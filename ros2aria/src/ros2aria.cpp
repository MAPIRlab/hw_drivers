#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "sensor_msgs/msg/battery_state.hpp"

#include <stdio.h>
#include <math.h>
#ifdef ADEPT_PKG
  #include <Aria.h>
  #include <ArRobotConfigPacketReader.h> // todo remove after ArRobotConfig implemented in AriaCoda
#else
  #include <Aria/Aria.h>
  #include <Aria/ArRobotConfigPacketReader.h> // todo remove after ArRobotConfig implemented in AriaCoda
#endif

#include <sstream>

using std::placeholders::_1;
using namespace std::chrono_literals;

class Ros2AriaNode : public rclcpp::Node
{
  public:
    Ros2AriaNode();
    ~Ros2AriaNode();
    
    int Setup();
    void cmdvel_cb(const geometry_msgs::msg::Twist::SharedPtr msg);
    void cmdvel_watchdog();
    void publish();
    //void dynamic_reconfigure(rosaria::RosAriaConfig &config,uint32_t level);
    //void readParameters();

  protected:

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pose_pub;  // odometry
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdvel_sub;

    //publishers for bumpers, sonars, sonar_pointcloud, voltage, recharge state, state of charge...
    //services for enabling/disabling motors

    //time when the last vel command was issue. If configured a watdhdog timer stop the robot when no new commnads are issued.
    rclcpp::Time veltime;
    rclcpp::TimerBase::SharedPtr cmdvel_watchdog_timer;

    std::string serial_port;
    int serial_baud;

    ArRobotConnector *conn;
    ArRobot *robot;
    ArPose pos;
    ArFunctorC<Ros2AriaNode> myPublishCB;

    nav_msgs::msg::Odometry position;
    sensor_msgs::msg::BatteryState bat_msg;

    std::unique_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster;

    geometry_msgs::msg::TransformStamped odom_trans;

    std::string frame_id_odom;
    std::string frame_id_base_link;
    std::string frame_id_bumper;
    std::string frame_id_sonar;


    // Robot Calibration Parameters (see readParameters() function)
    int TicksMM, DriftFactor, RevCount;  //If TicksMM or RevCount are <0, don't use. If DriftFactor is -99999, don't use (DriftFactor could be 0 or negative).

};


Ros2AriaNode::Ros2AriaNode(): Node("ros2aria_node"),myPublishCB(this,&Ros2AriaNode::publish),TicksMM(-1), DriftFactor(-99999), RevCount(-1)
{

  // Initialize ROS2 topic subs and pubs
  cmdvel_sub=this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel",1,std::bind(&Ros2AriaNode::cmdvel_cb,this,_1));

  pose_pub=this->create_publisher<nav_msgs::msg::Odometry>("/odom",1000);
  battery_pub = this->create_publisher<sensor_msgs::msg::BatteryState>("/battery",1000);

  // Initialize clock
  veltime=this->get_clock()->now();

  //Parameters
   // port and baud
  this->declare_parameter<std::string>("port", std::string("/dev/ttyUSB0") );

  //unsigned int does not work for parameter declaration...
  this->declare_parameter<int>("serial_baud", 115200);

  //if(serial_baud != 0)
  //  ROS_INFO("RosAria: set serial port baud rate %d", serial_baud);

  // handle debugging more elegantly
  //n.param( "debug_aria", debug_aria, false ); // default not to debug
  //n.param( "aria_log_filename", aria_log_filename, std::string("Aria.log") );

  // whether to connect to lasers using aria
  //n.param("publish_aria_lasers", publish_aria_lasers, false);

  // Get frame_ids to use.
  this->declare_parameter<std::string>("odom_frame", std::string("odom"));
  this->declare_parameter<std::string>("base_link_frame", std::string("base_link"));


  //n.param("bumpers_frame", frame_id_bumper, std::string("bumpers"));
  //n.param("sonar_frame", frame_id_sonar, std::string("sonar"));

  odom_broadcaster=std::make_unique<tf2_ros::TransformBroadcaster>(*this);
}

Ros2AriaNode::~Ros2AriaNode()
{
 // disable motors and sonar.
  robot->disableMotors();
  robot->disableSonar();

  robot->stopRunning();
  robot->waitForRunExit();
  Aria::shutdown();
}


int Ros2AriaNode::Setup()
{
  //Load ROS2 parameters
  rclcpp::Parameter port_param,baud_param,base_link_frame_param,odom_frame_param;
  this->get_parameter("port",port_param);
  this->get_parameter("serial_baud",baud_param);
  this->get_parameter("odom_frame",odom_frame_param);
  this->get_parameter("base_link_frame",base_link_frame_param);

  serial_port = port_param.as_string();
  serial_baud = baud_param.as_int();
  frame_id_odom = odom_frame_param.as_string();
  frame_id_base_link = base_link_frame_param.as_string();

  RCLCPP_INFO(this->get_logger(),"Setup: Port:[%s], Baud:[%d], Odom_frame:[%s], Base_link_frame:[%s]",serial_port.c_str(),serial_baud,frame_id_odom.c_str(),frame_id_base_link.c_str());
////////


  // Note, various objects are allocated here which are never deleted (freed), since Setup() is only supposed to be
  // called once per instance, and these objects need to persist until the process terminates.

  robot = new ArRobot();
  ArArgumentBuilder *args = new ArArgumentBuilder(); //  never freed
  ArArgumentParser *argparser = new ArArgumentParser(args); // Warning never freed
  argparser->loadDefaultArguments(); // adds any arguments given in /etc/Aria.args.  Useful on robots with unusual serial port or baud rate (e.g. pioneer lx)

  // Now add any parameters given via ros params (see RosAriaNode constructor):

  // if serial port parameter contains a ':' character, then interpret it as hostname:tcpport
  // for wireless serial connection. Otherwise, interpret it as a serial port name.
  size_t colon_pos = serial_port.find(":");
  if (colon_pos != std::string::npos)
  {
    args->add("-remoteHost"); // pass robot's hostname/IP address to Aria
    args->add(serial_port.substr(0, colon_pos).c_str());
    args->add("-remoteRobotTcpPort"); // pass robot's TCP port to Aria
    args->add(serial_port.substr(colon_pos+1).c_str());
  }
  else
  {
    args->add("-robotPort %s", serial_port.c_str()); // pass robot's serial port to Aria
  }

  // if a baud rate was specified in baud parameter
  if(serial_baud != 0)
  {
    args->add("-robotBaud %d", serial_baud);
  }

  /*
  if( debug_aria )
  {
    // turn on all ARIA debugging
    args->add("-robotLogPacketsReceived"); // log received packets
    args->add("-robotLogPacketsSent"); // log sent packets
    args->add("-robotLogVelocitiesReceived"); // log received velocities
    args->add("-robotLogMovementSent");
    args->add("-robotLogMovementReceived");
    ArLog::init(ArLog::File, ArLog::Verbose, aria_log_filename.c_str(), true);
  }

*/
  // Connect to the robot
  conn = new ArRobotConnector(argparser, robot); // warning never freed
  if (!conn->connectRobot()) {
    RCLCPP_ERROR(this->get_logger(),"RosAria: ARIA could not connect to robot! (Check ~port parameter is correct, and permissions on port device, or any errors reported above)");
    return 1;
  }

 // if(publish_aria_lasers)
 //   laserConnector = new ArLaserConnector(argparser, robot, conn);

  // causes ARIA to load various robot-specific hardware parameters from the robot parameter file in /usr/local/Aria/params
  if(!Aria::parseArgs())
  {
    RCLCPP_ERROR(this->get_logger(),"RosAria: ARIA error parsing ARIA startup parameters!");
    return 1;
  }

    RCLCPP_INFO(this->get_logger(),"Connected!!!");
  //readParameters();

  // Start dynamic_reconfigure server
  //dynamic_reconfigure_server = new dynamic_reconfigure::Server<rosaria::RosAriaConfig>;
  /*
  // Setup Parameter Minimums and maximums
  rosaria::RosAriaConfig dynConf_min;
  rosaria::RosAriaConfig dynConf_max;

  dynConf_max.trans_accel = robot->getAbsoluteMaxTransAccel() / 1000;
  dynConf_max.trans_decel = robot->getAbsoluteMaxTransDecel() / 1000;
  // TODO: Fix rqt dynamic_reconfigure gui to handle empty intervals
  // Until then, set unit length interval.
  dynConf_max.lat_accel = ((robot->getAbsoluteMaxLatAccel() > 0.0) ? robot->getAbsoluteMaxLatAccel() : 0.1) / 1000;
  dynConf_max.lat_decel = ((robot->getAbsoluteMaxLatDecel() > 0.0) ? robot->getAbsoluteMaxLatDecel() : 0.1) / 1000;
  dynConf_max.rot_accel = robot->getAbsoluteMaxRotAccel() * M_PI/180;
  dynConf_max.rot_decel = robot->getAbsoluteMaxRotDecel() * M_PI/180;

  dynConf_min.trans_accel = 0;
  dynConf_min.trans_decel = 0;
  dynConf_min.lat_accel = 0;
  dynConf_min.lat_decel = 0;
  dynConf_min.rot_accel = 0;
  dynConf_min.rot_decel = 0;

  dynConf_min.TicksMM     = 0;
  dynConf_max.TicksMM     = 200;
  dynConf_min.DriftFactor = -99999;
  dynConf_max.DriftFactor = 32767;
  dynConf_min.RevCount    = 0;
  dynConf_max.RevCount    = 65535;

  dynamic_reconfigure_server->setConfigMax(dynConf_max);
  dynamic_reconfigure_server->setConfigMin(dynConf_min);


  rosaria::RosAriaConfig dynConf_default;
  dynConf_default.trans_accel = robot->getTransAccel() / 1000;
  dynConf_default.trans_decel = robot->getTransDecel() / 1000;
  dynConf_default.lat_accel   = robot->getLatAccel() / 1000;
  dynConf_default.lat_decel   = robot->getLatDecel() / 1000;
  dynConf_default.rot_accel   = robot->getRotAccel() * M_PI/180;
  dynConf_default.rot_decel   = robot->getRotDecel() * M_PI/180;

  dynConf_default.TicksMM     = 0;
  dynConf_default.DriftFactor = -99999;
  dynConf_default.RevCount    = 0;

  dynamic_reconfigure_server->setConfigDefault(dynConf_default);

  dynamic_reconfigure_server->setCallback(boost::bind(&RosAriaNode::dynamic_reconfigureCB, this, _1, _2));
*/

  // Enable the motors
  robot->enableMotors();

  // disable sonars on startup
  robot->disableSonar();

  // callback will  be called by ArRobot background processing thread for every SIP data packet received from robot
  robot->addSensorInterpTask("ROSPublishingTask", 100, &myPublishCB);

  // Initialize bumpers with robot number of bumpers
//  bumpers.front_bumpers.resize(robot->getNumFrontBumpers());
//  bumpers.rear_bumpers.resize(robot->getNumRearBumpers());

  // Run ArRobot background processing thread
  robot->runAsync(true);

  // connect to lasers and create publishers
 /* if(publish_aria_lasers)
  {
    ROS_INFO_NAMED("rosaria", "rosaria: Connecting to laser(s) configured in ARIA parameter file(s)...");
    if (!laserConnector->connectLasers())
    {
      ROS_FATAL_NAMED("rosaria", "rosaria: Error connecting to laser(s)...");
      return 1;
    }

    robot->lock();
    const std::map<int, ArLaser*> *lasers = robot->getLaserMap();
    ROS_INFO_NAMED("rosaria", "rosaria: there are %lu connected lasers", lasers->size());
    for(std::map<int, ArLaser*>::const_iterator i = lasers->begin(); i != lasers->end(); ++i)
    {
      ArLaser *l = i->second;
      int ln = i->first;
      std::string tfname("laser");
      if(lasers->size() > 1 || ln > 1) // no number if only one laser which is also laser 1
        tfname += ln;
      tfname += "_frame";
      ROS_INFO_NAMED("rosaria", "rosaria: Creating publisher for laser #%d named %s with tf frame name %s", ln, l->getName(), tfname.c_str());
      new LaserPublisher(l, n, true, tfname);
    }
    robot->unlock();
    ROS_INFO_NAMED("rosaria", "rosaria: Done creating laser publishers");
  }

  // subscribe to command topics
  //ROS2->This is done in the class constructor
  cmdvel_sub = n.subscribe( "cmd_vel", 1, (boost::function <void(const geometry_msgs::TwistConstPtr&)>)
      boost::bind(&RosAriaNode::cmdvel_cb, this, _1 ));
*/
  // register a watchdog for cmd_vel timeout
  double cmdvel_timeout_param = 0.6;
  //n.param("cmd_vel_timeout", cmdvel_timeout_param, 0.6);


//  cmdvel_timeout = ros::Duration(cmdvel_timeout_param);
  if (cmdvel_timeout_param > 0.0)
    cmdvel_watchdog_timer = this->create_wall_timer(
      100ms, std::bind(&Ros2AriaNode::cmdvel_watchdog, this));

  RCLCPP_INFO(this->get_logger(),"ros2aria: Setup complete");
  return 0;
}

void Ros2AriaNode::publish()
{
  //tf::poseTFToMsg(tf::Transform(tf::createQuaternionFromYaw(pos.getTh()*M_PI/180), tf::Vector3(pos.getX()/1000,
  // pos.getY()/1000, 0)), position.pose.pose); //Aria returns pose in mm.

  // Set Odometry "position" from current Aria pose estimation "pos"
  pos = robot->getPose();
  tf2::Quaternion q;
  q.setRPY(0,0,pos.getTh()*M_PI/180);

  position.pose.pose.position.x = pos.getX()/1000;
  position.pose.pose.position.y = pos.getY()/1000;
  position.pose.pose.position.z = 0;
  position.pose.pose.orientation.x = q.getX();
  position.pose.pose.orientation.y = q.getY();
  position.pose.pose.orientation.z = q.getZ();
  position.pose.pose.orientation.w = q.getW();
  // set twist (part of odometry)
  position.twist.twist.linear.x = robot->getVel()/1000.0; //Aria returns velocity in mm/s.
  position.twist.twist.linear.y = robot->getLatVel()/1000.0;
  position.twist.twist.angular.z = robot->getRotVel()*M_PI/180;
  // set headers and timestamp
  position.header.frame_id = frame_id_odom;
  position.child_frame_id = frame_id_base_link;
  position.header.stamp = this->get_clock()->now();

  // 1. Publish Odometry as a Topic
  pose_pub->publish(position);

  // 2. Publish transform odom->base_link as TF
  odom_trans.header.stamp = this->get_clock()->now();
  odom_trans.header.frame_id = frame_id_odom;
  odom_trans.child_frame_id = frame_id_base_link;

  odom_trans.transform.translation.x = pos.getX()/1000;
  odom_trans.transform.translation.y = pos.getY()/1000;
  odom_trans.transform.translation.z = 0.0;  
  odom_trans.transform.rotation.x=q.getX();
  odom_trans.transform.rotation.y=q.getY();
  odom_trans.transform.rotation.z=q.getZ();
  odom_trans.transform.rotation.w=q.getW();

  odom_broadcaster->sendTransform(odom_trans);

  // 3. Publish Battery
  bat_msg.header.stamp = this->get_clock()->now();
  bat_msg.header.frame_id = frame_id_base_link;
  bat_msg.voltage = robot->getRealBatteryVoltage();
  bat_msg.percentage = robot->getStateOfCharge();
  bat_msg.power_supply_status = int(robot->isChargerPowerGood());
  bat_msg.temperature = float(robot->getTemperature());

  battery_pub->publish(bat_msg);
}

void Ros2AriaNode::cmdvel_cb(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  veltime = this->get_clock()->now();
  //veltime = ros::Time::now();

  //RCLCPP_INFO(this->get_logger(), "new speed: [%0.2f,%0.2f](%0.3f)", msg->linear.x*1e3, msg->angular.z, veltime.nanoseconds()/1e9 );

  robot->lock();
  robot->setVel(msg->linear.x*1e3);
  if(robot->hasLatVel())
    robot->setLatVel(msg->linear.y*1e3);
  robot->setRotVel(msg->angular.z*180/M_PI);
  robot->unlock();

  //RCLCPP_INFO(this->get_logger(),"RosAria: sent vels to to aria (time %f): x vel %f mm/s, y vel %f mm/s, ang vel %f deg/s", veltime.nanoseconds()/1e9, (double) msg->linear.x * 1e3, (double) msg->linear.y * 1e3, (double) msg->angular.z * 180/M_PI);
}


void Ros2AriaNode::cmdvel_watchdog()
{
  // stop robot if no cmd_vel message was received for 0.6 seconds
  if (this->get_clock()->now() - veltime > rclcpp::Duration(0.6,0))
  {
    robot->lock();
    robot->setVel(0.0);
    if(robot->hasLatVel())
      robot->setLatVel(0.0);
    robot->setRotVel(0.0);
    robot->unlock();
  }
}



// ===================================================== MAIN ==============================
int main(int argc, char** argv)
{
  // init ROS2 node
  rclcpp::init(argc,argv);

  // Crete Aria object
  auto robot = std::make_shared<Ros2AriaNode>();
  Aria::init();
  robot->Setup();
  rclcpp::Rate loop_rate(90);
  //trying to publish at 90 Hz, but Aria is also publishing in a thread at 10 Hz...
  while (rclcpp::ok())
  {
    // publish odometry 
    robot->publish();

    // check for cmd_vel
    rclcpp::spin_some(robot);

    // sleep
    loop_rate.sleep();
  }
  return 0;
}
