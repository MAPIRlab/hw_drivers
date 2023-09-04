/***********************************************************************/
/**                                                                    */
/** giraff_node.h                                                      */
/**                                                                    */
/** Copyright (c) 2015, Service Robotics Lab.                          */
/**                     http://robotics.upo.es                         */
/**                                                                    */
/** All rights reserved.                                               */
/**                                                                    */
/** Authors:                                                           */
/** Ignacio Perez-Hurtado (maintainer)                                 */
/** Noe Perez                                                          */
/** Rafael Ramon                                                       */
/** Fernando Caballero                                                 */
/** Jesus Capitan                                                      */
/** Luis Merino                                                        */
/**                                                                    */
/** This software may be modified and distributed under the terms      */
/** of the BSD license. See the LICENSE file for details.              */
/**                                                                    */
/** http://www.opensource.org/licenses/BSD-3-Clause                    */
/**                                                                    */
/***********************************************************************/

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>

#include "giraff_ros2_driver/giraff_manager.h"
#include <boost/bind.hpp>
#include <limits>

// msgs (to move to mapir_sensor_msgs)
#include <giraff_interfaces/msg/cmd_vel_avr.hpp>

// srv
#include <giraff_interfaces/srv/giraff_srv_msg.hpp>

#define NODE_VERSION 2.1

// Global vars
//-----------
GiraffManager* giraff = NULL;                                                        // Communications with AVR (serial port)
rclcpp::Publisher<giraff_interfaces::msg::CmdVelAvr>::SharedPtr cmd_vel_avr_pub_ptr; // Publish the cmd_vel command sent to AVR (debug)
rclcpp::Time cmd_vel_time;                                                           // Last time a cmd_vel command was received.
rclcpp::Time e_stop_time;                                                            // Last time a e-stop command was received.
bool e_stop_timer;
bool e_stop;
double tilt_bias;
bool robot_stopped = true;
u_int32_t current_red, current_green, current_dial;
u_int32_t old_red, old_green;
int current_dial_int, old_dial_int;
bool verbose;
bool publish_other_tf;

// Keep values to publish odometry
double yaw = 0.0;
double roll = 0.0;
double pitch = 0.0;
double ang_vel = 0.0;
double lin_vel = 0.0;
double pos_x = 0.0;
double pos_y = 0.0;

// CallBack Functions (subscription topics)
//-------------------------------------------

// New cmd_vel msg --> Send to AVR
void cmdVelReceived(const geometry_msgs::msg::Twist cmd_vel)
{
    // Check if E-stop is pressed (Emergency)
    // If so, do nothing!
    if (e_stop)
        return;

    // No emergency, therefore, send command to robot base
    cmd_vel_time = rclcpp::Clock().now();
    GiraffState state = giraff->setVelocity(cmd_vel.linear.x, cmd_vel.angular.z);
    // RCLCPP_INFO("[Giraff_ros_driver] New cmd_vel command: %.3f m/s, %.3f rad/s",cmd_vel->linear.x, cmd_vel->angular.z);
    robot_stopped = false;

    // Publish topic over ROS (to see the cmd_vel_avr (AVR state))
    if (cmd_vel_avr_pub_ptr != NULL)
    {
        giraff_interfaces::msg::CmdVelAvr cmd_vel_avr_msg;
        cmd_vel_avr_msg.header.stamp = rclcpp::Clock().now();
        cmd_vel_avr_msg.cmd_vel.linear.x = state.lin_speed;  // filtered speed sent to AVR
        cmd_vel_avr_msg.cmd_vel.angular.z = state.ang_speed; // filtered speed sent to AVR
        cmd_vel_avr_msg.mode = state.mode;
        cmd_vel_avr_msg.a = state.a;              // lineal acceleration
        cmd_vel_avr_msg.aw = state.aw;            // lineal acceleration
        cmd_vel_avr_msg.v = state.v * state.p;    // speed (m/s)
        cmd_vel_avr_msg.vg = state.vg * PI / 180; // angular speed (rad/s)
        cmd_vel_avr_msg.p = state.p;
        cmd_vel_avr_pub_ptr->publish(cmd_vel_avr_msg);
    }
}

// Attend service requests
bool process_srv_request(const std::shared_ptr<giraff_interfaces::srv::GiraffSrvMsg::Request> req,
    std::shared_ptr<giraff_interfaces::srv::GiraffSrvMsg::Response> res)
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[Giraff_ros_driver] SRV [%s] requested...", req->command.c_str());

    if (req->command.compare("flush") == 0)
        giraff->flush_serial_interface();

    else if (req->command.compare("restart") == 0)
        giraff->restart_serial_interface();

    else if (req->command.compare("set_stalk") == 0) // 0-1000
        giraff->setStalk((int)req->arg);

    else if (req->command.compare("inc_stalk") == 0) // h + 10steps in range [0, 1000]
        giraff->incStalk();

    else if (req->command.compare("dec_stalk") == 0) // h - 10steps in range [0, 1000]
        giraff->decStalk();

    else if (req->command.compare("set_tilt") == 0) // range: -2PI/3 to 2PI/3
        giraff->setTilt(req->arg + tilt_bias);

    else if (req->command.compare("inc_tilt") == 0) // tilt + 0.02 rad
        giraff->incTilt();

    else if (req->command.compare("dec_tilt") == 0) // tilt + 0.02 rad
        giraff->decTilt();

    else
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[Giraff_ros_driver] SRV requested [%s] is not implemeted!", req->command.c_str());
    res->result = true;
    return true;
}

// Publish button state change! Add buttons in order (red, greeen, dial, e-stop)
void publish_buttons(rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr buttons_pub, std::string& base_frame_id)
{
    sensor_msgs::msg::Joy joy;
    joy.header.stamp = rclcpp::Clock().now();
    joy.header.frame_id == base_frame_id;

    // red
    // published data is state-change: 0=no_change, 1=pressed
    joy.buttons.push_back(current_red - old_red);

    // green
    // published data is state-change:  0=no_change, 1=pressed
    joy.buttons.push_back(current_green - old_green);

    // dial
    joy.buttons.push_back(current_dial_int - old_dial_int); // dial increment steps

    // e-stop (emergency)
    if (e_stop)
        joy.buttons.push_back(1);
    else
        joy.buttons.push_back(0);

    // publish msg over topic
    buttons_pub->publish(joy);
}

//-----------------------------------------------------------------------------------
//                                   MAIN
//-----------------------------------------------------------------------------------
int main(int argc, char** argv)
{
    try
    {
        rclcpp::init(argc, argv);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[Giraff_ros_driver] Initializing node...");
        std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("GiraffNode");
        // Read Parameters
        //----------------
        std::string giraff_avr_port;
        bool publish_odometry_over_tf;
        std::string odom_topic;
        double freq;

        giraff_avr_port = node->declare_parameter<std::string>("giraff_avr_port", "/dev/ttyS1");

        publish_odometry_over_tf = node->declare_parameter<bool>("publish_odometry_over_tf", false);

        publish_other_tf = node->declare_parameter<bool>("publish_other_tf", false);

        odom_topic = node->declare_parameter<std::string>("odom_topic", "odom");

        freq = node->declare_parameter<double>("freq", 100.0);

        std::string base_frame_id;
        std::string odom_frame_id;
        std::string head_frame_id;
        std::string stalk_frame_id;

        base_frame_id = node->declare_parameter<std::string>("base_frame_id", "base_link");

        odom_frame_id = node->declare_parameter<std::string>("odom_frame_id", "odom");

        head_frame_id = node->declare_parameter<std::string>("head_frame_id", "giraff_head");

        stalk_frame_id = node->declare_parameter<std::string>("stalk_frame_id", "giraff_stalk");

        std::string base_footprint_frame_id = node->declare_parameter<std::string>("base_footprint_frame_id", "giraff_base_footprint");
        std::string camera_frame_id = node->declare_parameter<std::string>("camera_frame_id", "giraff_camera");
        std::string laser_frame_id = node->declare_parameter<std::string>("laser_frame_id", "giraff_laser_frame");

        double max_lv, max_av, acc_lin, acc_ang, vgr;
        int controller_mode;
        int timeout;

        controller_mode = node->declare_parameter<int>("controller_mode", 2); // Valid modes are 0 and 2

        max_lv = node->declare_parameter<double>("max_linear_vel", 0.6); // m/s

        max_av = node->declare_parameter<double>("max_angular_vel", 0.7); // rad/s

        acc_lin = node->declare_parameter<double>("linear_acceleration", 0.2); // Linear acceleration (m/s2)

        acc_ang = node->declare_parameter<double>("angular_acceleration", 0.001); // Angular acceleration (rad/s2)

        vgr = node->declare_parameter<double>("virtual_gear_ratio", 20.0); // Virtual Gear Ratio (max spin difference between right and left wheels)

        timeout = node->declare_parameter<int>("cmd_vel_timeout", 1.0); // sec after which not receiving a cmd_vel command we will Stop the robot

        tilt_bias = node->declare_parameter<double>("tilt_bias", 0.6); // There's something weird with the tilt, which is not centered. A bias term may be needed

        std::string batt_tech, batt_serial;
        double batt_capacity;

        batt_tech = node->declare_parameter<std::string>("battery_technology", "NIMH"); // NIMH, LION, LIPO, LIFE, NICD, LIMN

        batt_capacity = node->declare_parameter<double>("battery_design_capacity", 4.4); // Capacity in Ah (design capacity)

        batt_serial = node->declare_parameter<std::string>("battery_serial_number", "giraff_battery"); // Serial number or identification name

        verbose = node->declare_parameter<bool>("verbose", true);

        // Create Giraff-Controller (access to AVR-serial port)
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[Giraff_ros_driver] Creating AVR-Manager...");
        giraff = new GiraffManager(giraff_avr_port, controller_mode, max_lv, max_av, acc_lin, acc_ang, vgr, batt_tech, batt_capacity, batt_serial);

        // Publishers and Subscribers
        //--------------------------
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[Giraff_ros_driver] Initializing Publishers...");
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub = node->create_publisher<nav_msgs::msg::Odometry>(odom_topic, 5);
        rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr batteries_pub = node->create_publisher<sensor_msgs::msg::BatteryState>("battery", 5);
        rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr buttons_pub = node->create_publisher<sensor_msgs::msg::Joy>("buttons", 5);
        rclcpp::Publisher<giraff_interfaces::msg::CmdVelAvr>::SharedPtr cmd_vel_avr_pub = node->create_publisher<giraff_interfaces::msg::CmdVelAvr>("cmd_vel_avr", 5);
        cmd_vel_avr_pub_ptr = node->create_publisher<giraff_interfaces::msg::CmdVelAvr>("cmd_vel_avr", 5);
        // cmd_vel_avr_pub_ptr = &cmd_vel_avr_pub; //Global pointer

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[Giraff_ros_driver] Initializing Subscribers...");
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub = node->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 1, std::bind(&cmdVelReceived, std::placeholders::_1));

        // Services
        //----------------
        rclcpp::Service<giraff_interfaces::srv::GiraffSrvMsg>::SharedPtr service = node->create_service<giraff_interfaces::srv::GiraffSrvMsg>("giraff_ros2_driver/command", process_srv_request);

        // Set init state
        //---------------
        rclcpp::Time current_time, last_time;
        last_time = rclcpp::Clock().now();
        cmd_vel_time = rclcpp::Clock().now();
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*node);
        rclcpp::Rate(8.0).sleep();
        giraff->setTilt(tilt_bias);
        giraff->get_button_data(current_red, current_green, current_dial);
        old_red = current_red;
        old_green = current_green;
        old_dial_int = (int32_t)current_dial;
        e_stop_time = rclcpp::Clock().now();
        e_stop_timer = false;
        e_stop = false;

        // Main Loop
        //----------
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[Giraff_ros_driver] Looping");
        rclcpp::Rate r(freq);
        while (rclcpp::ok())
        {
            // Read Odometry and velocities as estimated by base
            // Odometry is absolute in ROS --> Pose of the robot with respect to starting point
            giraff->get2DOdometry(pos_x, pos_y, yaw, lin_vel, ang_vel);
            if (verbose)
                RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "[Giraff_ros_driver] GiraffOdom [%f(m) %f(m) %f(rad)] [%f(m/s) %f(rad/s)]", pos_x, pos_y, yaw, lin_vel, ang_vel);

            // Check if time-out --> StopGiraff + flush serial port
            current_time = rclcpp::Clock().now();
            double cmd_vel_sec = (current_time - cmd_vel_time).seconds();
            if (!robot_stopped && cmd_vel_sec >= timeout)
            {
                if (verbose)
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[Giraff_ros_driver] TimeOut reached --> Stopping Robot");
                // giraff->flush_serial_interface();     //seems to carry problems -_-
                GiraffState state = giraff->setVelocity(0.0, 0.0);

                // Publish new cmd_vel_avr topic
                if (cmd_vel_avr_pub_ptr != NULL)
                {
                    giraff_interfaces::msg::CmdVelAvr cmd_vel_avr_msg;
                    cmd_vel_avr_msg.header.stamp = rclcpp::Clock().now();
                    cmd_vel_avr_msg.cmd_vel.linear.x = state.lin_speed;  // filtered speed (m/s) sent to AVR
                    cmd_vel_avr_msg.cmd_vel.angular.z = state.ang_speed; // filtered speed (rad/s) sent to AVR
                    cmd_vel_avr_msg.mode = state.mode;
                    cmd_vel_avr_msg.a = state.a;              // lineal acceleration
                    cmd_vel_avr_msg.aw = state.aw;            // lineal acceleration
                    cmd_vel_avr_msg.v = state.v * state.p;    // speed (m/s)
                    cmd_vel_avr_msg.vg = state.vg * PI / 180; // angular speed (rad/s)
                    cmd_vel_avr_msg.p = state.p;
                    cmd_vel_avr_pub_ptr->publish(cmd_vel_avr_msg);
                }
                if ((state.lin_speed == 0.0) && (state.ang_speed == 0.0))
                    robot_stopped = true;
            }

            // Publish the odometry message over ROS topic
            nav_msgs::msg::Odometry odom;
            odom.header.stamp = current_time;
            odom.header.frame_id = odom_frame_id;
            odom.pose.pose.position.x = pos_x;
            odom.pose.pose.position.y = pos_y;
            odom.pose.pose.position.z = 0.0;
            tf2::Quaternion q;
            q.setRPY(0, 0, yaw);
            odom.pose.pose.orientation = toMsg(q);
            // set the velocity
            odom.child_frame_id = base_frame_id;
            odom.twist.twist.linear.x = lin_vel;
            odom.twist.twist.linear.y = 0.0;
            odom.twist.twist.angular.z = ang_vel;
            // publish the odometry
            odom_pub->publish(odom);

            if (publish_odometry_over_tf)
            {
                // Publish TF (\odom -> \base_footprint)
                geometry_msgs::msg::TransformStamped odom_trans;
                odom_trans.header.stamp = current_time;
                odom_trans.header.frame_id = odom_frame_id;
                odom_trans.child_frame_id = base_footprint_frame_id;
                odom_trans.transform.translation.x = pos_x;
                odom_trans.transform.translation.y = pos_y;
                odom_trans.transform.translation.z = 0.0;
                tf2::Quaternion q;
                q.setRPY(0, 0, yaw);
                odom_trans.transform.rotation = toMsg(q);
                tf_broadcaster->sendTransform(odom_trans);
            }
            if (publish_other_tf)
            {
                // Publish TF (\base_footprint -> \base_link)
                geometry_msgs::msg::TransformStamped footprint_trans;
                footprint_trans.header.stamp = current_time;
                footprint_trans.header.frame_id = base_footprint_frame_id;
                footprint_trans.child_frame_id = base_frame_id;
                footprint_trans.transform.translation.x = 0.0;
                footprint_trans.transform.translation.y = 0.0;
                footprint_trans.transform.translation.z = 0.2;
                q.setRPY(0, 0, 0);
                footprint_trans.transform.rotation = toMsg(q);
                tf_broadcaster->sendTransform(footprint_trans);

                // Publish TF (\base_link -> \hokuyo_front)
                geometry_msgs::msg::TransformStamped laser_trans;
                laser_trans.header.stamp = current_time;
                laser_trans.header.frame_id = base_frame_id;
                laser_trans.child_frame_id = laser_frame_id;
                laser_trans.transform.translation.x = 0.2;
                laser_trans.transform.translation.y = 0.0;
                laser_trans.transform.translation.z = 0.0;
                q.setRPY(0, 0, 0);
                laser_trans.transform.rotation = toMsg(q);
                tf_broadcaster->sendTransform(laser_trans);

                // Publish TF (\head -> \camera)
                geometry_msgs::msg::TransformStamped camera_trans;
                camera_trans.header.stamp = current_time;
                camera_trans.header.frame_id = head_frame_id;
                camera_trans.child_frame_id = camera_frame_id;
                camera_trans.transform.translation.x = 0.0;
                camera_trans.transform.translation.y = 0.0;
                camera_trans.transform.translation.z = 0.0;
                q.setRPY(0, 0, 0);
                camera_trans.transform.rotation = toMsg(q);
                // tf_broadcaster->sendTransform(camera_trans);
            }

            float altura_en_metros = (116 + 0.025 * giraff->getStalk()) * 0.01;
            float tilt_en_rad = giraff->getTilt() - tilt_bias;

            // Publish TF (\base_link -> \stalk)
            geometry_msgs::msg::TransformStamped stalk_trans;
            stalk_trans.header.stamp = current_time;
            stalk_trans.header.frame_id = base_frame_id;
            stalk_trans.child_frame_id = stalk_frame_id;
            stalk_trans.transform.translation.x = 0.0;
            stalk_trans.transform.translation.y = 0.0;
            stalk_trans.transform.translation.z = altura_en_metros;
            q.setRPY(0, 0, 0);
            stalk_trans.transform.rotation = toMsg(q);
            tf_broadcaster->sendTransform(stalk_trans);

            // Publish TF (\stalk -> \giraff_head)
            geometry_msgs::msg::TransformStamped head_trans;
            head_trans.header.stamp = current_time;
            head_trans.header.frame_id = stalk_frame_id;
            head_trans.child_frame_id = head_frame_id;
            head_trans.transform.translation.x = 0.0;
            head_trans.transform.translation.y = 0.0;
            head_trans.transform.translation.z = 0.0;

            q.setRPY(0.0, -tilt_en_rad, 0.0);
            head_trans.transform.rotation = toMsg(q);
            tf_broadcaster->sendTransform(head_trans);

            // Publish the state of the batteries
            sensor_msgs::msg::BatteryState battmsg;
            giraff->getGiraffBatteryData(battmsg);
            batteries_pub->publish(battmsg);

            // Buttons pressed
            //-----------------
            giraff->get_button_data(current_red, current_green, current_dial);
            // buttons are cumulative uint_32 (number of hits since starting time and only considering RISE events!).
            int red_hit = current_red - old_red;
            int green_hit = current_green - old_green;

            // Cast from uint32 (all positive values) to int32_t (positive and negative)
            current_dial_int = (int32_t)(current_dial - std::numeric_limits<uint32_t>::max() - 1);
            int dial_hit = current_dial_int - old_dial_int;

            // update state
            old_red = current_red;
            old_green = current_green;
            old_dial_int = current_dial_int;

            // Detect E-Stop (Emergency)
            // Since we can no detect button releases, we use hits both to activate and deactivate the e-stop

            if (red_hit >= 1 && green_hit >= 1)
            {
                // EMERGENCY BUTTON PRESS DETECTED!
                if (!e_stop)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "[GIRAFF_ROS_DRIVER] E-Stop button Pressed! Closing connection with GiraffX base.");
                    // 1. Disable access to the motors (NO CMD_VEL msg sent to base)
                    // 2. Notify the task manager to Cancell all tasks in execution or planned
                    e_stop = true;
                }
                else
                {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "[GIRAFF_ROS_DRIVER] E-Stop button Released! Restoring connection with GiraffX base.");
                    e_stop = false;
                }
            }
            /*
            // PRO VERSION that requires release events (not currently available)
            if (!e_stop)
            {
                //Emergency is not active, check!
                if (!e_stop_timer)
                {
                    if (verbose)
                        ROS_INFO("[giraff_ros_driver] Starting E-Stop timer");

                    //first time both buttons are pressed
                    e_stop_timer = true;
                    e_stop_time = ros::Time::now();
                }
                else
                {
                    //If e-stop pressed for more than 1 sec --> rise EMERGENCY!
                    if ( (ros::Time::now() - e_stop_time).toSec() > 1.0 )
                    {
                        //EMERGENCY BUTTON PRESS DETECTED!
                        ROS_ERROR("[GIRAFF_ROS_DRIVER] E-Stop button Pressed! Closing connection with GiraffX base.");
                        //1. Disable access to the motor (NO CMD_VEL msg sent to base)
                        e_stop = true;
                        //2. Notify the task manager to Cancell all tasks in execution or planned
                        publish_buttons(buttons_pub, base_frame_id);
                    }
                }
            }
            */

            // Publish button state change! Add buttons in order (red, greeen, dial, e-stop)
            if (red_hit || green_hit || dial_hit)
            {
                if (verbose)
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[giraff_ros_driver] Buttons pressed are: RED=%u, GREEN=%u, DIAL=%i, E-STOP=%u", red_hit, green_hit, dial_hit, e_stop);

                publish_buttons(buttons_pub, base_frame_id);
            }

            r.sleep();
            rclcpp::spin_some(node); // check subscriptions!
        }
    }
    catch (GiraffManagerException e)
    {
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "%s", e.what());
        if (giraff != NULL)
        {
            delete giraff;
        }
        // RCLCPP_BREAK();
        RCLCPP_LOCAL(rclcpp::get_logger("rclcpp"), "no se para que es esta linea");
    }

    // Leaving
    delete giraff;
    return 0;
}
