# giraff_ros_driver
ROS2 package for interfacing the Giraff-X mobile robotic platform.

## Parameters

* **giraff_avr_port**: Path of the USB port connected to the AVR (robot motor controller). e.g. /dev/ttyUSB0

* **publish_odometry_over_tf**: Boolean to indicate if odometry estimation from wheel encoders must be published over tf

* **controller_mode**: Motor controller mode: 0 (default), 2(old mode with possibility to set v/w directly)

* **freq**: Frequency in hertzs of the main loop.


* **odom_topic**: Topic name where to publish the odometry msgs

* **base_frame_id**: Name of the tf_frame to identify the robot

* **odom_frame_id**: Name of the tf_frame to identify the odometry

* **head_frame_id**: Name of the tf_frame to identify the robot's head

* **stalk_frame_id**: Name of the tf_frame to identify the robot's stalk


* **max_linear_vel**: Maximun linear velocity in m/s accepted by the driver.

* **max_angular_vel**: Maximun angular velocity in rad/s accepted by the driver.
 
* **linear_acceleration**: Maximun linear acceleration m/s^2

* **angular_acceleration**: Maximun angular acceleration rad/2^2

* **virtual_gear_ratio**: Virtual Gear Ratio (max spin difference between right and left wheels)

* **cmd_vel_timeout**: [sec] after which not receiving a cmd_vel command we will Stop the robot

* **tilt_bias**: Angle (rad) to set the tilt=0 as vertical-screen. By default the tilt=0 is pointing down from teleoperation requirements.



## Subscribed topics

* **/cmd_vel** of type **geometry_msgs::msg::Twist** to set the angular and linear velocities.



## Published topics

* **[odom_topic]** of type **nav_msgs::msg::Odometry**, the odometry estimation from wheel encoders (not very reliable)

* **/giraff_node/battery** of type **sensor_msgs::msg::BatteryState**, the current status of the Giraff internal battery.

* **/giraff_node/buttons** of type **sensor_msgs::msg::Joy**, published when any of the red/green buttons are pressed, or when the dial is rotated.

* **/giraff_node/cmd_vel_avr** of type **giraff_interfaces::msg::CmdVelAvr**, in order to publish the parameters sent to the AVR for each motion command received (useful for debug).



## Services

**giraff_interfaces/GiraffSrvMsg**: A service to set/increase/decrease the Tilt and Stalk, as well as other serial-port related operations.

* set_tilt: Sets the tilt in (rad) in the range [-2PI/3 , 2PI/3]
* set_stalk: Sets the head height in the range [0 , 1000]

