# Braccio Robotic Arm

ROS pkg to interact with a Braccio robotic arm, designed specifically for university students. This pkg implements the Direct and Inverse Kinematics of Braccio, and offers a series of interfaces to interact with it from ROS. Additionally, the Arduino sketch is also included in this repository, necessary to interact with the robotic arm. Communication Arduino-ROS is handled via Serial Port.

![Braccio Robotic Arm](https://cdn.shopify.com/s/files/1/0438/4735/2471/products/T050000_04.back_710x468.jpg?v=1600873229)

The [Tinkerkit Braccio robot](https://store.arduino.cc/products/tinkerkit-braccio-robot) is a fully operational robotic arm, controlled via Arduino.
The braccio has a total of 6 axis:
  - Axis 1 – It is located at the base of a robot, and helps it to rotate from left to right.
  - Axis 2 – It helps the lower arm of a robot to move in an up and down motion.
  - Axis 3 – It allows the upper arm of a robot to move forward and backward.
  - Axis 4 – This axis is known as wrist roll, and it rotates the upper arm of a robot in a circular movement.
  - Axis 5 – It permits the wrist of the robot’s arm to raise and lower.
  - Axis 6 – It allows the wrist of the robot’s arm to rotate freely in a circular motion.
  - All the above six axes are controlled with the help of servo motors. 
   
 ## Action Server
 This pkg implements an action server to interact with the arm. The action allows two ways of contol:
 1. Providing as goal the angle value of each of the robot joints (goal_joints)
 2. Providing as goal a point to reach with the end effector (goal_position). In this mode the braccio_pkg uses the inverse kinematics model to estimate the joint angles to apply.

## Status Service
Additionally, the pkg offers an service server (Status) that returns the current position of the end efector, and the current values of all joints.
