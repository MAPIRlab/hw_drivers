cmake_minimum_required(VERSION 3.8)
project(giraff_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AvrComms.msg"
  "msg/Batteries.msg"
  "msg/CmdVelAvr.msg"
  "msg/Pilot.msg"
  "msg/Stalk.msg"
  "msg/StalkRef.msg"
  "srv/GiraffSrvMsg.srv"
  DEPENDENCIES std_msgs geometry_msgs nav_msgs sensor_msgs
 )

ament_package()
