cmake_minimum_required(VERSION 3.8)
project(giraff_ros2_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(giraff_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


include_directories(include)

add_executable(giraff_node src/giraff_node.cpp)
ament_target_dependencies(giraff_node rclcpp giraff_interfaces std_msgs sensor_msgs tf2 tf2_ros tf2_geometry_msgs)
#target_compile_definitions(giraff_node PRIVATE -D_GIRAFF_AVR_DEBUG_)

install(TARGETS
  giraff_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
launch
DESTINATION share/${PROJECT_NAME})

ament_package()












