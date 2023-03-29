cmake_minimum_required(VERSION 3.8)
project(braccio)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Declare Services/Actions
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Status.srv"
  "action/BraccioCMD.action"
  DEPENDENCIES geometry_msgs # Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg
)

# Includes
include_directories(
  include
)

# Add Executables
add_executable(braccio_driver src/braccio_driver.cpp)
add_executable(braccio_action_client src/braccio_action_client.cpp)

# Declare and install executables
install(TARGETS
  braccio_driver
  braccio_action_client
  DESTINATION lib/${PROJECT_NAME}
)

# Dependencies
ament_target_dependencies(braccio_driver rclcpp rclcpp_action geometry_msgs std_msgs)
ament_target_dependencies(braccio_action_client rclcpp rclcpp_action geometry_msgs std_msgs)


# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Service/Action Dependencies
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)
target_link_libraries(braccio_driver "${cpp_typesupport_target}")
target_link_libraries(braccio_action_client "${cpp_typesupport_target}")

# Pkg
ament_package()
