cmake_minimum_required(VERSION 3.5)
project(sicktoolbox_wrapper)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(sicktoolbox REQUIRED)
find_package(diagnostic_updater REQUIRED)

# let the compiler search for headers in the include folder
include_directories(include)

# Add Executables
#add_executable(sickld ros/sickld/sickld.cpp)
#target_link_libraries(sickld ${catkin_LIBRARIES})

add_executable(sicklms ros/sicklms/sicklms.cpp)
ament_target_dependencies(sicklms rclcpp sicktoolbox sensor_msgs diagnostic_updater) 
#target_link_libraries(sicklms rclcpp sensor_msgs)

#add_executable(print_scans standalone/print_scans.cpp)
#target_link_libraries(print_scans ${catkin_LIBRARIES})

#add_executable(time_scans standalone/time_scans.cpp)
#target_link_libraries(time_scans ${catkin_LIBRARIES})

#add_executable(log_scans standalone/log_scans.cpp)
#target_link_libraries(log_scans ${catkin_LIBRARIES})

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS 
  #sickld sicklms print_scans time_scans
  sicklms
  DESTINATION lib/${PROJECT_NAME}
  #LIBRARY DESTINATION lib/${PROJECT_NAME}
  #ARCHIVE DESTINATION lib/${PROJECT_NAME}
  #RUNTIME DESTINATION bin/${PROJECT_NAME}
)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
ament_package()

## Mark cpp header files for installation
#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#  FILES_MATCHING PATTERN "*.hh"
#  PATTERN ".svn" EXCLUDE
#)

## Mark other files for installation (e.g. launch and bag files, etc.)
#install(FILES
#   sick_test.vcg
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)