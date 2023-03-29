cmake_minimum_required(VERSION 3.5)
project(sicktoolbox)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Threads REQUIRED)

# Include Headers (for libraries)
include_directories(include)

# Driver libraries
#add_library(SickLD c++/drivers/ld/sickld/SickLD.cc c++/drivers/ld/sickld/SickLDBufferMonitor.cc c++/drivers/ld/sickld/SickLDMessage.cc)
#target_link_libraries(SickLD ${catkin_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})

#add_library(SickLMS1xx c++/drivers/lms1xx/sicklms1xx/SickLMS1xx.cc c++/drivers/lms1xx/sicklms1xx/SickLMS1xxBufferMonitor.cc c++/drivers/lms1xx/sicklms1xx/SickLMS1xxMessage.cc)
#target_link_libraries(SickLMS1xx ${catkin_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})

#add_library(SickLMS2xx c++/drivers/lms2xx/sicklms2xx/SickLMS2xx.cc c++/drivers/lms2xx/sicklms2xx/SickLMS2xxBufferMonitor.cc c++/drivers/lms2xx/sicklms2xx/SickLMS2xxMessage.cc)
#target_link_libraries(SickLMS2xx rclcpp ${CMAKE_THREAD_LIBS_INIT})

# SICK LMS!
add_library(SickLMS2xx 
  c++/drivers/lms2xx/sicklms2xx/SickLMS2xx.cc 
  c++/drivers/lms2xx/sicklms2xx/SickLMS2xxBufferMonitor.cc 
  c++/drivers/lms2xx/sicklms2xx/SickLMS2xxMessage.cc
)
#ament_auto_add_library(SickLMS2xx SHARED
#  c++/drivers/lms2xx/sicklms2xx/SickLMS2xx.cc 
#  c++/drivers/lms2xx/sicklms2xx/SickLMS2xxBufferMonitor.cc 
#  c++/drivers/lms2xx/sicklms2xx/SickLMS2xxMessage.cc
#)
ament_target_dependencies(SickLMS2xx rclcpp ${CMAKE_THREAD_LIBS_INIT})
ament_export_targets(SickLMS2xx HAS_LIBRARY_TARGET)



#############
## Install ##
#############

# install the include/sicktoolbox
install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
)

install(
  TARGETS SickLMS2xx
  EXPORT SickLMS2xx
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()