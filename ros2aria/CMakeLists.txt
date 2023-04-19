cmake_minimum_required(VERSION 3.8)
project(ros2aria)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake QUIET)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)



find_package(Aria QUIET)
if(Aria_FOUND)
  message("CMake package for Aria was found, using that.")
  # Aria package for CMake was found
	if(EXISTS "${Aria_INCLUDE_DIRS}/Aria.h")
		add_definitions(-DADEPT_PKG)
	endif()
  include_directories(${Aria_INCLUDE_DIRS})
  link_directories(${Aria_LIBRARY_DIRS})
else()
  # No Aria package for CMake was found, search ourselves
	# The installation package provided by Adept doesn't follow normal Linux
	# installation locations, but the repackaged Debian package and AriaCoda do.
  # If user set ARIA environment variable, look there, otherwise look in
  # /usr/local/.
  if(DEFINED ENV{ARIA})
    message("ARIA environment variable defined, checking there...")
    set(prefix $ENV{ARIA})
  else()
    set(prefix "/usr/local")
  endif()
  message("Looking for Aria in ${prefix}/Aria and ${prefix}...")

	if(EXISTS "${prefix}/Aria/include/Aria.h")
    message("Found ${prefix}/Aria/include/Aria.h, assuming Adept ARIA package.")
	  add_definitions(-DADEPT_PKG)
	  include_directories(${prefix}/Aria/include)
	  link_directories(${prefix}/Aria/lib)
  else()
    if(EXISTS "${prefix}/include/Aria.h")
      message("Found ${prefix)/include/Aria.h, assuming Adept ARIA source directory.")
      add_definitions(-DADEPT_PKG)
      include_directories(${prefix}/include)
      link_directories(${prefix}/lib)
    else()
      if(EXISTS "${prefix}/include/Aria/Aria.h")
        message("Found ${prefix}/include/Aria/Aria.h, assuming AriaCoda or repackaged ARIA.")
        #add_definitions(-DARIACODA)
        include_directories(${prefix}/include)
        link_directories(${prefix}/lib)
      else()
        message("Aria.h not found in ${prefix}. Continuing with default header and library paths.")
      endif()
    endif()
	endif()
endif()




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

ament_package()


# Executables
add_executable(ros2aria src/ros2aria.cpp)

include_directories(
    include
    /usr/local/include
)

# Install launch files and related config files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)


install(TARGETS
    ros2aria
DESTINATION lib/${PROJECT_NAME})

target_link_libraries(ros2aria Aria)

ament_target_dependencies(ros2aria rclcpp std_msgs geometry_msgs nav_msgs sensor_msgs tf2 tf2_ros)
