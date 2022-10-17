cmake_minimum_required(VERSION 3.8)
project(mbzirc_joystick101)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system thread)

# using eigen3
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)
set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN3_DEFINITIONS})    

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(
  include
  lib
  src
)

add_executable(mbzirc_joystick101_node src/main.cpp 
lib/mbzirc_joy_ctrl_ros2_lib.cpp 
include/mbzirc_joy_ctrl_ros2_lib.h
lib/mbzirc_quad_imu_data_lib.cpp
include/mbzirc_quad_imu_data_lib.h 
lib/joy_xbox360_lib.cpp
include/joy_xbox360_lib.h 
lib/config_param.cpp
include/config_param.h 
lib/misc_func.cpp
include/misc_func.h
include/global_header.h
)
ament_target_dependencies(mbzirc_joystick101_node rclcpp std_msgs sensor_msgs tf2 tf2_ros)

install(TARGETS
mbzirc_joystick101_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
