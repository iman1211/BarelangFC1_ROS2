cmake_minimum_required(VERSION 3.5)
project(bfc_motion_bridge)

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
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(bfc_msgs REQUIRED)
find_package(darknet_ros_msgs REQUIRED)

add_executable(motion_bridge src/bfc_motion_bridge.cpp)
ament_target_dependencies(motion_bridge rclcpp geometry_msgs std_msgs sensor_msgs bfc_msgs)

add_executable(head_movement src/bfc_head_movement.cpp)
ament_target_dependencies(head_movement rclcpp geometry_msgs std_msgs sensor_msgs bfc_msgs darknet_ros_msgs)

install(TARGETS
  motion_bridge
  head_movement
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
  launch 
  config 
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
