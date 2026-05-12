cmake_minimum_required(VERSION 3.8)
project(outdoor_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)

# Motor controller node
add_executable(motor_controller src/motor_controller.cpp)
ament_target_dependencies(motor_controller
  rclcpp geometry_msgs)

# Odometry publisher node
add_executable(odometry_publisher src/odometry.cpp)
ament_target_dependencies(odometry_publisher
  rclcpp nav_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs)

# Install targets
install(TARGETS
  motor_controller
  odometry_publisher
  DESTINATION lib/${PROJECT_NAME})

# Install launch and config files
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
