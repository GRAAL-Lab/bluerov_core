cmake_minimum_required(VERSION 3.5)
project(udp_camera)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

install(
  PROGRAMS
    scripts/camera_node.py  # Path to your Python file (relative to the package root)
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

