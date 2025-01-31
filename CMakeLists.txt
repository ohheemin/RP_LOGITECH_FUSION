cmake_minimum_required(VERSION 3.8)
project(cali)

# Use Python version 3
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(laser_geometry REQUIRED)

# Install Python nodes
install(
  PROGRAMS
    src/cam_cali.py
    src/camera.py
    src/lidar.py
    src/main.py
    src/yolo.py
  DESTINATION lib/${PROJECT_NAME}
)

# Package export
ament_package()

