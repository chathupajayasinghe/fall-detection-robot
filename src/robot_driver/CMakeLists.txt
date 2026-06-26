cmake_minimum_required(VERSION 3.8)
project(robot_driver)

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)

# Install Python scripts
install(PROGRAMS
  robot_driver/motor_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
