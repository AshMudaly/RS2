cmake_minimum_required(VERSION 3.5)
project(svg_generation)

# Find dependencies for Python packages
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(visualization_msgs REQUIRED)

# Install Python executable
install(PROGRAMS
  svg_generation/svg_reader.py
  DESTINATION lib/${PROJECT_NAME}
)

# This ensures that your Python package is properly installed and accessible
ament_python_install_package(${PROJECT_NAME})

# Add other ROS2 specific configurations
ament_package()
