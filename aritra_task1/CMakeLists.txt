cmake_minimum_required(VERSION 3.5)
project(aritra_task1)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_generation REQUIRED)
find_package(setuptools REQUIRED)

# Generate messages

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

# Install Python package
ament_python_install_package(${PROJECT_NAME})

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
