cmake_minimum_required(VERSION 3.8)
project(szenzor_rendszer)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Szenzor node
add_executable(szenzor_node src/szenzor_node.cpp)
ament_target_dependencies(szenzor_node rclcpp sensor_msgs std_msgs)

# Figyelő node
add_executable(figyelo_node src/figyelo_node.cpp)
ament_target_dependencies(figyelo_node rclcpp sensor_msgs std_msgs)

install(TARGETS
  szenzor_node
  figyelo_node
  DESTINATION lib/${PROJECT_NAME}
)

# Launch fájl telepítése
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
