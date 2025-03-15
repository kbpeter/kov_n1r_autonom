cmake_minimum_required(VERSION 3.5)
project(szenzor_rendszer)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(szenzor_node src/szenzor_node.cpp)
ament_target_dependencies(szenzor_node rclcpp sensor_msgs std_msgs)

add_executable(figyelo_node src/figyelo_node.cpp)
ament_target_dependencies(figyelo_node rclcpp sensor_msgs std_msgs)

add_executable(adat_megjelenito src/adat_megjelenito.cpp)
ament_target_dependencies(adat_megjelenito rclcpp sensor_msgs std_msgs)

install(TARGETS
  szenzor_node
  figyelo_node
  adat_megjelenito
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
