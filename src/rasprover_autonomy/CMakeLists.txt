cmake_minimum_required(VERSION 3.8)
project(rasprover_autonomy)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(pure_pursuit_node src/pure_pursuit_node.cpp)
ament_target_dependencies(pure_pursuit_node 
  rclcpp geometry_msgs nav_msgs sensor_msgs tf2 tf2_ros tf2_geometry_msgs)

# Install targets
install(TARGETS pure_pursuit_node
  DESTINATION lib/${PROJECT_NAME})

# Install launch and config files
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()

