cmake_minimum_required(VERSION 3.5)
project(fp_ackermann_to_bicycle)

# Abhängigkeiten finden
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# Executable erstellen
add_executable(ackermann_to_bicycle_node src/ackermann_to_bicycle.cpp)

# Executable mit den Abhängigkeiten verknüpfen
ament_target_dependencies(ackermann_to_bicycle_node rclcpp geometry_msgs)

# Installationsanweisungen
install(TARGETS ackermann_to_bicycle_node
  DESTINATION lib/${PROJECT_NAME})

# Package exportieren
ament_package()
