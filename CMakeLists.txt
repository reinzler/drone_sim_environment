cmake_minimum_required(VERSION 3.5)
project(floatgen)

# Найти необходимые пакеты
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)  # Если используются узлы на C++
find_package(xacro REQUIRED)  # Если используются Xacro файлы
find_package(ros_gz_sim REQUIRED)  # Если используется Gazebo Sim

# Установка GZ_SIM_RESOURCE_PATH
ament_environment_hooks("hooks/resource_paths.dsv.in")

# Установка директорий с ресурсами
install(
  DIRECTORY meshes urdf launch models worlds
  DESTINATION share/${PROJECT_NAME}
)

# Если есть исполняемые файлы на C++
# add_executable(your_cpp_node src/your_cpp_node.cpp)
# ament_target_dependencies(your_cpp_node rclcpp)
# install(TARGETS your_cpp_node DESTINATION lib/${PROJECT_NAME})

# Export dependencies
ament_export_dependencies(rclcpp xacro ros_gz_sim)

# Завершение описания пакета
ament_package()