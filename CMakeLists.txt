cmake_minimum_required(VERSION 3.5)
project(larr_loader_project)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include)

find_package(ament_cmake REQUIRED)
find_package(wheel_loader_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(Eigen3 3.3 REQUIRED)
add_executable(loader_sim 
  src/loader_sim/loader_sim.cpp
  src/loader_sim/visualization.cpp
  src/loader_sim/topic_publisher.cpp
  src/loader_sim/loader_sim_node.cpp)
target_link_libraries(loader_sim Eigen3::Eigen)
ament_target_dependencies(loader_sim rclcpp std_msgs nav_msgs sensor_msgs wheel_loader_msgs visualization_msgs)

add_executable(steering_angle_control 
  src/steering_angle_control/steering_angle_control.cpp
  src/steering_angle_control/steering_angle_control_node.cpp)
target_link_libraries(steering_angle_control Eigen3::Eigen)
ament_target_dependencies(steering_angle_control rclcpp std_msgs nav_msgs sensor_msgs wheel_loader_msgs)

install(TARGETS
  loader_sim
  steering_angle_control
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/)

  install(DIRECTORY
  meshes rviz
  DESTINATION share/${PROJECT_NAME})

ament_package()