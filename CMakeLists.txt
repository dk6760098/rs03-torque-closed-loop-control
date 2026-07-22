cmake_minimum_required(VERSION 3.8)
project(rs03_torque_closed_loop_control)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(rs03_torque_closed_loop_node src/rs03_torque_closed_loop_node.cpp)
target_compile_features(rs03_torque_closed_loop_node PUBLIC cxx_std_17)
ament_target_dependencies(rs03_torque_closed_loop_node rclcpp std_msgs)

install(TARGETS rs03_torque_closed_loop_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
