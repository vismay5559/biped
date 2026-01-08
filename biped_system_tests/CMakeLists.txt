cmake_minimum_required(VERSION 3.8)
project(biped_system_tests)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_executable(biped_stand_controller
  src/biped_stand_controller.cpp
)

ament_target_dependencies(biped_stand_controller
  rclcpp
  rclcpp_action
  control_msgs
  trajectory_msgs
)

install(TARGETS
  biped_stand_controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
