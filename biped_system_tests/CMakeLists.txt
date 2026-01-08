cmake_minimum_required(VERSION 3.8)
project(biped_system_tests)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

# Build the executable
add_executable(biped_stand_controller
  src/biped_stand_controller.cpp
)

ament_target_dependencies(biped_stand_controller
  rclcpp
  rclcpp_action
  control_msgs
  trajectory_msgs
)

# CRITICAL: This makes the controller "findable" by ros2 run
install(TARGETS
  biped_stand_controller
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


# Export the ament index for this package
ament_package()