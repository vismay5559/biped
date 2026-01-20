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
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Build C++ executables
add_executable(biped_walker src/biped_walker.cpp)

# Link dependencies for biped_walker
ament_target_dependencies(biped_walker
  rclcpp
  rclcpp_action
  control_msgs
  trajectory_msgs
)

# Install C++ executables
install(TARGETS
  biped_walker
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/biped_stand_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(biped_closed_loop src/biped_closed_loop_walker.cpp)

ament_target_dependencies(biped_closed_loop rclcpp rclcpp_action control_msgs trajectory_msgs sensor_msgs tf2 tf2_geometry_msgs)

install(TARGETS biped_closed_loop DESTINATION lib/${PROJECT_NAME})

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