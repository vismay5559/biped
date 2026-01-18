/**
 * @file biped_walk.cpp
 * @brief ROS 2 MoveIt 2 program to make a biped robot take a step (Relative Frame Fix)
 */

#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <thread>
#include <tf2_eigen/tf2_eigen.hpp> 

// Helper function to plan and execute a Cartesian path
bool planAndExecuteCartesianPath(
    moveit::planning_interface::MoveGroupInterface& move_group,
    const std::vector<geometry_msgs::msg::Pose>& waypoints)
{
  moveit_msgs::msg::RobotTrajectory trajectory;
  const double eef_step = 0.01;      // 1 cm resolution
  bool avoid_collisions = false;     // Disable collision for debugging

  RCLCPP_INFO(rclcpp::get_logger("biped_walk"), "Computing Cartesian Path in frame: %s", move_group.getPoseReferenceFrame().c_str());

  double fraction = move_group.computeCartesianPath(
      waypoints, 
      eef_step, 
      trajectory, 
      avoid_collisions);

  RCLCPP_INFO(rclcpp::get_logger("biped_walk"), "Path fraction achieved: %.2f%%", fraction * 100.0);

  if (fraction < 0.5) 
  {
    RCLCPP_ERROR(rclcpp::get_logger("biped_walk"), "Path planning failed (fraction too low).");
    return false;
  }

  move_group.execute(trajectory);
  return true;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  node_options.parameter_overrides({{"use_sim_time", true}});

  auto const node = std::make_shared<rclcpp::Node>("biped_walker", node_options);
  auto const logger = rclcpp::get_logger("biped_walk");

  // Spin in background
  std::thread spinning_thread([node] { rclcpp::spin(node); });

  using moveit::planning_interface::MoveGroupInterface;

  // 1. Create Interfaces
  auto left_leg_group = MoveGroupInterface(node, "left_leg");
  auto right_leg_group = MoveGroupInterface(node, "right_leg");

  // SAFETY: Slow down
  left_leg_group.setMaxVelocityScalingFactor(0.5);
  right_leg_group.setMaxVelocityScalingFactor(0.5);

  // 2. Move to Stand Pose
  RCLCPP_INFO(logger, "Moving to Stand Pose...");
  left_leg_group.setNamedTarget("stand");
  left_leg_group.move();
  right_leg_group.setNamedTarget("stand");
  right_leg_group.move();

  // 3. Define Step Sequence
  RCLCPP_INFO(logger, "Attempting Left Leg Step...");
  rclcpp::sleep_for(std::chrono::seconds(2)); // Wait for settle

  // --- THE FIX: MANUAL RELATIVE TRANSFORM ---
  
  // A. Force MoveIt to plan relative to base_link (Pelvis)
  // This tells the solver: "Ignore the world frame. All my commands are relative to base_link."
  std::string reference_frame = "base_link"; 
  left_leg_group.setPoseReferenceFrame(reference_frame);

  // B. Get the robot state
  auto current_state = left_leg_group.getCurrentState();
  
  // C. Calculate the relative transform manually
  // T_world_to_base
  Eigen::Isometry3d text_base = current_state->getGlobalLinkTransform("base_link");
  // T_world_to_foot
  Eigen::Isometry3d text_foot = current_state->getGlobalLinkTransform(left_leg_group.getEndEffectorLink());
  // T_base_to_foot = (T_world_to_base)^-1 * T_world_to_foot
  Eigen::Isometry3d relative_transform = text_base.inverse() * text_foot;

  // D. Convert to Geometry Msg
  geometry_msgs::msg::Pose start_pose = tf2::toMsg(relative_transform);
  
  RCLCPP_INFO(logger, "Start Pose (relative to %s): X=%.3f, Y=%.3f, Z=%.3f", 
              reference_frame.c_str(),
              start_pose.position.x,
              start_pose.position.y,
              start_pose.position.z);

  // 4. Build Waypoints
  std::vector<geometry_msgs::msg::Pose> waypoints;
  geometry_msgs::msg::Pose target_pose = start_pose;

  // TEST 1: Lift Foot (Z + 5cm)
  target_pose.position.z += 0.05; 
  waypoints.push_back(target_pose);

  // TEST 2: Step Forward (X + 5cm)
  target_pose.position.x += 0.05; 
  waypoints.push_back(target_pose);

  // Execute
  bool success = planAndExecuteCartesianPath(left_leg_group, waypoints);

  if (success) {
    RCLCPP_INFO(logger, "Left step completed!");
  } else {
    RCLCPP_ERROR(logger, "Left step failed.");
  }

  rclcpp::shutdown();
  spinning_thread.join();
  return 0;
}