#include <chrono>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

using namespace std::chrono_literals;

class BipedStandController : public rclcpp::Node
{
public:
  using FollowJT = control_msgs::action::FollowJointTrajectory;

  BipedStandController()
  : Node("biped_stand_controller")
  {
    // Action client matches the controller name in your launch file
    client_ = rclcpp_action::create_client<FollowJT>(
      this,
      "/biped_joint_trajectory_controller/follow_joint_trajectory"
    );

    RCLCPP_INFO(get_logger(), "Waiting for biped controller action server...");
    if (!client_->wait_for_action_server(20s)) {
      RCLCPP_FATAL(get_logger(), "Controller not available! Ensure Gazebo and controllers are running.");
      rclcpp::shutdown();
      return;
    }

    // Joint names must exactly match your URDF
    joint_names_ = {
      "l_abduction", "l_hip_roll", "l_knee_roll", "l_foot_roll",
      "r_abduction", "r_hip_roll", "r_knee_roll", "r_foot_roll"
    };

    /**
     * STANDING POSE EXPLANATION:
     * We use a "ready" pose where the knees are slightly bent.
     * To keep the feet flat on the ground:
     * 1. Hip rolls back (negative)
     * 2. Knee rolls forward (negative)
     * 3. Foot/Ankle compensates (negative)
     */
    stand_pose_ = {
      0.0, 0.3, 0.6, 0,  // Left Leg
      0.0, -0.3, -0.6, 0   // Right Leg (Symmetrical to Left)
    };

    // Delay the command slightly to ensure the robot has fully spawned
    timer_ = this->create_wall_timer(2s, std::bind(&BipedStandController::send_stand, this));
  }

private:
  void send_stand()
  {
    // Stop the timer so we only send the command once
    timer_->cancel();

    auto goal_msg = FollowJT::Goal();
    goal_msg.trajectory.joint_names = joint_names_;

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = stand_pose_;
    
    // Smooth transition: 4 seconds prevents the robot from "jerking" and falling
    point.time_from_start.sec = 4;
    point.time_from_start.nanosec = 0;

    goal_msg.trajectory.points.push_back(point);

    RCLCPP_INFO(get_logger(), "Sending standing goal...");
    
    auto send_goal_options = rclcpp_action::Client<FollowJT>::SendGoalOptions();
    send_goal_options.result_callback = 
      [](const rclcpp_action::ClientGoalHandle<FollowJT>::WrappedResult & result) {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          std::cout << "Robot successfully reached standing pose!" << std::endl;
        }
      };

    client_->async_send_goal(goal_msg, send_goal_options);
  }

  rclcpp_action::Client<FollowJT>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<std::string> joint_names_;
  std::vector<double> stand_pose_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BipedStandController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}