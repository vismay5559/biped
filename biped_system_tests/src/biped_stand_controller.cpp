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
    client_ = rclcpp_action::create_client<FollowJT>(
      this,
      "/biped_joint_trajectory_controller/follow_joint_trajectory"
    );

    RCLCPP_INFO(get_logger(), "Waiting for biped controller...");
    if (!client_->wait_for_action_server(20s)) {
      RCLCPP_FATAL(get_logger(), "Controller not available");
      rclcpp::shutdown();
      return;
    }

    joint_names_ = {
      "l_abduction", "l_hip_roll", "l_knee_roll", "l_foot_roll",
      "r_abduction", "r_hip_roll", "r_knee_roll", "r_foot_roll"
    };

    stand_pose_ = {
      0.0, -0.2, -0.4, -0.2,
      0.0, -0.2, -0.4, -0.2
    };

    send_stand();
  }

private:
  void send_stand()
  {
    FollowJT::Goal goal;
    goal.trajectory.joint_names = joint_names_;

    trajectory_msgs::msg::JointTrajectoryPoint p;
    p.positions = stand_pose_;
    p.time_from_start.sec = 2;

    goal.trajectory.points.push_back(p);

    client_->async_send_goal(goal);
    RCLCPP_INFO(get_logger(), "Standing command sent");
  }

  rclcpp_action::Client<FollowJT>::SharedPtr client_;
  std::vector<std::string> joint_names_;
  std::vector<double> stand_pose_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BipedStandController>());
  rclcpp::shutdown();
  return 0;
}
