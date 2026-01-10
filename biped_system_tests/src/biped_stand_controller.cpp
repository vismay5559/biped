#include <chrono>
#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

using namespace std::chrono_literals;

class BipedStandController : public rclcpp::Node {
public:
  using FollowJT = control_msgs::action::FollowJointTrajectory;

  BipedStandController() : Node("biped_stand_controller") {
    client_ = rclcpp_action::create_client<FollowJT>(this, "/biped_joint_trajectory_controller/follow_joint_trajectory");
    
    joint_names_ = {"l_abduction", "l_hip_roll", "l_knee_roll", "l_foot_roll",
                    "r_abduction", "r_hip_roll", "r_knee_roll", "r_foot_roll"};

    // Mirrored values: Left negative, Right positive to move physically same way 
    stand_pose_ = {0.0, -0.2 , 0.2, 0.0,   // Left Leg
                   0.0,  0.2,  -0.2,  0.0};  // Right Leg (Mirrored)

    timer_ = this->create_wall_timer(500ms, std::bind(&BipedStandController::send_stand, this));
  }

private:
  void send_stand() {
    timer_->cancel();
    if (!client_->wait_for_action_server(5s)) return;

    auto goal_msg = FollowJT::Goal();
    goal_msg.trajectory.joint_names = joint_names_;

    trajectory_msgs::msg::JointTrajectoryPoint p;
    p.positions = stand_pose_;
    // Fast timing (0.5s) to beat gravity 
    p.time_from_start.sec = 0;
    p.time_from_start.nanosec = 500000000; 

    goal_msg.trajectory.points.push_back(p);
    client_->async_send_goal(goal_msg);
    RCLCPP_INFO(get_logger(), "Standing command sent!");
  }

  rclcpp_action::Client<FollowJT>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<std::string> joint_names_;
  std::vector<double> stand_pose_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BipedStandController>());
  rclcpp::shutdown();
  return 0;
}