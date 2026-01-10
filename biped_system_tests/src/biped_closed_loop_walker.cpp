#include <chrono>
#include <cmath>
#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

using namespace std::chrono_literals;

class ClosedLoopWalker : public rclcpp::Node {
public:
  using FollowJT = control_msgs::action::FollowJointTrajectory;

  ClosedLoopWalker() : Node("closed_loop_walker") {
    client_ = rclcpp_action::create_client<FollowJT>(this, "/biped_joint_trajectory_controller/follow_joint_trajectory");
    
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu", 10, std::bind(&ClosedLoopWalker::imu_callback, this, std::placeholders::_1));

    // UPDATED: Only 6 joints
    joint_names_ = {"l_abduction", "l_hip_roll", "l_knee_roll",
                    "r_abduction", "r_hip_roll", "r_knee_roll"};

    if (!client_->wait_for_action_server(10s)) {
      RCLCPP_ERROR(get_logger(), "Controller not found!");
    }

    timer_ = this->create_wall_timer(10ms, std::bind(&ClosedLoopWalker::control_loop, this));
    start_time_ = this->now();
    RCLCPP_INFO(get_logger(), "Walker Started: PASSIVE ANKLE MODE");
  }

private:
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    tf2::Quaternion q(
        msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    tf2::Matrix3x3 m(q);
    m.getRPY(current_roll_, current_pitch_, current_yaw_);
  }

  void control_loop() {
    double t = (this->now() - start_time_).seconds();
    double ramp = std::min(t / 4.0, 1.0); 

    // --- 1. POSTURE PARAMETERS ---
    double target_knee = -0.5;
    double target_hip = -0.05; 

    // Apply Soft Start
    double hip_base = target_hip * ramp;
    double knee_base = target_knee * ramp;

    // --- 2. WALKING GENERATOR ---
    double step_mag = 0.0; // Keep 0.0 for initial balance test
    double freq = 1.5;
    double l_phase = t * freq;
    double r_phase = t * freq + M_PI;

    double l_hip_walk = step_mag * std::sin(l_phase);
    double r_hip_walk = step_mag * std::sin(r_phase);
    
    // --- 3. STABILIZER (Hip Strategy Only) ---
    // Since feet are passive, we can only balance using hips
    double Kp_pitch = 0.0; 
    double Kp_roll = 0.0;

    double balance_pitch = current_pitch_ * Kp_pitch;
    double balance_roll  = current_roll_ * Kp_roll;

    // --- 4. COMMAND GENERATION ---
    
    // LEFT LEG
    double l_abd  = 0.0 + balance_roll;
    double l_hip  = hip_base + l_hip_walk + balance_pitch; 
    double l_knee = knee_base;
    // Ankle command removed

    // RIGHT LEG (Mirrored)
    // IMPORTANT: Check if your right leg needs negative signs. 
    // Assuming standard mirrored URDF:
    double r_abd  = 0.0 + balance_roll; 
    double r_hip  = (hip_base + r_hip_walk + balance_pitch) * -1.0; 
    double r_knee = knee_base * -1.0;
    // Ankle command removed

    // UPDATED: Sending vector of size 6
    send_command({l_abd, l_hip, l_knee, r_abd, r_hip, r_knee});
  }

  void send_command(std::vector<double> positions) {
    FollowJT::Goal goal;
    goal.trajectory.joint_names = joint_names_;
    trajectory_msgs::msg::JointTrajectoryPoint p;
    p.positions = positions;
    p.time_from_start = rclcpp::Duration::from_seconds(0.01); 
    goal.trajectory.points.push_back(p);
    client_->async_send_goal(goal);
  }

  rclcpp_action::Client<FollowJT>::SharedPtr client_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<std::string> joint_names_;
  rclcpp::Time start_time_;

  double current_roll_ = 0.0;
  double current_pitch_ = 0.0;
  double current_yaw_ = 0.0;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ClosedLoopWalker>());
  rclcpp::shutdown();
  return 0;
}