#include <chrono>
#include <cmath>
#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

class BipedWalker : public rclcpp::Node {
public:
  using FollowJT = control_msgs::action::FollowJointTrajectory;

  BipedWalker() : Node("biped_walker") {
    client_ = rclcpp_action::create_client<FollowJT>(this, "/biped_joint_trajectory_controller/follow_joint_trajectory");
    
    // Joint names matching your URDF
    joint_names_ = {"l_abduction", "l_hip_roll", "l_knee_roll", "l_foot_roll",
                    "r_abduction", "r_hip_roll", "r_knee_roll", "r_foot_roll"};

    // Wait for controller
    if (!client_->wait_for_action_server(10s)) {
      RCLCPP_ERROR(get_logger(), "Controller action server not found!");
    }

    // Timer for the gait loop (20Hz control loop)
    timer_ = this->create_wall_timer(50ms, std::bind(&BipedWalker::gait_loop, this));
    start_time_ = this->now();
    RCLCPP_INFO(get_logger(), "Walker Started! Press Ctrl+C to stop.");
  }

private:
  void gait_loop() {
    auto current_time = this->now();
    double t = (current_time - start_time_).seconds();

    // GAIT PARAMETERS
    double freq = 2.0;            // Speed of walking (Hz)
    double sway_amp = 0.05;       // How much to lean Left/Right (Abduction)
    double step_amp = 0.3;        // How high/far to lift legs (Hip/Knee)
    
    // BASE POSE (Crouch) - Keeps knees bent so they can extend to push
    // Using negative values because your URDF limits are -2.5 to 0.0
    double hip_base = -0.2;  
    double knee_base = -0.1; 
    double ankle_base = 0;

    // SINE WAVES GENERATOR
    // Phase shift PI (3.14) ensures legs move opposite to each other
    double left_phase = t * freq;
    double right_phase = t * freq + M_PI;

    // 1. SWAY (Abduction) - Lean weight onto the supporting leg
    // We lean LEFT when lifting RIGHT leg, and vice versa.
    double l_abd_cmd = sway_amp * std::sin(left_phase);
    double r_abd_cmd = sway_amp * std::sin(left_phase); // Move same direction to sway body

    // 2. STEP (Hip/Knee) - Lift and move
    // We modify the "Base" crouch with a sine wave
    double l_hip_cmd = hip_base + (step_amp * std::sin(left_phase));
    double r_hip_cmd = hip_base + (step_amp * std::sin(right_phase));

    double l_knee_cmd = knee_base + (step_amp * std::sin(left_phase)); 
    double r_knee_cmd = knee_base + (step_amp * std::sin(right_phase));

    // 3. ANKLE (Keep foot flat)
    // Inverse of hip/knee usually keeps foot parallel to ground
    double l_foot_cmd = ankle_base - (0.5 * step_amp * std::sin(left_phase));
    double r_foot_cmd = ankle_base - (0.5 * step_amp * std::sin(right_phase));

    // Construct Message
    FollowJT::Goal goal;
    goal.trajectory.joint_names = joint_names_;
    
    trajectory_msgs::msg::JointTrajectoryPoint p;
    p.positions = {
        l_abd_cmd, l_hip_cmd, l_knee_cmd, l_foot_cmd, // Left
        r_abd_cmd, r_hip_cmd, r_knee_cmd, r_foot_cmd  // Right
    };
    
    p.time_from_start = rclcpp::Duration::from_seconds(0.05); // Instant command
    goal.trajectory.points.push_back(p);

    // Send without waiting for result (Fire and Forget for smooth loop)
    client_->async_send_goal(goal);
  }

  rclcpp_action::Client<FollowJT>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<std::string> joint_names_;
  rclcpp::Time start_time_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BipedWalker>());
  rclcpp::shutdown();
  return 0;
}