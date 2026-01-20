#!/bin/bash
# ============================================================
# Single script to launch the Biped robot with Gazebo + ROS 2 Control
# ============================================================
#!/bin/bash

# --- SOURCE ROS ENVIRONMENT (CRITICAL) ---
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

cleanup() {
    echo "Cleaning up ROS 2 and Gazebo processes..."
    sleep 3.0
    pkill -9 -f "ros2|gazebo|gz|rviz2|robot_state_publisher|joint_state_publisher|controller_manager|spawner|biped"
}

# Trap Ctrl+C and kill everything cleanly
trap 'cleanup' SIGINT SIGTERM

echo "Launching Biped Gazebo simulation..."

ros2 launch biped_gazebo biped_gazebo.launch.py \
    load_controllers:=true \
    use_rviz:=true \
    use_sim_time:=true \
    x:=0.0 \
    y:=0.0 \
    z:=0.90 \
    roll:=0.0 \
    pitch:=0.0 \
    yaw:=0.0
