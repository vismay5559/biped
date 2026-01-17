#!/bin/bash
# Single script to launch the biped with Gazebo, RViz, and MoveIt 2

cleanup() {
    echo "Cleaning up..."
    sleep 5.0
    # Kill common ROS 2 and Gazebo processes
    pkill -9 -f "ros2|gazebo|gz|nav2|amcl|bt_navigator|nav_to_pose|rviz2|assisted_teleop|cmd_vel_relay|robot_state_publisher|joint_state_publisher|move_to_free|mqtt|autodock|cliff_detection|moveit|move_group|basic_navigator"
}

# Set up cleanup trap to run the function above when you press Ctrl+C
trap 'cleanup' SIGINT SIGTERM

echo "Launching Gazebo simulation..."
# NOTE: Ensure you have a package named 'biped_gazebo' with a launch file 'biped.gazebo.launch.py'
# If your gazebo launch file is in 'biped_description', change the package name below.
ros2 launch biped_gazebo biped_gazebo.launch.py \
    load_controllers:=true \
    world_file:=empty.world \
    use_rviz:=false \
    use_robot_state_pub:=true \
    use_sim_time:=true \
    x:=0.0 \
    y:=0.0 \
    z:=0.5 \
    roll:=0.0 \
    pitch:=0.0 \
    yaw:=0.0 &

# Wait for Gazebo to fully load before starting MoveIt
echo "Waiting for Gazebo to load..."
sleep 15

echo "Launching MoveIt 2..."
# This launches the file we created earlier in 'biped_moveit_config'
ros2 launch biped_moveit_config move_group.launch.py \
    use_sim_time:=true \
    use_rviz:=true &

echo "Adjusting camera position..."
# Moves the Gazebo camera to a good viewing angle for a biped
gz service -s /gui/move_to/pose --reqtype gz.msgs.GUICamera --reptype gz.msgs.Boolean --timeout 2000 --req "pose: {position: {x: 2.0, y: 2.0, z: 1.5} orientation: {x: -0.1, y: 0.2, z: 0.5, w: 0.8}}"

# Keep the script running until Ctrl+C
wait