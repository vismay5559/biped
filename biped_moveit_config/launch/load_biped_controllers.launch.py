#!/usr/bin/env python3
"""
Launch ROS 2 controllers for the biped robot.

This launch file sequentially starts:
1. Joint State Broadcaster
2. Leg Controller (JointTrajectoryController)

This ensures correct controller initialization order.
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit


def generate_launch_description():

    # ----------------------------------------------------
    # Start joint state broadcaster
    # ----------------------------------------------------
    start_joint_state_broadcaster_cmd = ExecuteProcess(
        cmd=[
            'ros2', 'control', 'load_controller',
            '--set-state', 'active',
            'joint_state_broadcaster'
        ],
        output='screen'
    )

    # ----------------------------------------------------
    # Start leg controller (after joint states are active)
    # ----------------------------------------------------
    start_leg_controller_cmd = ExecuteProcess(
        cmd=[
            'ros2', 'control', 'load_controller',
            '--set-state', 'active',
            'leg_controller'
        ],
        output='screen'
    )

    # ----------------------------------------------------
    # Ensure leg controller starts AFTER joint_state_broadcaster
    # ----------------------------------------------------
    load_leg_controller_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=start_joint_state_broadcaster_cmd,
            on_exit=[start_leg_controller_cmd]
        )
    )

    # ----------------------------------------------------
    # Create launch description
    # ----------------------------------------------------
    ld = LaunchDescription()

    # Start controllers in order
    ld.add_action(start_joint_state_broadcaster_cmd)
    ld.add_action(load_leg_controller_after_jsb)

    return ld
