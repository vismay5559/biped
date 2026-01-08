#!/usr/bin/env python3

import os
import subprocess

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_ros_gz_sim = FindPackageShare('ros_gz_sim').find('ros_gz_sim')
    pkg_description = FindPackageShare('biped_description').find('biped_description')

    # Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items()
    )

    # robot_state_publisher (xacro expanded like mycobot)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'use_sim_time': True,
            'robot_description': subprocess.check_output([
                'xacro',
                os.path.join(pkg_description, 'urdf', 'robots', 'biped.xacro')
            ]).decode()
        }]
    )

    # Spawn robot
    spawn_biped = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'biped',
            '-topic', '/robot_description',
            '-z', '0.95'
        ],
        output='screen'
    )

    # Spawn controllers AFTER gz_ros2_control is alive
    spawn_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    spawn_biped_ctrl = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['biped_joint_trajectory_controller'],
        output='screen'
    )


    controllers_delayed = TimerAction(
        period=5.0,
        actions=[spawn_jsb, spawn_biped_ctrl]
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_biped,
        controllers_delayed
    ])
