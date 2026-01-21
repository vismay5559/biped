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

    # Gazebo Sim
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items()
    )

    # Robot State Publisher
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
    spawn_biped = Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'biped',
                '-topic', '/robot_description',
                '-z', '0.5', # Lower z slightly so feet touch ground immediately
                # Left Leg (Matches your C++ Stand Pose)
                '-J', 'l_abduction', '0.0',
                '-J', 'l_hip_roll', '-0.1',
                '-J', 'l_knee_roll', '0.1',
                # Right Leg (Matches your C++ Stand Pose)
                '-J', 'r_abduction', '0.0',
                '-J', 'r_hip_roll', '0.1',
                '-J', 'r_knee_roll', '-0.4'
            ],
            output='screen'
        )

    # ROS-GZ Bridge - Updated with IMU mapping
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU'
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # Controller Spawners
# Controller Spawners
    spawn_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        # CORRECT WAY to set use_sim_time
        parameters=[{'use_sim_time': True}], 
        output='screen'
    )

    spawn_biped_ctrl = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['biped_joint_trajectory_controller'],
        # CORRECT WAY to set use_sim_time
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    controllers_delayed = TimerAction(
        period=1.0,
        actions=[spawn_jsb, spawn_biped_ctrl]
    )

    # RViz2
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_description, 'rviz', 'display.rviz')],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    rviz_delayed = TimerAction(
        period=6.0,
        actions=[rviz]
    )

    biped_stand = Node(
        package='biped_system_tests',
        executable='biped_stand_controller',
        output='screen'
    )

    delayed_biped_stand = TimerAction(
        period=4.0,  # seconds
        actions=[biped_stand]
    )


    return LaunchDescription([
        ros_gz_bridge,
        gazebo,
        robot_state_publisher,
        spawn_biped,
        controllers_delayed,
        #delayed_biped_stand,
        rviz_delayed
    ])