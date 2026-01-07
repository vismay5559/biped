#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    AppendEnvironmentVariable
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # =========================
    # Packages
    # =========================
    description_pkg = 'biped_description'
    gazebo_pkg = 'biped_gazebo'
    moveit_pkg = 'biped_moveit_config'

    # =========================
    # Launch args
    # =========================
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_gazebo = LaunchConfiguration('use_gazebo')
    use_rviz = LaunchConfiguration('use_rviz')
    load_controllers = LaunchConfiguration('load_controllers')

    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    roll = LaunchConfiguration('roll')
    pitch = LaunchConfiguration('pitch')
    yaw = LaunchConfiguration('yaw')

    # =========================
    # Declare args
    # =========================
    declare_args = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_gazebo', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('load_controllers', default_value='true'),

        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.95'),  # IMPORTANT
        DeclareLaunchArgument('roll', default_value='0.0'),
        DeclareLaunchArgument('pitch', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
    ]

    # =========================
    # Paths
    # =========================
    pkg_description = FindPackageShare(description_pkg).find(description_pkg)
    pkg_gazebo = FindPackageShare(gazebo_pkg).find(gazebo_pkg)
    pkg_moveit = FindPackageShare(moveit_pkg).find(moveit_pkg)
    pkg_ros_gz = FindPackageShare('ros_gz_sim').find('ros_gz_sim')

    world_path = os.path.join(pkg_gazebo, 'worlds', 'empty.world')
    bridge_config = os.path.join(pkg_gazebo, 'config', 'ros_gz_bridge.yaml')

    # =========================
    # Environment (Gazebo meshes)
    # =========================
    set_gz_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(pkg_gazebo, 'models') + ':' + pkg_description
    )

    # =========================
    # Robot State Publisher
    # =========================
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_description, 'launch', 'robot_state_publisher.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )

    # =========================
    # Gazebo
    # =========================
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': [
                '-r -v 4 ',
                world_path
            ]
        }.items(),
        condition=IfCondition(use_gazebo)
    )

    # =========================
    # Bridge
    # =========================
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_config}],
        output='screen'
    )

    # =========================
    # Spawn robot
    # =========================
    spawn_biped = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'biped',
            '-topic', '/robot_description',
            '-x', x,
            '-y', y,
            '-z', z,
            '-R', roll,
            '-P', pitch,
            '-Y', yaw,
        ],
        output='screen'
    )

    # =========================
    # Controllers (DELAYED)
    # =========================
    controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_moveit, 'launch', 'load_biped_controllers.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items(),
        condition=IfCondition(load_controllers)
    )

    # =========================
    # RViz (DELAYED)
    # =========================
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=[
            '-d',
            os.path.join(pkg_description, 'rviz', 'display.rviz')
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
        condition=IfCondition(use_rviz)
    )

    # =========================
    # Launch ordering
    # =========================
    ld = LaunchDescription()

    for arg in declare_args:
        ld.add_action(arg)

    ld.add_action(set_gz_resources)
    ld.add_action(robot_state_publisher)
    ld.add_action(gazebo)
    ld.add_action(bridge)
    ld.add_action(spawn_biped)

    # controllers AFTER spawn
    ld.add_action(
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_biped,
                on_exit=[controllers]
            )
        )
    )

    # rviz AFTER controllers
    ld.add_action(
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_biped,
                on_exit=[rviz]
            )
        )
    )

    return ld
