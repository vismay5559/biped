#!/usr/bin/env python3
"""
Launch RViz visualization for the biped robot.

This launch file:
- Processes ros2_control config (template → final yaml)
- Processes the biped xacro
- Starts robot_state_publisher
- Starts joint_state_publisher (GUI optional)
- Launches RViz
"""

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


# ----------------------------------------------------
# ros2_control config processing (CRITICAL)
# ----------------------------------------------------
def process_ros2_controllers_config(context):

    robot_name = LaunchConfiguration('robot_name').perform(context)
    prefix = LaunchConfiguration('prefix').perform(context)

    home = str(Path.home())

    # Source + install paths (same logic as myCobot)
    src_config_path = os.path.join(
        home,
        'ros2_ws/src/biped/biped_moveit_config/config',
        robot_name
    )

    install_config_path = os.path.join(
        home,
        'ros2_ws/install/biped/share/biped_moveit_config/config',
        robot_name
    )

    template_path = os.path.join(
        src_config_path,
        'ros2_controllers_template.yaml'
    )

    # Read template
    with open(template_path, 'r', encoding='utf-8') as file:
        template_content = file.read()

    # Replace placeholders
    processed_content = template_content.replace('${prefix}', prefix)

    # Write final yaml to both places
    for path in [src_config_path, install_config_path]:
        os.makedirs(path, exist_ok=True)
        output_file = os.path.join(path, 'ros2_controllers.yaml')
        with open(output_file, 'w', encoding='utf-8') as file:
            file.write(processed_content)

    return []


# ----------------------------------------------------
# Launch arguments for XACRO + control
# ----------------------------------------------------
ARGUMENTS = [
    DeclareLaunchArgument(
        'robot_name',
        default_value='biped',
        description='Robot name'),

    DeclareLaunchArgument(
        'prefix',
        default_value='',
        description='Joint name prefix (for multi-robot setups)'),

    DeclareLaunchArgument(
        'jsp_gui',
        default_value='true',
        choices=['true', 'false'],
        description='Use joint_state_publisher_gui'),

    DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz'),

    DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock'),
]


def generate_launch_description():

    # ------------------------------------------------
    # Paths
    # ------------------------------------------------
    package_name = 'biped_description'
    xacro_file = 'biped.xacro'
    rviz_file = 'display.rviz'

    pkg_share = FindPackageShare(package_name)

    urdf_path = PathJoinSubstitution([
        pkg_share,
        'urdf',
        'robots',
        xacro_file
    ])

    rviz_path = PathJoinSubstitution([
        pkg_share,
        'rviz',
        rviz_file
    ])

    # ------------------------------------------------
    # Launch configs
    # ------------------------------------------------
    jsp_gui = LaunchConfiguration('jsp_gui')
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz_config_file = rviz_path

    # ------------------------------------------------
    # Robot description (XACRO)
    # ------------------------------------------------
    robot_description = ParameterValue(
        Command([
            'xacro ', urdf_path, ' ',
            'prefix:=', LaunchConfiguration('prefix')
        ]),
        value_type=str
    )

    # ------------------------------------------------
    # Nodes
    # ------------------------------------------------
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }]
    )

    joint_state_publisher = Node(
        condition=UnlessCondition(jsp_gui),
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    joint_state_publisher_gui = Node(
        condition=IfCondition(jsp_gui),
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    rviz = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # ------------------------------------------------
    # Launch description
    # ------------------------------------------------
    ld = LaunchDescription(ARGUMENTS)

    # IMPORTANT: process controller config FIRST
    ld.add_action(OpaqueFunction(function=process_ros2_controllers_config))

    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher)
    ld.add_action(joint_state_publisher_gui)
    ld.add_action(rviz)

    return ld
