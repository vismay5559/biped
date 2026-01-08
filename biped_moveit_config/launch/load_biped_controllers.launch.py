from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction


def generate_launch_description():

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    biped_position_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "biped_position_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    return LaunchDescription([
        # Give controller_manager time to come up
        TimerAction(period=2.0, actions=[joint_state_broadcaster]),
        TimerAction(period=3.0, actions=[biped_position_controller]),
    ])
