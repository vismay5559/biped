import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler, OpaqueFunction, LogInfo
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # ---------------------------------------------------------
    # CONFIGURATION
    package_name_moveit_config = 'biped_moveit_config'
    package_name_description = 'biped_description' 
    urdf_file_name = 'biped.xacro'
    # ---------------------------------------------------------

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    rviz_config_package = LaunchConfiguration('rviz_config_package')

    pkg_share_moveit_config_temp = FindPackageShare(package=package_name_moveit_config)
    pkg_share_description_temp = FindPackageShare(package=package_name_description)

    declare_robot_name_cmd = DeclareLaunchArgument(
        name='robot_name',
        default_value='biped',
        description='Name of the robot to use')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        name='use_rviz',
        default_value='true',
        description='Whether to start RViz')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        name='rviz_config_file',
        default_value='move_group.rviz',
        description='RViz configuration file')

    declare_rviz_config_package_cmd = DeclareLaunchArgument(
        name='rviz_config_package',
        default_value=package_name_moveit_config,
        description='Package containing the RViz configuration file')

    def configure_setup(context):
        robot_name_str = LaunchConfiguration('robot_name').perform(context)
        
        # 1. FIND PACKAGES
        try:
            pkg_share_description = pkg_share_description_temp.find(package_name_description)
            pkg_share_moveit_config = pkg_share_moveit_config_temp.find(package_name_moveit_config)
        except Exception:
            raise Exception("Could not find required packages. Run 'colcon build' and 'source install/setup.bash'.")

        # 2. FIND URDF
        urdf_path_1 = os.path.join(pkg_share_description, 'urdf', 'robots', urdf_file_name)
        urdf_path_2 = os.path.join(pkg_share_description, 'urdf', urdf_file_name)
        
        if os.path.exists(urdf_path_1):
            urdf_file_path = urdf_path_1
        elif os.path.exists(urdf_path_2):
            urdf_file_path = urdf_path_2
        else:
            raise Exception(f"URDF NOT FOUND at {urdf_path_1} or {urdf_path_2}")

        print(f"\n>>> DEBUG: URDF FOUND: {urdf_file_path}")

        # 3. FIND CONFIGS
        config_path = os.path.join(pkg_share_moveit_config, 'config')
        srdf_model_path = os.path.join(config_path, 'biped', 'biped.srdf')
        initial_positions_file_path = os.path.join(config_path, 'biped', 'initial_positions.yaml')
        joint_limits_file_path = os.path.join(config_path, 'biped', 'joint_limits.yaml')
        kinematics_file_path = os.path.join(config_path, 'biped', 'kinematics.yaml')
        moveit_controllers_file_path = os.path.join(config_path, 'biped', 'moveit_controllers.yaml')
        pilz_cartesian_limits_file_path = os.path.join(config_path, 'biped', 'pilz_cartesian_limits.yaml')
        # Check if they exist
        for path in [srdf_model_path, joint_limits_file_path, kinematics_file_path, moveit_controllers_file_path]:
            if not os.path.exists(path):
                print(f"\n>>> DEBUG ERROR: MISSING CONFIG FILE: {path}\n")

        # 4. BUILD MOVEIT CONFIG
        # Using OMPL only to reduce complexity for now
        moveit_config_obj = (
            MoveItConfigsBuilder(robot_name_str, package_name=package_name_moveit_config)
            .robot_description(file_path=urdf_file_path)
            .trajectory_execution(file_path=moveit_controllers_file_path)
            .robot_description_semantic(file_path=srdf_model_path)
            .joint_limits(file_path=joint_limits_file_path)
            .robot_description_kinematics(file_path=kinematics_file_path)
            .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
            .planning_scene_monitor(
                publish_robot_description=False,
                publish_robot_description_semantic=True,
                publish_planning_scene=True,
            )
            .to_moveit_configs()
        )

        moveit_config_dict = moveit_config_obj.to_dict()

        # ---------------------------------------------------------
        # THE FIX: SANITIZE DICTIONARY
        # This removes any None values that cause the crash and prints them
        # ---------------------------------------------------------
        sanitized_config = {}
        for key, value in moveit_config_dict.items():
            if value is None:
                print(f"\n\033[91m>>> CRITICAL WARNING: Parameter '{key}' is None! It was removed to prevent crash.\033[0m")
            else:
                sanitized_config[key] = value

        move_group_capabilities = {"capabilities": "move_group/ExecuteTaskSolutionCapability"}

        # Define Move Group Node
        start_move_group_node_cmd = Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[
                sanitized_config,
                {'use_sim_time': use_sim_time},
                {'start_state': {'content': initial_positions_file_path}},
                move_group_capabilities,
            ],
        )

        # Define RViz Node
        start_rviz_node_cmd = Node(
            condition=IfCondition(use_rviz),
            package="rviz2",
            executable="rviz2",
            arguments=[
                "-d",
                [FindPackageShare(rviz_config_package), "/rviz/", rviz_config_file]
            ],
            output="screen",
            parameters=[
                moveit_config_obj.robot_description,
                moveit_config_obj.robot_description_semantic,
                moveit_config_obj.planning_pipelines,
                moveit_config_obj.robot_description_kinematics,
                moveit_config_obj.joint_limits,
                {'use_sim_time': use_sim_time}
            ],
        )

        exit_event_handler = RegisterEventHandler(
            condition=IfCondition(use_rviz),
            event_handler=OnProcessExit(
                target_action=start_rviz_node_cmd,
                on_exit=EmitEvent(event=Shutdown(reason='rviz exited')),
            ),
        )

        return [start_move_group_node_cmd, start_rviz_node_cmd, exit_event_handler]

    ld = LaunchDescription()
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_rviz_config_package_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(OpaqueFunction(function=configure_setup))

    return ld