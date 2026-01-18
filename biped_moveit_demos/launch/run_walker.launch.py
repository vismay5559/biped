import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # 1. Get Package Paths
    pkg_share_description = FindPackageShare(package='biped_description').find('biped_description')
    pkg_share_config = FindPackageShare(package='biped_moveit_config').find('biped_moveit_config')

    # 2. Define Explicit File Paths
    # URDF: Check locations
    urdf_file_path = os.path.join(pkg_share_description, 'urdf', 'robots', 'biped.xacro')

    
  

    # CONFIGS: Point to the 'biped' subdirectory
    # This is the key change to support your preferred structure
    config_dir = os.path.join(pkg_share_config, 'config', 'biped')

    srdf_file_path = os.path.join(config_dir, 'biped.srdf')
    joint_limits_path = os.path.join(config_dir, 'joint_limits.yaml')
    kinematics_path = os.path.join(config_dir, 'kinematics.yaml')
    controllers_path = os.path.join(config_dir, 'moveit_controllers.yaml')
    pilz_limits_path = os.path.join(config_dir, 'pilz_cartesian_limits.yaml')

    # 3. Load MoveIt Configs with Explicit Paths
    moveit_config = (
        MoveItConfigsBuilder("biped", package_name="biped_moveit_config")
        .robot_description(file_path=urdf_file_path)
        .robot_description_semantic(file_path=srdf_file_path)
        .robot_description_kinematics(file_path=kinematics_path)
        .joint_limits(file_path=joint_limits_path)
        .trajectory_execution(file_path=controllers_path)
        .pilz_cartesian_limits(file_path=pilz_limits_path)
        .to_moveit_configs()
    )

    # 4. Define the Node
    walker_node = Node(
        package="biped_moveit_demos",
        executable="biped_walk",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': True}
        ]
    )

    return LaunchDescription([walker_node])