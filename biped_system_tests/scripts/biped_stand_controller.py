#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class BipedStandController(Node):
    def __init__(self):
        super().__init__('biped_stand_controller')

        # Action client for your biped trajectory controller
        self.arm_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/biped_joint_trajectory_controller/follow_joint_trajectory'
        )

        self.get_logger().info('Waiting for biped controller...')
        self.arm_client.wait_for_server()
        
        # Joint names from your URDF
        self.joint_names = [
            'l_abduction', 'l_hip_roll', 'l_knee_roll', 
            'r_abduction', 'r_hip_roll', 'r_knee_roll'
        ]

        # Define a "Standing" pose (slight knee bend helps stability)
        # Sequence: [L_abd, L_hip, L_knee, L_foot, R_abd, R_hip, R_knee, R_foot]
        self.stand_pose = [0.0, -0.1, 0.1, 0.0, 0.1, -1]

        self.get_logger().info('Sending Stand Command...')
        self.send_stand_command()

    def send_stand_command(self):
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = self.stand_pose
        point.time_from_start = Duration(sec=1, nanosec=0) # Move to stand in 1 second

        goal_msg.trajectory.points = [point]
        self.arm_client.send_goal_async(goal_msg)
        self.get_logger().info('Standing pose sent!')

def main(args=None):
    rclpy.init(args=args)
    node = BipedStandController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
