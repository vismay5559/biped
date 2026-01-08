#!/usr/bin/env python3
"""
Simple biped controller (myCobot-style)

Publishes joint positions to:
  /biped_position_controller/commands

Controller type:
  forward_command_controller/ForwardCommandController

Author: You (with sanity restored)
"""

import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class BipedController(Node):

    def __init__(self):
        super().__init__('biped_basic_controller')

        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/biped_position_controller/commands',
            10
        )

        self.get_logger().info('Waiting for controller...')
        time.sleep(2.0)
        self.get_logger().info('Biped controller started')

        # Joint order MUST match controller YAML
        self.stand = [
            0.0,   # l_abduction
            0.0,   # l_hip_roll
            -0.2,  # l_knee_roll
            0.0,   # r_abduction
            0.0,   # r_hip_roll
            -0.2   # r_knee_roll
        ]

        self.squat = [
            0.0,
            0.0,
            -0.6,
            0.0,
            0.0,
            -0.6
        ]

        self.timer = self.create_timer(0.05, self.control_loop)

        self.state = 0
        self.last_switch = time.time()

    def send(self, positions):
        msg = Float64MultiArray()
        msg.data = positions
        self.publisher.publish(msg)

    def control_loop(self):

        now = time.time()

        # Switch state every 3 seconds
        if now - self.last_switch > 3.0:
            self.state = (self.state + 1) % 2
            self.last_switch = now

        if self.state == 0:
            self.send(self.stand)
        else:
            self.send(self.squat)


def main():
    rclpy.init()
    node = BipedController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
