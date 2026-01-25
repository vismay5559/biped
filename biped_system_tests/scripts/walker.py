#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import math
import numpy as np

# ================== PARAMETERS (MATCHING YOUR WALKER) ==================
DT = 0.02           
THIGH_LENGTH = 0.245 
SHIN_LENGTH  = 0.215
MAX_REACH    = THIGH_LENGTH + SHIN_LENGTH

# The "Zero" position for the foot relative to the hip
# Usually directly underneath: x=0, y=0, z = -(thigh + shin)
DEFAULT_Z = -0.42 # A bit bent, not fully extended to avoid singularity

def Rx(a):
    ca, sa = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0],
                     [0, ca, -sa],
                     [0, sa,  ca]])

def solve_leg_ik(x, y, z, is_left):
    """
    Your exact IK function.
    Input: Vector from HIP to FOOT.
    """
    l1 = THIGH_LENGTH
    l2 = SHIN_LENGTH

    p = np.array([x, y, z])

    # 1) Abduction about x
    # Note: If z is negative (foot below hip), atan2(y, -z) works for standard orientation
    theta_abd = math.atan2(y, -z)

    if not is_left:
        theta_abd = -theta_abd

    # Rotate into sagittal plane (x–z)
    p_sag = Rx(-theta_abd) @ p
    x_p, _, z_p = p_sag

    # 2) Planar IK in the x–z plane for hip/knee (about y)
    D = math.sqrt(x_p**2 + z_p**2)

    # Safety clamping for reach
    if D >= (l1 + l2):
        D = l1 + l2 - 1e-4

    cos_knee = (l1**2 + l2**2 - D**2) / (2 * l1 * l2)
    cos_knee = max(-1.0, min(1.0, cos_knee)) # Clamp for domain errors

    gamma = math.acos(cos_knee)
    theta_knee = math.pi - gamma

    alpha = math.asin((l2 * math.sin(gamma)) / D)
    beta = math.atan2(x_p, -z_p)

    theta_hip = beta + alpha

    return theta_abd, theta_hip, theta_knee

class IKVerifier(Node):

    def __init__(self):
        super().__init__("ik_verifier")
        
        # Publisher
        self.publisher = self.create_publisher(JointTrajectory, "/biped_joint_trajectory_controller/joint_trajectory", 10)
        
        # Timer
        self.timer = self.create_timer(DT, self.control_loop)
        self.t = 0.0
        
        self.joint_names = [
            "l_abduction", "l_hip_roll", "l_knee_roll",
            "r_abduction", "r_hip_roll", "r_knee_roll"
        ]
        
        self.get_logger().info("IK Verifier Started.")
        self.get_logger().info("Drawing a circle with the RIGHT FOOT...")

    def control_loop(self):
        # ================== TEST TRAJECTORY ==================
        # We will move the RIGHT foot in a circle relative to the RIGHT hip.
        
        # Radius of the test circle (meters)
        radius = 0.05 
        speed = 2.0 # Rad/s

        # Calculate changing X and Z
        # Center of circle is (0, 0, DEFAULT_Z)
        target_x = radius * math.cos(speed * self.t)
        target_z = DEFAULT_Z + (radius * math.sin(speed * self.t))
        target_y = 0.0 # Keep it centered in Y for now

        # ================== COMPUTE IK ==================
        
        # 1. LEFT LEG: Keep Static (Reference)
        # Vector from Hip to Foot: (0, 0, DEFAULT_Z)
        l_abd, l_hip, l_knee = solve_leg_ik(0.0, 0.0, DEFAULT_Z, True)

        # 2. RIGHT LEG: Moving
        # Vector from Hip to Foot: (target_x, target_y, target_z)
        r_abd, r_hip, r_knee = solve_leg_ik(target_x, target_y, target_z, False)

        # ================== VISUALIZATION & LOGGING ==================
        # Print every 20 iterations to avoid spamming console
        if int(self.t / DT) % 20 == 0:
            print(f"--- Time: {self.t:.2f} ---")
            print(f"Target (X, Z):  [{target_x:.3f}, {target_z:.3f}]")
            print(f"Right Joints:   Abd: {r_abd:.2f} | Hip: {r_hip:.2f} | Knee: {r_knee:.2f}")

        # ================== PUBLISH ==================
        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = [l_abd, l_hip, l_knee, r_abd, r_hip, r_knee]
        point.time_from_start = Duration(sec=0, nanosec=0)
        traj.points.append(point)
        self.publisher.publish(traj)

        self.t += DT

def main():
    rclpy.init()
    node = IKVerifier()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()