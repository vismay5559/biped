#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import Imu
from builtin_interfaces.msg import Duration
import math
import numpy as np

# ================== PARAMETERS ==================
DT = 0.01           
STEP_TIME = 4.0     # Slow walk
STEP_LENGTH = 0.10  
STEP_HEIGHT = 0.04  

# ---- GAINS (POSITIVE = Standard Stabilization) ----
KP_PITCH = 0.8  
KP_ROLL  = 0.5  

# ---- ROBOT PHYSICAL PARAMETERS ----
THIGH_LENGTH = 0.245 
SHIN_LENGTH  = 0.215
MAX_REACH    = THIGH_LENGTH + SHIN_LENGTH
TARGET_COM_HEIGHT = 0.42  
HIP_OFFSET_Y = 0.072 

# ================== HELPERS ==================
def quintic(t, T, p0, pT):
    if t <= 0.0: return p0
    if t >= T:   return pT
    tau = t / T
    return p0 + (pT - p0) * (10*tau**3 - 15*tau**4 + 6*tau**5)

def euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    
    return roll_x, pitch_y, yaw_z 

def solve_leg_ik(x, y, z, is_left):
    l1 = THIGH_LENGTH
    l2 = SHIN_LENGTH
    
    theta_abd = math.atan2(y, -z)
    if not is_left: theta_abd = -theta_abd
    
    z_prime = z / math.cos(theta_abd)
    D = math.sqrt(x**2 + z_prime**2)
    
    if D >= (l1 + l2): D = l1 + l2 - 0.0001
    
    cos_knee = (l1**2 + l2**2 - D**2) / (2 * l1 * l2)
    cos_knee = max(-1.0, min(1.0, cos_knee))
    gamma = math.acos(cos_knee)
    theta_knee = (math.pi - gamma)
    
    alpha = math.asin((l2 * math.sin(gamma)) / D)
    beta = math.atan2(x, -z_prime)
    theta_hip = beta + alpha
    
    return theta_abd, theta_hip, theta_knee

# ================== CONTROLLER NODE ==================
class BipedWalkingController(Node):

    def __init__(self):
        super().__init__("biped_walking_controller")
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        
        self.publisher = self.create_publisher(JointTrajectory, "/biped_joint_trajectory_controller/joint_trajectory", 10)
        self.imu_sub = self.create_subscription(Imu, "/imu", self.imu_callback, 10)
        self.timer = self.create_timer(DT, self.control_loop)
        
        self.t = 0.0
        
        # Calibration State
        self.current_roll = 0.0
        self.current_pitch = 0.0
        self.target_roll = 0.0
        self.target_pitch = 0.0
        self.calibration_samples = 0
        self.CALIBRATION_LIMIT = 50 
        self.is_calibrated = False

        self.state = "CALIBRATING"
        
        self.joint_names = [
            "l_abduction", "l_hip_roll", "l_knee_roll",
            "r_abduction", "r_hip_roll", "r_knee_roll"
        ]
        self.get_logger().info("Controller Started. DO NOT MOVE ROBOT (Calibrating IMU)...")

    def imu_callback(self, msg):
        r, p, y = euler_from_quaternion(
            msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
        )
        self.current_roll = r
        self.current_pitch = p

        if not self.is_calibrated:
            self.target_roll += r
            self.target_pitch += p
            self.calibration_samples += 1
            
            if self.calibration_samples >= self.CALIBRATION_LIMIT:
                self.target_roll /= self.CALIBRATION_LIMIT
                self.target_pitch /= self.CALIBRATION_LIMIT
                self.is_calibrated = True
                self.state = "INIT"
                self.get_logger().info(f"CALIBRATION DONE. Zero Roll: {self.target_roll:.3f}")

    def control_loop(self):
        if not self.is_calibrated:
            return 

        # --- STATE MACHINE ---
        target_x = 0.0
        target_y = 0.0
        target_z = MAX_REACH - 0.005
        foot_x = 0.0
        foot_z = 0.0

        if self.state == "INIT":
            if self.t > 1.0:
                self.t = 0.0
                self.state = "SQUAT"
                self.get_logger().info(">>> SQUATTING <<<")
            t_cmd = 0.0

        elif self.state == "SQUAT":
            DURATION = 2.0
            if self.t > DURATION:
                self.t = 0.0
                self.state = "SWAY"
                self.get_logger().info(">>> SWAYING LEFT <<<")
                target_z = TARGET_COM_HEIGHT
            else:
                target_z = quintic(self.t, DURATION, MAX_REACH - 0.005, TARGET_COM_HEIGHT)
            t_cmd = 0.0

        elif self.state == "SWAY":
            DURATION = 2.0
            if self.t > DURATION:
                self.t = 0.0
                self.state = "STEP"
                self.get_logger().info(">>> STEPPING RIGHT <<<")
                target_y = HIP_OFFSET_Y
            else:
                target_y = quintic(self.t, DURATION, 0.0, HIP_OFFSET_Y)
            target_z = TARGET_COM_HEIGHT

        elif self.state == "STEP":
            if self.t > STEP_TIME:
                self.state = "HOLD"
                t_cmd = STEP_TIME
                self.get_logger().info("Step Finished.")
            else:
                t_cmd = self.t
            
            target_z = TARGET_COM_HEIGHT
            target_y = HIP_OFFSET_Y
            target_x = quintic(t_cmd, STEP_TIME, 0.0, STEP_LENGTH / 2)
            foot_x = quintic(t_cmd, STEP_TIME, 0.0, STEP_LENGTH)
            foot_z = STEP_HEIGHT * math.sin(math.pi * t_cmd / STEP_TIME)

        elif self.state == "HOLD":
            t_cmd = STEP_TIME
            target_z = TARGET_COM_HEIGHT
            target_y = HIP_OFFSET_Y
            target_x = STEP_LENGTH / 2
            foot_x = STEP_LENGTH
            foot_z = 0.0

        # --- INVERSE KINEMATICS ---
        l_foot_pos = np.array([0.0, HIP_OFFSET_Y, 0.0])
        r_foot_pos = np.array([foot_x, -HIP_OFFSET_Y, foot_z])
        l_hip_pos = np.array([target_x, target_y + HIP_OFFSET_Y, target_z])
        r_hip_pos = np.array([target_x, target_y - HIP_OFFSET_Y, target_z])
        
        l_vec = l_foot_pos - l_hip_pos
        r_vec = r_foot_pos - r_hip_pos

        try:
            l_abd, l_hip, l_knee = solve_leg_ik(l_vec[0], l_vec[1], l_vec[2], True)
            r_abd, r_hip, r_knee = solve_leg_ik(r_vec[0], r_vec[1], r_vec[2], False)
            
            # --- POSITIVE FEEDBACK (Correct for standard IMU) ---
            # If we pitch forward (Positive Error), we pitch hips back (Positive Correction)
            pitch_correction = KP_PITCH * (self.current_pitch - self.target_pitch)
            
            # If we roll right (Positive Error), we push hips left (Positive Correction)
            roll_correction  = KP_ROLL * (self.current_roll - self.target_roll)

            l_hip += pitch_correction
            r_hip += pitch_correction
            
            l_abd += roll_correction
            r_abd += roll_correction

        except ValueError:
            return

        # --- PUBLISH ---
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
    node = BipedWalkingController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()