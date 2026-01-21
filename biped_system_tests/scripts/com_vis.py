#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Point, Vector3
from visualization_msgs.msg import Marker
from std_msgs.msg import String
from tf2_ros import TransformListener, Buffer
import xml.etree.ElementTree as ET
import numpy as np
import math

class COMVisualizer(Node):
    def __init__(self):
        super().__init__('com_visualizer')

        # --- CONFIGURATION ---
        # The frame you want to calculate CoM relative to (usually base_link or world)
        self.target_frame = 'base_link' 
        
        # --- SUBSCRIBERS ---
        # We need the URDF to know masses and inertial offsets
        # "transient_local" is required to read latched topics like /robot_description
        qos_profile = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.sub_urdf = self.create_subscription(
            String, 
            '/robot_description', 
            self.urdf_callback, 
            qos_profile
        )

        # --- PUBLISHERS ---
        self.marker_pub = self.create_publisher(Marker, '/visual/com', 1)
        self.ground_proj_pub = self.create_publisher(Marker, '/visual/com_ground', 1)

        # --- TF BUFFER ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- STATE ---
        self.link_data = {} # Stores mass and inertial offsets for each link
        self.total_mass = 0.0
        self.urdf_loaded = False

        # Run the calculation loop at 50Hz
        self.create_timer(0.02, self.calculate_com)
        self.get_logger().info("Waiting for /robot_description...")

    def urdf_callback(self, msg):
        """Parses URDF XML to get Link Masses and Inertial Offsets"""
        try:
            root = ET.fromstring(msg.data)
            total_mass = 0.0
            
            for link in root.findall('link'):
                name = link.get('name')
                inertial = link.find('inertial')
                
                if inertial is not None:
                    # 1. Get Mass
                    mass_elem = inertial.find('mass')
                    if mass_elem is not None:
                        mass = float(mass_elem.get('value'))
                        
                        # 2. Get Inertial Offset (Center of Mass relative to Link Origin)
                        origin = inertial.find('origin')
                        xyz = [0.0, 0.0, 0.0]
                        rpy = [0.0, 0.0, 0.0]
                        
                        if origin is not None:
                            if origin.get('xyz'):
                                xyz = [float(x) for x in origin.get('xyz').split()]
                            if origin.get('rpy'):
                                rpy = [float(x) for x in origin.get('rpy').split()]

                        # Only store links that actually have mass
                        if mass > 0.001:
                            self.link_data[name] = {
                                'mass': mass,
                                'xyz': np.array(xyz),
                                'rpy': np.array(rpy)
                            }
                            total_mass += mass
                            
            self.total_mass = total_mass
            self.urdf_loaded = True
            self.get_logger().info(f"URDF Parsed! Total Mass: {self.total_mass:.3f} kg. Tracking {len(self.link_data)} links.")
            
        except Exception as e:
            self.get_logger().error(f"Failed to parse URDF: {str(e)}")

    def get_transform(self, target_frame, source_frame):
        """Helper to get 4x4 homogenous transform matrix from TF"""
        try:
            t = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
            
            # Translation
            trans = np.array([
                t.transform.translation.x,
                t.transform.translation.y,
                t.transform.translation.z
            ])
            
            # Rotation (Quaternion to Matrix)
            q = [
                t.transform.rotation.x, 
                t.transform.rotation.y, 
                t.transform.rotation.z, 
                t.transform.rotation.w
            ]
            
            # Basic Quaternion to Rotation Matrix conversion
            x, y, z, w = q
            R = np.array([
                [1 - 2*y*y - 2*z*z,  2*x*y - 2*z*w,      2*x*z + 2*y*w],
                [2*x*y + 2*z*w,      1 - 2*x*x - 2*z*z,  2*y*z - 2*x*w],
                [2*x*z - 2*y*w,      2*y*z + 2*x*w,      1 - 2*x*x - 2*y*y]
            ])
            
            # Combine into 4x4 matrix
            T = np.eye(4)
            T[:3, :3] = R
            T[:3, 3] = trans
            return T
            
        except Exception:
            return None

    def calculate_com(self):
        if not self.urdf_loaded:
            return

        weighted_position_sum = np.zeros(3)
        current_mass_sum = 0.0

        for link_name, properties in self.link_data.items():
            mass = properties['mass']
            inertial_offset = properties['xyz'] # [x, y, z] relative to link frame
            
            # Get transform from Target Frame (base_link) -> Link Frame
            T_target_link = self.get_transform(self.target_frame, link_name)
            
            if T_target_link is not None:
                # Transform the inertial offset into the target frame
                # P_global = T_link * P_local
                local_com = np.append(inertial_offset, 1.0) # Make homogeneous [x,y,z,1]
                global_com = T_target_link @ local_com
                
                # Add to weighted sum: (Position * Mass)
                weighted_position_sum += global_com[:3] * mass
                current_mass_sum += mass

        if current_mass_sum > 0:
            final_com = weighted_position_sum / current_mass_sum
            self.publish_markers(final_com)

    def publish_markers(self, com_pos):
        timestamp = self.get_clock().now().to_msg()
        
        # 1. The Actual 3D CoM (Red Sphere)
        marker = Marker()
        marker.header.frame_id = self.target_frame
        marker.header.stamp = timestamp
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.scale = Vector3(x=0.05, y=0.05, z=0.05) # 5cm size
        marker.color.a = 1.0
        marker.color.r = 1.0 # Red
        marker.pose.position.x = com_pos[0]
        marker.pose.position.y = com_pos[1]
        marker.pose.position.z = com_pos[2]
        self.marker_pub.publish(marker)

        # 2. The Ground Projection (Blue Flat Cylinder)
        # This helps you see if the CoM is inside the support polygon (feet)
        shadow = Marker()
        shadow.header.frame_id = self.target_frame
        shadow.header.stamp = timestamp
        shadow.type = Marker.CYLINDER
        shadow.action = Marker.ADD
        shadow.scale = Vector3(x=0.05, y=0.05, z=0.005) # Flat disk
        shadow.color.a = 0.5
        shadow.color.b = 1.0 # Blue
        shadow.pose.position.x = com_pos[0]
        shadow.pose.position.y = com_pos[1]
        shadow.pose.position.z = 0.0 # Force to floor (relative to base_link)
        self.ground_proj_pub.publish(shadow)

def main(args=None):
    rclpy.init(args=args)
    node = COMVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()