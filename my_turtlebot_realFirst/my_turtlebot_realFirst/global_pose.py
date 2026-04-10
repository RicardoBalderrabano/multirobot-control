#!/usr/bin/env python3
"""
Multi-Robot Global Coordinate Transformer
-----------------------------------------
This ROS 2 node bridges decentralized local robot odometry with an absolute global 
ground truth system (OptiTrack). It dynamically discovers an N-agent fleet 
at runtime, calculates the initial Homogeneous Transformation Matrix offset 
(T_offset = T_opti * T_odom^-1), and applies this mapping to all subsequent 
high-frequency state estimations in real-time.

Author: Ricardo Balderrabano Rodriguez
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import numpy as np
import tf_transformations
import re
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class MultiGlobalTransformer(Node):
    def __init__(self):
        super().__init__('multi_global_transformer')
        
        # State management for N-agents. 
        # Dictionaries allow O(1) lookups for scalable fleet management.
        self.initial_opti = {}
        self.initial_odom = {}
        self.offset_matrices = {}  # Stores the static T_offset matrices
        self.active_robots = set()

        # Dynamic Discovery: Polls the ROS 2 graph every 2 seconds for new agents
        self.create_timer(2.0, self.discover_robots)
        self.get_logger().info("Multi-Robot Matrix Transformer Started (With Inverse Odom Fix).")

    def get_4x4_matrix(self, x, y, z, q):
        """ Converts ROS 2 quaternion and translation into an SE(3) Homogeneous Transformation Matrix """
        matrix = tf_transformations.quaternion_matrix(q)
        matrix[0, 3] = x
        matrix[1, 3] = y
        matrix[2, 3] = z
        return matrix

    def discover_robots(self):
        """ 
        Fleet Scalability: Uses regex to dynamically discover topics matching '/tbX/pose'.
        Eliminates the need to hardcode namespaces, allowing seamless scaling at runtime.
        """
        topic_list = self.get_topic_names_and_types()
        pattern = r'^/tb(\d+)/pose$'
        for topic_name, _ in topic_list:
            match = re.match(pattern, topic_name)
            if match:
                robot_id = f"tb{match.group(1)}"
                if robot_id not in self.active_robots:
                    self.setup_robot(robot_id)

    def setup_robot(self, robot_id):
        """ Initializes publishers and subscribers with network-aware QoS profiles """
        self.get_logger().info(f"Setting up listeners for {robot_id}...")
        
        # Network Resilience: High-frequency UDP MoCap data requires Best-Effort QoS 
        # to prevent queue latency or network choking over Wi-Fi.
        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, 
            history=HistoryPolicy.KEEP_LAST, 
            depth=1
        )
        
        # Use lambda functions to inject the robot_id context into the shared callbacks
        self.create_subscription(PoseStamped, f'/{robot_id}/pose', 
            lambda msg, rid=robot_id: self.opti_callback(msg, rid), qos_best_effort)
        
        self.create_subscription(Odometry, f'/{robot_id}/odom', 
            lambda msg, rid=robot_id: self.ekf_callback(msg, rid), 10)
        
        pub = self.create_publisher(PoseStamped, f'/{robot_id}/global_pose_estimated', 10)
        setattr(self, f"pub_{robot_id}", pub)
        
        self.active_robots.add(robot_id)

    def opti_callback(self, msg, robot_id):
        """ Captures the global ground truth initial state (T_opti,0) """
        if robot_id not in self.initial_opti:
            q = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
            self.initial_opti[robot_id] = self.get_4x4_matrix(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, q)
            self.check_initialization(robot_id)

    def ekf_callback(self, ekf_msg, robot_id):
        """ 
        High-frequency callback for local EKF odometry.
        Maps local estimates to the global frame in real-time.
        """
        q_local = [ekf_msg.pose.pose.orientation.x, ekf_msg.pose.pose.orientation.y, 
                   ekf_msg.pose.pose.orientation.z, ekf_msg.pose.pose.orientation.w]
        T_local = self.get_4x4_matrix(ekf_msg.pose.pose.position.x, ekf_msg.pose.pose.position.y, ekf_msg.pose.pose.position.z, q_local)

        # Capture initial local state (T_odom,0)
        if robot_id not in self.initial_odom:
            self.initial_odom[robot_id] = T_local
            self.check_initialization(robot_id)
            return

        # Real-Time CPU Optimization: Apply the pre-calculated offset via lightweight dot product.
        # Avoids continuous matrix inversion during high-frequency execution.
        if robot_id in self.offset_matrices:
            T_global = np.dot(self.offset_matrices[robot_id], T_local)
            q_global = tf_transformations.quaternion_from_matrix(T_global)

            out = PoseStamped()
            out.header = ekf_msg.header
            out.header.frame_id = "world"
            out.pose.position.x = T_global[0, 3]
            out.pose.position.y = T_global[1, 3]
            out.pose.position.z = T_global[2, 3]
            out.pose.orientation.x = q_global[0]
            out.pose.orientation.y = q_global[1]
            out.pose.orientation.z = q_global[2]
            out.pose.orientation.w = q_global[3]
            
            pub = getattr(self, f"pub_{robot_id}")
            pub.publish(out)

    def check_initialization(self, robot_id):
        """ 
        Calculates the True Offset by resolving initial frame misalignments.
        Executes exactly ONCE per robot to save CPU cycles. 
        Math: T_offset = T_opti * inverse(T_odom)
        """
        if robot_id in self.initial_opti and robot_id in self.initial_odom and robot_id not in self.offset_matrices:
            odom_inv = np.linalg.inv(self.initial_odom[robot_id])
            self.offset_matrices[robot_id] = np.dot(self.initial_opti[robot_id], odom_inv)
            self.get_logger().info(f">>> [ROBOT {robot_id}] True Global Offset Locked!")

def main():
    rclpy.init()
    node = MultiGlobalTransformer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()