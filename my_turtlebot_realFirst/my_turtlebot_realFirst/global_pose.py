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
        
        # Stores the initial T matrix for each robot: { "tb1": matrix, "tb3": matrix }
        self.initial_matrices = {}
        
        # Keep track of which robots we have already set up
        self.active_robots = set()

        # Timer to "discover" new robots on the network every 2 seconds
        self.create_timer(2.0, self.discover_robots)
        
        self.get_logger().info("Multi-Robot Transformer Started. Searching for /tbX/pose topics...")

    def get_matrix_from_pose(self, x, y, quaternion):
        _, _, yaw = tf_transformations.euler_from_quaternion(quaternion)
        return np.array([
            [np.cos(yaw), -np.sin(yaw), x],
            [np.sin(yaw),  np.cos(yaw), y],
            [0,            0,           1]
        ])

    def discover_robots(self):
        topic_list = self.get_topic_names_and_types()
        # Look for the OptiTrack pose topic pattern: /tb1/pose, /tb2/pose...
        pattern = r'^/tb(\d+)/pose$'
        
        for topic_name, _ in topic_list:
            match = re.match(pattern, topic_name)
            if match:
                robot_id = f"tb{match.group(1)}"
                if robot_id not in self.active_robots:
                    self.setup_robot(robot_id)

    def setup_robot(self, robot_id):
        self.get_logger().info(f"Setting up listeners for {robot_id}...")
        
        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscribe to OptiTrack
        self.create_subscription(
            PoseStamped, 
            f'/{robot_id}/pose', 
            lambda msg, rid=robot_id: self.opti_callback(msg, rid), 
            qos_best_effort)
        
        # Subscribe to EKF
        self.create_subscription(
            Odometry, 
            f'/{robot_id}/odometry/filtered', 
            lambda msg, rid=robot_id: self.ekf_callback(msg, rid), 
            10)
        
        # Publisher
        pub = self.create_publisher(PoseStamped, f'/{robot_id}/global_pose_estimated', 10)
        setattr(self, f"pub_{robot_id}", pub)
        
        self.active_robots.add(robot_id)

    def opti_callback(self, msg, robot_id):
        # Only compute the matrix if we haven't locked it yet
        if robot_id not in self.initial_matrices:
            q = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
            self.initial_matrices[robot_id] = self.get_matrix_from_pose(msg.pose.position.x, msg.pose.position.y, q)
            
            self.get_logger().info(f">>> [ROBOT {robot_id}] Global Origin Locked!")
            # We don't call destroy() here anymore to avoid the InvalidHandle error.
            # The 'if' check above handles ignoring future messages.

    def ekf_callback(self, ekf_msg, robot_id):
        if robot_id not in self.initial_matrices:
            return # Wait for OptiTrack initialization

        # 1. Apply Transformation Matrix to Position
        local_point = np.array([ekf_msg.pose.pose.position.x, ekf_msg.pose.pose.position.y, 1.0])
        global_point = np.dot(self.initial_matrices[robot_id], local_point)

        # 2. Combine Orientations
        q_local = [ekf_msg.pose.pose.orientation.x, ekf_msg.pose.pose.orientation.y, 
                   ekf_msg.pose.pose.orientation.z, ekf_msg.pose.pose.orientation.w]
        _, _, yaw_local = tf_transformations.euler_from_quaternion(q_local)
        
        # Extract initial yaw from stored matrix
        _, _, yaw_init = tf_transformations.euler_from_matrix(self.initial_matrices[robot_id])
        
        total_yaw = yaw_init + yaw_local
        q_global = tf_transformations.quaternion_from_euler(0, 0, total_yaw)

        # 3. Publish
        out = PoseStamped()
        out.header = ekf_msg.header
        out.header.frame_id = "world"
        out.pose.position.x = global_point[0]
        out.pose.position.y = global_point[1]
        out.pose.orientation.x = q_global[0]
        out.pose.orientation.y = q_global[1]
        out.pose.orientation.z = q_global[2]
        out.pose.orientation.w = q_global[3]
        
        pub = getattr(self, f"pub_{robot_id}")
        pub.publish(out)

def main():
    rclpy.init()
    node = MultiGlobalTransformer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
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
        
        # Stores the initial 4x4 T matrix for each robot: { "tb1": matrix }
        self.initial_matrices = {}
        self.active_robots = set()

        # Discovery timer
        self.create_timer(2.0, self.discover_robots)
        self.get_logger().info("Multi-Robot Matrix Transformer Started.")

    def get_4x4_matrix(self, x, y, z, q):
        """Creates a 4x4 Homogeneous Transformation Matrix."""
        # This matches the 3D kinematic model logic in robot_localization
        matrix = tf_transformations.quaternion_matrix(q)
        matrix[0, 3] = x
        matrix[1, 3] = y
        matrix[2, 3] = z
        return matrix

    def discover_robots(self):
        topic_list = self.get_topic_names_and_types()
        pattern = r'^/tb(\d+)/pose$'
        for topic_name, _ in topic_list:
            match = re.match(pattern, topic_name)
            if match:
                robot_id = f"tb{match.group(1)}"
                if robot_id not in self.active_robots:
                    self.setup_robot(robot_id)

    def setup_robot(self, robot_id):
        self.get_logger().info(f"Setting up listeners for {robot_id}...")
        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscriber: OptiTrack (Initial Anchor)
        self.create_subscription(PoseStamped, f'/{robot_id}/pose', 
            lambda msg, rid=robot_id: self.opti_callback(msg, rid), qos_best_effort)
        
        # Subscriber: EKF (Continuous Local Estimate) [cite: 9-10]
        self.create_subscription(Odometry, f'/{robot_id}/odometry/filtered', 
            lambda msg, rid=robot_id: self.ekf_callback(msg, rid), 10)
        
        # Global Publisher
        pub = self.create_publisher(PoseStamped, f'/{robot_id}/global_pose_estimated', 10)
        setattr(self, f"pub_{robot_id}", pub)
        self.active_robots.add(robot_id)

    def opti_callback(self, msg, robot_id):
        if robot_id not in self.initial_matrices:
            q = [msg.pose.orientation.x, msg.pose.orientation.y, 
                 msg.pose.orientation.z, msg.pose.orientation.w]
            # Lock the initial global offset T_world_odom [cite: 96-98]
            self.initial_matrices[robot_id] = self.get_4x4_matrix(
                msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, q)
            self.get_logger().info(f">>> [ROBOT {robot_id}] Global Matrix Locked!")

    def ekf_callback(self, ekf_msg, robot_id):
        if robot_id not in self.initial_matrices:
            return 

        # 1. Build Local EKF Matrix (T_odom_base)
        q_local = [ekf_msg.pose.pose.orientation.x, ekf_msg.pose.pose.orientation.y, 
                   ekf_msg.pose.pose.orientation.z, ekf_msg.pose.pose.orientation.w]
        T_local = self.get_4x4_matrix(
            ekf_msg.pose.pose.position.x, 
            ekf_msg.pose.pose.position.y, 
            ekf_msg.pose.pose.position.z, 
            q_local
        )

        # 2. Matrix Multiplication: T_world_base = T_world_odom * T_odom_base
        # This handles position and orientation simultaneously
        T_global = np.dot(self.initial_matrices[robot_id], T_local)

        # 3. Extract Global Pose from Matrix [cite: 96-98]
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

def main():
    rclpy.init()
    node = MultiGlobalTransformer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()