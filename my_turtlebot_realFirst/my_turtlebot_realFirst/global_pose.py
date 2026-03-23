"""
#THIS WAS THE PREVIOUS CONSIDERING THE 0,0 AT STARTING
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
        Creates a 4x4 Homogeneous Transformation Matrix.
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
        
        # Store separate initial states to calculate the true mathematical offset
        self.initial_opti = {}
        self.initial_odom = {}
        self.offset_matrices = {}  # The final T_world_odom matrices
        self.active_robots = set()

        self.create_timer(2.0, self.discover_robots)
        self.get_logger().info("Multi-Robot Matrix Transformer Started (With Inverse Odom Fix).")

    def get_4x4_matrix(self, x, y, z, q):
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
        qos_best_effort = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        
        self.create_subscription(PoseStamped, f'/{robot_id}/pose', 
            lambda msg, rid=robot_id: self.opti_callback(msg, rid), qos_best_effort)
        
        self.create_subscription(Odometry, f'/{robot_id}/odometry/filtered', 
            lambda msg, rid=robot_id: self.ekf_callback(msg, rid), 10)
        
        pub = self.create_publisher(PoseStamped, f'/{robot_id}/global_pose_estimated', 10)
        setattr(self, f"pub_{robot_id}", pub)
        self.active_robots.add(robot_id)

    def opti_callback(self, msg, robot_id):
        if robot_id not in self.initial_opti:
            q = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
            self.initial_opti[robot_id] = self.get_4x4_matrix(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, q)
            self.check_initialization(robot_id)

    def ekf_callback(self, ekf_msg, robot_id):
        q_local = [ekf_msg.pose.pose.orientation.x, ekf_msg.pose.pose.orientation.y, 
                   ekf_msg.pose.pose.orientation.z, ekf_msg.pose.pose.orientation.w]
        T_local = self.get_4x4_matrix(ekf_msg.pose.pose.position.x, ekf_msg.pose.pose.position.y, ekf_msg.pose.pose.position.z, q_local)

        # Grab the very first odometry reading to subtract it from the global offset
        if robot_id not in self.initial_odom:
            self.initial_odom[robot_id] = T_local
            self.check_initialization(robot_id)
            return

        # Once initialized, apply the true offset
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
        """ Calculates the True Offset by removing the initial Odometry drift """
        if robot_id in self.initial_opti and robot_id in self.initial_odom and robot_id not in self.offset_matrices:
            # T_offset = T_opti * inverse(T_odom)
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