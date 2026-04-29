
#!/usr/bin/env python3
"""
Distributed Swarm Controller (Master Node)
Uses the Strategy Pattern to dynamically switch between control laws.
"""

import rclpy
from rclpy.node import Node
import numpy as np
import re
import time
import csv
import os

from geometry_msgs.msg import TwistStamped, PoseStamped
from tf_transformations import euler_from_quaternion
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# Import the strategies from your new library file
# Note: Adjust the import path if your ROS 2 package requires it 
# (e.g., from my_turtlebot_realFirst.control_strategies import ...)
from my_turtlebot_realFirst.control_strategies import (
    ConsensusCollision, 
    AbsoluteGoalTracking, 
    DynamicTrajectory, 
    AnisotropicEllipse,
    T1PointMassFlocking,
    T2DimensionAwareFlocking,
    TangentMappingFlocking
)

class DistributedControllerOptitrack(Node):
    def __init__(self):
        super().__init__('distributed_controller_optitrack')
        
        # --- Core Parameters ---
        self.declare_parameter('robot_namespace', 'tb1')
        self.declare_parameter('pose_topic', '/tb1/pose')
        self.ns = self.get_parameter('robot_namespace').value
        self.pose_topic = self.get_parameter('pose_topic').value

        # --- Control Strategy Selector ---
        self.declare_parameter('control_mode', 'dynamic_trajectory')
        self.declare_parameter('traj_type', 'circle')
        
        mode_name = self.get_parameter('control_mode').value
        self.traj_type = self.get_parameter('traj_type').value

        # Initialize the Strategy Dictionary
        self.strategies = {
            'consensus_collision': ConsensusCollision(),
            'absolute_goal': AbsoluteGoalTracking(),
            'dynamic_trajectory': DynamicTrajectory(),
            'anisotropic_ellipse': AnisotropicEllipse(),
            't1_flocking':T1PointMassFlocking(),
            't2_flocking':T2DimensionAwareFlocking(),
            'tangent_mapping': TangentMappingFlocking()
        }
        
        if mode_name not in self.strategies:
            self.get_logger().error(f"Unknown control_mode '{mode_name}'. Defaulting to 'consensus_collision'.")
            mode_name = 'consensus_collision'
            
        self.active_strategy = self.strategies[mode_name]
        self.get_logger().info(f"--- Swarm Controller Started ({self.ns.upper()}) ---")
        self.get_logger().info(f"ACTIVE MODE: {mode_name.upper()}")
        
        # --- Declare All Possible Math Parameters ---
        params_to_declare = [
            ('alpha', 1.0), ('beta', 0.25), ('rho', 0.5),
            ('goal_x', 0.0), ('goal_y', 0.0),
            ('traj_center_x', 0.0), ('traj_center_y', 0.0), 
            ('traj_radius', 0.8), ('traj_w', 0.12),
            ('m11', 1.0), ('m22', 1.0), ('is_rotating', True), ('ellipse_alpha', 0.0),
            ('b', 0.1), ('max_v', 0.18), ('max_w', 1.5),
            ('K', 2.0), ('y_bar', 0.0), ('safe_dist', 0.25) # <--- ADDED HERE
        ]
        for name, default in params_to_declare:
            self.declare_parameter(name, default)

        # Retrieve Kinematics
        self.b = self.get_parameter('b').value
        self.max_v = self.get_parameter('max_v').value
        self.max_w = self.get_parameter('max_w').value
        
        # --- State Variables ---
        self.x, self.y, self.theta = 0.0, 0.0, 0.0
        self.xB, self.yB = 0.0, 0.0 
        self.pose_received = False
        self.last_pose_time = 0.0
        self.experiment_start_time = None 
        self.neighbors = {}

        # --- Logging Setup ---
        save_dir = '/home/ricardo/multirobot_ws/src/multirobot_control/my_turtlebot_realFirst/plotting'
        os.makedirs(save_dir, exist_ok=True) 
        self.csv_filename = os.path.join(save_dir, f'swarm_effort_{mode_name}.csv')
        
        if not os.path.exists(self.csv_filename):
            try:
                with open(self.csv_filename, 'w', newline='') as f:
                    csv.writer(f).writerow(['timestamp', 'robot_id', 'v_cmd', 'w_cmd'])
            except Exception: pass 
        
        # --- ROS 2 Setup ---
        self.pub_cmd = self.create_publisher(TwistStamped, f'/{self.ns}/cmd_vel', 10)
        
        optitrack_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST, depth=1
        )
        self.sub_pose = self.create_subscription(PoseStamped, self.pose_topic, self.pose_callback, optitrack_qos)
        
        time.sleep(1.0)
        self.discover_neighbors_once()
        self.timer = self.create_timer(0.05, self.control_loop) # 20Hz Loop

    def discover_neighbors_once(self):
        topic_list = self.get_topic_names_and_types()
        kalman_pattern = rf'^/{self.ns}/tracked_(tb\d+)$'
        for topic_name, _ in topic_list:
            match = re.match(kalman_pattern, topic_name)
            if match:
                nid = match.group(1)
                self.create_subscription(PoseStamped, topic_name, lambda msg, n=nid: self.neighbor_callback(msg, n), 10)
                self.get_logger().info(f"✅ Subscribed to Neighbor: {topic_name}")

    def pose_callback(self, msg):
        self.pose_received = True
        curr = self.get_clock().now().nanoseconds / 1e9
        self.last_pose_time = curr
        if self.experiment_start_time is None:
            self.experiment_start_time = curr
            
        self.x, self.y = msg.pose.position.x, msg.pose.position.y
        orientation = msg.pose.orientation
        try:
            (_, _, self.theta) = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
            self.xB = self.x + self.b * np.cos(self.theta)
            self.yB = self.y + self.b * np.sin(self.theta)
        except Exception: return

    def neighbor_callback(self, msg, neighbor_id):
        self.neighbors[neighbor_id] = {
            'x': msg.pose.position.x, 'y': msg.pose.position.y,
            'last_time': self.get_clock().now().nanoseconds / 1e9
        }

    def get_trajectory_reference(self, t):
        """Generates the requested trajectory shape."""
        c_x = self.get_parameter('traj_center_x').value
        c_y = self.get_parameter('traj_center_y').value
        R = self.get_parameter('traj_radius').value
        w = self.get_parameter('traj_w').value

        if self.traj_type == 'figure8':
            # Vertical Figure-8 Math
            r_x = c_x + (R / 2.0) * np.sin(2 * w * t)
            r_y = c_y + 1.25 * np.sin(w * t)
            r_dot_x = R * w * np.cos(2 * w * t)
            r_dot_y = 1.25 * w * np.cos(w * t)
        else:
            # Default Circle Math
            r_x = c_x + R * np.cos(w * t)
            r_y = c_y + R * np.sin(w * t)
            r_dot_x = -R * w * np.sin(w * t)
            r_dot_y =  R * w * np.cos(w * t)
            
        return r_x, r_y, r_dot_x, r_dot_y

    def get_current_params_dict(self):
        """Packages all current ROS 2 parameters into a dictionary for the strategy."""
        return {
            'alpha': self.get_parameter('alpha').value,
            'beta': self.get_parameter('beta').value,
            'rho': self.get_parameter('rho').value,
            'goal_x': self.get_parameter('goal_x').value,
            'goal_y': self.get_parameter('goal_y').value,
            'm11': self.get_parameter('m11').value,
            'm22': self.get_parameter('m22').value,
            'is_rotating': self.get_parameter('is_rotating').value,
            'ellipse_alpha': self.get_parameter('ellipse_alpha').value,
            'K': self.get_parameter('K').value,                 # <--- ADDED HERE
            'y_bar': self.get_parameter('y_bar').value,         # <--- ADDED HERE
            'safe_dist': self.get_parameter('safe_dist').value
        }

    def control_loop(self):
        now = self.get_clock().now().nanoseconds / 1e9
        if not self.pose_received or self.experiment_start_time is None: return
        
        if (now - self.last_pose_time) > 2.0:
            self.stop_robot()
            return

        # 1. Package the State Data
        robot_state = {'xB': self.xB, 'yB': self.yB}
        
        # 2. Package the Target Data
        t = now - self.experiment_start_time
        traj_ref = self.get_trajectory_reference(t)
        
        # 3. Clean up inactive neighbors
        active_neighbors = {nid: data for nid, data in self.neighbors.items() if (now - data['last_time']) <= 2.0}
        
        if len(active_neighbors) == 0 and self.get_parameter('control_mode').value != 'absolute_goal':
            self.stop_robot()
            return

        # 4. EXECUTE STRATEGY
        params_dict = self.get_current_params_dict()
        u_x, u_y = self.active_strategy.compute(robot_state, active_neighbors, traj_ref, params_dict)

        # 5. I/O LINEARIZATION
        c_th, s_th = np.cos(self.theta), np.sin(self.theta)
        v_cmd = u_x * c_th + u_y * s_th
        w_cmd = (1.0 / self.b) * (-u_x * s_th + u_y * c_th)

        # 6. SATURATION
        scale = min(self.max_v / abs(v_cmd) if abs(v_cmd) > self.max_v else 1.0,
                    self.max_w / abs(w_cmd) if abs(w_cmd) > self.max_w else 1.0)
        v_cmd *= scale
        w_cmd *= scale

        # 7. LOG & PUBLISH
        try:
            with open(self.csv_filename, 'a', newline='') as f:
                csv.writer(f).writerow([now, self.ns, v_cmd, w_cmd])
        except Exception: pass 

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.linear.x, msg.twist.angular.z = float(v_cmd), float(w_cmd)
        self.pub_cmd.publish(msg)

    def stop_robot(self):
        msg = TwistStamped()
        self.pub_cmd.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DistributedControllerOptitrack()
    try: rclpy.spin(node)
    except KeyboardInterrupt: node.stop_robot()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()