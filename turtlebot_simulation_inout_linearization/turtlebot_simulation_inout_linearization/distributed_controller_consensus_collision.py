#!/usr/bin/env python3
"""
Distributed Swarm Controller (Master Node)
Uses dynamic 2-second heartbeat discovery and logs X/Y for Matplotlib trajectories.
"""

import rclpy
from rclpy.node import Node
import numpy as np
import re
import csv
import os

from geometry_msgs.msg import TwistStamped, PoseStamped, Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# Import the strategies from your simulation package
from turtlebot_simulation_inout_linearization.control_strategies import (
    ConsensusCollision, 
    AbsoluteGoalTracking, 
    DynamicTrajectory, 
    AnisotropicEllipse,
    T1PointMassFlocking,
    T2DimensionAwareFlocking
)

class DistributedControllerOptitrack(Node):
    def __init__(self):
        super().__init__('distributed_controller_optitrack')
        
        # --- Core Parameters ---
        self.declare_parameter('robot_namespace', 'tb1')
        self.declare_parameter('pose_topic', '/tb1/pose')
        self.declare_parameter('sim_mode', False)
        self.declare_parameter('use_twist_msg', False)
        
        self.ns = self.get_parameter('robot_namespace').value
        self.pose_topic = self.get_parameter('pose_topic').value
        self.is_sim = self.get_parameter('sim_mode').value
        self.use_twist = self.get_parameter('use_twist_msg').value

        # --- Control Strategy Selector ---
        self.declare_parameter('control_mode', 'dynamic_trajectory')
        self.declare_parameter('traj_type', 'circle')
        
        mode_name = self.get_parameter('control_mode').value
        self.traj_type = self.get_parameter('traj_type').value

        self.strategies = {
            'consensus_collision': ConsensusCollision(),
            'absolute_goal': AbsoluteGoalTracking(),
            'dynamic_trajectory': DynamicTrajectory(),
            'anisotropic_ellipse': AnisotropicEllipse(),
            't1_flocking':T1PointMassFlocking(),
            't2_flocking':T2DimensionAwareFlocking()
        }
        
        if mode_name not in self.strategies:
            self.get_logger().error(f"Unknown control_mode '{mode_name}'. Defaulting to 'consensus_collision'.")
            mode_name = 'consensus_collision'
            
        self.active_strategy = self.strategies[mode_name]
        self.get_logger().info(f"--- Swarm Controller Started ({self.ns.upper()}) ---")
        
        # --- Math Parameters ---
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

        # --- UPDATED: Logging Setup ---
        save_dir = '/home/ricardo/multirobot_ws/src/multirobot_control/turtlebot_simulation_inout_linearization/plotting'
        os.makedirs(save_dir, exist_ok=True) 
        self.csv_filename = os.path.join(save_dir, f'swarm_effort_{mode_name}.csv')
        
        if not os.path.exists(self.csv_filename):
            try:
                with open(self.csv_filename, 'w', newline='') as f:
                    # ADDED 'x' and 'y' to the header
                    csv.writer(f).writerow(['timestamp', 'robot_id', 'x', 'y', 'v_cmd', 'w_cmd'])
            except Exception: pass 
        
        # --- ROS 2 Setup ---
        self.cmd_type = Twist if self.use_twist else TwistStamped
        self.pub_cmd = self.create_publisher(self.cmd_type, f'/{self.ns}/cmd_vel', 10)
        
        optitrack_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST, depth=1
        )
        
        msg_type = Odometry if self.is_sim else PoseStamped
        self.sub_pose = self.create_subscription(msg_type, self.pose_topic, self.pose_callback, optitrack_qos)
        
        # CONTINUOUS TIMERS
        self.discovery_timer = self.create_timer(2.0, self.discover_neighbors) # Scan every 2 seconds
        self.timer = self.create_timer(0.05, self.control_loop)                # 20Hz Loop

    def extract_pose(self, msg):
        if hasattr(msg, 'pose') and hasattr(msg.pose, 'pose'):
            return msg.pose.pose
        else:
            return msg.pose

    def discover_neighbors(self):
        topic_list = self.get_topic_names_and_types()
        
        if self.is_sim:
            pattern = r'^/(tb\d+)/ground_truth_pose$'
        else:
            pattern = rf'^/{self.ns}/tracked_(tb\d+)$'
            
        for topic_name, topic_types in topic_list:
            match = re.match(pattern, topic_name)
            if match:
                nid = match.group(1)
                
                if self.is_sim and nid == self.ns:
                    continue 
                
                # Only subscribe if we haven't seen them yet
                if nid not in self.neighbors:
                    self.neighbors[nid] = {'x': 0.0, 'y': 0.0, 'last_time': 0.0}
                    msg_type = Odometry if 'nav_msgs/msg/Odometry' in topic_types else PoseStamped
                    self.create_subscription(
                        msg_type, topic_name, lambda msg, n=nid: self.neighbor_callback(msg, n), 10)
                    self.get_logger().info(f"✅ CONTROLLER ({self.ns.upper()}): Linked with {nid}!")

    def pose_callback(self, msg):
        self.pose_received = True
        curr = self.get_clock().now().nanoseconds / 1e9
        self.last_pose_time = curr
        if self.experiment_start_time is None:
            self.experiment_start_time = curr
            
        actual_pose = self.extract_pose(msg)
        self.x, self.y = actual_pose.position.x, actual_pose.position.y
        orientation = actual_pose.orientation
        
        try:
            (_, _, self.theta) = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
            self.xB = self.x + self.b * np.cos(self.theta)
            self.yB = self.y + self.b * np.sin(self.theta)
        except Exception: return

    def neighbor_callback(self, msg, neighbor_id):
        actual_pose = self.extract_pose(msg)
        self.neighbors[neighbor_id] = {
            'x': actual_pose.position.x, 'y': actual_pose.position.y,
            'last_time': self.get_clock().now().nanoseconds / 1e9
        }

    def get_trajectory_reference(self, t):
        c_x = self.get_parameter('traj_center_x').value
        c_y = self.get_parameter('traj_center_y').value
        R = self.get_parameter('traj_radius').value
        w = self.get_parameter('traj_w').value

        if self.traj_type == 'figure8':
            r_x = c_x + (R / 2.0) * np.sin(2 * w * t)
            r_y = c_y + 1.25 * np.sin(w * t)
            r_dot_x = R * w * np.cos(2 * w * t)
            r_dot_y = 1.25 * w * np.cos(w * t)
        else:
            r_x = c_x + R * np.cos(w * t)
            r_y = c_y + R * np.sin(w * t)
            r_dot_x = -R * w * np.sin(w * t)
            r_dot_y =  R * w * np.cos(w * t)
            
        return r_x, r_y, r_dot_x, r_dot_y

    def get_current_params_dict(self):
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
            'safe_dist': self.get_parameter('safe_dist').value  # <--- ADDED HERE
        }

    def control_loop(self):
        now = self.get_clock().now().nanoseconds / 1e9
        if not self.pose_received or self.experiment_start_time is None: return
        
        if (now - self.last_pose_time) > 2.0:
            self.stop_robot()
            return

        robot_state = {'xB': self.xB, 'yB': self.yB}
        t = now - self.experiment_start_time
        traj_ref = self.get_trajectory_reference(t)
        
        active_neighbors = {nid: data for nid, data in self.neighbors.items() if (now - data['last_time']) <= 2.0}
        
        if len(active_neighbors) == 0 and self.get_parameter('control_mode').value != 'absolute_goal':
            self.stop_robot()
            return

        params_dict = self.get_current_params_dict()
        u_x, u_y = self.active_strategy.compute(robot_state, active_neighbors, traj_ref, params_dict)

        c_th, s_th = np.cos(self.theta), np.sin(self.theta)
        v_cmd = u_x * c_th + u_y * s_th
        w_cmd = (1.0 / self.b) * (-u_x * s_th + u_y * c_th)

        scale = min(self.max_v / abs(v_cmd) if abs(v_cmd) > self.max_v else 1.0,
                    self.max_w / abs(w_cmd) if abs(w_cmd) > self.max_w else 1.0)
        v_cmd *= scale
        w_cmd *= scale

        # --- UPDATED: LOG & PUBLISH ---
        try:
            with open(self.csv_filename, 'a', newline='') as f:
                # ADDED self.x and self.y
                csv.writer(f).writerow([now, self.ns, self.x, self.y, v_cmd, w_cmd])
        except Exception: pass 

        msg = self.cmd_type()
        if not self.use_twist:
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "base_link"
            msg.twist.linear.x = float(v_cmd)
            msg.twist.angular.z = float(w_cmd)
        else:
            msg.linear.x = float(v_cmd)
            msg.angular.z = float(w_cmd)

        self.pub_cmd.publish(msg)

    def stop_robot(self):
        msg = self.cmd_type()
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