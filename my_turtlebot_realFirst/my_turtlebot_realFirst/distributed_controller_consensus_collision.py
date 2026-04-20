"""
# 1
#!/usr/bin/env python3

Distributed Consensus & Collision Avoidance Controller (Real Robot Version)
Updates:
- Replaced static Goal I/O Linearization with Artificial Potential Fields
- Auto-discovers local EKF tracking topics
- Maintains OptiTrack QoS and TwistStamped publishers
- Added Latching Velocity Deadband to prevent equilibrium shivering
- Added informative terminal logging


import rclpy
from rclpy.node import Node
import numpy as np
import re
import time

from geometry_msgs.msg import TwistStamped, PoseStamped
from tf_transformations import euler_from_quaternion
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

class DistributedControllerOptitrack(Node):
    def __init__(self):
        super().__init__('distributed_controller_optitrack')
        
        # --- Parameters ---
        self.declare_parameter('robot_namespace', 'tb1')
        self.declare_parameter('pose_topic', '/tb1/pose')
        
        # Swarm Control Gains
        self.declare_parameter('alpha', 1.0) # Cohesion (Attraction)
        self.declare_parameter('beta', 5.0)  # Dispersion (Repulsion)
        
        # Kinematics
        self.declare_parameter('b', 0.1)
        self.declare_parameter('max_v', 0.22)
        self.declare_parameter('max_w', 1.5)
        
        self.ns = self.get_parameter('robot_namespace').value
        self.pose_topic = self.get_parameter('pose_topic').value
        
        self.alpha = self.get_parameter('alpha').value
        self.beta = self.get_parameter('beta').value
        self.b = self.get_parameter('b').value
        self.max_v = self.get_parameter('max_v').value
        self.max_w = self.get_parameter('max_w').value
        
        # --- State ---
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.xB = 0.0
        self.yB = 0.0
        
        self.pose_received = False
        self.last_pose_time = 0.0
        
        # Latching Mechanism State
        self.equilibrium_locked = False
        
        # Dictionary to store latest neighbor positions from the local EKF
        self.neighbors = {}
        
        self.pub_cmd = self.create_publisher(TwistStamped, f'/{self.ns}/cmd_vel', 10)
        
        # --- QoS Profile (Best Effort / Volatile for OptiTrack) ---
        optitrack_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscribe to Ego Ground Truth
        self.sub_pose = self.create_subscription(
            PoseStamped,
            self.pose_topic,
            self.pose_callback,
            optitrack_qos
        )
        
        self.get_logger().info(f"--- Distributed Controller Started ({self.ns.upper()}) ---")
        self.get_logger().info(f"Waiting for Best-Effort data on: {self.pose_topic}")
        
        # Auto-Discover Local EKF Tracking Topics
        time.sleep(1.0)
        self.discover_neighbors_once()
        
        self.timer = self.create_timer(0.05, self.control_loop)

    def discover_neighbors_once(self):
        #Automatically finds and subscribes to locally tracked neighbors
        topic_list = self.get_topic_names_and_types()
        
        # Only look for topics published by THIS robot's isolated EKF
        kalman_pattern = rf'^/{self.ns}/tracked_(tb\d+)$'
        
        discovered_count = 0
        
        for topic_name, _ in topic_list:
            match = re.match(kalman_pattern, topic_name)
            if match:
                nid = match.group(1) # Extracts 'tb2', 'tb3', etc.
                
                self.create_subscription(
                    PoseStamped,
                    topic_name,
                    lambda msg, n=nid: self.neighbor_callback(msg, n),
                    10
                )
                discovered_count += 1
                self.get_logger().info(f"✅ Auto-subscribed to local EKF tracking: {topic_name}")
                    
        if discovered_count == 0:
            self.get_logger().warn("⚠️ No local tracking topics discovered. Is the EKF running?")

    def pose_callback(self, msg):
        self.pose_received = True
        self.last_pose_time = self.get_clock().now().nanoseconds / 1e9
        
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y

        orientation = msg.pose.orientation
        try:
            (_, _, self.theta) = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        except Exception:
            return
            
        self.xB = self.x + self.b * np.cos(self.theta)
        self.yB = self.y + self.b * np.sin(self.theta)

    def neighbor_callback(self, msg, neighbor_id):
        self.neighbors[neighbor_id] = {
            'x': msg.pose.position.x,
            'y': msg.pose.position.y,
            'last_time': self.get_clock().now().nanoseconds / 1e9
        }

    def control_loop(self):
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        # SAFETY CHECK: Ego pose timeout
        if self.pose_received and (current_time - self.last_pose_time) > 2.0:
            self.get_logger().warn(f"TIMEOUT: Last Ego data was {current_time - self.last_pose_time:.1f}s ago", throttle_duration_sec=1.0)
            self.stop_robot()
            return

        if not self.pose_received:
            return

        # --- DISTRIBUTED CONTROL LAW (Sum of Forces) ---
        u_x = 0.0
        u_y = 0.0
        active_neighbors = 0
        
        for nid, data in self.neighbors.items():
            # Ignore stale neighbor data (> 2.0 seconds old)
            if (current_time - data['last_time']) > 2.0:
                continue
                
            active_neighbors += 1
            n_x = data['x']
            n_y = data['y']
            
            dx = self.xB - n_x
            dy = self.yB - n_y
            dist_sq = dx**2 + dy**2
            
            # Prevent division by zero singularity
            if dist_sq < 0.0001:
                dist_sq = 0.0001
                
            # Attraction (Cohesion) + Repulsion (Collision Avoidance)
            u_x += -self.alpha * dx + self.beta * (dx / dist_sq)
            u_y += -self.alpha * dy + self.beta * (dy / dist_sq)

        # If no neighbors are seen, safely stop
        if active_neighbors == 0:
            self.get_logger().info("No active neighbors. Holding position.", throttle_duration_sec=2.0)
            self.stop_robot()
            return

        # --- I/O LINEARIZATION MAPPING ---
        c_th = np.cos(self.theta)
        s_th = np.sin(self.theta)
        
        v_cmd = u_x * c_th + u_y * s_th
        w_cmd = (1.0 / self.b) * (-u_x * s_th + u_y * c_th)

        # --- SATURATION (SCALING METHOD) ---
        scale_v = 1.0
        scale_w = 1.0
        
        if abs(v_cmd) > self.max_v:
            scale_v = self.max_v / abs(v_cmd)
            
        if abs(w_cmd) > self.max_w:
            scale_w = self.max_w / abs(w_cmd)
            
        scale = min(scale_v, scale_w)
        
        v_cmd = v_cmd * scale
        w_cmd = w_cmd * scale

        # --- LATCHING DEADBAND FIX ---
        # 1. If currently locked, require a massive force to break the lock
        if self.equilibrium_locked:
            if abs(v_cmd) > 0.08 or abs(w_cmd) > 0.8:
                self.equilibrium_locked = False
                self.get_logger().warn("Formation broken! Resuming movement.")
            else:
                # Force motors to zero while locked
                v_cmd = 0.0
                w_cmd = 0.0
                self.get_logger().info(
                    f"[{active_neighbors} Neighbors] Locked in equilibrium.", 
                    throttle_duration_sec=5.0
                )

        # 2. If not locked, check if we should lock
        elif abs(v_cmd) < 0.02 and abs(w_cmd) < 0.2:
            self.equilibrium_locked = True
            v_cmd = 0.0
            w_cmd = 0.0
            self.get_logger().info("Equilibrium reached. Locking brakes!")
            
        # 3. Normal movement logging
        else:
            self.get_logger().info(
                f"[{active_neighbors} Neighbors] Adjusting formation -> v: {v_cmd:.3f} m/s, w: {w_cmd:.3f} rad/s", 
                throttle_duration_sec=2.0
            )

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.linear.x = float(v_cmd)
        msg.twist.angular.z = float(w_cmd)
        self.pub_cmd.publish(msg)

    def stop_robot(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        self.pub_cmd.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DistributedControllerOptitrack()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_robot()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

"""
#!/usr/bin/env python3
"""
Distributed Consensus, Collision Avoidance, and Formation Tracking
Updates:
- Added Absolute Goal Tracking (-rho * (bi - G))
- Auto-discovers local EKF tracking topics
- Maintains Latching Velocity Deadband
- Unified CSV Control Effort Logging to specific directory
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

class DistributedControllerOptitrack(Node):
    def __init__(self):
        super().__init__('distributed_controller_optitrack')
        
        # --- Parameters ---
        self.declare_parameter('robot_namespace', 'tb1')
        self.declare_parameter('pose_topic', '/tb1/pose')
        
        # Swarm Control Gains (Local)
        self.declare_parameter('alpha', 1.0) # Cohesion
        self.declare_parameter('beta', 5.0)  # Dispersion
        
        # Formation Tracking Gains & Goal (Absolute)
        self.declare_parameter('rho', 0.5)   # Goal Tracking Pull
        self.declare_parameter('goal_x', 0.0) # Global Goal X (OptiTrack Frame)
        self.declare_parameter('goal_y', -1.5) # Global Goal Y (OptiTrack Frame)
        
        # Kinematics
        self.declare_parameter('b', 0.1)
        self.declare_parameter('max_v', 0.22)
        self.declare_parameter('max_w', 1.5)
        
        self.ns = self.get_parameter('robot_namespace').value
        self.pose_topic = self.get_parameter('pose_topic').value
        
        self.alpha = self.get_parameter('alpha').value
        self.beta = self.get_parameter('beta').value
        self.rho = self.get_parameter('rho').value
        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value
        
        self.b = self.get_parameter('b').value
        self.max_v = self.get_parameter('max_v').value
        self.max_w = self.get_parameter('max_w').value
        
        # --- State ---
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.xB = 0.0 # This is b_i(x)
        self.yB = 0.0 # This is b_i(y)
        
        self.pose_received = False
        self.last_pose_time = 0.0
        
        # Latching Mechanism State
        self.equilibrium_locked = False
        self.noise_frames = 0
        
        self.neighbors = {}

        # --- UNIFIED LOGGING SETUP ---
        save_dir = '/home/ricardo/multirobot_ws/src/multirobot_control/my_turtlebot_realFirst/plotting'
        os.makedirs(save_dir, exist_ok=True)  # Safely create the directory if it doesn't exist
        self.csv_filename = os.path.join(save_dir, 'swarm_control_effort.csv')
        
        # Only write the header if the file doesn't exist yet
        if not os.path.exists(self.csv_filename):
            try:
                with open(self.csv_filename, 'w', newline='') as f:
                    csv.writer(f).writerow(['timestamp', 'robot_id', 'v_cmd', 'w_cmd'])
            except Exception:
                pass # Another robot process might have beaten us to it
        # ------------------------------
        
        self.pub_cmd = self.create_publisher(TwistStamped, f'/{self.ns}/cmd_vel', 10)
        
        optitrack_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.sub_pose = self.create_subscription(
            PoseStamped,
            self.pose_topic,
            self.pose_callback,
            optitrack_qos
        )
        
        self.get_logger().info(f"--- Formation Tracking Controller Started ({self.ns.upper()}) ---")
        self.get_logger().info(f"Target Goal: G=({self.goal_x}, {self.goal_y}) with Rho={self.rho}")
        
        time.sleep(1.0)
        self.discover_neighbors_once()
        
        self.timer = self.create_timer(0.05, self.control_loop)

    def discover_neighbors_once(self):
        topic_list = self.get_topic_names_and_types()
        kalman_pattern = rf'^/{self.ns}/tracked_(tb\d+)$'
        discovered_count = 0
        
        for topic_name, _ in topic_list:
            match = re.match(kalman_pattern, topic_name)
            if match:
                nid = match.group(1)
                self.create_subscription(PoseStamped, topic_name, lambda msg, n=nid: self.neighbor_callback(msg, n), 10)
                discovered_count += 1
                self.get_logger().info(f"✅ Auto-subscribed to local EKF tracking: {topic_name}")
                    
        if discovered_count == 0:
            self.get_logger().warn("⚠️ No local tracking topics discovered. Is the EKF running?")

    def pose_callback(self, msg):
        self.pose_received = True
        self.last_pose_time = self.get_clock().now().nanoseconds / 1e9
        
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y

        orientation = msg.pose.orientation
        try:
            (_, _, self.theta) = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        except Exception:
            return
            
        # Calculate b_i
        self.xB = self.x + self.b * np.cos(self.theta)
        self.yB = self.y + self.b * np.sin(self.theta)

    def neighbor_callback(self, msg, neighbor_id):
        self.neighbors[neighbor_id] = {
            'x': msg.pose.position.x,
            'y': msg.pose.position.y,
            'last_time': self.get_clock().now().nanoseconds / 1e9
        }

    def control_loop(self):
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        if self.pose_received and (current_time - self.last_pose_time) > 2.0:
            self.get_logger().warn(f"TIMEOUT: Last Ego data was {current_time - self.last_pose_time:.1f}s ago", throttle_duration_sec=1.0)
            self.stop_robot()
            return

        if not self.pose_received:
            return

        u_x = 0.0
        u_y = 0.0
        active_neighbors = 0
        
        # --- 1. LOCAL APF (Cohesion & Dispersion) ---
        for nid, data in self.neighbors.items():
            if (current_time - data['last_time']) > 2.0:
                continue
                
            active_neighbors += 1
            n_x = data['x'] # b_j(x)
            n_y = data['y'] # b_j(y)
            
            dx = self.xB - n_x
            dy = self.yB - n_y
            dist_sq = dx**2 + dy**2
            
            if dist_sq < 0.0001:
                dist_sq = 0.0001
                
            u_x += -self.alpha * dx + self.beta * (dx / dist_sq)
            u_y += -self.alpha * dy + self.beta * (dy / dist_sq)

        if active_neighbors == 0:
            self.get_logger().info("No active neighbors. Holding position.", throttle_duration_sec=2.0)
            self.stop_robot()
            return

        # --- 2. ABSOLUTE APF (Goal Tracking) ---
        # -rho * (b_i - G)
        u_x += -self.rho * (self.xB - self.goal_x)
        u_y += -self.rho * (self.yB - self.goal_y)

        # --- I/O LINEARIZATION MAPPING ---
        c_th = np.cos(self.theta)
        s_th = np.sin(self.theta)
        
        v_cmd = u_x * c_th + u_y * s_th
        w_cmd = (1.0 / self.b) * (-u_x * s_th + u_y * c_th)

        # --- SATURATION ---
        scale_v = self.max_v / abs(v_cmd) if abs(v_cmd) > self.max_v else 1.0
        scale_w = self.max_w / abs(w_cmd) if abs(w_cmd) > self.max_w else 1.0
        scale = min(scale_v, scale_w)
        
        v_cmd *= scale
        w_cmd *= scale

        # --- LATCHING DEADBAND FIX (With Noise Debouncing) ---
        if self.equilibrium_locked:
            if abs(v_cmd) > 0.08 or abs(w_cmd) > 0.8:
                self.noise_frames += 1
                if self.noise_frames >= 4:
                    self.equilibrium_locked = False
                    self.noise_frames = 0
                    self.get_logger().warn("Tracking error detected! Resuming movement.")
                else:
                    v_cmd = 0.0
                    w_cmd = 0.0
            else:
                self.noise_frames = 0
                v_cmd = 0.0
                w_cmd = 0.0
                self.get_logger().info(f"[{active_neighbors} Neighbors] Locked at Goal.", throttle_duration_sec=5.0)

        elif abs(v_cmd) < 0.02 and abs(w_cmd) < 0.20:
            self.equilibrium_locked = True
            self.noise_frames = 0
            v_cmd = 0.0
            w_cmd = 0.0
            self.get_logger().info("Goal reached. Locking brakes!")
            
        else:
            self.get_logger().info(
                f"[{active_neighbors} Neighbors] Tracking -> v: {v_cmd:.3f} m/s, w: {w_cmd:.3f} rad/s", 
                throttle_duration_sec=2.0
            )

        # --- UNIFIED LOGGING APPEND ---
        # Write the final commanded velocities (after deadband/saturation logic)
        try:
            with open(self.csv_filename, 'a', newline='') as f:
                csv.writer(f).writerow([current_time, self.ns, v_cmd, w_cmd])
        except Exception as e:
            self.get_logger().debug(f"Failed to write to CSV: {e}")
        # -----------------------------

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.linear.x = float(v_cmd)
        msg.twist.angular.z = float(w_cmd)
        self.pub_cmd.publish(msg)

    def stop_robot(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        self.pub_cmd.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DistributedControllerOptitrack()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_robot()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()