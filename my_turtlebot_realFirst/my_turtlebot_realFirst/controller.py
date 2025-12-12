

#!/usr/bin/env python3
"""
Input/Output Linearization Controller (Robust OptiTrack)
Updates:
- Increased safety timeout to 2.0 seconds
- Added debug prints to track connection health
"""

import rclpy
from rclpy.node import Node
import numpy as np

from geometry_msgs.msg import TwistStamped, Point, PoseStamped
from tf_transformations import euler_from_quaternion
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

class IOLinearizationOptitrack(Node):
    def __init__(self):
        super().__init__('io_linearization_optitrack')
        
        # --- Parameters ---
        self.declare_parameter('robot_namespace', 'burger1')
        self.declare_parameter('pose_topic', '/tb1/pose')
        self.declare_parameter('Kx', 1.5)
        self.declare_parameter('Ky', 1.5)
        self.declare_parameter('b', 0.1)
        self.declare_parameter('goal_tolerance', 0.05)
        self.declare_parameter('max_v', 0.22)
        self.declare_parameter('max_w', 1.5)
        
        self.ns = self.get_parameter('robot_namespace').value
        self.pose_topic = self.get_parameter('pose_topic').value
        self.Kx = self.get_parameter('Kx').value
        self.Ky = self.get_parameter('Ky').value
        self.b = self.get_parameter('b').value
        self.dist_tol = self.get_parameter('goal_tolerance').value
        self.max_v = self.get_parameter('max_v').value
        self.max_w = self.get_parameter('max_w').value
        
        # --- State ---
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.xB = 0.0
        self.yB = 0.0
        self.xB_goal = 0.0
        self.yB_goal = 0.0
        self.has_goal = False
        self.pose_received = False
        self.last_pose_time = 0.0
        
        self.pub_cmd = self.create_publisher(TwistStamped, f'/{self.ns}/cmd_vel', 10)
        
        # --- QoS Profile (Best Effort / Volatile) ---
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
        
        self.sub_goal = self.create_subscription(
            Point,
            f'/{self.ns}/robot_goal', 
            self.goal_callback,
            10
        )
        
        self.timer = self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info(f"--- Controller Started ---")
        self.get_logger().info(f"Waiting for Best-Effort data on: {self.pose_topic}")

    def pose_callback(self, msg):
        self.pose_received = True
        self.last_pose_time = self.get_clock().now().nanoseconds / 1e9
        
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        
        # Debug print every 1 second (approx) to confirm liveness
        # if int(self.last_pose_time) % 2 == 0 and int(self.last_pose_time * 10) % 5 == 0:
        #    self.get_logger().info(f"Data OK: x={self.x:.2f}, y={self.y:.2f}", throttle_duration_sec=2.0)

        orientation = msg.pose.orientation
        try:
            (_, _, self.theta) = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        except Exception:
            return
            
        self.xB = self.x + self.b * np.cos(self.theta)
        self.yB = self.y + self.b * np.sin(self.theta)

    def goal_callback(self, msg):
        if not self.pose_received:
            self.get_logger().warn("Goal ignored: No OptiTrack data yet.")
            return
        self.xB_goal = msg.x
        self.yB_goal = msg.y
        self.has_goal = True
        self.get_logger().info(f"NEW GOAL: ({self.xB_goal:.2f}, {self.yB_goal:.2f})")

    def control_loop(self):
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        # SAFETY CHECK: Increased timeout to 2.0 seconds
        if self.pose_received and (current_time - self.last_pose_time) > 2.0:
            self.get_logger().warn(f"TIMEOUT: Last data was {current_time - self.last_pose_time:.1f}s ago", throttle_duration_sec=1.0)
            self.stop_robot()
            return

        if not self.has_goal or not self.pose_received:
            return

        e_x = self.xB_goal - self.xB
        e_y = self.yB_goal - self.yB
        dist_error = np.sqrt(e_x**2 + e_y**2)

        if dist_error < self.dist_tol:
            self.get_logger().info(f"Goal Reached (Error: {dist_error:.3f}m)", throttle_duration_sec=2.0)
            self.stop_robot()
            self.has_goal = False
            return

        # Control Law
        u_x = self.Kx * e_x
        u_y = self.Ky * e_y
        
        c_th = np.cos(self.theta)
        s_th = np.sin(self.theta)
        
        v_cmd = u_x * c_th + u_y * s_th
        w_cmd = (1.0 / self.b) * (-u_x * s_th + u_y * c_th)

        # Saturation
        v_cmd = np.clip(v_cmd, -self.max_v, self.max_v)
        w_cmd = np.clip(w_cmd, -self.max_w, self.max_w)

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
    node = IOLinearizationOptitrack()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_robot()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()