#!/usr/bin/env python3
"""
Input/Output Linearization Controller for Real TurtleBot3 (Point B Control)

CONTROLLER TYPE: Exact input/output linearization
CONTROL OBJECTIVE: Regulation of Point B to a target coordinate.

REAL ROBOT:
- Uses TwistStamped
- Implements safety velocity saturation
- Checks for valid Odometry before controlling
"""

import rclpy
from rclpy.node import Node
import numpy as np

from geometry_msgs.msg import TwistStamped, Point
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion


class IOLinearizationReal(Node):
    def __init__(self):
        super().__init__('io_linearization_real_point_b')
        
        # --- Parameters ---
        self.declare_parameter('robot_namespace', 'burger1')
        self.declare_parameter('Kx', 1.5)  # Gain for X
        self.declare_parameter('Ky', 1.5)  # Gain for Y
        self.declare_parameter('b', 0.2)   # Distance to Point B (lookahead)
        self.declare_parameter('goal_tolerance', 0.05)
        
        # Safety limits (Real Robot)
        self.declare_parameter('max_linear_vel', 0.22)
        self.declare_parameter('max_angular_vel', 1.5)
        
        self.ns = self.get_parameter('robot_namespace').value
        self.Kx = self.get_parameter('Kx').value
        self.Ky = self.get_parameter('Ky').value
        self.b = self.get_parameter('b').value
        self.dist_tol = self.get_parameter('goal_tolerance').value
        self.max_v = self.get_parameter('max_linear_vel').value
        self.max_w = self.get_parameter('max_angular_vel').value
        
        # --- State ---
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Point B State
        self.xB = 0.0
        self.yB = 0.0
        
        # Goal State (Point B)
        self.xB_goal = 0.0
        self.yB_goal = 0.0
        self.has_goal = False
        self.odom_received = False
        
        # --- Communication ---
        self.pub_cmd = self.create_publisher(
            TwistStamped, 
            f'/{self.ns}/cmd_vel', 
            10
        )
        
        self.sub_odom = self.create_subscription(
            Odometry,
            f'/{self.ns}/odom',
            self.odom_callback,
            10
        )
        
        self.sub_goal = self.create_subscription(
            Point,
            f'/{self.ns}/robot_goal', # Publishes goal directly for Point B
            self.goal_callback,
            10
        )
        
        # Timer (20Hz is sufficient for real robot)
        self.timer = self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info("--- Real Robot IO Linearization (Point B) ---")
        self.get_logger().info(f"Namespace: {self.ns}")
        self.get_logger().info(f"Offset b: {self.b}m")
        self.get_logger().info(f"Safety Limits: Vmax={self.max_v}, Wmax={self.max_w}")

    def odom_callback(self, msg):
        """Update robot state and calculate current Point B"""
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        
        rot = msg.pose.pose.orientation
        try:
            _, _, self.theta = euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])
        except Exception:
            return

        # [cite_start]Calculate Point B (The point we are actually controlling) [cite: 26]
        self.xB = self.x + self.b * np.cos(self.theta)
        self.yB = self.y + self.b * np.sin(self.theta)
        
        self.odom_received = True

    def goal_callback(self, msg):
        """Receive goal coordinates for Point B"""
        if not self.odom_received:
            self.get_logger().warn("Ignored goal: Waiting for Odometry...")
            return

        # Directly map input message to Point B goal
        self.xB_goal = msg.x
        self.yB_goal = msg.y
        self.has_goal = True
        
        self.get_logger().info(f"New Point B Goal: ({self.xB_goal:.2f}, {self.yB_goal:.2f})")

    def control_loop(self):
        if not self.has_goal or not self.odom_received:
            return

        # 1. Calculate Error (in Point B frame)
        ex = self.xB_goal - self.xB
        ey = self.yB_goal - self.yB
        dist_error = np.sqrt(ex**2 + ey**2)

        # 2. Check Tolerance
        if dist_error < self.dist_tol:
            self.stop_robot()
            self.has_goal = False
            self.get_logger().info(f"Goal Reached. Final Error: {dist_error:.3f}m")
            return

        # 3. Virtual Inputs
        # We assume a static goal point, so velocity feedforward is 0
        ux = self.Kx * ex
        uy = self.Ky * ey

        # 4. Decoupling Matrix 
        # v = ux*cos(theta) + uy*sin(theta)
        # w = (-ux*sin(theta) + uy*cos(theta)) / b
        c_th = np.cos(self.theta)
        s_th = np.sin(self.theta)

        v_cmd = ux * c_th + uy * s_th
        w_cmd = (1.0 / self.b) * (-ux * s_th + uy * c_th)

        # 5. Safety Saturation (Real Robot Protection)
        v_cmd = np.clip(v_cmd, -self.max_v, self.max_v)
        w_cmd = np.clip(w_cmd, -self.max_w, self.max_w)

        # 6. Publish TwistStamped
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = float(v_cmd)
        msg.twist.angular.z = float(w_cmd)
        self.pub_cmd.publish(msg)

    def stop_robot(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        self.pub_cmd.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = IOLinearizationReal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_robot()
        node.get_logger().info("Node stopped by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()