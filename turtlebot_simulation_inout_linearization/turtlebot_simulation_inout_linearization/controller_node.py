#!/usr/bin/env python3
"""
Input/Output Linearization Controller for Point B

CONTROLLER TYPE: Exact input/output linearization via feedback linearization
CONTROL OBJECTIVE: Drive Point B (ahead of robot) to a target coordinates (xB_goal, yB_goal)

CONTROL LAW:
  [v]   [cos(θ)   sin(θ)] [ ux ]
  [ω] = [-sin(θ)/b cos(θ)/b] [ uy ]
  
  Where virtual inputs are:
  ux = xB_dot_ref + Kx * (xB_ref - xB)
  uy = yB_dot_ref + Ky * (yB_ref - yB)
"""

import rclpy
from rclpy.node import Node
import numpy as np

from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion

class IOLinearizationPointB(Node):
    def __init__(self):
        super().__init__('io_linearization_point_b')
        
        # --- Parameters ---
        self.declare_parameter('robot_namespace', 'burger1')
        self.declare_parameter('Kx', 1.0)  # Proportional gain for X
        self.declare_parameter('Ky', 1.0)  # Proportional gain for Y
        self.declare_parameter('b', 0.2)   # Distance from center to point B [cite: 25]
        self.declare_parameter('goal_tolerance', 0.05)
        
        self.ns = self.get_parameter('robot_namespace').value
        self.Kx = self.get_parameter('Kx').value
        self.Ky = self.get_parameter('Ky').value
        self.b = self.get_parameter('b').value
        self.dist_tol = self.get_parameter('goal_tolerance').value
        
        # --- State ---
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Point B current state
        self.xB = 0.0
        self.yB = 0.0
        
        # Point B Goal state
        self.xB_goal = 0.0
        self.yB_goal = 0.0
        self.goal_received = False
        
        # --- Communication ---
        self.pub_cmd = self.create_publisher(Twist, f'/{self.ns}/cmd_vel', 10)
        self.sub_odom = self.create_subscription(Odometry, f'/{self.ns}/odom', self.odom_callback, 10)
        self.sub_goal = self.create_subscription(Point, f'/{self.ns}/robot_goal', self.goal_callback, 10)
        
        self.timer = self.create_timer(0.02, self.control_loop) # 50Hz

        self.get_logger().info("I/O Linearization: Controlling Point B directly.")
        self.get_logger().info(f"Offset b={self.b}m. Gains Kx={self.Kx}, Ky={self.Ky}")

    def odom_callback(self, msg):
        # 1. Get Robot State (x, y, theta)
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        
        rot = msg.pose.pose.orientation
        (_, _, self.theta) = euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])
        
        # 2. Compute Point B Position 
        # xB = x + b*cos(theta)
        # yB = y + b*sin(theta)
        self.xB = self.x + self.b * np.cos(self.theta)
        self.yB = self.y + self.b * np.sin(self.theta)

    def goal_callback(self, msg):
        # We interpret the incoming Point msg as the TARGET FOR POINT B directly.
        self.xB_goal = msg.x
        self.yB_goal = msg.y
        self.goal_received = True
        self.get_logger().info(f"New Goal for B: ({self.xB_goal:.2f}, {self.yB_goal:.2f})")

    def control_loop(self):
        if not self.goal_received:
            return

        # 1. Calculate Errors for Point B
        ex = self.xB_goal - self.xB
        ey = self.yB_goal - self.yB
        dist_error = np.sqrt(ex**2 + ey**2)
        
        if dist_error < self.dist_tol:
            self.stop_robot()
            return

        # 2. Linear Control Law (Virtual Inputs) 
        # We assume a static goal (reference velocity = 0)
        # ux = dot_xB* + Kx(xB* - xB) -> ux = Kx * ex
        ux = self.Kx * ex
        uy = self.Ky * ey
        
        # 3. Decoupling Matrix / Feedback Linearization 
        # The inverse of the T matrix:
        # v = ux*cos(theta) + uy*sin(theta)
        # w = (-ux*sin(theta) + uy*cos(theta)) / b
        
        c_th = np.cos(self.theta)
        s_th = np.sin(self.theta)
        
        v_cmd = ux * c_th + uy * s_th
        w_cmd = (1.0 / self.b) * (-ux * s_th + uy * c_th)

        # 4. Safety Saturation
        v_cmd = np.clip(v_cmd, -0.22, 0.22)
        w_cmd = np.clip(w_cmd, -2.0, 2.0)

        # 5. Publish
        twist = Twist()
        twist.linear.x = float(v_cmd)
        twist.angular.z = float(w_cmd)
        self.pub_cmd.publish(twist)

    def stop_robot(self):
        twist = Twist()
        self.pub_cmd.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = IOLinearizationPointB()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_robot()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

