#!/usr/bin/env python3
"""
Input/Output Linearization Controller for Real TurtleBot3 with Robot Center Goals
Modified to work like simulation version (no trajectory generator needed)
"""

import rclpy
from rclpy.node import Node
import numpy as np

from geometry_msgs.msg import TwistStamped, Point
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion


class IOLinearizationControllerRobotCenter(Node):
    def __init__(self):
        super().__init__('io_linearization_controller_robot_center')
        
        # Get parameters - REAL ROBOT SAFETY LIMITS
        self.declare_parameter('robot_namespace', 'burger1')
        self.declare_parameter('Kx', 2.0)
        self.declare_parameter('Ky', 2.0)
        self.declare_parameter('b', 0.1)
        self.declare_parameter('feedforward_speed', 0.08)  # Added for direct goals
        self.declare_parameter('goal_tolerance', 0.08)     # Added for direct goals
        self.declare_parameter('max_linear_vel', 0.15)
        self.declare_parameter('max_angular_vel', 0.8)
        
        self.robot_namespace = self.get_parameter('robot_namespace').value
        self.Kx = self.get_parameter('Kx').value
        self.Ky = self.get_parameter('Ky').value
        self.b = self.get_parameter('b').value
        self.feedforward_speed = self.get_parameter('feedforward_speed').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value
        
        # Current robot state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.xB = 0.0
        self.yB = 0.0
        
        # Goal state (ROBOT CENTER goals - NEW)
        self.x_goal = 0.0      # Desired robot center x
        self.y_goal = 0.0      # Desired robot center y
        self.xB_goal = 0.0     # Computed Point B goal
        self.yB_goal = 0.0     # Computed Point B goal
        
        # Desired velocities for Point B
        self.xB_dot_desired = 0.0
        self.yB_dot_desired = 0.0
        
        # Control state
        self.has_goal = False
        self.goal_reached = False
        self.odom_data_received = False
        
        # Publishers and Subscribers
        self.cmd_pub = self.create_publisher(
            TwistStamped,
            f'/{self.robot_namespace}/cmd_vel', 
            10
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            f'/{self.robot_namespace}/odom',
            self.odom_callback,
            10
        )
        
        # CHANGED: Subscribe to robot center goals instead of trajectory points
        self.goal_sub = self.create_subscription(
            Point,
            f'/{self.robot_namespace}/robot_goal',  # Changed topic
            self.goal_callback,
            10
        )
        
        # Control timer (20 Hz)
        self.control_timer = self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info(f'════════════════════════════════════════')
        self.get_logger().info(f'Real Robot I/O Linearization Controller (Robot Center Goals)')
        self.get_logger().info(f'Robot: {self.robot_namespace}')
        self.get_logger().info(f'Gains: Kx={self.Kx}, Ky={self.Ky}, b={self.b}')
        self.get_logger().info(f'Feedforward speed: {self.feedforward_speed} m/s')
        self.get_logger().info(f'Goal tolerance: {self.goal_tolerance} m')
        self.get_logger().info(f'Max velocities: linear={self.max_linear_vel}, angular={self.max_angular_vel}')
        self.get_logger().info(f'Waiting for goals on: /{self.robot_namespace}/robot_goal')
        self.get_logger().info(f'════════════════════════════════════════')
        
    def odom_callback(self, msg):
        """Get robot state from odometry"""
        # Get robot position
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        
        # Get orientation from quaternion
        orientation = msg.pose.pose.orientation
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
        
        try:
            roll, pitch, yaw = euler_from_quaternion(quaternion)
            self.theta = yaw
        except Exception as e:
            self.get_logger().error(f'Quaternion conversion error: {e}')
            return
        
        # Calculate point B position (offset from center)
        self.xB = self.x + self.b * np.cos(self.theta)
        self.yB = self.y + self.b * np.sin(self.theta)
        
        # Mark that we've received odometry data
        self.odom_data_received = True
        
        # Debug logging at reduced frequency
        self.get_logger().debug(
            f'Odom: Robot=({self.x:.3f}, {self.y:.3f}), θ={np.degrees(self.theta):.1f}°',
            throttle_duration_sec=2.0
        )
        
    def goal_callback(self, msg):
        """
        Callback for robot center goals
        msg.x, msg.y: Desired ROBOT CENTER position
        msg.z: Optional desired feedforward speed (overrides parameter)
        """
        if not self.odom_data_received:
            self.get_logger().warn('Cannot accept goal: No odometry data received yet!')
            return
            
        # Store robot center goal
        self.x_goal = msg.x
        self.y_goal = msg.y
        self.has_goal = True
        self.goal_reached = False
        
        # Optional: Use msg.z as feedforward speed if provided
        if abs(msg.z) > 0.001:
            self.feedforward_speed = msg.z
        
        # Calculate desired heading: point from current position to goal
        dx = self.x_goal - self.x
        dy = self.y_goal - self.y
        distance = np.sqrt(dx**2 + dy**2)
        
        if distance > 1e-6:
            desired_heading = np.arctan2(dy, dx)
        else:
            desired_heading = self.theta  # Keep current heading if at goal
        
        # Calculate Point B goal: b meters in front of robot center along desired heading
        self.xB_goal = self.x_goal + self.b * np.cos(desired_heading)
        self.yB_goal = self.y_goal + self.b * np.sin(desired_heading)
        
        self.get_logger().info(f'════════════════════════════════════════')
        self.get_logger().info(f'🎯 NEW ROBOT CENTER GOAL RECEIVED')
        self.get_logger().info(f'Robot Goal: ({self.x_goal:.3f}, {self.y_goal:.3f})')
        self.get_logger().info(f'Current Robot: ({self.x:.3f}, {self.y:.3f})')
        self.get_logger().info(f'Computed Point B Goal: ({self.xB_goal:.3f}, {self.yB_goal:.3f})')
        self.get_logger().info(f'Current Point B: ({self.xB:.3f}, {self.yB:.3f})')
        self.get_logger().info(f'Robot Distance: {distance:.3f} m')
        self.get_logger().info(f'Desired Heading: {np.degrees(desired_heading):.1f}°')
        self.get_logger().info(f'Current Heading: {np.degrees(self.theta):.1f}°')
        self.get_logger().info(f'════════════════════════════════════════')
        
    def compute_velocity_feedforward(self):
        """
        Compute desired velocity vector pointing from current Point B to goal Point B
        Returns: xB_dot_desired, yB_dot_desired
        """
        # Direction vector from current Point B to goal Point B
        dx = self.xB_goal - self.xB
        dy = self.yB_goal - self.yB
        distance = np.sqrt(dx**2 + dy**2)
        
        # If we're close to goal, no feedforward
        if distance < self.goal_tolerance * 2:
            return 0.0, 0.0
            
        # Normalize direction vector and scale by desired speed
        if distance > 1e-6:
            direction_x = dx / distance
            direction_y = dy / distance
            
            # Scale to desired feedforward speed
            xB_dot_desired = self.feedforward_speed * direction_x
            yB_dot_desired = self.feedforward_speed * direction_y
        else:
            xB_dot_desired = 0.0
            yB_dot_desired = 0.0
            
        return xB_dot_desired, yB_dot_desired
        
    def compute_control(self):
        """
        Implement the exact I/O linearization control law
        """
        if not self.has_goal or self.goal_reached:
            return 0.0, 0.0, 0.0, 0.0
        
        # First, check if robot center has reached its goal
        dx_robot = self.x_goal - self.x
        dy_robot = self.y_goal - self.y
        robot_distance = np.sqrt(dx_robot**2 + dy_robot**2)
        
        if robot_distance < self.goal_tolerance:
            self.goal_reached = True
            self.get_logger().info(f'✨ ROBOT GOAL REACHED! ✨')
            self.get_logger().info(f'Final Position: ({self.x:.3f}, {self.y:.3f})')
            self.get_logger().info(f'Goal Position: ({self.x_goal:.3f}, {self.y_goal:.3f})')
            return 0.0, 0.0, robot_distance, 0.0
        
        # Calculate Point B errors
        e_x = self.xB_goal - self.xB
        e_y = self.yB_goal - self.yB
        pointB_distance = np.sqrt(e_x**2 + e_y**2)
        
        # Compute velocity feedforward (provides implicit heading control)
        self.xB_dot_desired, self.yB_dot_desired = self.compute_velocity_feedforward()
        
        # Desired output dynamics (linear system)
        u_x = self.xB_dot_desired + self.Kx * e_x
        u_y = self.yB_dot_desired + self.Ky * e_y
        
        # Compute decoupling matrix inverse
        det = self.b  # determinant = b·cos²(θ) + b·sin²(θ) = b
        
        # Apply control law: u = G(θ)⁻¹ · [u_x, u_y]ᵀ
        v = (self.b * np.cos(self.theta) * u_x + self.b * np.sin(self.theta) * u_y) / det
        omega = (-np.sin(self.theta) * u_x + np.cos(self.theta) * u_y) / det
        
        # Apply saturation limits for safety
        v = np.clip(v, -self.max_linear_vel, self.max_linear_vel)
        omega = np.clip(omega, -self.max_angular_vel, self.max_angular_vel)
        
        return v, omega, robot_distance, pointB_distance

    def control_loop(self):
        """Main control loop"""
        if not self.has_goal or self.goal_reached:
            # Send stop command and return
            cmd_msg = TwistStamped()
            cmd_msg.header.stamp = self.get_clock().now().to_msg()
            cmd_msg.header.frame_id = 'base_link'
            cmd_msg.twist.linear.x = 0.0
            cmd_msg.twist.angular.z = 0.0
            self.cmd_pub.publish(cmd_msg)
            return
        
        # Check if we have valid odometry data
        if not self.odom_data_received:
            self.get_logger().warn('Skipping control cycle: No odometry data')
            return
            
        try:
            v, omega, robot_distance, pointB_distance = self.compute_control()
            
            # Publish control commands (TwistStamped for consistency)
            cmd_msg = TwistStamped()
            cmd_msg.header.stamp = self.get_clock().now().to_msg()
            cmd_msg.header.frame_id = 'base_link'
            cmd_msg.twist.linear.x = float(v)
            cmd_msg.twist.angular.z = float(omega)
            self.cmd_pub.publish(cmd_msg)
            
            # Logging at reduced frequency
            self.get_logger().info(
                f'[Control] v={v:.3f}, ω={omega:.3f} | '
                f'Robot Dist: {robot_distance:.3f}m | '
                f'PointB Dist: {pointB_distance:.3f}m | '
                f'Robot: ({self.x:.2f}, {self.y:.2f}) | '
                f'Goal: ({self.x_goal:.2f}, {self.y_goal:.2f})',
                throttle_duration_sec=1.0
            )
            
        except Exception as e:
            self.get_logger().error(f'Control error: {str(e)}')
            # Send stop command on error
            cmd_msg = TwistStamped()
            cmd_msg.header.stamp = self.get_clock().now().to_msg()
            cmd_msg.header.frame_id = 'base_link'
            cmd_msg.twist.linear.x = 0.0
            cmd_msg.twist.angular.z = 0.0
            self.cmd_pub.publish(cmd_msg)

def main(args=None):
    rclpy.init(args=args)
    controller = IOLinearizationControllerRobotCenter()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Controller shutdown by user')
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()