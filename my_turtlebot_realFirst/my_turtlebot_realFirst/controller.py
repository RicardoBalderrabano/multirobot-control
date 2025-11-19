

#!/usr/bin/env python3
"""
Input/Output Linearization Controller for Real TurtleBot3

"""

import rclpy
from rclpy.node import Node
import numpy as np

from geometry_msgs.msg import TwistStamped, Point
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion


class IOLinearizationController(Node):
    def __init__(self):
        super().__init__('io_linearization_controller')
        
        # Get parameters - REAL ROBOT SAFETY LIMITS
        self.declare_parameter('robot_namespace', 'burger1')
        self.declare_parameter('Kx', 2.0)
        self.declare_parameter('Ky', 2.0)
        self.declare_parameter('b', 0.1)
        self.declare_parameter('max_linear_vel', 0.15)
        self.declare_parameter('max_angular_vel', 0.8)
        
        self.robot_namespace = self.get_parameter('robot_namespace').value
        self.Kx = self.get_parameter('Kx').value
        self.Ky = self.get_parameter('Ky').value
        self.b = self.get_parameter('b').value
        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value
        
        # Current state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.xB = 0.0
        self.yB = 0.0
        
        # Desired state
        self.xB_desired = 0.0
        self.yB_desired = 0.0
        self.xB_dot_desired = 0.0
        self.yB_dot_desired = 0.0
        
        # Control state
        self.control_enabled = False
        self.emergency_stop = False
        self.trajectory_completed = False
        
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
        
        self.traj_sub = self.create_subscription(
            Point,
            f'/{self.robot_namespace}/pointB_desired_state',
            self.traj_callback,
            10
        )
        
        # Control timer (20 Hz)
        self.control_timer = self.create_timer(0.05, self.control_loop)
        
        # Safety timer - less aggressive
        self.last_trajectory_time = self.get_clock().now()
        self.safety_timer = self.create_timer(0.5, self.safety_check)
        
        self.get_logger().info(f'Real TurtleBot3 I/O Linearization Controller started for {self.robot_namespace}')
        self.get_logger().info(f'Gains: Kx={self.Kx}, Ky={self.Ky}, b={self.b}')
        self.get_logger().info(f'Safety limits: v_max={self.max_linear_vel}, ω_max={self.max_angular_vel}')
        
    def odom_callback(self, msg):
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
        
        # Calculate point B position
        self.xB = self.x + self.b * np.cos(self.theta)
        self.yB = self.y + self.b * np.sin(self.theta)
        
        # Debug at lower frequency
        current_time = self.get_clock().now()
        if (current_time - self.last_trajectory_time).nanoseconds * 1e-9 < 15.0:
            self.get_logger().info(
                f'[{self.robot_namespace}] Pose: ({self.x:.2f}, {self.y:.2f}) | '
                f'Theta: {np.degrees(self.theta):.1f}° | '
                f'Point B: ({self.xB:.2f}, {self.yB:.2f})',
                throttle_duration_sec=5.0
            )
        
    def traj_callback(self, msg):
        self.xB_desired = msg.x
        self.yB_desired = msg.y
        self.xB_dot_desired = msg.z
        self.yB_dot_desired = 0.0
        
        # Detect trajectory completion
        distance_to_goal = np.sqrt((msg.x - 1.0)**2 + (msg.y - 0.0)**2)
        if distance_to_goal < 0.01 and abs(msg.z) < 0.001:
            if not self.trajectory_completed:
                self.get_logger().info(f'[{self.robot_namespace}] Trajectory completion detected')
                self.trajectory_completed = True
        else:
            self.trajectory_completed = False
        
        self.last_trajectory_time = self.get_clock().now()
        self.control_enabled = True

    def safety_check(self):
        """Less aggressive safety check"""
        time_since_last_traj = (self.get_clock().now() - self.last_trajectory_time).nanoseconds * 1e-9
        
        # Only stop if trajectory was active and no commands for longer time
        if (time_since_last_traj > 5.0 and self.control_enabled and 
            not self.trajectory_completed):  # Increased to 5 seconds
            self.get_logger().warn(f'[{self.robot_namespace}] No trajectory received for 5s - stopping robot')
            self.emergency_stop_robot()
            self.control_enabled = False

    def compute_control(self):
        """I/O Linearization control law"""
        if self.emergency_stop:
            return 0.0, 0.0

        # Calculate errors
        e_x = self.xB_desired - self.xB
        e_y = self.yB_desired - self.yB
        
        # Check if target is reached
        distance = np.sqrt(e_x**2 + e_y**2)
        if distance < 0.05:  # 5cm threshold
            return 0.0, 0.0
        
        # Desired output dynamics
        u_x = self.xB_dot_desired + self.Kx * e_x
        u_y = self.yB_dot_desired + self.Ky * e_y
        
        # Compute control inputs
        det = self.b
        v = (self.b * np.cos(self.theta) * u_x + self.b * np.sin(self.theta) * u_y) / det
        omega = (-np.sin(self.theta) * u_x + np.cos(self.theta) * u_y) / det
        
        # Apply saturation limits
        v = np.clip(v, -self.max_linear_vel, self.max_linear_vel)
        omega = np.clip(omega, -self.max_angular_vel, self.max_angular_vel)
        
        return v, omega

    def control_loop(self):
        if not self.control_enabled or self.emergency_stop:
            return
            
        try:
            v, omega = self.compute_control()
            
            # DEBUG: Check if commands are being sent
            if abs(v) > 0.01 or abs(omega) > 0.01:
                self.get_logger().info(
                    f'[{self.robot_namespace}] SENDING COMMANDS: v={v:.3f}, ω={omega:.3f}'
                )

            # Publish to robot
            cmd_msg = TwistStamped()
            cmd_msg.header.stamp = self.get_clock().now().to_msg()
            cmd_msg.header.frame_id = 'base_link'
            cmd_msg.twist.linear.x = float(v)
            cmd_msg.twist.angular.z = float(omega)
            
            try:
                self.cmd_pub.publish(cmd_msg)
            except Exception as e:
                self.get_logger().debug(f'Command publish failed: {e}')
            
            # Reduced frequency logging
            self.get_logger().info(
                f'[{self.robot_namespace}] Control: v={v:.3f}, ω={omega:.3f} | '
                f'Point B: ({self.xB:.2f}, {self.yB:.2f}) | '
                f'Target B: ({self.xB_desired:.2f}, {self.yB_desired:.2f})',
                throttle_duration_sec=2.0
            )
            
        except Exception as e:
            self.get_logger().error(f'[{self.robot_namespace}] Control error: {str(e)}')
            self.emergency_stop_robot()

    def emergency_stop_robot(self):
        """Safe emergency stop"""
        try:
            if rclpy.ok():  # Check if ROS2 is still running
                cmd_msg = TwistStamped()
                cmd_msg.header.stamp = self.get_clock().now().to_msg()
                cmd_msg.header.frame_id = 'base_link'
                cmd_msg.twist.linear.x = 0.0
                cmd_msg.twist.angular.z = 0.0
                self.cmd_pub.publish(cmd_msg)
                self.get_logger().warn(f'[{self.robot_namespace}] Emergency stop activated')
        except Exception as e:
            self.get_logger().debug(f'Emergency stop failed: {e}')
        finally:
            self.emergency_stop = True

    def destroy_node(self):
        """Safe shutdown"""
        self.emergency_stop_robot()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    controller = IOLinearizationController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info(f'[{controller.robot_namespace}] Controller shutdown by user')
    except Exception as e:
        controller.get_logger().error(f'Controller error: {e}')
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()