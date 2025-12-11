#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose2D
from tf_transformations import euler_from_quaternion
from rclpy.qos import qos_profile_sensor_data
import numpy as np


class OptitrackBridgeNode(Node):
    def __init__(self):
        super().__init__('optitrack_bridge_node')
        
        # Parameters
        self.declare_parameter('robot_namespace', 'burger1')
        self.declare_parameter('input_topic', '/tb1/pose')
        
        self.robot_namespace = self.get_parameter('robot_namespace').value
        input_topic = self.get_parameter('input_topic').value
        
        # Use sensor data QoS profile
        self.subscription = self.create_subscription(
            PoseStamped,
            input_topic,
            self.pose_callback,
            qos_profile=qos_profile_sensor_data
        )
        
        self.publisher = self.create_publisher(
            Pose2D,
            f'/{self.robot_namespace}/optitrack_pose2d',
            10
        )
        
        self.get_logger().info(f'OptiTrack Bridge Node started for {self.robot_namespace}')
        self.get_logger().info(f'Subscribed to: {input_topic}')
    
    def quaternion_to_yaw(self, x, y, z, w):
        """
        Convert quaternion to yaw angle (rotation around Z axis)
        Returns yaw in radians in the range [-pi, pi]
        """
        # Method 1: Using arctan2 (most reliable)
        yaw = np.arctan2(2.0 * (w * z + x * y),
                         w * w + x * x - y * y - z * z)
        
        # Alternative: Using tf_transformations
        # (roll, pitch, yaw) = euler_from_quaternion([x, y, z, w])
        # return yaw
        
        return yaw
    
    def pose_callback(self, msg):
        try:
            # Extract position (direct copy)
            x = msg.pose.position.x
            y = msg.pose.position.y
            
            # Log original OptiTrack data for debugging
            self.get_logger().info(f'OptiTrack raw: x={x:.3f}, y={y:.3f}', 
                                   throttle_duration_sec=2.0)
            
            # Convert quaternion to yaw using our function
            q = msg.pose.orientation
            yaw = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)
            
            # Alternative: Using tf_transformations
            # (roll, pitch, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])
            
            # Create Pose2D message
            pose2d = Pose2D()
            pose2d.x = x
            pose2d.y = y
            pose2d.theta = yaw
            
            # Publish
            self.publisher.publish(pose2d)
            
            # Log the result
            self.get_logger().info(
                f'Published: x={x:.3f}, y={y:.3f}, θ={yaw:.3f} rad ({np.degrees(yaw):.1f}°)',
                throttle_duration_sec=2.0
            )
            
        except Exception as e:
            self.get_logger().error(f'Error in callback: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = OptitrackBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()