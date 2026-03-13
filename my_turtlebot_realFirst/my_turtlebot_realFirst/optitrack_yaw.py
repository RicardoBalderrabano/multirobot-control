#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import math
import sys
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy

class OptiTrackYaw(Node):
    def __init__(self):
        super().__init__('optitrack_yaw')
        
        # Create a QoS profile that matches OptiTrack (typically Best Effort)
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,  # Often OptiTrack uses Best Effort
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Alternative: Use Sensorbata QoS (another common choice for OptiTrack)
        # from rclpy.qos import qos_profile_sensor_data
        # self.subscription = self.create_subscription(
        #     PoseStamped,
        #     '/tb1/pose',
        #     self.pose_callback,
        #     qos_profile_sensor_data)
        
        self.subscription = self.create_subscription(
            PoseStamped,
            '/tb1/pose',
            self.pose_callback,
            qos_profile)  # Use the custom QoS profile
        
        self.get_logger().info('OptiTrack yaw extractor started - listening to /tb1/pose')
        self.get_logger().info(f'Using QoS: {qos_profile.reliability}')

    def quaternion_to_yaw(self, x, y, z, w):
        """Convert quaternion to yaw angle in degrees"""
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw_rad = math.atan2(siny_cosp, cosy_cosp)
        return math.degrees(yaw_rad)

    def pose_callback(self, msg):
        q = msg.pose.orientation
        yaw = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)
        
        pos = msg.pose.position
        sys.stdout.write(f"\rOptiTrack Yaw: {yaw:6.2f}° | Position: ({pos.x:.3f}, {pos.y:.3f}) | Frame: {msg.header.frame_id}")
        sys.stdout.flush()

def main(args=None):
    rclpy.init(args=args)
    node = OptiTrackYaw()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nShutting down")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
