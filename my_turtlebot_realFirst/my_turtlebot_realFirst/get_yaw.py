#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math
import sys

class YawExtractor(Node):
    def __init__(self):
        super().__init__('yaw_extractor')
        self.subscription = self.create_subscription(
            Odometry,
            '/tb1/odometry/filtered',
            self.odom_callback,
            10)
        self.get_logger().info('Yaw extractor started - listening to /tb1/odometry/filtered')

    def odom_callback(self, msg):
        # Extract quaternion
        q = msg.pose.pose.orientation
        
        # Convert quaternion to yaw (rotation around Z axis)
        # Formula: yaw = atan2(2*(w*z + x*y), 1-2*(y*y + z*z))
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        # Convert to degrees
        yaw_deg = math.degrees(yaw)
        
        # Clear line and print yaw
        sys.stdout.write(f"\rYaw: {yaw_deg:6.2f} degrees | Radians: {yaw:6.3f} | Position: ({msg.pose.pose.position.x:.3f}, {msg.pose.pose.position.y:.3f})")
        sys.stdout.flush()

def main(args=None):
    rclpy.init(args=args)
    yaw_extractor = YawExtractor()
    
    try:
        rclpy.spin(yaw_extractor)
    except KeyboardInterrupt:
        print("\nShutting down yaw extractor")
    finally:
        yaw_extractor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

