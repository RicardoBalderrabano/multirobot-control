'''
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
'''

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose2D
from rclpy.qos import qos_profile_sensor_data
import numpy as np
from functools import partial # Crucial for identifying which robot triggered the callback

class MultiOptitrackBridgeNode(Node):
    def __init__(self):
        super().__init__('multi_optitrack_bridge_node')
        
        # Define the robots you want to track
        # You can also turn this into a ROS parameter later
        self.robot_names = ['tb1', 'tb3'] 
        
        self.subs = {}
        self.pubs = {}

        for name in self.robot_names:
            # 1. Create a specific publisher for each robot
            # Result: /tb1/optitrack_pose2d, /tb2/optitrack_pose2d, etc.
            self.pubs[name] = self.create_publisher(
                Pose2D,
                f'/{name}/optitrack_pose2d',
                10
            )

            # 2. Create a subscription for each robot's OptiTrack Pose
            # Use partial to pass the 'name' string into the callback function
            input_topic = f'/{name}/pose'
            self.subs[name] = self.create_subscription(
                PoseStamped,
                input_topic,
                partial(self.pose_callback, robot_id=name),
                qos_profile=qos_profile_sensor_data
            )
            
            self.get_logger().info(f'Tracking {name} on {input_topic}')

    def quaternion_to_yaw(self, q):
        """Convert quaternion to yaw angle."""
        yaw = np.arctan2(2.0 * (q.w * q.z + q.x * q.y),
                         q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z)
        return yaw
    
    def pose_callback(self, msg, robot_id):
        """
        Shared callback for all robots.
        'robot_id' is provided by the partial function during subscription.
        """
        try:
            # Create Pose2D message
            pose2d = Pose2D()
            pose2d.x = msg.pose.position.x
            pose2d.y = msg.pose.position.y
            pose2d.theta = self.quaternion_to_yaw(msg.pose.orientation)
            
            # Publish to the specific robot's publisher
            self.pubs[robot_id].publish(pose2d)
            
            # Throttle logs so the terminal isn't overwhelmed by multiple robots
            self.get_logger().info(
                f'[{robot_id}] Ground Truth: x={pose2d.x:.2f}, y={pose2d.y:.2f}',
                throttle_duration_sec=5.0
            )
            
        except Exception as e:
            self.get_logger().error(f'Error tracking {robot_id}: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = MultiOptitrackBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()