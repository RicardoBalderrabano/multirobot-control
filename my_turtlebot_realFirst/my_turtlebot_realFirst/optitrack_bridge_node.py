"""
# BRIDGE WITHOUT TIME STAMP
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
        #Convert quaternion to yaw angle
        yaw = np.arctan2(2.0 * (q.w * q.z + q.x * q.y),
                         q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z)
        return yaw
    
    def pose_callback(self, msg, robot_id):
        
        Shared callback for all robots.
        'robot_id' is provided by the partial function during subscription.
        
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
"""

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
# Changed from Pose2D to PoseStamped to keep the header/timestamp
from geometry_msgs.msg import PoseStamped, Pose2D 
from rclpy.qos import qos_profile_sensor_data
import numpy as np
from functools import partial

class MultiOptitrackBridgeNode(Node):
    def __init__(self):
        super().__init__('multi_optitrack_bridge_node')
        
        self.robot_names = ['tb1', 'tb3'] 
        self.subs = {}
        self.pubs = {}

        for name in self.robot_names:
            # MODIFICATION: Publishing PoseStamped to retain the timestamp header
            self.pubs[name] = self.create_publisher(
                PoseStamped,
                f'/{name}/optitrack_pose_stamped',
                10
            )

            input_topic = f'/{name}/pose'
            self.subs[name] = self.create_subscription(
                PoseStamped,
                input_topic,
                partial(self.pose_callback, robot_id=name),
                qos_profile=qos_profile_sensor_data
            )
            
            self.get_logger().info(f'Tracking {name} with Timestamp preservation on {input_topic}')

    def pose_callback(self, msg, robot_id):
        """
        Callback that preserves the original OptiTrack timestamp.
        """
        try:
            # We create a NEW PoseStamped message
            out_msg = PoseStamped()
            
            # CRITICAL MODIFICATION: Copy the original header (timestamp + frame_id)
            # This ensures your CSV logger gets the OptiTrack time, not the Laptop time.
            out_msg.header = msg.header 
            
            # Copy position data
            out_msg.pose.position.x = msg.pose.position.x
            out_msg.pose.position.y = msg.pose.position.y
            out_msg.pose.position.z = msg.pose.position.z
            
            # Copy orientation data
            out_msg.pose.orientation = msg.pose.orientation
            
            # Publish the stamped message
            self.pubs[robot_id].publish(out_msg)
            
            self.get_logger().info(
                f'[{robot_id}] Stamped Data: sec={out_msg.header.stamp.sec}',
                throttle_duration_sec=10.0
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