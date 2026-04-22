#!/usr/bin/env python3
"""
Mock EKF Observer for Simulation
Reads Gazebo ground truth (Odometry/Pose) and publishes it as a local EKF tracker.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import re

class MockKalmanObserver(Node):
    def __init__(self):
        super().__init__('mock_kalman_observer')
        
        self.declare_parameter('observer_namespace', 'tb1')
        self.obs_ns = self.get_parameter('observer_namespace').value
        
        self.trackers = {} 
        
        self.get_logger().info(f"--- STARTING MOCK EKF OBSERVER ({self.obs_ns.upper()}) ---")
        
        # CONTINUOUS DISCOVERY: Scan for new robots every 2 seconds
        self.discovery_timer = self.create_timer(2.0, self.discover_fleet)

    def discover_fleet(self):
        topic_list = self.get_topic_names_and_types()
        gt_pattern = r'^/(tb\d+)/ground_truth_pose$'

        for topic_name, topic_types in topic_list:
            match = re.match(gt_pattern, topic_name)
            if match:
                rid = match.group(1) 
                
                # If it's a neighbor we haven't seen before, track it!
                if rid != self.obs_ns and rid not in self.trackers:
                    msg_type = Odometry if 'nav_msgs/msg/Odometry' in topic_types else PoseStamped
                    self.setup_mock_tracker(rid, topic_name, msg_type)
                    self.get_logger().info(f"✅ MOCK EKF ({self.obs_ns}): Began tracking {rid}")

    def setup_mock_tracker(self, rid, topic_name, msg_type):
        pub_topic = f'/{self.obs_ns}/tracked_{rid}'
        pub = self.create_publisher(PoseStamped, pub_topic, 10)
        self.trackers[rid] = pub
        
        self.create_subscription(
            msg_type,
            topic_name,
            lambda msg, r=rid: self.gt_to_tracked_cb(msg, r),
            10
        )

    def gt_to_tracked_cb(self, msg, rid):
        tracked_msg = PoseStamped()
        tracked_msg.header.stamp = self.get_clock().now().to_msg()
        tracked_msg.header.frame_id = "world"
        
        if hasattr(msg, 'pose') and hasattr(msg.pose, 'pose'):
            tracked_msg.pose = msg.pose.pose
        else:
            tracked_msg.pose = msg.pose
            
        self.trackers[rid].publish(tracked_msg)

def main():
    rclpy.init()
    node = MockKalmanObserver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()