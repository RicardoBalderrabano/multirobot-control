import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import math
import csv
import numpy as np
import tf_transformations
import re
import time
from sklearn.cluster import DBSCAN
import message_filters

class MultiKalmanObserver(Node):
    def __init__(self):
        super().__init__('multi_kalman_observer')
        
        # --- 1. Fleet State ---
        self.trackers = {}   
        self.gt_registry = {} 
        self.csv_filename = 'fleet_tracking_results.csv'

        # --- 2. Filter Gains ---
        self.q_base = 0.001
        self.r_meas = 2.5
        self.gate_threshold = 0.4
        
        # --- 3. Initial Discovery with Debugging ---
        self.get_logger().info("--- STARTING ROBOT DISCOVERY ---")
        # Small wait to allow ROS graph to settle
        time.sleep(1.0) 
        self.discover_fleet_once()
        
        if not self.trackers:
            self.get_logger().warn("CRITICAL: No target robots found! Check if Transformer is running.")
        else:
            self.get_logger().info(f"SUCCESS: Tracking {list(self.trackers.keys())}")

        self.init_unified_csv()
        
        # --- 4. Observer Sync (TB1) ---
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        self.scan_sub = message_filters.Subscriber(self, LaserScan, '/tb1/scan', qos_profile=qos)
        self.obs_pose_sub = message_filters.Subscriber(self, PoseStamped, '/tb1/global_pose_estimated')
        
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.scan_sub, self.obs_pose_sub], 
            queue_size=10, slop=0.05
        )
        self.ts.registerCallback(self.sync_lidar_callback)

    def discover_fleet_once(self):
            topic_list = self.get_topic_names_and_types()
            self.get_logger().info(f"Scanning {len(topic_list)} total topics on network...")
            
            gt_pattern = r'^/(tb\d+)/pose$'
            kf_pattern = r'^/(tb\d+)/global_pose_estimated$'

            # Define the Best Effort QoS for OptiTrack/LiDAR
            qos_best_effort = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=1
            )

            for topic_name, _ in topic_list:
                # 1. Ground Truth Matching
                gt_match = re.match(gt_pattern, topic_name)
                if gt_match:
                    rid = gt_match.group(1)
                    self.get_logger().info(f"FOUND Ground Truth topic for: {rid}")
                    self.gt_registry[rid] = [0.0, 0.0]
                    # APPLY BEST_EFFORT HERE
                    self.create_subscription(PoseStamped, topic_name, 
                        lambda msg, r=rid: self.gt_callback(msg, r), qos_best_effort)

                # 2. Target Tracker Matching
                kf_match = re.match(kf_pattern, topic_name)
                if kf_match:
                    rid = kf_match.group(1)
                    if rid != 'tb1':
                        self.get_logger().info(f"MATCHED Target Robot: {rid}. Setting up Kalman Filter.")
                        self.setup_target_tracker(rid)

    def setup_target_tracker(self, rid):
        self.trackers[rid] = {
            'x': 0.0, 'y': 0.0, 'p': np.eye(2) * 0.1,
            'last_odom_x': None, 'last_odom_y': None,
            'initialized': False,
            'pub': self.create_publisher(PoseStamped, f'/{rid}/kalman', 10)
        }
        self.create_subscription(PoseStamped, f'/{rid}/global_pose_estimated', 
            lambda msg, r=rid: self.predict_step_cb(msg, r), 10)

    def gt_callback(self, msg, rid):
        self.gt_registry[rid] = [msg.pose.position.x, msg.pose.position.y]

    def predict_step_cb(self, msg, rid):
        t = self.trackers[rid]
        if t['last_odom_x'] is not None and t['initialized']:
            t['x'] += (msg.pose.position.x - t['last_odom_x'])
            t['y'] += (msg.pose.position.y - t['last_odom_y'])
            t['p'] += np.eye(2) * self.q_base
        elif not t['initialized']:
            t['x'], t['y'] = msg.pose.position.x, msg.pose.position.y
        t['last_odom_x'], t['last_odom_y'] = msg.pose.position.x, msg.pose.position.y

    def sync_lidar_callback(self, scan_msg, obs_pose_msg):
        obs_x, obs_y = obs_pose_msg.pose.position.x, obs_pose_msg.pose.position.y
        q = [obs_pose_msg.pose.orientation.x, obs_pose_msg.pose.orientation.y, 
             obs_pose_msg.pose.orientation.z, obs_pose_msg.pose.orientation.w]
        obs_yaw = tf_transformations.euler_from_quaternion(q)[2]

        clusters = self.get_all_clusters(scan_msg)
        
        # Debug: Print cluster count if there are robots tracked
        if self.trackers and clusters:
            # self.get_logger().info(f"DBSCAN found {len(clusters)} clusters.")
            pass

        for rid, t in self.trackers.items():
            best_gx, best_gy, min_d = None, None, self.gate_threshold
            for cx, cy in clusters:
                gx = obs_x + (cx * math.cos(obs_yaw) - cy * math.sin(obs_yaw))
                gy = obs_y + (cx * math.sin(obs_yaw) + cy * math.cos(obs_yaw))
                dist = math.sqrt((gx - t['x'])**2 + (gy - t['y'])**2)
                if dist < min_d:
                    min_d, best_gx, best_gy = dist, gx, gy

            if best_gx is not None:
                if not t['initialized']:
                    self.get_logger().info(f"Initialized Tracking for {rid}")
                    t['x'], t['y'], t['initialized'] = best_gx, best_gy, True
                else:
                    K = t['p'] @ np.linalg.inv(t['p'] + np.eye(2) * self.r_meas)
                    new_state = np.array([t['x'], t['y']]) + K @ (np.array([best_gx, best_gy]) - np.array([t['x'], t['y']]))
                    t['x'], t['y'] = new_state[0], new_state[1]
                    t['p'] = (np.eye(2) - K) @ t['p']
            
            t['pub'].publish(self.create_pose_msg(t['x'], t['y']))
        
        self.log_unified_data()

    def get_all_clusters(self, msg):
        points = [[d * math.cos(msg.angle_min + i * msg.angle_increment), 
                   d * math.sin(msg.angle_min + i * msg.angle_increment)] 
                  for i, d in enumerate(msg.ranges) if msg.range_min < d < msg.range_max]
        if len(points) < 5: return []
        db = DBSCAN(eps=0.15, min_samples=5).fit(np.array(points))
        centroids = []
        for label in set(db.labels_):
            if label == -1: continue
            c = np.mean(np.array(points)[db.labels_ == label], axis=0)
            centroids.append(c - (c / np.linalg.norm(c)) * 0.04)
        return centroids

    def init_unified_csv(self):
        header = ['timestamp', 'tb1_gt_x', 'tb1_gt_y']
        # Use sorted keys for consistent column order
        for rid in sorted(self.trackers.keys()):
            header += [f'{rid}_kf_x', f'{rid}_kf_y', f'{rid}_gt_x', f'{rid}_gt_y']
        with open(self.csv_filename, 'w', newline='') as f:
            csv.writer(f).writerow(header)

    def log_unified_data(self):
        # Only log if at least one robot is initialized
        if not any(t['initialized'] for t in self.trackers.values()) and not self.gt_registry:
            return

        row = [self.get_clock().now().nanoseconds / 1e9]
        row += self.gt_registry.get('tb1', [0.0, 0.0])
        for rid in sorted(self.trackers.keys()):
            t = self.trackers[rid]
            gt = self.gt_registry.get(rid, [0.0, 0.0])
            row += [t['x'], t['y'], gt[0], gt[1]]
        with open(self.csv_filename, 'a', newline='') as f:
            csv.writer(f).writerow(row)

    def create_pose_msg(self, x, y):
        m = PoseStamped()
        m.header.frame_id, m.header.stamp = "world", self.get_clock().now().to_msg()
        m.pose.position.x, m.pose.position.y = x, y
        return m

def main():
    rclpy.init()
    node = MultiKalmanObserver()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__': main()