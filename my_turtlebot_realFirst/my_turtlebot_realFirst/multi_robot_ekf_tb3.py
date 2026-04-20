"""
#LAST WORKING WITHOUT NAMEPACE AUTOMATION
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
        
        self.trackers = {}   
        self.gt_registry = {} 
        self.csv_filename = 'fleet_tracking_results.csv'

        # --- EKF Gains & Tuning ---
        # Q (Process Noise): 3x3 matrix for [x, y, theta]
        self.q_base = np.diag([0.2, 0.2, 0.1])        
        self.ema_alpha = 0.4       
        
        # --- ADAPTIVE EKF: Dynamic Measurement Noise Parameters ---
        self.r_meas_base = 0.15       # Clean line-of-sight noise
        self.r_meas_expansion = 0.4   # Penalty per missed frame
        self.r_meas_max = 2.0         # Hard cap for maximum measurement distrust
        
        # --- Dynamic Gating Parameters ---
        self.gate_threshold = 0.10  # Strict base gate: 10cm
        self.gate_expansion = 0.02  # Expand slowly: 2cm per missed frame
        self.max_gate = 0.15        # Hard cap: Reject ghost clusters jumping > 15cm
        
        self.get_logger().info("--- STARTING ROBOT DISCOVERY (ANNEALED ADAPTIVE EKF) ---")
        time.sleep(1.0) 
        self.discover_fleet_once()
        self.init_unified_csv()
        
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.scan_sub = message_filters.Subscriber(
            self, LaserScan, '/tb1/scan_filtered', qos_profile=qos
        )
        self.obs_pose_sub = message_filters.Subscriber(
            self, PoseStamped, '/tb1/global_pose_estimated'
        )
        
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.scan_sub, self.obs_pose_sub], 
            queue_size=10, slop=0.05
        )
        self.ts.registerCallback(self.sync_lidar_callback)

    # ===============================
    # 1. CIRCLE FIT FUNCTION
    # ===============================
    def fit_circle(self, points):
        x = points[:, 0]
        y = points[:, 1]

        A = np.c_[2*x, 2*y, np.ones(len(points))]
        b = x**2 + y**2

        try:
            c = np.linalg.lstsq(A, b, rcond=None)[0]
            cx, cy = c[0], c[1]
            return np.array([cx, cy])
        except:
            return None

    # ===============================
    # 2. DISCOVERY & SETUP
    # ===============================
    def discover_fleet_once(self):
        topic_list = self.get_topic_names_and_types()
        gt_pattern = r'^/(tb\d+)/pose$'
        kf_pattern = r'^/(tb\d+)/global_pose_estimated$'

        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        for topic_name, _ in topic_list:
            gt_match = re.match(gt_pattern, topic_name)
            if gt_match:
                rid = gt_match.group(1)
                self.gt_registry[rid] = [0.0, 0.0]
                self.create_subscription(
                    PoseStamped, topic_name,
                    lambda msg, r=rid: self.gt_callback(msg, r),
                    qos_best_effort
                )

            kf_match = re.match(kf_pattern, topic_name)
            if kf_match:
                rid = kf_match.group(1)
                if rid != 'tb1':
                    self.setup_target_tracker(rid)

        if not self.trackers:
            self.get_logger().warn("⚠️ CRITICAL: No target robots discovered!")
        else:
            self.get_logger().info(f"✅ DISCOVERY COMPLETE! Tracking: {list(self.trackers.keys())}")

    def setup_target_tracker(self, rid):
        self.trackers[rid] = {
            'x': 0.0, 'y': 0.0, 'theta': 0.0, 
            'p': np.eye(3) * 0.1, # 3x3 Covariance Matrix
            'last_odom_x': None, 'last_odom_y': None, 'last_odom_theta': None,
            'initialized': False,
            'ema_x': None, 'ema_y': None,
            'missed_frames': 0,
            'pub': self.create_publisher(PoseStamped, f'/{rid}/kalman', 10)
        }

        self.create_subscription(
            PoseStamped,
            f'/{rid}/global_pose_estimated',
            lambda msg, r=rid: self.predict_step_cb(msg, r),
            10
        )

    # ===============================
    # 3. CALLBACKS & EKF ESTIMATION
    # ===============================
    def gt_callback(self, msg, rid):
        self.gt_registry[rid] = [msg.pose.position.x, msg.pose.position.y]

    def predict_step_cb(self, msg, rid):
        t = self.trackers[rid]

        # Extract Theta from Quaternions
        q = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        try:
            _, _, new_theta = tf_transformations.euler_from_quaternion(q)
        except Exception:
            new_theta = 0.0

        if t['last_odom_x'] is not None and t['initialized']:
            # Calculate physical displacement (v * dt) and rotation (w * dt)
            dx = msg.pose.position.x - t['last_odom_x']
            dy = msg.pose.position.y - t['last_odom_y']
            
            # Directional displacement
            ds = math.hypot(dx, dy)
            # Reverse direction if moving backward (dot product check)
            if (dx * math.cos(t['last_odom_theta']) + dy * math.sin(t['last_odom_theta'])) < 0:
                ds = -ds
                
            raw_dtheta = new_theta - t['last_odom_theta']
            
            # Safely wrap the angle delta to [-pi, pi]
            dtheta = math.atan2(math.sin(raw_dtheta), math.cos(raw_dtheta))

            # --- 1. EKF Non-Linear State Prediction ---
            t['x'] += ds * math.cos(t['theta'])
            t['y'] += ds * math.sin(t['theta'])
            t['theta'] += dtheta
            t['theta'] = math.atan2(math.sin(t['theta']), math.cos(t['theta'])) # Normalize

            # --- 2. Calculate Jacobian Matrix (F) ---
            F = np.eye(3)
            F[0, 2] = -ds * math.sin(t['theta'])
            F[1, 2] =  ds * math.cos(t['theta'])

            # --- 3. Covariance Prediction ---
            t['p'] = F @ t['p'] @ F.T + self.q_base

        elif not t['initialized']:
            t['x'], t['y'], t['theta'] = msg.pose.position.x, msg.pose.position.y, new_theta

        # Store for next iteration
        t['last_odom_x'] = msg.pose.position.x
        t['last_odom_y'] = msg.pose.position.y
        t['last_odom_theta'] = new_theta

    def sync_lidar_callback(self, scan_msg, obs_pose_msg):
            obs_x = obs_pose_msg.pose.position.x
            obs_y = obs_pose_msg.pose.position.y

            q = [obs_pose_msg.pose.orientation.x, obs_pose_msg.pose.orientation.y, obs_pose_msg.pose.orientation.z, obs_pose_msg.pose.orientation.w]
            obs_yaw = tf_transformations.euler_from_quaternion(q)[2]
            clusters = self.get_all_clusters(scan_msg)

            # 1. Generate Matches
            possible_matches = []
            for rid, t in self.trackers.items():
                current_gate = min(self.max_gate, self.gate_threshold + (t['missed_frames'] * self.gate_expansion))
                for idx, (cx, cy) in enumerate(clusters):
                    gx = obs_x + (cx * math.cos(obs_yaw) - cy * math.sin(obs_yaw))
                    gy = obs_y + (cx * math.sin(obs_yaw) + cy * math.cos(obs_yaw))
                    dist = math.hypot(gx - t['x'], gy - t['y'])
                    if dist < current_gate:
                        possible_matches.append({'rid': rid, 'cluster_idx': idx, 'dist': dist, 'gx': gx, 'gy': gy})

            # 2. Sort Matches
            possible_matches.sort(key=lambda item: item['dist'])

            # 3. Assign Clusters and UPDATE EKF
            used_clusters = set()
            updated_robots = set()

            for match in possible_matches:
                rid = match['rid']
                idx = match['cluster_idx']

                if idx in used_clusters or rid in updated_robots:
                    continue

                used_clusters.add(idx)
                updated_robots.add(rid)
                
                t = self.trackers[rid]
                best_gx, best_gy = match['gx'], match['gy']
                
                if not t['initialized']:
                    t['x'], t['y'] = best_gx, best_gy
                    # Initialize with the true odometry heading, not 0.0
                    t['theta'] = t['last_odom_theta'] if t['last_odom_theta'] is not None else 0.0
                    t['initialized'] = True
                    t['ema_x'], t['ema_y'] = best_gx, best_gy
                else:
                    # --- THE STALE EMA FIX ---
                    if t['missed_frames'] > 0:
                        # Robot just emerged from occlusion. Bypass the stale EMA!
                        t['ema_x'] = best_gx
                        t['ema_y'] = best_gy
                    else:
                        # Normal EMA smoothing
                        t['ema_x'] = self.ema_alpha * best_gx + (1 - self.ema_alpha) * t['ema_x']
                        t['ema_y'] = self.ema_alpha * best_gy + (1 - self.ema_alpha) * t['ema_y']

                    # --- ADAPTIVE R CALCULATION ---
                    current_r_val = min(self.r_meas_max, self.r_meas_base + (t['missed_frames'] * self.r_meas_expansion))
                    R_adaptive = np.diag([current_r_val, current_r_val])

                    # --- EKF UPDATE STEP ---
                    # H Matrix maps 3D state [x, y, theta] to 2D measurement [x, y]
                    H = np.array([[1.0, 0.0, 0.0],
                                  [0.0, 1.0, 0.0]])
                    
                    # Innovation (Measurement Residual)
                    z = np.array([t['ema_x'], t['ema_y']])
                    h_x = np.array([t['x'], t['y']])
                    y_res = z - h_x
                    
                    # Innovation Covariance (S) using Adaptive R Matrix
                    S = H @ t['p'] @ H.T + R_adaptive
                    
                    # Kalman Gain (K) -> 3x2 Matrix
                    K = t['p'] @ H.T @ np.linalg.inv(S)
                    
                    # Update State
                    state_update = K @ y_res
                    t['x'] += state_update[0]
                    t['y'] += state_update[1]
                    t['theta'] += state_update[2]
                    t['theta'] = math.atan2(math.sin(t['theta']), math.cos(t['theta']))
                    
                    # Update Covariance
                    I = np.eye(3)
                    t['p'] = (I - K @ H) @ t['p']

                # --- [OPTION 1 FIX]: Gradual R Recovery (Annealing) ---
                # Decrease the penalty gracefully instead of instantly dropping to 0
                t['missed_frames'] = max(0, t['missed_frames'] - 2)

            # 4. Handle Occlusions & Publish
            for rid, t in self.trackers.items():
                if rid not in updated_robots and t['initialized']:
                    t['missed_frames'] += 1
                
                t['pub'].publish(self.create_pose_msg(t['x'], t['y']))

            self.log_unified_data()

    # ===============================
    # 4. CLUSTER EXTRACTION & LOGGING
    # ===============================
    def get_all_clusters(self, msg):
        points = []
        for i, d in enumerate(msg.ranges):
            if np.isfinite(d) and msg.range_min < d < msg.range_max:
                angle = msg.angle_min + i * msg.angle_increment
                points.append([d * math.cos(angle), d * math.sin(angle)])

        if len(points) < 5: return []
        points = np.array(points)
        db = DBSCAN(eps=0.12, min_samples=3).fit(points)

        valid_labels = set(db.labels_) - {-1}
        centroids = []

        for label in valid_labels:
            cluster_points = points[db.labels_ == label]
            if 4 <= len(cluster_points) <= 25:
                center = self.fit_circle(cluster_points)
                if center is not None: centroids.append(center)

        return centroids

    def init_unified_csv(self):
        header = ['timestamp', 'tb1_gt_x', 'tb1_gt_y']
        for rid in sorted(self.trackers.keys()): header += [f'{rid}_kf_x', f'{rid}_kf_y', f'{rid}_gt_x', f'{rid}_gt_y']
        with open(self.csv_filename, 'w', newline='') as f: csv.writer(f).writerow(header)

    def log_unified_data(self):
        row = [self.get_clock().now().nanoseconds / 1e9] + self.gt_registry.get('tb1', [0.0, 0.0])
        for rid in sorted(self.trackers.keys()):
            t = self.trackers[rid]
            gt = self.gt_registry.get(rid, [0.0, 0.0])
            row += [t['x'], t['y'], gt[0], gt[1]]
        with open(self.csv_filename, 'a', newline='') as f: csv.writer(f).writerow(row)

    def create_pose_msg(self, x, y):
        m = PoseStamped()
        m.header.frame_id = "world"
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose.position.x = x
        m.pose.position.y = y
        return m

def main():
    rclpy.init()
    node = MultiKalmanObserver()
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
        
        # --- Namespace Parameterization ---
        self.declare_parameter('observer_namespace', 'tb3')
        self.obs_ns = self.get_parameter('observer_namespace').value
        
        self.trackers = {}   
        self.gt_registry = {} 
        
        # Dynamic CSV logging to prevent file overwrites when running multiple nodes locally
        self.csv_filename = f'{self.obs_ns}_fleet_tracking_results.csv'

        # --- EKF Gains & Tuning ---
        self.q_base = np.diag([0.2, 0.2, 0.1])        
        self.ema_alpha = 0.4       
        
        # --- ADAPTIVE EKF: Dynamic Measurement Noise Parameters ---
        self.r_meas_base = 0.15       
        self.r_meas_expansion = 0.4   
        self.r_meas_max = 2.0         
        
        # --- Dynamic Gating Parameters ---
        self.gate_threshold = 0.10  
        self.gate_expansion = 0.02  
        self.max_gate = 0.15        
        
        self.get_logger().info(f"--- STARTING ROBOT DISCOVERY ({self.obs_ns.upper()}: ANNEALED ADAPTIVE EKF) ---")
        time.sleep(1.0) 
        self.discover_fleet_once()
        self.init_unified_csv()
        
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Dynamically subscribe to this specific robot's sensors
        self.scan_sub = message_filters.Subscriber(
            self, LaserScan, f'/{self.obs_ns}/scan_filtered', qos_profile=qos
        )
        self.obs_pose_sub = message_filters.Subscriber(
            self, PoseStamped, f'/{self.obs_ns}/global_pose_estimated'
        )
        
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.scan_sub, self.obs_pose_sub], 
            queue_size=10, slop=0.05
        )
        self.ts.registerCallback(self.sync_lidar_callback)

    # ===============================
    # 1. CIRCLE FIT FUNCTION
    # ===============================
    def fit_circle(self, points):
        x = points[:, 0]
        y = points[:, 1]

        A = np.c_[2*x, 2*y, np.ones(len(points))]
        b = x**2 + y**2

        try:
            c = np.linalg.lstsq(A, b, rcond=None)[0]
            cx, cy = c[0], c[1]
            return np.array([cx, cy])
        except:
            return None

    # ===============================
    # 2. DISCOVERY & SETUP
    # ===============================
    def discover_fleet_once(self):
        topic_list = self.get_topic_names_and_types()
        gt_pattern = r'^/(tb\d+)/pose$'
        kf_pattern = r'^/(tb\d+)/global_pose_estimated$'

        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        for topic_name, _ in topic_list:
            gt_match = re.match(gt_pattern, topic_name)
            if gt_match:
                rid = gt_match.group(1)
                self.gt_registry[rid] = [0.0, 0.0]
                self.create_subscription(
                    PoseStamped, topic_name,
                    lambda msg, r=rid: self.gt_callback(msg, r),
                    qos_best_effort
                )

            kf_match = re.match(kf_pattern, topic_name)
            if kf_match:
                rid = kf_match.group(1)
                # Ignore self during discovery
                if rid != self.obs_ns:
                    self.setup_target_tracker(rid)

        if not self.trackers:
            self.get_logger().warn("⚠️ CRITICAL: No target robots discovered!")
        else:
            self.get_logger().info(f"✅ DISCOVERY COMPLETE! Tracking: {list(self.trackers.keys())}")

    def setup_target_tracker(self, rid):
        self.trackers[rid] = {
            'x': 0.0, 'y': 0.0, 'theta': 0.0, 
            'p': np.eye(3) * 0.1, 
            'last_odom_x': None, 'last_odom_y': None, 'last_odom_theta': None,
            'initialized': False,
            'ema_x': None, 'ema_y': None,
            'missed_frames': 0,
            # Strict Data Provenance: Publish using observer's namespace
            'pub': self.create_publisher(PoseStamped, f'/{self.obs_ns}/tracked_{rid}', 10)
        }

        self.create_subscription(
            PoseStamped,
            f'/{rid}/global_pose_estimated',
            lambda msg, r=rid: self.predict_step_cb(msg, r),
            10
        )

    # ===============================
    # 3. CALLBACKS & EKF ESTIMATION
    # ===============================
    def gt_callback(self, msg, rid):
        self.gt_registry[rid] = [msg.pose.position.x, msg.pose.position.y]

    def predict_step_cb(self, msg, rid):
        t = self.trackers[rid]

        # Extract Theta from Quaternions
        q = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        try:
            _, _, new_theta = tf_transformations.euler_from_quaternion(q)
        except Exception:
            new_theta = 0.0

        if t['last_odom_x'] is not None and t['initialized']:
            dx = msg.pose.position.x - t['last_odom_x']
            dy = msg.pose.position.y - t['last_odom_y']
            
            ds = math.hypot(dx, dy)
            if (dx * math.cos(t['last_odom_theta']) + dy * math.sin(t['last_odom_theta'])) < 0:
                ds = -ds
                
            raw_dtheta = new_theta - t['last_odom_theta']
            dtheta = math.atan2(math.sin(raw_dtheta), math.cos(raw_dtheta))

            t['x'] += ds * math.cos(t['theta'])
            t['y'] += ds * math.sin(t['theta'])
            t['theta'] += dtheta
            t['theta'] = math.atan2(math.sin(t['theta']), math.cos(t['theta'])) 

            F = np.eye(3)
            F[0, 2] = -ds * math.sin(t['theta'])
            F[1, 2] =  ds * math.cos(t['theta'])

            t['p'] = F @ t['p'] @ F.T + self.q_base

        elif not t['initialized']:
            t['x'], t['y'], t['theta'] = msg.pose.position.x, msg.pose.position.y, new_theta

        t['last_odom_x'] = msg.pose.position.x
        t['last_odom_y'] = msg.pose.position.y
        t['last_odom_theta'] = new_theta

    def sync_lidar_callback(self, scan_msg, obs_pose_msg):
            obs_x = obs_pose_msg.pose.position.x
            obs_y = obs_pose_msg.pose.position.y

            q = [obs_pose_msg.pose.orientation.x, obs_pose_msg.pose.orientation.y, obs_pose_msg.pose.orientation.z, obs_pose_msg.pose.orientation.w]
            obs_yaw = tf_transformations.euler_from_quaternion(q)[2]
            clusters = self.get_all_clusters(scan_msg)

            possible_matches = []
            for rid, t in self.trackers.items():
                current_gate = min(self.max_gate, self.gate_threshold + (t['missed_frames'] * self.gate_expansion))
                for idx, (cx, cy) in enumerate(clusters):
                    gx = obs_x + (cx * math.cos(obs_yaw) - cy * math.sin(obs_yaw))
                    gy = obs_y + (cx * math.sin(obs_yaw) + cy * math.cos(obs_yaw))
                    dist = math.hypot(gx - t['x'], gy - t['y'])
                    if dist < current_gate:
                        possible_matches.append({'rid': rid, 'cluster_idx': idx, 'dist': dist, 'gx': gx, 'gy': gy})

            possible_matches.sort(key=lambda item: item['dist'])

            used_clusters = set()
            updated_robots = set()

            for match in possible_matches:
                rid = match['rid']
                idx = match['cluster_idx']

                if idx in used_clusters or rid in updated_robots:
                    continue

                used_clusters.add(idx)
                updated_robots.add(rid)
                
                t = self.trackers[rid]
                best_gx, best_gy = match['gx'], match['gy']
                
                if not t['initialized']:
                    t['x'], t['y'] = best_gx, best_gy
                    t['theta'] = t['last_odom_theta'] if t['last_odom_theta'] is not None else 0.0
                    t['initialized'] = True
                    t['ema_x'], t['ema_y'] = best_gx, best_gy
                else:
                    if t['missed_frames'] > 0:
                        t['ema_x'] = best_gx
                        t['ema_y'] = best_gy
                    else:
                        t['ema_x'] = self.ema_alpha * best_gx + (1 - self.ema_alpha) * t['ema_x']
                        t['ema_y'] = self.ema_alpha * best_gy + (1 - self.ema_alpha) * t['ema_y']

                    current_r_val = min(self.r_meas_max, self.r_meas_base + (t['missed_frames'] * self.r_meas_expansion))
                    R_adaptive = np.diag([current_r_val, current_r_val])

                    H = np.array([[1.0, 0.0, 0.0],
                                  [0.0, 1.0, 0.0]])
                    
                    z = np.array([t['ema_x'], t['ema_y']])
                    h_x = np.array([t['x'], t['y']])
                    y_res = z - h_x
                    
                    S = H @ t['p'] @ H.T + R_adaptive
                    K = t['p'] @ H.T @ np.linalg.inv(S)
                    
                    state_update = K @ y_res
                    t['x'] += state_update[0]
                    t['y'] += state_update[1]
                    t['theta'] += state_update[2]
                    t['theta'] = math.atan2(math.sin(t['theta']), math.cos(t['theta']))
                    
                    I = np.eye(3)
                    t['p'] = (I - K @ H) @ t['p']

                t['missed_frames'] = max(0, t['missed_frames'] - 2)

            for rid, t in self.trackers.items():
                if rid not in updated_robots and t['initialized']:
                    t['missed_frames'] += 1
                
                t['pub'].publish(self.create_pose_msg(t['x'], t['y']))

            self.log_unified_data()

    # ===============================
    # 4. CLUSTER EXTRACTION & LOGGING
    # ===============================
    def get_all_clusters(self, msg):
        points = []
        for i, d in enumerate(msg.ranges):
            if np.isfinite(d) and msg.range_min < d < msg.range_max:
                angle = msg.angle_min + i * msg.angle_increment
                points.append([d * math.cos(angle), d * math.sin(angle)])

        if len(points) < 5: return []
        points = np.array(points)
        db = DBSCAN(eps=0.12, min_samples=3).fit(points)

        valid_labels = set(db.labels_) - {-1}
        centroids = []

        for label in valid_labels:
            cluster_points = points[db.labels_ == label]
            if 4 <= len(cluster_points) <= 25:
                center = self.fit_circle(cluster_points)
                if center is not None: centroids.append(center)

        return centroids

    def init_unified_csv(self):
        header = ['timestamp', f'{self.obs_ns}_gt_x', f'{self.obs_ns}_gt_y']
        for rid in sorted(self.trackers.keys()): 
            header += [f'{rid}_kf_x', f'{rid}_kf_y', f'{rid}_gt_x', f'{rid}_gt_y']
        with open(self.csv_filename, 'w', newline='') as f: csv.writer(f).writerow(header)

    def log_unified_data(self):
        row = [self.get_clock().now().nanoseconds / 1e9] + self.gt_registry.get(self.obs_ns, [0.0, 0.0])
        for rid in sorted(self.trackers.keys()):
            t = self.trackers[rid]
            gt = self.gt_registry.get(rid, [0.0, 0.0])
            row += [t['x'], t['y'], gt[0], gt[1]]
        with open(self.csv_filename, 'a', newline='') as f: csv.writer(f).writerow(row)

    def create_pose_msg(self, x, y):
        m = PoseStamped()
        m.header.frame_id = "world"
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose.position.x = x
        m.pose.position.y = y
        return m

def main():
    rclpy.init()
    node = MultiKalmanObserver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()