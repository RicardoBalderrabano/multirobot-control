"""
#WORKS FOR ONLY DBSCAN, WITHOUT MODIFY Q PROCESS NEITHER VELOCITY IN KF
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import math
import csv
import numpy as np
import tf_transformations
from sklearn.cluster import DBSCAN

class KalmanObserver(Node):
    def __init__(self):
        super().__init__('kalman_observer')
        
        # --- 1. State Setup ---
        self.state_x = 0.0  # Fused KF State
        self.state_y = 0.0
        
        # Raw EKF positions for comparison
        self.ekf_raw_x = 0.0
        self.ekf_raw_y = 0.0
        
        # Observers for Global Transformation
        self.tb1_global_x = 0.0
        self.tb1_global_y = 0.0
        self.tb1_global_yaw = 0.0
        
        self.last_tb3_global_x = None
        self.last_tb3_global_y = None

        # Ground Truth Storage
        self.gt_tb1_x, self.gt_tb1_y = 0.0, 0.0
        self.gt_tb3_x, self.gt_tb3_y = 0.0, 0.0

        # --- 2. Filter Gains ---
        self.p_matrix = 0.1
        self.q_process_noise = 0.001
        self.r_measure_noise = 1.5  # Optimized tuned value
        self.initialized = False

        # --- 3. Publishers/Subscribers ---
        self.pub_kf = self.create_publisher(PoseStamped, '/tb3/kalman', 10)
        
        qos_lidar = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        qos_opti = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)

        self.create_subscription(PoseStamped, '/tb1/pose', self.gt_tb1_callback, qos_opti)
        self.create_subscription(PoseStamped, '/tb3/pose', self.gt_tb3_callback, qos_opti)
        self.create_subscription(PoseStamped, '/tb1/global_pose_estimated', self.tb1_global_callback, 10)
        self.create_subscription(PoseStamped, '/tb3/global_pose_estimated', self.tb3_global_callback, 10)
        self.create_subscription(LaserScan, '/tb1/scan', self.lidar_callback, qos_lidar)

        # --- 4. Logging ---
        self.csv_filename = 'tracking_dbscan5.csv'
        self.init_csv()

    # --- CALLBACKS ---
    def gt_tb1_callback(self, msg):
        self.gt_tb1_x, self.gt_tb1_y = msg.pose.position.x, msg.pose.position.y

    def gt_tb3_callback(self, msg):
        self.gt_tb3_x, self.gt_tb3_y = msg.pose.position.x, msg.pose.position.y

    def tb1_global_callback(self, msg):
        self.tb1_global_x, self.tb1_global_y = msg.pose.position.x, msg.pose.position.y
        q = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        _, _, self.tb1_global_yaw = tf_transformations.euler_from_quaternion(q)

    def tb3_global_callback(self, msg):
        self.ekf_raw_x, self.ekf_raw_y = msg.pose.position.x, msg.pose.position.y
        if self.last_tb3_global_x is not None:
            dx = self.ekf_raw_x - self.last_tb3_global_x
            dy = self.ekf_raw_y - self.last_tb3_global_y
            # Prediction Step
            self.state_x += dx
            self.state_y += dy
            self.p_matrix += self.q_process_noise
        else:
            self.state_x, self.state_y = self.ekf_raw_x, self.ekf_raw_y
        self.last_tb3_global_x, self.last_tb3_global_y = self.ekf_raw_x, self.ekf_raw_y

    def lidar_callback(self, msg):
        rx_rel, ry_rel = self.get_lidar_dbscan(msg)
        
        if rx_rel is not None:
            # Transform LiDAR to Global
            gx = self.tb1_global_x + (rx_rel * math.cos(self.tb1_global_yaw) - ry_rel * math.sin(self.tb1_global_yaw))
            gy = self.tb1_global_y + (rx_rel * math.sin(self.tb1_global_yaw) + ry_rel * math.cos(self.tb1_global_yaw))

            # Proper Initialization on first hit
            if not self.initialized:
                self.state_x, self.state_y = gx, gy
                self.initialized = True

            # Update Fused State
            self.perform_kalman_update(gx, gy)
            
            # Log all sources
            self.log_to_csv()
            
            # Console Debug
            kf_err = math.sqrt((self.state_x - self.gt_tb3_x)**2 + (self.state_y - self.gt_tb3_y)**2)
            print(f"DBSCAN KF Error: {kf_err:.4f}m")

        self.pub_kf.publish(self.create_pose_msg(self.state_x, self.state_y))

    def get_lidar_dbscan(self, msg):
        points = []
        for i, dist in enumerate(msg.ranges):
            if dist < msg.range_min or dist > msg.range_max: continue
            angle = msg.angle_min + (i * msg.angle_increment)
            points.append([dist * math.cos(angle), dist * math.sin(angle)])
        
        if len(points) < 5: return None, None
        data = np.array(points)
        
        # Parameters optimized during characterization
        db = DBSCAN(eps=0.15, min_samples=5).fit(data)
        
        # Local predicted target for gating
        dx = self.state_x - self.tb1_global_x
        dy = self.state_y - self.tb1_global_y
        tx = dx * math.cos(self.tb1_global_yaw) + dy * math.sin(self.tb1_global_yaw)
        ty = -dx * math.sin(self.tb1_global_yaw) + dy * math.cos(self.tb1_global_yaw)

        best_c, min_d = None, 0.5
        for label in set(db.labels_):
            if label == -1: continue
            c = np.mean(data[db.labels_ == label], axis=0)
            dist = math.sqrt((c[0]-tx)**2 + (c[1]-ty)**2)
            if dist < min_d:
                min_d = dist
                best_c = c

        if best_c is not None:
            # Final optimized 5cm center shift
            vec = np.array([tx - best_c[0], ty - best_c[1]])
            v_norm = np.linalg.norm(vec)
            if v_norm > 0:
                res = best_c + (vec / v_norm) * 0.05
                return res[0], res[1]
        return None, None

    def perform_kalman_update(self, z_x, z_y):
        k = self.p_matrix / (self.p_matrix + self.r_measure_noise)
        self.state_x += k * (z_x - self.state_x)
        self.state_y += k * (z_y - self.state_y)
        self.p_matrix = (1 - k) * self.p_matrix

    def create_pose_msg(self, x, y):
        m = PoseStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "world"
        m.pose.position.x, m.pose.position.y = x, y
        return m

    def init_csv(self):
        with open(self.csv_filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['timestamp', 'kf_x', 'kf_y', 'ekf_raw_x', 'ekf_raw_y', 'gt_x', 'gt_y', 'gt_tb1_x', 'gt_tb1_y'])

    def log_to_csv(self):
        with open(self.csv_filename, 'a', newline='') as f:
            ts = self.get_clock().now().nanoseconds / 1e9
            csv.writer(f).writerow([ts, self.state_x, self.state_y, self.ekf_raw_x, self.ekf_raw_y, 
                                    self.gt_tb3_x, self.gt_tb3_y, self.gt_tb1_x, self.gt_tb1_y])

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(KalmanObserver())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
"""
"""
# EXPERIMENT 5
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import math
import csv
import numpy as np
import tf_transformations
from sklearn.cluster import DBSCAN

class KalmanObserver(Node):
    def __init__(self):
        super().__init__('kalman_observer')
        
        # --- 1. State Setup (Constant Velocity Model) ---
        self.state_x = 0.0
        self.state_y = 0.0
        self.state_vx = 0.0
        self.state_vy = 0.0
        
        self.ekf_raw_x = 0.0
        self.ekf_raw_y = 0.0
        
        # Observer State & Velocity
        self.obs_x, self.obs_y, self.obs_yaw = 0.0, 0.0, 0.0
        self.obs_vx, self.obs_vy = 0.0, 0.0
        self.last_obs_x, self.last_obs_y = None, None
        
        self.last_tb3_x, self.last_tb3_y = None, None
        self.last_time = None

        # --- 2. Filter Gains ---
        self.p_matrix = np.eye(2) * 0.1 # Simplified 2x2 for position
        self.q_base = 0.001
        self.r_meas = 2.5  # Increased R to trust smooth odom more during high-speed turns
        self.initialized = False

        # --- 3. Pubs/Subs ---
        self.pub_kf = self.create_publisher(PoseStamped, '/tb3/kalman', 10)
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)

        self.create_subscription(PoseStamped, '/tb1/pose', self.gt_tb1_cb, qos)
        self.create_subscription(PoseStamped, '/tb3/pose', self.gt_tb3_cb, qos)
        self.create_subscription(PoseStamped, '/tb1/global_pose_estimated', self.tb1_global_cb, 10)
        self.create_subscription(PoseStamped, '/tb3/global_pose_estimated', self.tb3_global_cb, 10)
        self.create_subscription(LaserScan, '/tb1/scan', self.lidar_callback, qos)

        self.csv_filename = 'tracking_dbscan15.csv'
        self.gt_tb1_x, self.gt_tb1_y, self.gt_tb3_x, self.gt_tb3_y = 0.0, 0.0, 0.0, 0.0
        self.init_csv()

    # --- CALLBACKS ---
    def gt_tb1_cb(self, msg): self.gt_tb1_x, self.gt_tb1_y = msg.pose.position.x, msg.pose.position.y
    def gt_tb3_cb(self, msg): self.gt_tb3_x, self.gt_tb3_y = msg.pose.position.x, msg.pose.position.y

    def tb1_global_cb(self, msg):
        #Tracks Observer Position and Velocity
        self.obs_x, self.obs_y = msg.pose.position.x, msg.pose.position.y
        q = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        _, _, self.obs_yaw = tf_transformations.euler_from_quaternion(q)
        
        # Calculate Observer Velocity to inform the Filter uncertainty
        if self.last_obs_x is not None:
            self.obs_vx = self.obs_x - self.last_obs_x
            self.obs_vy = self.obs_y - self.last_obs_y
        self.last_obs_x, self.last_obs_y = self.obs_x, self.obs_y

    def tb3_global_cb(self, msg):
        #Prediction Step: Observer-Aware Process Noise Inflation
        current_time = self.get_clock().now().nanoseconds / 1e9
        self.ekf_raw_x, self.ekf_raw_y = msg.pose.position.x, msg.pose.position.y

        if self.last_time is not None:
            dt = current_time - self.last_time
            dx = self.ekf_raw_x - self.last_tb3_x
            dy = self.ekf_raw_y - self.last_tb3_y

            # 1. State Prediction (Target Displacement)
            self.state_x += dx
            self.state_y += dy

            # 2. Dynamic Q-Inflation
            # The faster TB1 moves, the more we inflate P to allow LiDAR to correct the 'lag'
            v_obs = math.sqrt(self.obs_vx**2 + self.obs_vy**2)
            dynamic_q = self.q_base * (1.0 + v_obs * 15.0) # Scaling factor 15.0
            
            self.p_matrix[0,0] += dynamic_q
            self.p_matrix[1,1] += dynamic_q
        else:
            self.state_x, self.state_y = self.ekf_raw_x, self.ekf_raw_y
        
        self.last_tb3_x, self.last_tb3_y = self.ekf_raw_x, self.ekf_raw_y
        self.last_time = current_time

    def lidar_callback(self, msg):
        rx_rel, ry_rel = self.get_lidar_dbscan(msg)
        
        if rx_rel is not None:
            # Transform Local LiDAR to Global Map
            gx = self.obs_x + (rx_rel * math.cos(self.obs_yaw) - ry_rel * math.sin(self.obs_yaw))
            gy = self.obs_y + (rx_rel * math.sin(self.obs_yaw) + ry_rel * math.cos(self.obs_yaw))

            if not self.initialized:
                self.state_x, self.state_y = gx, gy
                self.initialized = True

            # 3. Update Step
            # Use separate p_matrix elements for X and Y
            kx = self.p_matrix[0,0] / (self.p_matrix[0,0] + self.r_meas)
            ky = self.p_matrix[1,1] / (self.p_matrix[1,1] + self.r_meas)
            
            self.state_x += kx * (gx - self.state_x)
            self.state_y += ky * (gy - self.state_y)
            
            self.p_matrix[0,0] *= (1 - kx)
            self.p_matrix[1,1] *= (1 - ky)
            
            self.log_to_csv()

        self.pub_kf.publish(self.create_pose_msg(self.state_x, self.state_y))

    def get_lidar_dbscan(self, msg):
        points = []
        for i, dist in enumerate(msg.ranges):
            if dist < msg.range_min or dist > msg.range_max: continue
            angle = msg.angle_min + (i * msg.angle_increment)
            points.append([dist * math.cos(angle), dist * math.sin(angle)])
        
        if len(points) < 5: return None, None
        data = np.array(points)
        db = DBSCAN(eps=0.15, min_samples=5).fit(data)
        
        # Local predicted target for gating
        dx, dy = self.state_x - self.obs_x, self.state_y - self.obs_y
        tx = dx * math.cos(self.obs_yaw) + dy * math.sin(self.obs_yaw)
        ty = -dx * math.sin(self.obs_yaw) + dy * math.cos(self.obs_yaw)

        best_c, min_d = None, 0.5
        for label in set(db.labels_):
            if label == -1: continue
            c = np.mean(data[db.labels_ == label], axis=0)
            dist = math.sqrt((c[0]-tx)**2 + (c[1]-ty)**2)
            if dist < min_d: min_d, best_c = dist, c

        if best_c is not None:
            vec = np.array([tx - best_c[0], ty - best_c[1]])
            v_norm = np.linalg.norm(vec)
            if v_norm > 0:
                # Reduced radius offset to 4cm to pull the circle inward
                res = best_c + (vec / v_norm) * 0.04
                return res[0], res[1]
        return None, None

    # --- CSV & MESSAGE HELPERS ---
    def init_csv(self):
        with open(self.csv_filename, 'w', newline='') as f:
            csv.writer(f).writerow(['timestamp', 'kf_x', 'kf_y', 'ekf_raw_x', 'ekf_raw_y', 'gt_x', 'gt_y', 'gt_obs_x', 'gt_obs_y'])

    def log_to_csv(self):
        with open(self.csv_filename, 'a', newline='') as f:
            ts = self.get_clock().now().nanoseconds / 1e9
            csv.writer(f).writerow([ts, self.state_x, self.state_y, self.ekf_raw_x, self.ekf_raw_y, 
                                    self.gt_tb3_x, self.gt_tb3_y, self.gt_tb1_x, self.gt_tb1_y])

    def create_pose_msg(self, x, y):
        m = PoseStamped(); m.header.frame_id = "world"; m.header.stamp = self.get_clock().now().to_msg()
        m.pose.position.x, m.pose.position.y = x, y
        return m

def main():
    rclpy.init(); rclpy.spin(KalmanObserver()); rclpy.shutdown()

if __name__ == '__main__': main()
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
from sklearn.cluster import DBSCAN

# --- NEW: Message Filters for Time Synchronization ---
import message_filters

class KalmanObserver(Node):
    def __init__(self):
        super().__init__('kalman_observer')
        
        # --- 1. State Setup (2D Position + Odometry Control) ---
        self.state_x = 0.0
        self.state_y = 0.0
        self.p_matrix = np.eye(2) * 0.1
        
        # --- 2. Filter Gains ---
        self.q_base = 0.001       # Process noise (trust in Odometry step)
        self.r_meas = 2.5         # Measurement noise (trust in LiDAR)
        self.gate_threshold = 0.4 # METERS: Ignore LiDAR if it jumps further than this
        
        # Buffers
        self.ekf_raw_x, self.ekf_raw_y = 0.0, 0.0
        self.last_tb3_x, self.last_tb3_y = None, None
        self.initialized = False

        # --- 3. Pubs/Subs ---
        self.pub_kf = self.create_publisher(PoseStamped, '/tb3/kalman', 10)
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)

        # Standard Subscribers for Ground Truth (Logging only)
        self.create_subscription(PoseStamped, '/tb1/pose', self.gt_tb1_cb, qos)
        self.create_subscription(PoseStamped, '/tb3/pose', self.gt_tb3_cb, qos)
        
        # Standard Subscriber for Target Prediction (TB3 Global)
        self.create_subscription(PoseStamped, '/tb3/global_pose_estimated', self.tb3_global_cb, 10)

        # --- 4. THE TIME SYNCHRONIZER (The Fix) ---
        # Instead of normal subscriptions, we create "Filter Subscribers"
        self.scan_sub = message_filters.Subscriber(self, LaserScan, '/tb1/scan', qos_profile=qos)
        self.obs_pose_sub = message_filters.Subscriber(self, PoseStamped, '/tb1/global_pose_estimated')

        # Synchronizer: Waits until it has a scan and a pose with nearly identical timestamps
        # slop=0.05 means it allows a 50-millisecond difference between the two messages
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.scan_sub, self.obs_pose_sub], 
            queue_size=10, 
            slop=0.05
        )
        self.ts.registerCallback(self.sync_lidar_callback)

        # Logging
        self.csv_filename = 'tracking_dbscan37.csv'
        self.gt_tb1_x, self.gt_tb1_y, self.gt_tb3_x, self.gt_tb3_y = 0.0, 0.0, 0.0, 0.0
        self.init_csv()
        self.get_logger().info("Kalman Filter running with Message Filters Synchronization!")

    # --- CALLBACKS ---
    def gt_tb1_cb(self, msg): self.gt_tb1_x, self.gt_tb1_y = msg.pose.position.x, msg.pose.position.y
    def gt_tb3_cb(self, msg): self.gt_tb3_x, self.gt_tb3_y = msg.pose.position.x, msg.pose.position.y

    def tb3_global_cb(self, msg):
        """ Prediction Step: Driven by Target's EKF Odometry """
        self.ekf_raw_x, self.ekf_raw_y = msg.pose.position.x, msg.pose.position.y

        if self.last_tb3_x is not None and self.initialized:
            dx = self.ekf_raw_x - self.last_tb3_x
            dy = self.ekf_raw_y - self.last_tb3_y

            self.state_x += dx
            self.state_y += dy

            Q = np.eye(2) * self.q_base
            self.p_matrix = self.p_matrix + Q
        elif not self.initialized:
            self.state_x, self.state_y = self.ekf_raw_x, self.ekf_raw_y
        
        self.last_tb3_x, self.last_tb3_y = self.ekf_raw_x, self.ekf_raw_y

    def sync_lidar_callback(self, scan_msg, obs_pose_msg):
        """ Update Step: Triggers only when Scan and Observer Pose are perfectly aligned in time """
        
        # 1. Extract synchronized observer pose from your custom node
        obs_x = obs_pose_msg.pose.position.x
        obs_y = obs_pose_msg.pose.position.y
        q = [obs_pose_msg.pose.orientation.x, obs_pose_msg.pose.orientation.y, 
             obs_pose_msg.pose.orientation.z, obs_pose_msg.pose.orientation.w]
        _, _, obs_yaw = tf_transformations.euler_from_quaternion(q)

        # 2. Process LiDAR with exact pose
        rx_rel, ry_rel = self.get_lidar_dbscan(scan_msg, obs_x, obs_y, obs_yaw)
        
        if rx_rel is not None:
            # Transform Local LiDAR to Global Map
            gx = obs_x + (rx_rel * math.cos(obs_yaw) - ry_rel * math.sin(obs_yaw))
            gy = obs_y + (rx_rel * math.sin(obs_yaw) + ry_rel * math.cos(obs_yaw))

            if not self.initialized:
                self.state_x, self.state_y = gx, gy
                self.initialized = True
                return

            # --- INNOVATION GATE ---
            distance_error = math.sqrt((gx - self.state_x)**2 + (gy - self.state_y)**2)
            
            if distance_error < self.gate_threshold:
                # Update logic
                R = np.eye(2) * self.r_meas
                z = np.array([gx, gy])
                h = np.array([self.state_x, self.state_y])
                innovation = z - h
                
                S = self.p_matrix + R
                K = self.p_matrix @ np.linalg.inv(S)
                
                new_state = h + K @ innovation
                self.state_x, self.state_y = new_state[0], new_state[1]
                self.p_matrix = (np.eye(2) - K) @ self.p_matrix
                
            self.log_to_csv()

        self.pub_kf.publish(self.create_pose_msg(self.state_x, self.state_y))

    def get_lidar_dbscan(self, msg, obs_x, obs_y, obs_yaw):
        points = []
        for i, dist in enumerate(msg.ranges):
            if dist < msg.range_min or dist > msg.range_max: continue
            angle = msg.angle_min + (i * msg.angle_increment)
            points.append([dist * math.cos(angle), dist * math.sin(angle)])
        
        if len(points) < 5: return None, None
        data = np.array(points)
        db = DBSCAN(eps=0.15, min_samples=5).fit(data)
        
        # Gating prediction
        dx, dy = self.state_x - obs_x, self.state_y - obs_y
        tx = dx * math.cos(obs_yaw) + dy * math.sin(obs_yaw)
        ty = -dx * math.sin(obs_yaw) + dy * math.cos(obs_yaw)

        best_c, min_d = None, 0.5
        for label in set(db.labels_):
            if label == -1: continue
            mask = (db.labels_ == label)
            c = np.mean(data[mask], axis=0)
            dist = math.sqrt((c[0]-tx)**2 + (c[1]-ty)**2)
            
            if dist < min_d: 
                min_d = dist
                best_c = c

        if best_c is not None:
            vec = np.array([tx - best_c[0], ty - best_c[1]])
            v_norm = np.linalg.norm(vec)
            if v_norm > 0:
                res = best_c + (vec / v_norm) * 0.04
                return res[0], res[1]
        return None, None

    def init_csv(self):
        with open(self.csv_filename, 'w', newline='') as f:
            csv.writer(f).writerow(['timestamp', 'kf_x', 'kf_y', 'ekf_raw_x', 'ekf_raw_y', 'gt_x', 'gt_y', 'gt_obs_x', 'gt_obs_y'])

    def log_to_csv(self):
        with open(self.csv_filename, 'a', newline='') as f:
            ts = self.get_clock().now().nanoseconds / 1e9
            csv.writer(f).writerow([ts, self.state_x, self.state_y, self.ekf_raw_x, self.ekf_raw_y, 
                                    self.gt_tb3_x, self.gt_tb3_y, self.gt_tb1_x, self.gt_tb1_y])

    def create_pose_msg(self, x, y):
        m = PoseStamped()
        m.header.frame_id = "world" # Matches your transformer node
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose.position.x, m.pose.position.y = x, y
        return m

def main(): 
    rclpy.init()
    node = KalmanObserver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__': 
    main()