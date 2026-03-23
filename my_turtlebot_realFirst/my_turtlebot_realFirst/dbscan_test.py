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
        
        # --- 1. State Setup (Dual Filters) ---
        # Legacy Filter State
        self.state_leg_x, self.state_leg_y = 0.0, 0.0
        self.p_leg = 0.1
        
        # DBSCAN Filter State
        self.state_db_x, self.state_db_y = 0.0, 0.0
        self.p_db = 0.1

        # Global Transformation & EKF Storage
        self.ekf_raw_x, self.ekf_raw_y = 0.0, 0.0
        self.tb1_x, self.tb1_y, self.tb1_yaw = 0.0, 0.0, 0.0
        self.last_tb3_x, self.last_tb3_y = None, None

        # Ground Truth
        self.gt_tb1_x, self.gt_tb1_y = 0.0, 0.0
        self.gt_tb3_x, self.gt_tb3_y = 0.0, 0.0

        # --- 2. Filter Gains ---
        self.q_proc = 0.001
        self.r_meas = 1.5  # Using your tuned value
        self.initialized = False

        # --- 3. Pubs/Subs ---
        self.pub_kf = self.create_publisher(PoseStamped, '/tb3/kalman_comparison', 10)
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)

        self.create_subscription(PoseStamped, '/tb1/pose', self.gt_tb1_cb, qos)
        self.create_subscription(PoseStamped, '/tb3/pose', self.gt_tb3_cb, qos)
        self.create_subscription(PoseStamped, '/tb1/global_pose_estimated', self.tb1_global_cb, 10)
        self.create_subscription(PoseStamped, '/tb3/global_pose_estimated', self.tb3_global_cb, 10)
        self.create_subscription(LaserScan, '/tb1/scan', self.lidar_callback, qos)

        self.csv_filename = 'dbscan_vs_legacy_comparison7.csv'
        self.init_csv()

    # --- CALLBACKS ---
    def gt_tb1_cb(self, msg): self.gt_tb1_x, self.gt_tb1_y = msg.pose.position.x, msg.pose.position.y
    def gt_tb3_cb(self, msg): self.gt_tb3_x, self.gt_tb3_y = msg.pose.position.x, msg.pose.position.y

    def tb1_global_cb(self, msg):
        self.tb1_x, self.tb1_y = msg.pose.position.x, msg.pose.position.y
        q = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        _, _, self.tb1_yaw = tf_transformations.euler_from_quaternion(q)

    def tb3_global_cb(self, msg):
        self.ekf_raw_x, self.ekf_raw_y = msg.pose.position.x, msg.pose.position.y
        if self.last_tb3_x is not None:
            dx, dy = self.ekf_raw_x - self.last_tb3_x, self.ekf_raw_y - self.last_tb3_y
            # Predict both filters using EKF displacement
            self.state_leg_x += dx; self.state_leg_y += dy
            self.state_db_x += dx; self.state_db_y += dy
            self.p_leg += self.q_proc; self.p_db += self.q_proc
        else:
            self.state_leg_x, self.state_leg_y = self.ekf_raw_x, self.ekf_raw_y
            self.state_db_x, self.state_db_y = self.ekf_raw_x, self.ekf_raw_y
        self.last_tb3_x, self.last_tb3_y = self.ekf_raw_x, self.ekf_raw_y

    def lidar_callback(self, msg):
        # 1. Get Measurements from both algorithms
        legacy_res = self.get_lidar_legacy(msg)
        dbscan_res = self.get_lidar_dbscan(msg)

        # 2. Update Legacy Filter
        if legacy_res[0] is not None:
            gx, gy = self.to_global(legacy_res[0], legacy_res[1])
            self.state_leg_x, self.state_leg_y, self.p_leg = self.kf_update(gx, gy, self.state_leg_x, self.state_leg_y, self.p_leg)

        # 3. Update DBSCAN Filter
        if dbscan_res[0] is not None:
            gx, gy = self.to_global(dbscan_res[0], dbscan_res[1])
            if not self.initialized: # Initialize on first real LiDAR hit
                self.state_db_x, self.state_db_y = gx, gy
                self.initialized = True
            self.state_db_x, self.state_db_y, self.p_db = self.kf_update(gx, gy, self.state_db_x, self.state_db_y, self.p_db)

        self.log_to_csv()
        self.pub_kf.publish(self.create_pose_msg(self.state_db_x, self.state_db_y))

    # --- ALGORITHMS ---
    def get_lidar_legacy(self, msg):
        points = []
        # Local predicted target for gating
        tx = (self.state_leg_x - self.tb1_x) * math.cos(self.tb1_yaw) + (self.state_leg_y - self.tb1_y) * math.sin(self.tb1_yaw)
        ty = -(self.state_leg_x - self.tb1_x) * math.sin(self.tb1_yaw) + (self.state_leg_y - self.tb1_y) * math.cos(self.tb1_yaw)

        for i, dist in enumerate(msg.ranges):
            if dist < msg.range_min or dist > msg.range_max: continue
            angle = msg.angle_min + (i * msg.angle_increment)
            lx, ly = dist * math.cos(angle), dist * math.sin(angle)
            if math.sqrt((lx - tx)**2 + (ly - ty)**2) < 0.3: points.append((lx, ly))

        if len(points) > 2:
            rx, ry = sum(p[0] for p in points)/len(points), sum(p[1] for p in points)/len(points)
            d = math.sqrt(rx**2 + ry**2)
            return rx * (1 + 0.045/d), ry * (1 + 0.045/d) # Original 4.5cm fix
        return None, None

    def get_lidar_dbscan(self, msg):
        points = []
        for i, dist in enumerate(msg.ranges):
            if dist < msg.range_min or dist > msg.range_max: continue
            angle = msg.angle_min + (i * msg.angle_increment)
            points.append([dist * math.cos(angle), dist * math.sin(angle)])
        
        if len(points) < 5: return None, None
        data = np.array(points)
        
        # Stricter clustering for a more consistent centroid
        db = DBSCAN(eps=0.15, min_samples=5).fit(data)
        
        # Local target for gating (where we think TB3 is)
        tx = (self.state_db_x - self.tb1_x) * math.cos(self.tb1_yaw) + (self.state_db_y - self.tb1_y) * math.sin(self.tb1_yaw)
        ty = -(self.state_db_x - self.tb1_x) * math.sin(self.tb1_yaw) + (self.state_db_y - self.tb1_y) * math.cos(self.tb1_yaw)

        best_c, min_d = None, 0.5 # Gating radius
        for label in set(db.labels_):
            if label == -1: continue
            # Instead of the average (mean), we could use the point closest to TB1
            # but mean is usually more stable for centroids.
            c = np.mean(data[db.labels_ == label], axis=0)
            dist = math.sqrt((c[0]-tx)**2 + (c[1]-ty)**2)
            if dist < min_d: min_d, best_c = dist, c

        if best_c is not None:
            # Pushing the center in toward the predicted position
            vec = np.array([tx - best_c[0], ty - best_c[1]])
            v_norm = np.linalg.norm(vec)
            if v_norm > 0:
                # REDUCED OFFSET: Moving from 0.07 to 0.05 to pull the line inward
                res = best_c + (vec / v_norm) * 0.05 
                return res[0], res[1]
        return None, None

    # --- HELPERS ---
    def to_global(self, rx, ry):
        gx = self.tb1_x + (rx * math.cos(self.tb1_yaw) - ry * math.sin(self.tb1_yaw))
        gy = self.tb1_y + (rx * math.sin(self.tb1_yaw) + ry * math.cos(self.tb1_yaw))
        return gx, gy

    def kf_update(self, zx, zy, sx, sy, p):
        k = p / (p + self.r_meas)
        return sx + k*(zx - sx), sy + k*(zy - sy), (1-k)*p

    def init_csv(self):
        with open(self.csv_filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['ts', 'leg_x', 'leg_y', 'db_x', 'db_y', 'odom_x', 'odom_y', 'gt_x', 'gt_y', 'gt_obs_x', 'gt_obs_y'])

    def log_to_csv(self):
        with open(self.csv_filename, 'a', newline='') as f:
            ts = self.get_clock().now().nanoseconds / 1e9
            csv.writer(f).writerow([ts, self.state_leg_x, self.state_leg_y, self.state_db_x, self.state_db_y, 
                                    self.ekf_raw_x, self.ekf_raw_y, self.gt_tb3_x, self.gt_tb3_y, self.gt_tb1_x, self.gt_tb1_y])

    def create_pose_msg(self, x, y):
        m = PoseStamped(); m.header.frame_id = "world"; m.header.stamp = self.get_clock().now().to_msg()
        m.pose.position.x, m.pose.position.y = x, y
        return m

def main():
    rclpy.init(); rclpy.spin(KalmanObserver()); rclpy.shutdown()

if __name__ == '__main__': main()
