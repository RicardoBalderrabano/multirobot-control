
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
        self.r_measure_noise = 2.0 

        # --- 3. Publishers/Subscribers ---
        self.pub_kf = self.create_publisher(PoseStamped, '/tb3/kalman', 10)
        
        qos_lidar = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        qos_opti = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)

        # Optitrack Ground Truth
        self.create_subscription(PoseStamped, '/tb1/pose', self.gt_tb1_callback, qos_opti)
        self.create_subscription(PoseStamped, '/tb3/pose', self.gt_tb3_callback, qos_opti)
        
        # Global EKF Positions
        self.create_subscription(PoseStamped, '/tb1/global_pose_estimated', self.tb1_global_callback, 10)
        self.create_subscription(PoseStamped, '/tb3/global_pose_estimated', self.tb3_global_callback, 10)
        
        # LiDAR Scan
        self.create_subscription(LaserScan, '/tb1/scan', self.lidar_callback, qos_lidar)

        # --- 4. Logging ---
        self.csv_filename = 'tracking_dbscan2.csv'
        self.init_csv()

    # --- CALLBACKS ---
    def gt_tb1_callback(self, msg):
        self.gt_tb1_x = msg.pose.position.x
        self.gt_tb1_y = msg.pose.position.y

    def gt_tb3_callback(self, msg):
        self.gt_tb3_x = msg.pose.position.x
        self.gt_tb3_y = msg.pose.position.y

    def tb1_global_callback(self, msg):
        self.tb1_global_x = msg.pose.position.x
        self.tb1_global_y = msg.pose.position.y
        q = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        _, _, self.tb1_global_yaw = tf_transformations.euler_from_quaternion(q)

    def tb3_global_callback(self, msg):
        #Prediction Module + Raw EKF Storage
        self.ekf_raw_x = msg.pose.position.x
        self.ekf_raw_y = msg.pose.position.y

        if self.last_tb3_global_x is not None:
            dx = self.ekf_raw_x - self.last_tb3_global_x
            dy = self.ekf_raw_y - self.last_tb3_global_y
            
            # Predict Fused State
            self.state_x += dx
            self.state_y += dy
            self.p_matrix += self.q_process_noise
        else:
            self.state_x = self.ekf_raw_x
            self.state_y = self.ekf_raw_y

        self.last_tb3_global_x = self.ekf_raw_x
        self.last_tb3_global_y = self.ekf_raw_y

    def lidar_callback(self, msg):
        rx_rel, ry_rel = self.get_lidar_measurement(msg)
        
        if rx_rel is not None:
            # Transform LiDAR to Global
            gx = self.tb1_global_x + (rx_rel * math.cos(self.tb1_global_yaw) - ry_rel * math.sin(self.tb1_global_yaw))
            gy = self.tb1_global_y + (rx_rel * math.sin(self.tb1_global_yaw) + ry_rel * math.cos(self.tb1_global_yaw))

            # Update Fused State
            self.perform_kalman_update(gx, gy)
            
            # Log all three sources + TB1 Ground Truth
            self.log_to_csv()
            
            # Detailed Console Output
            print(f"\n--- POSITION COMPARISON ---")
            print(f"OPTITRACK (Truth): [{self.gt_tb3_x:.3f}, {self.gt_tb3_y:.3f}]")
            print(f"RAW EKF (Odom)  : [{self.ekf_raw_x:.3f}, {self.ekf_raw_y:.3f}]")
            print(f"FUSED KF (Lidar): [{self.state_x:.3f}, {self.state_y:.3f}]")
            
            # Calculate Errors
            ekf_err = math.sqrt((self.ekf_raw_x - self.gt_tb3_x)**2 + (self.ekf_raw_y - self.gt_tb3_y)**2)
            kf_err = math.sqrt((self.state_x - self.gt_tb3_x)**2 + (self.state_y - self.gt_tb3_y)**2)
            print(f"ERROR -> EKF: {ekf_err:.4f}m | KF: {kf_err:.4f}m")

        self.pub_kf.publish(self.create_pose_msg(self.state_x, self.state_y))

    def get_lidar_measurement(self, msg):
        points = []
        for i, dist in enumerate(msg.ranges):
            if dist < msg.range_min or dist > msg.range_max: continue
            angle = msg.angle_min + (i * msg.angle_increment)
            lx, ly = dist * math.cos(angle), dist * math.sin(angle)

            dx = self.state_x - self.tb1_global_x
            dy = self.state_y - self.tb1_global_y
            tx = dx * math.cos(self.tb1_global_yaw) + dy * math.sin(self.tb1_global_yaw)
            ty = -dx * math.sin(self.tb1_global_yaw) + dy * math.cos(self.tb1_global_yaw)

            if math.sqrt((lx - tx)**2 + (ly - ty)**2) < 0.3:
                points.append((lx, ly))

        if len(points) > 2:
            rx, ry = sum(p[0] for p in points)/len(points), sum(p[1] for p in points)/len(points)
            d = math.sqrt(rx**2 + ry**2)
            return rx * (1 + 0.045/d), ry * (1 + 0.045/d)
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
            # Added gt_tb1_x and gt_tb1_y to headers
            writer.writerow([
                'timestamp', 
                'kf_x', 'kf_y', 
                'ekf_raw_x', 'ekf_raw_y', 
                'gt_x', 'gt_y',
                'gt_tb1_x', 'gt_tb1_y'
            ])

    def log_to_csv(self):
        with open(self.csv_filename, 'a', newline='') as f:
            writer = csv.writer(f)
            ts = self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec * 1e-9
            # Added self.gt_tb1_x and self.gt_tb1_y to the logged row
            writer.writerow([
                ts, 
                self.state_x, self.state_y, 
                self.ekf_raw_x, self.ekf_raw_y, 
                self.gt_tb3_x, self.gt_tb3_y,
                self.gt_tb1_x, self.gt_tb1_y
            ])

def main(args=None):
    rclpy.init(args=args)
    node = KalmanObserver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()