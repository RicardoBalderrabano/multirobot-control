"""
================================================================================
PROJECT: Multi-Robot State Estimation using Kalman Filter
MODULE: KalmanObserver Node (ROS2)
================================================================================
DESCRIPTION:
    This node implements a 2D Kalman Filter to track the position of a target 
    robot (TB3) relative to an observer robot (TB1). It achieves high-accuracy 
    localization by fusing two main data sources:
    
    1. PREDICTION (Unicycle Model): 
       Uses high-frequency wheel encoder data (Odometry) and a unicycle 
       kinematic model to predict the next state.
       
    2. MEASUREMENT (LiDAR Centroid): 
       Uses 2D LaserScan data to detect the target robot's geometric center 
       (centroid) and applies a radial offset correction to align with the 
       robot's physical center.

    The final fused output is compared against OptiTrack Ground Truth to 
    calculate performance metrics (RMSE and Improvement %).
================================================================================
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Pose2D
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import math
import csv

class KalmanObserver(Node):
    def __init__(self):
        super().__init__('kalman_observer')
        
        # --- 1. State Setup ---
        # Initial relative position of TB3 as seen by TB1
        self.state_x = 0.42
        self.state_y = 0.006
        self.init_theta = -3.13 # Relative moving angle (Orientation)
        
        # --- 2. Filter Gains ---
        self.p_matrix = 0.01             # Initial State Covariance 
        self.q_process_noise = 0.0001    # Uncertainty in unicycle model
        self.r_measure_noise = 5.0       # High value to prioritize smoothness over jitter

        # Internal variables for tracking and logging
        self.last_odom_x = 0.0
        self.last_odom_y = 0.0
        self.raw_odom_x = self.state_x  
        self.raw_odom_y = self.state_y
        self.true_x, self.true_y = self.state_x, self.state_y

        # --- 3. Logging Setup ---
        self.csv_filename = 'tracking_results_with_optitrack36.csv'
        self.init_csv()
        
        # Publishers/Subscribers
        self.pub_kf = self.create_publisher(PoseStamped, '/tracking/kalman_filter', 10)
        self.pub_raw_lidar = self.create_publisher(PoseStamped, '/tracking/raw_lidar', 10)
        
        qos_lidar = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(LaserScan, '/tb1/scan', self.lidar_callback, qos_lidar)
        self.create_subscription(Odometry, '/tb3/odom', self.odom_callback, 10)
        self.create_subscription(Pose2D, '/tb3/optitrack_pose2d', self.opti_callback, 10)

    # --- MODULE 1: UNICYCLE KINEMATICS ---
    def get_unicycle_prediction(self, msg):
        """Calculates the state displacement by rotatinf local TB3 wheel movements into the TB1 observer frame."""
        dx_local = msg.pose.pose.position.x - self.last_odom_x
        dy_local = msg.pose.pose.position.y - self.last_odom_y
        
        # Unicycle model: Rotate local displacement into Global frame
        dx = dx_local * math.cos(self.init_theta) - dy_local * math.sin(self.init_theta)
        dy = dx_local * math.sin(self.init_theta) + dy_local * math.cos(self.init_theta)
        
        # Update Predicted Sate and Uncertainty
        self.state_x += dx
        self.state_y += dy
        self.raw_odom_x += dx
        self.raw_odom_y += dy
        self.p_matrix += self.q_process_noise
        
        self.last_odom_x = msg.pose.pose.position.x
        self.last_odom_y = msg.pose.pose.position.y
        return self.state_x, self.state_y

    # --- MODULE 2: LIDAR CENTROID (MEASUREMENT) ---
    def get_lidar_measurement(self, msg):
        """Detects the target robot cluster and calculates its geometric center. Applies a radial correction to account for th
        robot's physical radius."""

        points = []
        for i, dist in enumerate(msg.ranges):
            if dist < msg.range_min or dist > msg.range_max: continue
            angle = msg.angle_min + (i * msg.angle_increment)
            lx, ly = dist * math.cos(angle), dist * math.sin(angle)

            # Distance-based clustering (Gating)
            if math.sqrt((lx - self.state_x)**2 + (ly - self.state_y)**2) < 0.25:
                points.append((lx, ly))

        if len(points) > 2:
            # Simple centroid with the radial offset expansion
            rx = sum(p[0] for p in points) / len(points)
            ry = sum(p[1] for p in points) / len(points)
            d = math.sqrt(rx**2 + ry**2)
            offset = 0.045 
            return rx * (1 + offset/d), ry * (1 + offset/d)
        return None, None

    # --- MODULE 3: KALMAN FUSION (Correction) ---
    def perform_kalman_update(self, z_x, z_y):
        """ Computes Kalman Gain and fuses the prediction and measurement to produce the optimal state estimate."""
        # Calculate Kalman Gain (Weighting of trust)
        k = self.p_matrix / (self.p_matrix + self.r_measure_noise)

        # State correction
        self.state_x += k * (z_x - self.state_x)
        self.state_y += k * (z_y - self.state_y)

        # Update Uncertainty Matrix
        self.p_matrix = (1 - k) * self.p_matrix
        return self.state_x, self.state_y

    # --- COORDINATION & LOGGING ---
    def odom_callback(self, msg):
        self.get_unicycle_prediction(msg)

    def lidar_callback(self, msg):
        mx, my = self.get_lidar_measurement(msg)
        if mx is not None:
            self.perform_kalman_update(mx, my)
            self.pub_raw_lidar.publish(self.create_pose_msg(mx, my))
            
            # Print Real-Time Statistics 
            print(f"--- LOG ---")
            print(f"ODOM+DYN: [{self.raw_odom_x:.3f}, {self.raw_odom_y:.3f}]")
            print(f"RAW LIDAR: [{mx:.3f}, {my:.3f}]")
            print(f"KALMAN:    [{self.state_x:.3f}, {self.state_y:.3f}]")
            print(f"GROUND T:  [{self.true_x:.3f}, {self.true_y:.3f}]")
            self.log_to_csv(mx, my)

        self.pub_kf.publish(self.create_pose_msg(self.state_x, self.state_y))

    def opti_callback(self, msg):
        self.true_x, self.true_y = msg.x, msg.y

    def create_pose_msg(self, x, y):
        m = PoseStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "tb1/odom"
        m.pose.position.x, m.pose.position.y = x, y
        return m

    def init_csv(self):
        with open(self.csv_filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['timestamp', 'odom_x', 'odom_y', 'lidar_x', 'lidar_y', 'kf_x', 'kf_y', 'opti_x', 'opti_y'])

    def log_to_csv(self, lx, ly):
        with open(self.csv_filename, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([self.get_clock().now().to_msg().sec, self.raw_odom_x, self.raw_odom_y, lx, ly, self.state_x, self.state_y, self.true_x, self.true_y])

def main(args=None):
    rclpy.init(args=args)
    node = KalmanObserver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()