"""
================================================================================
PROJECT: Multi-Robot State Estimation - Distributed Observer
MODULE: KalmanObserver Node (ROS2) - Optimized for Timestamped Tracking
================================================================================
DESCRIPTION:
    Tracks TB2 relative to TB1 using a Linear Kalman Filter. 
    Updates:
    1. STAMPED DATA: Processes PoseStamped messages from the new bridge.
    2. PRECISION LOGGING: Saves full sec + nanosec for accurate post-processing.
    3. DATA CONSISTENCY: Uses the actual OptiTrack capture time for the log.
================================================================================
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Pose2D # Pose2D kept for backward compatibility if needed
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import math
import csv

class KalmanObserver(Node):
    def __init__(self):
        super().__init__('kalman_observer')
        
        # --- 1. State Setup ---
        self.state_x = 0.41
        self.state_y = 0.006
        self.init_theta = 3.10 
        
        # --- 2. Filter Gains ---
        self.p_matrix = 0.01             
        self.q_process_noise = 0.0001    
        self.r_measure_noise = 5.0       

        # Internal Variables
        self.last_odom_x = 0.0
        self.last_odom_y = 0.0
        self.raw_odom_x = self.state_x  
        self.raw_odom_y = self.state_y
        
        # Buffers for Timer-based Logging
        self.last_mx, self.last_my = 0.0, 0.0
        self.true_tb1_x, self.true_tb1_y = 0.0, 0.0 
        self.true_tb2_x, self.true_tb2_y = self.state_x, self.state_y 
        
        # NEW: Timestamp buffers for post-processing alignment
        self.last_opti_sec = 0
        self.last_opti_nanosec = 0

        # --- 3. Optimized I/O Setup ---
        self.csv_filename = 'distributed_tracking_results_stamped.csv'
        self.csv_file = open(self.csv_filename, 'w', newline='')
        self.writer = csv.writer(self.csv_file)
        # Added 'nanosec' column for precision plotting
        self.writer.writerow(['sec', 'nanosec', 'odom_x', 'odom_y', 'lidar_x', 'lidar_y', 
                             'kf_x', 'kf_y', 'opti_tb2_x', 'opti_tb2_y', 'opti_tb1_x', 'opti_tb1_y'])
        
        # --- 4. Publishers/Subscribers ---
        self.pub_kf = self.create_publisher(PoseStamped, '/tracking/kalman_filter', 10)
        self.pub_raw_lidar = self.create_publisher(PoseStamped, '/tracking/raw_lidar', 10)
        
        qos_lidar = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)
        
        self.create_subscription(LaserScan, '/tb1/scan', self.lidar_callback, qos_lidar)
        self.create_subscription(Odometry, '/tb3/odom', self.odom_callback, 10)
        
        # MODIFICATION: Changed message type to PoseStamped and corrected topic names
        self.create_subscription(PoseStamped, '/tb1/optitrack_pose_stamped', self.opti_tb1_callback, 10)
        self.create_subscription(PoseStamped, '/tb3/optitrack_pose_stamped', self.opti_tb2_callback, 10)

        # --- 5. Logging Timer (20Hz) ---
        self.timer_period = 0.05 
        self.timer = self.create_timer(self.timer_period, self.timer_logging_callback)
        self.get_logger().info("Stamped Observer Initialized at 20Hz")

    def timer_logging_callback(self):
        """Dedicated logging function using high-precision timestamps."""
        self.writer.writerow([
            self.last_opti_sec, 
            self.last_opti_nanosec,
            self.raw_odom_x, self.raw_odom_y, 
            self.last_mx, self.last_my, 
            self.state_x, self.state_y, 
            self.true_tb2_x, self.true_tb2_y, 
            self.true_tb1_x, self.true_tb1_y
        ])

    def get_unicycle_prediction(self, msg):
        """Relative prediction logic."""
        dx_local = msg.pose.pose.position.x - self.last_odom_x
        dy_local = msg.pose.pose.position.y - self.last_odom_y
        
        dx = dx_local * math.cos(self.init_theta) - dy_local * math.sin(self.init_theta)
        dy = dx_local * math.sin(self.init_theta) + dy_local * math.cos(self.init_theta)
        
        self.state_x += dx
        self.state_y += dy
        self.raw_odom_x += dx
        self.raw_odom_y += dy
        self.p_matrix += self.q_process_noise
        
        self.last_odom_x = msg.pose.pose.position.x
        self.last_odom_y = msg.pose.pose.position.y

    def get_lidar_measurement(self, msg):
        """Clustering and offset correction."""
        points = []
        for i, dist in enumerate(msg.ranges):
            if dist < msg.range_min or dist > msg.range_max: continue
            angle = msg.angle_min + (i * msg.angle_increment)
            lx, ly = dist * math.cos(angle), dist * math.sin(angle)

            if math.sqrt((lx - self.state_x)**2 + (ly - self.state_y)**2) < 0.25:
                points.append((lx, ly))

        if len(points) > 2:
            rx = sum(p[0] for p in points) / len(points)
            ry = sum(p[1] for p in points) / len(points)
            d = math.sqrt(rx**2 + ry**2)
            offset = 0.045 
            return rx * (1 + offset/d), ry * (1 + offset/d)
        return None, None

    def perform_kalman_update(self, z_x, z_y):
        """Standard LKF update."""
        k = self.p_matrix / (self.p_matrix + self.r_measure_noise)
        self.state_x += k * (z_x - self.state_x)
        self.state_y += k * (z_y - self.state_y)
        self.p_matrix = (1 - k) * self.p_matrix

    # --- UPDATED CALLBACKS FOR STAMPED DATA ---
    def odom_callback(self, msg):
        self.get_unicycle_prediction(msg)

    def lidar_callback(self, msg):
        mx, my = self.get_lidar_measurement(msg)
        if mx is not None:
            self.last_mx, self.last_my = mx, my 
            self.perform_kalman_update(mx, my)
            self.pub_raw_lidar.publish(self.create_pose_msg(mx, my))

        self.pub_kf.publish(self.create_pose_msg(self.state_x, self.state_y))

    def opti_tb1_callback(self, msg):
        # Access nested PoseStamped structure
        self.true_tb1_x = msg.pose.position.x
        self.true_tb1_y = msg.pose.position.y
        # Store high-precision capture time
        self.last_opti_sec = msg.header.stamp.sec
        self.last_opti_nanosec = msg.header.stamp.nanosec

    def opti_tb2_callback(self, msg):
        # Access nested PoseStamped structure
        self.true_tb2_x = msg.pose.position.x
        self.true_tb2_y = msg.pose.position.y

    def create_pose_msg(self, x, y):
        m = PoseStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "tb1/odom"
        m.pose.position.x, m.pose.position.y = x, y
        return m

    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
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