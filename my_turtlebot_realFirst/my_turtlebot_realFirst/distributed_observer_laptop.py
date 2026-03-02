"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import math
import csv

class KalmanObserver(Node):
    def __init__(self):
        super().__init__('kalman_observer')
        
        # --- 1. Coordinate & Orientation Setup ---
        # Starting position of TB2 relative to TB1
        self.state_x = 0.39 
        self.state_y = -0.03
        
        # Heading of TB2 relative to TB1 
        # (3.14159 if facing each other, 0.0 if facing same way)
        self.init_theta = -3.13 
        
        # --- 2. Kalman Filter Variables ---
        self.p_matrix = 0.1             # Initial uncertainty
        self.q_process_noise = 0.01     # Trust in Odometry
        self.r_measure_noise = 0.05     # Trust in LIDAR
        
        self.last_odom_x = 0.0
        self.last_odom_y = 0.0
        self.raw_odom_x = self.state_x  # Transformed Odom for comparison
        self.raw_odom_y = self.state_y

        # --- 3. Communication & Logging ---
        self.csv_filename = 'tracking_results_aligned.csv'
        self.init_csv()
        
        self.pub_kf = self.create_publisher(PoseStamped, '/tracking/kalman_filter', 10)
        self.pub_raw_lidar = self.create_publisher(PoseStamped, '/tracking/raw_lidar', 10)
        self.pub_odom_only = self.create_publisher(PoseStamped, '/tracking/odom_only', 10)

        qos_lidar = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10)

        self.lidar_sub = self.create_subscription(LaserScan, '/tb1/scan', self.lidar_callback, qos_lidar)
        self.odom_sub = self.create_subscription(Odometry, '/tb2/odom', self.odom_callback, 10)
        
        self.get_logger().info('Kalman Observer Aligned Node Started.')

    def init_csv(self):
        with open(self.csv_filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['timestamp', 'odom_x', 'odom_y', 'lidar_x', 'lidar_y', 'kf_x', 'kf_y'])

    def create_pose_msg(self, x, y):

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "tb1/odom"
        msg.pose.position.x = x
        msg.pose.position.y = y
        return msg

    def odom_callback(self, msg):
        # 1. Get local delta movement from TB2
        dx_local = msg.pose.pose.position.x - self.last_odom_x
        dy_local = msg.pose.pose.position.y - self.last_odom_y
        
        # 2. APPLY ROTATION MATRIX (Transforming TB2 movement to TB1 frame)
        # This aligns the coordinate systems so "Forward" is the same for both.
        dx = dx_local * math.cos(self.init_theta) - dy_local * math.sin(self.init_theta)
        dy = dx_local * math.sin(self.init_theta) + dy_local * math.cos(self.init_theta)
        
        # 3. Predict Step: Update Kalman State with Odometry
        self.state_x += dx
        self.state_y += dy
        self.p_matrix += self.q_process_noise
        
        # 4. Update Transformed Odom Path (for mirroring fix in plot)
        self.raw_odom_x += dx
        self.raw_odom_y += dy
        
        # Store for next delta
        self.last_odom_x = msg.pose.pose.position.x
        self.last_odom_y = msg.pose.pose.position.y
        
        # Publish Odom-Only for visual comparison
        self.pub_odom_only.publish(self.create_pose_msg(self.raw_odom_x, self.raw_odom_y))

    def lidar_callback(self, msg):
        # Clustering: Find LIDAR points near the current predicted state
        detected_points = []
        for i, distance in enumerate(msg.ranges):
            if distance < msg.range_min or distance > msg.range_max: continue
            angle = msg.angle_min + (i * msg.angle_increment)
            lx = distance * math.cos(angle)
            ly = distance * math.sin(angle)
            
            if math.sqrt((lx - self.state_x)**2 + (ly - self.state_y)**2) < 0.25:
                detected_points.append((lx, ly))

        if len(detected_points) > 2:
            # Observation Centroid (The raw measurement Z)
            meas_x = sum(p[0] for p in detected_points) / len(detected_points)
            meas_y = sum(p[1] for p in detected_points) / len(detected_points)
            
            # Publish raw measurement for RViz
            self.pub_raw_lidar.publish(self.create_pose_msg(meas_x, meas_y))

            # --- Update Step: The Kalman Correction ---
            # 1. Calculate Kalman Gain
            k_gain = self.p_matrix / (self.p_matrix + self.r_measure_noise)
            
            # 2. Adjust State based on LIDAR measurement
            self.state_x += k_gain * (meas_x - self.state_x)
            self.state_y += k_gain * (meas_y - self.state_y)
            
            # 3. Update Covariance (Uncertainty)
            self.p_matrix = (1 - k_gain) * self.p_matrix
            
            # --- Logging & Visualizing ---
            self.get_logger().info(f'POS: ({self.state_x:.2f}, {self.state_y:.2f}) | Gain: {k_gain:.2f}')
            self.pub_kf.publish(self.create_pose_msg(self.state_x, self.state_y))
            self.log_to_csv(meas_x, meas_y)
        else:
            self.get_logger().warn('TB2 Tracking Lost (LIDAR)! Relying on Odometry prediction.')

    def log_to_csv(self, lx, ly):
        with open(self.csv_filename, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                self.get_clock().now().to_msg().sec, 
                self.raw_odom_x, self.raw_odom_y, 
                lx, ly, 
                self.state_x, self.state_y
            ])

def main(args=None):
    rclpy.init(args=args)
    node = KalmanObserver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

"""
"""
last working, noisy
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Pose2D # Added Pose2D
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import math
import csv

class KalmanObserver(Node):
    def __init__(self):
        super().__init__('kalman_observer')
        
        # --- 1. Coordinate & Orientation Setup ---
        # Updated with your precise OptiTrack initial values
        self.state_x = -0.039
        self.state_y = -0.37
        self.init_theta = 1.56
        
        # --- 2. Kalman Filter Variables ---
        self.p_matrix = 0.01             
        self.q_process_noise = 0.0001  
        self.r_measure_noise = 2.0     


        self.last_odom_x = 0.0
        self.last_odom_y = 0.0
        self.raw_odom_x = self.state_x  
        self.raw_odom_y = self.state_y

        # --- 3. Ground Truth Variables ---
        self.true_x = self.state_x # Default to start pos until first OptiTrack msg
        self.true_y = self.state_y

        # Buffer for Moving Average Filter
        self.lidar_buffer_x = []
        self.lidar_buffer_y = []
        self.window_size = 15 # Smoothing window size

        # --- 4. Communication & Logging ---
        self.csv_filename = 'tracking_results_with_optitrack20.csv'
        self.init_csv()
        
        self.pub_kf = self.create_publisher(PoseStamped, '/tracking/kalman_filter', 10)
        self.pub_raw_lidar = self.create_publisher(PoseStamped, '/tracking/raw_lidar', 10)
        self.pub_odom_only = self.create_publisher(PoseStamped, '/tracking/odom_only', 10)

        qos_lidar = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10)

        # Subscribers
        self.lidar_sub = self.create_subscription(LaserScan, '/tb1/scan', self.lidar_callback, qos_lidar)
        self.odom_sub = self.create_subscription(Odometry, '/tb2/odom', self.odom_callback, 10)
        
        # New: Subscription to the OptiTrack Bridge for Ground Truth
        self.opti_sub = self.create_subscription(
            Pose2D, 
            '/tb2/optitrack_pose2d', 
            self.opti_callback, 
            10)
        
        self.get_logger().info('Kalman Observer with Ground Truth Logging Started.')

    def init_csv(self):
        with open(self.csv_filename, 'w', newline='') as f:
            writer = csv.writer(f)
            # Added opti_x and opti_y to the header
            writer.writerow(['timestamp', 'odom_x', 'odom_y', 'lidar_x', 'lidar_y', 'kf_x', 'kf_y', 'opti_x', 'opti_y'])

    def opti_callback(self, msg):
        self.true_x = msg.x
        self.true_y = msg.y

    def create_pose_msg(self, x, y):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "tb1/odom"
        msg.pose.position.x = x
        msg.pose.position.y = y
        return msg

    def odom_callback(self, msg):
        dx_local = msg.pose.pose.position.x - self.last_odom_x
        dy_local = msg.pose.pose.position.y - self.last_odom_y
        
        # Rotation Matrix alignment
        dx = dx_local * math.cos(self.init_theta) - dy_local * math.sin(self.init_theta)
        dy = dx_local * math.sin(self.init_theta) + dy_local * math.cos(self.init_theta)
        
        self.state_x += dx
        self.state_y += dy
        self.p_matrix += self.q_process_noise
        
        self.raw_odom_x += dx
        self.raw_odom_y += dy
        
        self.last_odom_x = msg.pose.pose.position.x
        self.last_odom_y = msg.pose.pose.position.y
        
        self.pub_odom_only.publish(self.create_pose_msg(self.raw_odom_x, self.raw_odom_y))

    def lidar_callback(self, msg):
        detected_points = []
        for i, distance in enumerate(msg.ranges):
            if distance < msg.range_min or distance > msg.range_max: continue
            angle = msg.angle_min + (i * msg.angle_increment)
            lx = distance * math.cos(angle)
            ly = distance * math.sin(angle)
                
            if math.sqrt((lx - self.state_x)**2 + (ly - self.state_y)**2) < 0.25:
                detected_points.append((lx, ly))


        if len(detected_points) > 2:
            raw_x = sum(p[0] for p in detected_points) / len(detected_points)
            raw_y = sum(p[1] for p in detected_points) / len(detected_points)
                
                # 1. REMOVE the constant +0.049 Y-offset. 
                # 2. Use a single Radial Correction Factor.
                # If the Lidar "circle" is too small, we multiply by a factor > 1.0.
                # Based on your logs, the Lidar circle is roughly 5cm too small in radius.
            dist_to_surf = math.sqrt(raw_x**2 + raw_y**2)
                
                # This factor scales the X and Y proportionally based on distance.
                # Try 0.035m (3.5cm) to expand the circle to match the Black line.
            radial_offset = 0.035 
                
            cx = raw_x * (1 + radial_offset / dist_to_surf)
            cy = raw_y * (1 + radial_offset / dist_to_surf)
                
                # --- MOVING AVERAGE FILTER ---
            self.lidar_buffer_x.append(cx)
            self.lidar_buffer_y.append(cy)
            if len(self.lidar_buffer_x) > self.window_size:
                self.lidar_buffer_x.pop(0)
                self.lidar_buffer_y.pop(0)
                
            meas_x = sum(self.lidar_buffer_x) / len(self.lidar_buffer_x)
            meas_y = sum(self.lidar_buffer_y) / len(self.lidar_buffer_y)

                # This should now show X close to 0.458 and Y close to 0.017
            self.get_logger().info(f'PRECISION MEAS: x={meas_x:.4f}, y={meas_y:.4f}')

                # --- KALMAN UPDATE ---
            k_gain = self.p_matrix / (self.p_matrix + self.r_measure_noise)
            self.state_x += k_gain * (meas_x - self.state_x)
            self.state_y += k_gain * (meas_y - self.state_y)
            self.p_matrix = (1 - k_gain) * self.p_matrix
                
            self.pub_kf.publish(self.create_pose_msg(self.state_x, self.state_y))
            self.log_to_csv(meas_x, meas_y)

    def log_to_csv(self, lx, ly):
        with open(self.csv_filename, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                self.get_clock().now().to_msg().sec, 
                self.raw_odom_x, self.raw_odom_y, 
                lx, ly, 
                self.state_x, self.state_y,
                self.true_x, self.true_y
            ])

def main(args=None):
    rclpy.init(args=args)
    node = KalmanObserver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

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
        self.state_x = 0.39
        self.state_y = 0.006
        self.init_theta = 3.1 # Orientation of TB2 relative to TB1
        
        # --- 2. Filter Gains ---
        self.p_matrix = 0.01             
        self.q_process_noise = 0.0001  
        self.r_measure_noise = 5.0     

        self.last_odom_x = 0.0
        self.last_odom_y = 0.0
        self.raw_odom_x = self.state_x  
        self.raw_odom_y = self.state_y
        self.true_x, self.true_y = self.state_x, self.state_y

        # --- 3. Logging Setup ---
        self.csv_filename = 'tracking_results_with_optitrack35.csv'
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
        """Updates position based on the unicycle motion model."""
        dx_local = msg.pose.pose.position.x - self.last_odom_x
        dy_local = msg.pose.pose.position.y - self.last_odom_y
        
        # Unicycle model: Rotate local displacement into observer frame
        dx = dx_local * math.cos(self.init_theta) - dy_local * math.sin(self.init_theta)
        dy = dx_local * math.sin(self.init_theta) + dy_local * math.cos(self.init_theta)
        
        self.state_x += dx
        self.state_y += dy
        self.raw_odom_x += dx
        self.raw_odom_y += dy
        self.p_matrix += self.q_process_noise
        
        self.last_odom_x = msg.pose.pose.position.x
        self.last_odom_y = msg.pose.pose.position.y
        return self.state_x, self.state_y

    # --- MODULE 2: LIDAR CENTROID (NO FILTER) ---
    def get_lidar_measurement(self, msg):
        """Calculates the raw geometric centroid of detected points."""
        points = []
        for i, dist in enumerate(msg.ranges):
            if dist < msg.range_min or dist > msg.range_max: continue
            angle = msg.angle_min + (i * msg.angle_increment)
            lx, ly = dist * math.cos(angle), dist * math.sin(angle)
            
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

    # --- MODULE 3: KALMAN FUSION ---
    def perform_kalman_update(self, z_x, z_y):
        """Fuses the prediction and measurement."""
        k = self.p_matrix / (self.p_matrix + self.r_measure_noise)
        self.state_x += k * (z_x - self.state_x)
        self.state_y += k * (z_y - self.state_y)
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
            
            # Global Real-Time Log
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