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
import matplotlib.pyplot as plt

class KalmanObserver(Node):
    def __init__(self):
        super().__init__('kalman_observer')
        
        # --- 1. Global State Setup ---
        self.state_x = 0.0 
        self.state_y = 0.0
        self.tb1_global_x = 0.0
        self.tb1_global_y = 0.0
        self.tb1_global_yaw = 0.0
        self.last_tb3_global_x = None
        self.last_tb3_global_y = None
        self.gt_tb1_x, self.gt_tb1_y = 0.0, 0.0
        self.gt_tb3_x, self.gt_tb3_y = 0.0, 0.0

        # --- 2. Filter Gains ---
        self.p_matrix = 0.1
        self.q_process_noise = 0.001
        self.r_measure_noise = 0.5 

        # --- 3. Real-Time Plotting Buffers ---
        self.plot_x_kf, self.plot_y_kf = [], []
        self.plot_x_gt, self.plot_y_gt = [], []
        self.plot_x_obs, self.plot_y_obs = [], []
        
        # Initialize Matplotlib
        plt.ion() # Interactive mode on
        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        self.line_kf, = self.ax.plot([], [], 'r:', label='KF Estimate', linewidth=2)
        self.line_gt, = self.ax.plot([], [], 'g-', label='Target GT', linewidth=2)
        self.line_obs, = self.ax.plot([], [], 'k--', label='Observer GT', alpha=0.4)
        self.ax.set_xlabel('Global X [m]')
        self.ax.set_ylabel('Global Y [m]')
        self.ax.set_title('Real-Time Global Tracking')
        self.ax.legend()
        self.ax.grid(True)

        # --- 4. ROS Setup ---
        self.pub_kf = self.create_publisher(PoseStamped, '/tb3/kalman', 10)
        qos_best_effort = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)

        self.create_subscription(PoseStamped, '/tb1/pose', self.gt_tb1_callback, qos_best_effort)
        self.create_subscription(PoseStamped, '/tb3/pose', self.gt_tb3_callback, qos_best_effort)
        self.create_subscription(PoseStamped, '/tb1/global_pose_estimated', self.tb1_global_callback, 10)
        self.create_subscription(PoseStamped, '/tb3/global_pose_estimated', self.tb3_global_callback, 10)
        self.create_subscription(LaserScan, '/tb1/scan', self.lidar_callback, qos_best_effort)

        self.csv_filename = 'global_tracking_results2.csv'
        self.init_csv()

    def gt_tb1_callback(self, msg):
        self.gt_tb1_x, self.gt_tb1_y = msg.pose.position.x, msg.pose.position.y

    def gt_tb3_callback(self, msg):
        self.gt_tb3_x, self.gt_tb3_y = msg.pose.position.x, msg.pose.position.y

    def tb1_global_callback(self, msg):
        self.tb1_global_x, self.tb1_global_y = msg.pose.position.x, msg.pose.position.y
        q = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        _, _, self.tb1_global_yaw = tf_transformations.euler_from_quaternion(q)

    def tb3_global_callback(self, msg):
        curr_x, curr_y = msg.pose.position.x, msg.pose.position.y
        if self.last_tb3_global_x is not None:
            self.state_x += (curr_x - self.last_tb3_global_x)
            self.state_y += (curr_y - self.last_tb3_global_y)
            self.p_matrix += self.q_process_noise
        else:
            self.state_x, self.state_y = curr_x, curr_y
        self.last_tb3_global_x, self.last_tb3_global_y = curr_x, curr_y

    def lidar_callback(self, msg):
        rx_rel, ry_rel = self.get_lidar_measurement(msg)
        if rx_rel is not None:
            gx = self.tb1_global_x + (rx_rel * math.cos(self.tb1_global_yaw) - ry_rel * math.sin(self.tb1_global_yaw))
            gy = self.tb1_global_y + (rx_rel * math.sin(self.tb1_global_yaw) + ry_rel * math.cos(self.tb1_global_yaw))
            self.perform_kalman_update(gx, gy)
            self.log_to_csv(gx, gy)
            self.update_plot() # Real-time plot trigger

        self.pub_kf.publish(self.create_pose_msg(self.state_x, self.state_y))

    def update_plot(self):
        # Append data for plotting
        self.plot_x_kf.append(self.state_x)
        self.plot_y_kf.append(self.state_y)
        self.plot_x_gt.append(self.gt_tb3_x)
        self.plot_y_gt.append(self.gt_tb3_y)
        self.plot_x_obs.append(self.gt_tb1_x)
        self.plot_y_obs.append(self.gt_tb1_y)

        # Limit buffer size to keep plot fast (last 100 points)
        if len(self.plot_x_kf) > 100:
            self.plot_x_kf.pop(0); self.plot_y_kf.pop(0)
            self.plot_x_gt.pop(0); self.plot_y_gt.pop(0)
            self.plot_x_obs.pop(0); self.plot_y_obs.pop(0)

        # Update lines
        self.line_kf.set_data(self.plot_x_kf, self.plot_y_kf)
        self.line_gt.set_data(self.plot_x_gt, self.plot_y_gt)
        self.line_obs.set_data(self.plot_x_obs, self.plot_y_obs)

        # Auto-rescale axis
        self.ax.relim()
        self.ax.autoscale_view()
        
        # Non-blocking pause
        plt.pause(0.001)

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
            csv.writer(f).writerow(['timestamp', 'kf_global_x', 'kf_global_y', 'gt_tb3_x', 'gt_tb3_y', 'gt_tb1_x', 'gt_tb1_y'])

    def log_to_csv(self, gx, gy):
        with open(self.csv_filename, 'a', newline='') as f:
            ts = self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec * 1e-9
            csv.writer(f).writerow([ts, self.state_x, self.state_y, self.gt_tb3_x, self.gt_tb3_y, self.gt_tb1_x, self.gt_tb1_y])

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
