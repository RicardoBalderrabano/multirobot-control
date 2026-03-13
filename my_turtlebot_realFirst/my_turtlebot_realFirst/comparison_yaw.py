import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np
import math
import csv
import matplotlib.pyplot as plt
from datetime import datetime

class YawComparator(Node):
    def __init__(self):
        super().__init__('yaw_comparator')
        
        self.data_log = []
        self.start_time = None

        # QoS for OptiTrack (Best Effort)
        qos_opti = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.create_subscription(Odometry, '/tb1/odom', self.odom_callback, 10)
        self.create_subscription(Odometry, '/tb1/odometry/filtered', self.ekf_callback, 10)
        self.create_subscription(PoseStamped, '/tb1/pose', self.optitrack_callback, qos_opti)

        self.current_yaw_odom = 0.0
        self.current_yaw_ekf = 0.0
        self.current_yaw_opti = 0.0

        self.create_timer(0.1, self.log_data)
        self.get_logger().info('Logging with RMSE calculation. Press Ctrl+C to stop.')

    def quaternion_to_yaw(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.degrees(math.atan2(siny_cosp, cosy_cosp))

    def odom_callback(self, msg):
        self.current_yaw_odom = self.quaternion_to_yaw(msg.pose.pose.orientation)

    def ekf_callback(self, msg):
        self.current_yaw_ekf = self.quaternion_to_yaw(msg.pose.pose.orientation)

    def optitrack_callback(self, msg):
        self.current_yaw_opti = self.quaternion_to_yaw(msg.pose.orientation)

    def log_data(self):
        now = self.get_clock().now()
        if self.start_time is None: self.start_time = now
        rel_time = (now - self.start_time).nanoseconds / 1e9
        self.data_log.append([rel_time, self.current_yaw_odom, self.current_yaw_ekf, self.current_yaw_opti])

    def calculate_rmse(self, predictions, ground_truth):
        # Handle angle wrapping (differences like 359 vs 1 degree)
        diff = np.mod(predictions - ground_truth + 180, 360) - 180
        return np.sqrt(np.mean(diff**2))

    def save_and_plot(self):
        data = np.array(self.data_log)
        if len(data) == 0: return

        # Calculate RMSE
        rmse_odom = self.calculate_rmse(data[:, 1], data[:, 3])
        rmse_ekf = self.calculate_rmse(data[:, 2], data[:, 3])

        # Plotting
        plt.figure(figsize=(12, 7))
        plt.plot(data[:, 0], data[:, 1], label=f'Raw Odom (RMSE: {rmse_odom:.2f}°)', alpha=0.6)
        plt.plot(data[:, 0], data[:, 2], label=f'EKF Filtered (RMSE: {rmse_ekf:.2f}°)', linewidth=2)
        plt.plot(data[:, 0], data[:, 3], label='OptiTrack (Ground Truth)', linestyle='--', color='green')
        
        plt.xlabel('Time (seconds)')
        plt.ylabel('Yaw Angle (degrees)')
        plt.title('TurtleBot 3 Orientation: Performance Analysis')
        plt.legend(loc='upper right')
        plt.grid(True)
        plt.savefig('yaw_rmse_analysis.png')
        plt.show()

def main():
    rclpy.init()
    node = YawComparator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.save_and_plot()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()