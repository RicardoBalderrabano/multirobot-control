"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import numpy as np
import tf_transformations
import csv
import matplotlib.pyplot as plt
import signal
import sys
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class TB1BenchmarkHeading(Node):
    def __init__(self):
        super().__init__('tb1_benchmark_heading')
        
        self.opti_anchor_matrix = None
        self.initial_opti_yaw = None
        self.raw_odom_inverse_matrix = None
        self.data_log = [] 
        self.start_time = None

        # Current orientations (Heading)
        self.current_opti_yaw = 0.0
        self.current_ekf_yaw = 0.0
        self.current_raw_yaw = 0.0
        
        self.current_opti_pos = [0.0, 0.0]
        self.current_ekf_pos = [0.0, 0.0]
        self.current_raw_pos = [0.0, 0.0]

        # Inside __init__
        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Update the subscription line
        self.create_subscription(PoseStamped, '/tb1/pose', self.opti_callback, qos_best_effort)

        # Subscriptions
        
        self.create_subscription(PoseStamped, '/tb1/global_pose_estimated', self.ekf_callback, 10)        
        self.create_subscription(Odometry, '/tb1/odom', self.raw_odom_callback, 10)

        self.get_logger().info("Benchmark Active: Tracking Position AND Heading.")

    def get_yaw(self, q_obj):
        
        q = [q_obj.x, q_obj.y, q_obj.z, q_obj.w]
        _, _, yaw = tf_transformations.euler_from_quaternion(q)
        return yaw

    def opti_callback(self, msg):
        yaw = self.get_yaw(msg.pose.orientation)
        if self.opti_anchor_matrix is None:
            q = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
            self.opti_anchor_matrix = np.array([[np.cos(yaw), -np.sin(yaw), msg.pose.position.x], [np.sin(yaw), np.cos(yaw), msg.pose.position.y], [0, 0, 1]])
            self.initial_opti_yaw = yaw
            self.start_time = self.get_clock().now().nanoseconds / 1e9
        
        self.current_opti_yaw = yaw
        self.current_opti_pos = [msg.pose.position.x, msg.pose.position.y]

    def raw_odom_callback(self, msg):
        if self.opti_anchor_matrix is None: return
        
        # 1. Position Transformation
        if self.raw_odom_inverse_matrix is None:
            q = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
            m_start = np.array([[np.cos(self.get_yaw(msg.pose.pose.orientation)), -np.sin(self.get_yaw(msg.pose.pose.orientation)), msg.pose.pose.position.x], [np.sin(self.get_yaw(msg.pose.pose.orientation)), np.cos(self.get_yaw(msg.pose.pose.orientation)), msg.pose.pose.position.y], [0, 0, 1]])
            self.raw_odom_inverse_matrix = np.linalg.inv(m_start)

        # Apply matrix
        q_curr = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        m_curr = np.array([[np.cos(self.get_yaw(msg.pose.pose.orientation)), -np.sin(self.get_yaw(msg.pose.pose.orientation)), msg.pose.pose.position.x], [np.sin(self.get_yaw(msg.pose.pose.orientation)), np.cos(self.get_yaw(msg.pose.pose.orientation)), msg.pose.pose.position.y], [0, 0, 1]])
        rel = np.dot(self.raw_odom_inverse_matrix, m_curr)
        glob = np.dot(self.opti_anchor_matrix, rel)
        self.current_raw_pos = [glob[0, 2], glob[1, 2]]
        
        # 2. Heading Alignment (Local + Initial Offset)
        self.current_raw_yaw = self.get_yaw(msg.pose.pose.orientation) + self.initial_opti_yaw

    def ekf_callback(self, msg):
        self.current_ekf_yaw = self.get_yaw(msg.pose.orientation)
        self.current_ekf_pos = [msg.pose.position.x, msg.pose.position.y]
        self.log_step()

    def log_step(self):
        if self.start_time is None or self.raw_odom_inverse_matrix is None: return
        elapsed = (self.get_clock().now().nanoseconds / 1e9) - self.start_time
        self.data_log.append([elapsed, 
                             self.current_opti_pos[0], self.current_opti_pos[1], self.current_opti_yaw,
                             self.current_ekf_pos[0], self.current_ekf_pos[1], self.current_ekf_yaw,
                             self.current_raw_pos[0], self.current_raw_pos[1], self.current_raw_yaw])

    def finalize_and_plot(self):
            if not self.data_log:
                print("No data recorded.")
                return

            data = np.array(self.data_log)
            time = data[:, 0]
            
            # --- POSITION RMSE CALCULATION ---
            rmse_pos_ekf = np.sqrt(np.mean((data[:,1] - data[:,4])**2 + (data[:,2] - data[:,5])**2))
            rmse_pos_odom = np.sqrt(np.mean((data[:,1] - data[:,7])**2 + (data[:,2] - data[:,8])**2))

            # --- HEADING RMSE CALCULATION ---
            # We calculate the shortest angular difference to be precise
            def get_angular_diff(target, current):
                diff = target - current
                return (diff + np.pi) % (2 * np.pi) - np.pi

            yaw_diff_ekf = get_angular_diff(data[:,3], data[:,6])
            yaw_diff_odom = get_angular_diff(data[:,3], data[:,9])
            
            rmse_yaw_ekf = np.sqrt(np.mean(yaw_diff_ekf**2))
            rmse_yaw_odom = np.sqrt(np.mean(yaw_diff_odom**2))

            # Convert to Degrees for the report
            rmse_yaw_ekf_deg = np.degrees(rmse_yaw_ekf)
            rmse_yaw_odom_deg = np.degrees(rmse_yaw_odom)

            # --- TERMINAL REPORT ---
            print("\n" + "="*45)
            print(f"{'METRIC':<20} | {'EKF':<10} | {'ODOM':<10}")
            print("-" * 45)
            print(f"{'Pos RMSE (m)':<20} | {rmse_pos_ekf:.4f}     | {rmse_pos_odom:.4f}")
            print(f"{'Yaw RMSE (deg)':<20} | {rmse_yaw_ekf_deg:.4f}     | {rmse_yaw_odom_deg:.4f}")
            print("="*45 + "\n")

            # --- PLOTTING ---
            fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 12))
            
            # Position Plot
            ax1.plot(data[:, 1], data[:, 2], 'k--', label=f'Truth')
            ax1.plot(data[:, 4], data[:, 5], 'b-', label=f'EKF (RMSE: {rmse_pos_ekf:.3f}m)')
            ax1.plot(data[:, 7], data[:, 8], 'r:', label=f'Odom (RMSE: {rmse_pos_odom:.3f}m)')
            ax1.set_title("XY Position Comparison")
            ax1.legend(); ax1.axis('equal'); ax1.grid(True)

            # Heading Plot
            opti_h = np.degrees(np.unwrap(data[:, 3]))
            ekf_h = np.degrees(np.unwrap(data[:, 6]))
            odom_h = np.degrees(np.unwrap(data[:, 9]))

            ax2.plot(time, opti_h, 'k--', label='Truth')
            ax2.plot(time, ekf_h, 'b-', label=f'EKF (RMSE: {rmse_yaw_ekf_deg:.2f}°)')
            ax2.plot(time, odom_h, 'r:', label=f'Odom (RMSE: {rmse_yaw_odom_deg:.2f}°)')
            ax2.set_title("Heading Comparison")
            ax2.set_ylabel("Degrees"); ax2.legend(); ax2.grid(True)
            
            plt.tight_layout()
            plt.show()

def main():
    rclpy.init()
    node = TB1BenchmarkHeading()
    signal.signal(signal.SIGINT, lambda s, f: (node.finalize_and_plot(), rclpy.shutdown(), sys.exit(0)))
    try: rclpy.spin(node)
    except: node.finalize_and_plot()
    finally: rclpy.shutdown()

if __name__ == '__main__': main()
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import numpy as np
import tf_transformations
import matplotlib.pyplot as plt
import signal
import sys
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class TB1BenchmarkInterpolated(Node):
    def __init__(self):
        super().__init__('tb1_benchmark_interpolated')
        
        # Buffers and State
        self.opti_buffer = [] 
        self.buffer_max_size = 300 
        self.opti_anchor_matrix = None
        self.initial_opti_yaw = None
        self.raw_odom_inverse_matrix = None
        self.data_log = [] 
        self.start_time = None
        
        # Latest known values
        self.current_raw_pos = None # Start as None to detect initialization
        self.current_raw_yaw = None

        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscriptions
        self.create_subscription(PoseStamped, '/tb1/pose', self.opti_callback, qos_best_effort)
        self.create_subscription(PoseStamped, '/tb1/global_pose_estimated', self.ekf_callback, 10)        
        self.create_subscription(Odometry, '/tb1/odom', self.raw_odom_callback, 10)

        self.get_logger().info("Benchmark Active: Waiting for all sensors to align...")

    def get_yaw(self, q_obj):
        q = [q_obj.x, q_obj.y, q_obj.z, q_obj.w]
        _, _, yaw = tf_transformations.euler_from_quaternion(q)
        return yaw

    def opti_callback(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        yaw = self.get_yaw(msg.pose.orientation)
        
        self.opti_buffer.append((t, msg.pose.position.x, msg.pose.position.y, yaw))
        if len(self.opti_buffer) > self.buffer_max_size:
            self.opti_buffer.pop(0)

        # Initial Global Anchor
        if self.opti_anchor_matrix is None:
            self.opti_anchor_matrix = np.array([
                [np.cos(yaw), -np.sin(yaw), msg.pose.position.x],
                [np.sin(yaw),  np.cos(yaw), msg.pose.position.y],
                [0, 0, 1]
            ])
            self.initial_opti_yaw = yaw

    def interpolate_opti(self, target_t):
        if len(self.opti_buffer) < 2: return None
        
        buffer_start = self.opti_buffer[0][0]
        buffer_end = self.opti_buffer[-1][0]
        
        if target_t < buffer_start or target_t > buffer_end:
            return None

        for i in range(len(self.opti_buffer) - 1):
            t1, x1, y1, yaw1 = self.opti_buffer[i]
            t2, x2, y2, yaw2 = self.opti_buffer[i+1]
            
            if t1 <= target_t <= t2:
                factor = (target_t - t1) / (t2 - t1)
                interp_x = x1 + factor * (x2 - x1)
                interp_y = y1 + factor * (y2 - y1)
                diff = (yaw2 - yaw1 + np.pi) % (2 * np.pi) - np.pi
                interp_yaw = yaw1 + factor * diff
                return interp_x, interp_y, interp_yaw
        return None

    def raw_odom_callback(self, msg):
        # Ignore Odom until OptiTrack has defined the world origin
        if self.opti_anchor_matrix is None: return
        
        yaw_now = self.get_yaw(msg.pose.pose.orientation)
        m_curr = np.array([[np.cos(yaw_now), -np.sin(yaw_now), msg.pose.pose.position.x], 
                           [np.sin(yaw_now), np.cos(yaw_now), msg.pose.pose.position.y], 
                           [0, 0, 1]])

        if self.raw_odom_inverse_matrix is None:
            self.raw_odom_inverse_matrix = np.linalg.inv(m_curr)

        rel = np.dot(self.raw_odom_inverse_matrix, m_curr)
        glob = np.dot(self.opti_anchor_matrix, rel)
        
        self.current_raw_pos = [glob[0, 2], glob[1, 2]]
        self.current_raw_yaw = yaw_now + self.initial_opti_yaw

    def ekf_callback(self, msg):
        # Only log once we have OptiTrack AND transformed Odometry ready
        if self.opti_anchor_matrix is None or self.current_raw_pos is None:
            return
        
        t_ekf = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        truth = self.interpolate_opti(t_ekf)
        
        if truth is not None:
            # Sync the plot start to the first valid measurement
            if self.start_time is None:
                self.start_time = t_ekf
                self.get_logger().info("Synchronization Successful. Recording Data...")
                
            tx, ty, tyaw = truth
            ekf_yaw = self.get_yaw(msg.pose.orientation)
            elapsed = t_ekf - self.start_time
            
            self.data_log.append([
                elapsed, tx, ty, tyaw,
                msg.pose.position.x, msg.pose.position.y, ekf_yaw,
                self.current_raw_pos[0], self.current_raw_pos[1], self.current_raw_yaw
            ])

    def finalize_and_plot(self):
        if not self.data_log:
            print("[ERROR] No synced data. Did you drive the robot?")
            return
            
        data = np.array(self.data_log)
        
        # Calculate RMSE
        rmse_pos_ekf = np.sqrt(np.mean((data[:,1]-data[:,4])**2 + (data[:,2]-data[:,5])**2))
        rmse_pos_odom = np.sqrt(np.mean((data[:,1]-data[:,7])**2 + (data[:,2]-data[:,8])**2))
        
        def ang_diff(a, b): return (a - b + np.pi) % (2 * np.pi) - np.pi
        rmse_yaw_ekf = np.sqrt(np.mean(ang_diff(data[:,3], data[:,6])**2))
        rmse_yaw_odom = np.sqrt(np.mean(ang_diff(data[:,3], data[:,9])**2))
        
        print(f"\nFINAL METRICS")
        print(f"EKF Pos RMSE: {rmse_pos_ekf:.4f}m | Yaw RMSE: {np.degrees(rmse_yaw_ekf):.2f}°")
        print(f"Odom Pos RMSE: {rmse_pos_odom:.4f}m | Yaw RMSE: {np.degrees(rmse_yaw_odom):.2f}°")

        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 12))
        
        # XY Position
        ax1.plot(data[:, 1], data[:, 2], 'k--', label='Ground Truth')
        ax1.plot(data[:, 4], data[:, 5], 'b-', label='EKF Filtered')
        ax1.plot(data[:, 7], data[:, 8], 'r:', label='Raw Odometry')
        ax1.set_title("XY Position Comparison (Time & Start Aligned)")
        ax1.legend(); ax1.axis('equal'); ax1.grid(True)

        # Heading
        ax2.plot(data[:,0], np.degrees(np.unwrap(data[:,3])), 'k--', label='Truth')
        ax2.plot(data[:,0], np.degrees(np.unwrap(data[:,6])), 'b-', label='EKF')
        ax2.plot(data[:,0], np.degrees(np.unwrap(data[:,9])), 'r:', label='Raw Odom')
        ax2.set_title("Heading (Yaw) Comparison")
        ax2.set_xlabel("Time (s)"); ax2.set_ylabel("Degrees"); ax2.legend(); ax2.grid(True)
        
        plt.tight_layout()
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    node = TB1BenchmarkInterpolated()
    signal.signal(signal.SIGINT, lambda s, f: (node.finalize_and_plot(), rclpy.shutdown(), sys.exit(0)))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()