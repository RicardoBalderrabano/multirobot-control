import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
import math
import csv
import time
import os

# --- CONFIGURATION ---
ROBOT_TOPIC = '/tb1/scan'  # Change this to /tb2/scan or /tb3/scan as needed
CSV_FILENAME = 'lidar_experiment_data.csv'
CLUSTER_THRESHOLD = 0.10   # Meters. Points closer than this are the "same object"
MIN_OBJ_WIDTH = 0.05       # Filter out tiny noise (5cm)
MAX_OBJ_WIDTH = 0.50       # Filter out long walls (50cm)
# ---------------------

class LidarLogger(Node):
    def __init__(self):
        super().__init__('lidar_logger')

        # 1. Setup LiDAR Subscriber with Correct QoS
        qos_policy = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )
        self.subscription = self.create_subscription(
            LaserScan,
            ROBOT_TOPIC,
            self.scan_callback,
            qos_policy
        )

        # 2. Setup CSV Logging
        self.file_path = CSV_FILENAME
        # Write headers if file doesn't exist
        if not os.path.exists(self.file_path):
            with open(self.file_path, mode='w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['Timestamp', 'Distance_m', 'Width_m', 'Num_Points', 'Label'])
        
        self.start_time = time.time()
        self.get_logger().info(f"--- LOGGER STARTED ---")
        self.get_logger().info(f"Listening to: {ROBOT_TOPIC}")
        self.get_logger().info(f"Saving data to: {self.file_path}")

    def scan_callback(self, msg):
        # --- STEP 1: Process Raw Data into Points (X, Y) ---
        angle_min = msg.angle_min
        angle_inc = msg.angle_increment
        points = []

        # Convert valid ranges to (x,y) coordinates
        for i, dist in enumerate(msg.ranges):
            # Filter noise and infinite distances
            if 0.05 < dist < 3.5: 
                angle = angle_min + (i * angle_inc)
                x = dist * math.cos(angle)
                y = dist * math.sin(angle)
                points.append([x, y, dist])

        if not points:
            return

        # --- STEP 2: Clustering (Group nearby points) ---
        clusters = []
        current_cluster = [points[0]]

        for i in range(1, len(points)):
            prev_p = points[i-1]
            curr_p = points[i]
            
            # Euclidean distance between consecutive points
            dist_between = math.sqrt((curr_p[0] - prev_p[0])**2 + (curr_p[1] - prev_p[1])**2)

            if dist_between < CLUSTER_THRESHOLD:
                current_cluster.append(curr_p)
            else:
                clusters.append(current_cluster)
                current_cluster = [curr_p]
        
        # Append the last cluster
        if current_cluster:
            clusters.append(current_cluster)

        # --- STEP 3: Analyze Clusters ---
        closest_dist = float('inf')
        target_cluster = None
        target_width = 0.0

        for cluster in clusters:
            if len(cluster) < 3: continue # Ignore 1 or 2 dot noise

            # Calculate Width (Distance between first and last point)
            p_first = cluster[0]
            p_last = cluster[-1]
            width = math.sqrt((p_first[0]-p_last[0])**2 + (p_first[1]-p_last[1])**2)

            # Filter by size (Must look like a robot, not a wall)
            if MIN_OBJ_WIDTH < width < MAX_OBJ_WIDTH:
                # Find center distance of this cluster
                avg_dist = sum(p[2] for p in cluster) / len(cluster)

                # Keep the closest object found
                if avg_dist < closest_dist:
                    closest_dist = avg_dist
                    target_cluster = cluster
                    target_width = width

        # --- STEP 4: Log and Print Data ---
        if target_cluster:
            num_points = len(target_cluster)
            current_time = time.time() - self.start_time
            
            # Print to Terminal
            print(f"[Time: {current_time:.1f}s] TARGET FOUND! "
                  f"Dist: {closest_dist:.3f}m | Width: {target_width:.3f}m | Points: {num_points}")

            # Save to CSV
            with open(self.file_path, mode='a', newline='') as f:
                writer = csv.writer(f)
                # 'Label' is empty for now, you can fill it in Excel later (e.g., "Experiment 1")
                writer.writerow([f"{current_time:.2f}", f"{closest_dist:.4f}", f"{target_width:.4f}", num_points, ""])

def main(args=None):
    rclpy.init(args=args)
    logger = LidarLogger()
    try:
        rclpy.spin(logger)
    except KeyboardInterrupt:
        pass
    logger.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()