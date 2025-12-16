'''
#!/usr/bin/env python3
"""
Input/Output Linearization Controller for Point B (Ground Truth Version)
"""

import rclpy
from rclpy.node import Node
import numpy as np

# Change 1: Import PoseStamped instead of Odometry (if you don't use Odom anymore)
from geometry_msgs.msg import Twist, Point, PoseStamped
from nav_msgs.msg import Odometry # (Optional: keep if you want to switch back later)
from tf_transformations import euler_from_quaternion


'''

#!/usr/bin/env python3
"""
Input/Output Linearization Controller (Simulation Version)
ADAPTED FROM: Real Robot "IOLinearizationOptitrack"
FEATURES:
- Uses Ground Truth from Gazebo bridge
- Safety Timeout (Stops if sim data lags)
- Saturation (max_v, max_w)
- Robust Goal Handling
"""

import rclpy
from rclpy.node import Node
import numpy as np

# NOTE: Bridge uses Twist, Real robot used TwistStamped. We use Twist here.
from geometry_msgs.msg import Twist, Point, PoseStamped
from tf_transformations import euler_from_quaternion

class IOLinearizationSim(Node):
    def __init__(self):
        super().__init__('io_linearization_sim')
        
        # --- Parameters ---
        self.declare_parameter('robot_namespace', 'burger1')
        self.declare_parameter('pose_topic', 'ground_truth_pose') # Bridge topic name
        self.declare_parameter('Kx', 2.0)
        self.declare_parameter('Ky', 2.0)
        self.declare_parameter('b', 0.2)
        self.declare_parameter('goal_tolerance', 0.10)
        self.declare_parameter('max_v', 0.5)
        self.declare_parameter('max_w', 2.0)
        
        self.ns = self.get_parameter('robot_namespace').value
        pose_topic_name = self.get_parameter('pose_topic').value
        
        # Construct full topic string: /burger1/ground_truth_pose
        self.pose_topic = f'/{self.ns}/{pose_topic_name}'
        
        self.Kx = self.get_parameter('Kx').value
        self.Ky = self.get_parameter('Ky').value
        self.b = self.get_parameter('b').value
        self.dist_tol = self.get_parameter('goal_tolerance').value
        self.max_v = self.get_parameter('max_v').value
        self.max_w = self.get_parameter('max_w').value
        
        # --- State ---
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.xB = 0.0
        self.yB = 0.0
        self.xB_goal = 0.0
        self.yB_goal = 0.0
        
        self.has_goal = False
        self.pose_received = False
        self.last_pose_time = 0.0
        
        # Publisher: Using Twist to match Gazebo Bridge configuration
        self.pub_cmd = self.create_publisher(Twist, f'/{self.ns}/cmd_vel', 10)
        
        # Subscriber: Simulation bridge is usually Reliable, so we use default QoS (10)
        self.sub_pose = self.create_subscription(
            PoseStamped,
            self.pose_topic,
            self.pose_callback,
            10 
        )
        
        self.sub_goal = self.create_subscription(
            Point,
            f'/{self.ns}/robot_goal', 
            self.goal_callback,
            10
        )
        
        self.timer = self.create_timer(0.05, self.control_loop) # 20Hz
        
        self.get_logger().info(f"--- Sim Controller Started for {self.ns} ---")
        self.get_logger().info(f"Listening for Ground Truth on: {self.pose_topic}")

    def pose_callback(self, msg):
        self.pose_received = True
        # Get simulation time in seconds
        self.last_pose_time = self.get_clock().now().nanoseconds / 1e9
        
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        
        orientation = msg.pose.orientation
        try:
            (_, _, self.theta) = euler_from_quaternion(
                [orientation.x, orientation.y, orientation.z, orientation.w]
            )
        except Exception:
            return
            
        # Update Point B location
        self.xB = self.x + self.b * np.cos(self.theta)
        self.yB = self.y + self.b * np.sin(self.theta)

    def goal_callback(self, msg):
        if not self.pose_received:
            self.get_logger().warn(f"{self.ns}: Goal ignored - waiting for pose data...")
            return
        
        self.xB_goal = msg.x
        self.yB_goal = msg.y
        self.has_goal = True
        self.get_logger().info(f"{self.ns}: NEW GOAL RECEIVED ({self.xB_goal:.2f}, {self.yB_goal:.2f})")

    def control_loop(self):
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        # 1. Safety Timeout
        # If simulation time moves forward but we don't get data, stop.
        if self.pose_received and (current_time - self.last_pose_time) > 2.0:
            self.get_logger().warn(f"{self.ns}: TIMEOUT! Data lag > 2.0s", throttle_duration_sec=2.0)
            self.stop_robot()
            return

        if not self.has_goal or not self.pose_received:
            return

        # 2. Error Calculation
        e_x = self.xB_goal - self.xB
        e_y = self.yB_goal - self.yB
        dist_error = np.sqrt(e_x**2 + e_y**2)

        if dist_error < self.dist_tol:
            self.get_logger().info(f"{self.ns}: GOAL REACHED (Err: {dist_error:.3f}m)", throttle_duration_sec=3.0)
            self.stop_robot()
            self.has_goal = False
            return

        # 3. Control Law (I/O Linearization)
        u_x = self.Kx * e_x
        u_y = self.Ky * e_y
        
        c_th = np.cos(self.theta)
        s_th = np.sin(self.theta)
        
        # Exact Linearization Transformation
        v_cmd = u_x * c_th + u_y * s_th
        w_cmd = (1.0 / self.b) * (-u_x * s_th + u_y * c_th)

        # 4. Saturation (SCALING METHOD)
        # Check if v or w exceed their limits
        scale_v = 1.0
        scale_w = 1.0
        
        if abs(v_cmd) > self.max_v:
            scale_v = self.max_v / abs(v_cmd)
            
        if abs(w_cmd) > self.max_w:
            scale_w = self.max_w / abs(w_cmd)
            
        # Apply the strictest scaling factor to BOTH to preserve trajectory shape
        scale = min(scale_v, scale_w)
        
        v_cmd = v_cmd * scale
        w_cmd = w_cmd * scale

        # 5. Publish Command
        msg = Twist() # Using Twist for Gazebo Bridge
        msg.linear.x = float(v_cmd)
        msg.angular.z = float(w_cmd)
        self.pub_cmd.publish(msg)

    def stop_robot(self):
        msg = Twist()
        self.pub_cmd.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = IOLinearizationSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_robot()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()