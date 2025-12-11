#!/usr/bin/env python3
"""
Input/Output Linearization Controller for Point B

CONTROLLER TYPE: Exact input/output linearization via feedback linearization
CONTROL OBJECTIVE: Drive Point B (ahead of robot) to a target coordinates (xB_goal, yB_goal)
THEORY: Slides 3-6 (IOlincontrol.pdf)

CONTROL LAW:
  [v]   [cos(θ)   sin(θ)] [ ux ]
  [ω] = [-sin(θ)/b cos(θ)/b] [ uy ]
  
  Where virtual inputs are:
  ux = xB_dot_ref + Kx * (xB_ref - xB)
  uy = yB_dot_ref + Ky * (yB_ref - yB)
"""

import rclpy
from rclpy.node import Node
import numpy as np

from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion

class IOLinearizationPointB(Node):
    def __init__(self):
        super().__init__('io_linearization_point_b')
        
        # --- Parameters ---
        self.declare_parameter('robot_namespace', 'burger1')
        self.declare_parameter('Kx', 1.0)  # Proportional gain for X
        self.declare_parameter('Ky', 1.0)  # Proportional gain for Y
        self.declare_parameter('b', 0.2)   # Distance from center to point B [cite: 25]
        self.declare_parameter('goal_tolerance', 0.05)
        
        self.ns = self.get_parameter('robot_namespace').value
        self.Kx = self.get_parameter('Kx').value
        self.Ky = self.get_parameter('Ky').value
        self.b = self.get_parameter('b').value
        self.dist_tol = self.get_parameter('goal_tolerance').value
        
        # --- State ---
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Point B current state
        self.xB = 0.0
        self.yB = 0.0
        
        # Point B Goal state
        self.xB_goal = 0.0
        self.yB_goal = 0.0
        self.goal_received = False
        
        # --- Communication ---
        self.pub_cmd = self.create_publisher(Twist, f'/{self.ns}/cmd_vel', 10)
        self.sub_odom = self.create_subscription(Odometry, f'/{self.ns}/odom', self.odom_callback, 10)
        self.sub_goal = self.create_subscription(Point, f'/{self.ns}/robot_goal', self.goal_callback, 10)
        
        self.timer = self.create_timer(0.02, self.control_loop) # 50Hz

        self.get_logger().info("I/O Linearization: Controlling Point B directly.")
        self.get_logger().info(f"Offset b={self.b}m. Gains Kx={self.Kx}, Ky={self.Ky}")

    def odom_callback(self, msg):
        # 1. Get Robot State (x, y, theta)
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        
        rot = msg.pose.pose.orientation
        (_, _, self.theta) = euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])
        
        # 2. Compute Point B Position [cite: 26]
        # xB = x + b*cos(theta)
        # yB = y + b*sin(theta)
        self.xB = self.x + self.b * np.cos(self.theta)
        self.yB = self.y + self.b * np.sin(self.theta)

    def goal_callback(self, msg):
        # We interpret the incoming Point msg as the TARGET FOR POINT B directly.
        # We do NOT calculate heading or robot center goals.
        self.xB_goal = msg.x
        self.yB_goal = msg.y
        self.goal_received = True
        self.get_logger().info(f"New Goal for B: ({self.xB_goal:.2f}, {self.yB_goal:.2f})")

    def control_loop(self):
        if not self.goal_received:
            return

        # 1. Calculate Errors for Point B
        ex = self.xB_goal - self.xB
        ey = self.yB_goal - self.yB
        dist_error = np.sqrt(ex**2 + ey**2)
        
        if dist_error < self.dist_tol:
            self.stop_robot()
            return

        # 2. Linear Control Law (Virtual Inputs) 
        # We assume a static goal (reference velocity = 0)
        # ux = dot_xB* + Kx(xB* - xB) -> ux = Kx * ex
        ux = self.Kx * ex
        uy = self.Ky * ey
        
        # 3. Decoupling Matrix / Feedback Linearization 
        # The inverse of the T matrix:
        # v = ux*cos(theta) + uy*sin(theta)
        # w = (-ux*sin(theta) + uy*cos(theta)) / b
        
        c_th = np.cos(self.theta)
        s_th = np.sin(self.theta)
        
        v_cmd = ux * c_th + uy * s_th
        w_cmd = (1.0 / self.b) * (-ux * s_th + uy * c_th)

        # 4. Safety Saturation
        v_cmd = np.clip(v_cmd, -0.22, 0.22)
        w_cmd = np.clip(w_cmd, -2.0, 2.0)

        # 5. Publish
        twist = Twist()
        twist.linear.x = float(v_cmd)
        twist.angular.z = float(w_cmd)
        self.pub_cmd.publish(twist)

    def stop_robot(self):
        twist = Twist()
        self.pub_cmd.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = IOLinearizationPointB()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_robot()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

'''
#!/usr/bin/env python3

"""
Unicycle Controller for TurtleBot3 Simulation with Point Offset Control

CONTROLLER TYPE: Body-frame error regulation with point B offset
CONTROL STRATEGY: Geometric transformation of world-frame errors to body frame
MODEL: Unicycle kinematics with controlled point at offset from center
INPUTS: Desired Point B position (xB_desired, yB_desired, xB_dot_desired) via Point messages
OUTPUTS: Twist commands (v, ω) for simulated robot

MATHEMATICAL BASIS:
- Controls Point B at offset b from robot center: xB = x + b·cos(θ), yB = y + b·sin(θ)
- Transforms world-frame errors to body frame for intuitive control
- Implements: v = xB_dot_desired + kp·e_x_body, ω = (yB_dot_desired + kp·e_y_body)/b
- Applies velocity saturation for simulation stability

KEY PARAMETERS:
- kp = 4.0 (proportional gain for error correction)
- b = 0.2m (offset distance from robot center to controlled point B)
- Control rate: 50Hz for smooth simulation performance

FEATURES: Multi-robot namespace support, extensive debug logging, velocity saturation
USE CASE: Gazebo simulation with smooth trajectory tracking and clear debugging output
"""

import rclpy
from rclpy.node import Node
import numpy as np

from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion


class UnicycleController(Node):
    def __init__(self):
        super().__init__('unicycle_controller')
        
        # Get parameters
        self.declare_parameter('robot_namespace', 'burger1')
        self.declare_parameter('kp', 4.0)
        self.declare_parameter('b', 0.2)  # Distance from center to point B
        
        self.robot_namespace = self.get_parameter('robot_namespace').value
        self.kp = self.get_parameter('kp').value
        self.b = self.get_parameter('b').value
        
        # Current state
        self.xB = 0.0
        self.yB = 0.0
        self.theta = 0.0
        self.v = 0.0
        
        # Desired state
        self.xB_desired = 0.0
        self.yB_desired = 0.0
        self.xB_dot_desired = 0.0
        self.yB_dot_desired = 0.0
        
        # Publishers and Subscribers
        self.cmd_pub = self.create_publisher(
            Twist, 
            f'/{self.robot_namespace}/cmd_vel', 
            10
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            f'/{self.robot_namespace}/odom',
            self.odom_callback,
            10
        )
        
        self.traj_sub = self.create_subscription(
            Point,
            f'/{self.robot_namespace}/pointB_desired_state',
            self.traj_callback,
            10
        )
        
        # Control timer (50 Hz)
        self.control_timer = self.create_timer(0.02, self.control_loop)  # 50Hz
        
        self.get_logger().info(f'Controller started for {self.robot_namespace}')
        
    def odom_callback(self, msg):
        # DEBUG: Print raw odometry data
        self.get_logger().info(
            f'Raw Odom - Position: ({msg.pose.pose.position.x:.2f}, {msg.pose.pose.position.y:.2f}) | '
            f'Orientation: ({msg.pose.pose.orientation.x:.2f}, {msg.pose.pose.orientation.y:.2f}, '
            f'{msg.pose.pose.orientation.z:.2f}, {msg.pose.pose.orientation.w:.2f})',
            throttle_duration_sec=2.0
        )
        
        # Get robot position
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        
        # Get orientation from quaternion
        orientation = msg.pose.pose.orientation
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
        
        try:
            roll, pitch, yaw = euler_from_quaternion(quaternion)
            self.theta = yaw  # Use yaw for 2D orientation
        except Exception as e:
            self.get_logger().error(f'Quaternion conversion error: {e}')
            return
        
        # Get linear velocity
        self.v = msg.twist.twist.linear.x
        
        # Calculate point B position (offset from center)
        self.xB = self.x + self.b * np.cos(self.theta)
        self.yB = self.y + self.b * np.sin(self.theta)
        
        # DEBUG: Print processed data
        self.get_logger().info(
            f'Processed - Center: ({self.x:.2f}, {self.y:.2f}) | '
            f'Theta: {np.degrees(self.theta):.1f}° | '
            f'Point B: ({self.xB:.2f}, {self.yB:.2f}) | '
            f'Velocity: {self.v:.2f}',
            throttle_duration_sec=2.0
        )

    def traj_callback(self, msg):
        # msg.x = desired x position
        # msg.y = desired y position  
        # msg.z = desired x velocity (we'll use y velocity as 0 for simplicity)
        self.get_logger().info(f'Received target: ({msg.x:.2f}, {msg.y:.2f})')
        self.xB_desired = msg.x
        self.yB_desired = msg.y
        self.xB_dot_desired = msg.z
        self.yB_dot_desired = 0.0  # Simple assumption

    def compute_control(self):
        # Calculate errors in WORLD frame
        e_x_world = self.xB_desired - self.xB 
        e_y_world = self.yB_desired - self.yB 
        
        # Transform errors to BODY frame
        e_x_body = e_x_world * np.cos(self.theta) + e_y_world * np.sin(self.theta)
        e_y_body = -e_x_world * np.sin(self.theta) + e_y_world * np.cos(self.theta)
        
        # DEBUG: Print errors
        self.get_logger().info(
            f'Errors - World: ({e_x_world:.3f}, {e_y_world:.3f}) | '
            f'Body: ({e_x_body:.3f}, {e_y_body:.3f}) | '
            f'Actual B: ({self.xB:.3f}, {self.yB:.3f}) | '
            f'Target B: ({self.xB_desired:.3f}, {self.yB_desired:.3f})',
            throttle_duration_sec=1.0
        )

        # Virtual controls (first-order error dynamics) - use BODY frame errors
        w1 = self.xB_dot_desired + self.kp * e_x_body
        w2 = self.yB_dot_desired + self.kp * e_y_body
        
        # Apply control law for point B
        v = w1  # In body frame, x-direction is forward
        omega = w2 / self.b
        
        # Apply saturation limits
        v = np.clip(v, -0.1, 0.1)        # Max linear velocity
        omega = np.clip(omega, -1.0, 1.0) # Max angular velocity
        
        # DEBUG: Print control calculations
        self.get_logger().info(
            f'Control calc: w1={w1:.3f}, w2={w2:.3f}, v={v:.3f}, omega={omega:.3f}',
            throttle_duration_sec=1.0
        )

        return v, omega

    def control_loop(self):
        try:
            # Compute control commands
            v, omega = self.compute_control()

            # DEBUG: Print current state and commands
            self.get_logger().info(
                f'Actual: ({self.xB:.2f}, {self.yB:.2f}) | '
                f'Target: ({self.xB_desired:.2f}, {self.yB_desired:.2f}) | '
                f'Control: v={v:.2f}, ω={omega:.2f}',
                throttle_duration_sec=1.0  # Only print every 1 second
            )
            
            # Publish to robot
            cmd_msg = Twist()
            cmd_msg.linear.x = float(v)
            cmd_msg.angular.z = float(omega)
            self.cmd_pub.publish(cmd_msg)
            
            # Logging (optional - can comment out for less noise)
            # self.get_logger().info(f'Control: v={v:.2f}, ω={omega:.2f}')
            
        except Exception as e:
            self.get_logger().error(f'Control error: {str(e)}')
            
def main(args=None):
    rclpy.init(args=args)
    controller = UnicycleController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

'''