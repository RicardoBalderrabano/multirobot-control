#!/usr/bin/env python3

"""
Trajectory Generator for TurtleBot3 Simulation with Smooth Polynomial Motion

TRAJECTORY TYPE: 5th-order polynomial minimum-jerk trajectory
INTERPOLATION: Smooth step function for continuous position, velocity, acceleration
OUTPUT: Point messages containing (x_desired, y_desired, x_velocity_desired)
FEATURES: Multi-robot namespace support, smooth motion profiles, automatic completion

MATHEMATICAL BASIS:
- 5th-order polynomial: s(t) = 6t⁵ - 15t⁴ + 10t³
- Provides continuous position, velocity, and acceleration (C² continuity)
- Zero start/end velocity and acceleration for smooth motion
- Natural bell-shaped velocity profile

TRAJECTORY PARAMETERS:
- Start position: (start_x, start_y) - initial robot position
- Goal position: (goal_x, goal_y) - target destination  
- Total time: total_time seconds for complete trajectory
- Update rate: 50Hz for smooth simulation performance

MESSAGE FORMAT:
- Point.x: Desired x-position (meters)
- Point.y: Desired y-position (meters)
- Point.z: Desired x-velocity (m/s) - y-velocity assumed zero for simplicity

USE CASE: Gazebo simulation with smooth, predictable motion for controller testing
"""

import rclpy
from rclpy.node import Node
import numpy as np

from geometry_msgs.msg import Point


class TrajectoryGenerator(Node):
    def __init__(self):
        super().__init__('trajectory_generator')
        
        # Get parameters
        self.declare_parameter('robot_namespace', 'burger1')
        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('goal_x', 2.0)
        self.declare_parameter('goal_y', 1.0)
        self.declare_parameter('total_time', 10.0)
        
        self.robot_namespace = self.get_parameter('robot_namespace').value
        self.start_x = self.get_parameter('start_x').value
        self.start_y = self.get_parameter('start_y').value
        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value
        self.total_time = self.get_parameter('total_time').value
        
        # Trajectory state
        self.start_time = self.get_clock().now()
        self.trajectory_completed = False
        
        # Publisher
        self.traj_pub = self.create_publisher(
            Point, 
            f'/{self.robot_namespace}/pointB_desired_state', 
            10
        )
        
        # Trajectory timer (50 Hz)
        self.traj_timer = self.create_timer(0.02, self.publish_trajectory)  # 50Hz
        
        self.get_logger().info(f'Trajectory generator started for {self.robot_namespace}')
        self.get_logger().info(f'Moving from ({self.start_x}, {self.start_y}) to ({self.goal_x}, {self.goal_y}) in {self.total_time}s')
        
    def generate_trajectory(self, current_time):
        # Normalized time (0 to 1)
        t = current_time / self.total_time
        
        if t >= 1.0:
            self.trajectory_completed = True
            t = 1.0
        
        # 5th order polynomial for smooth motion
        # s(t) = 6t^5 - 15t^4 + 10t^3 (smooth step function)
        s = 6 * t**5 - 15 * t**4 + 10 * t**3
        s_dot = (30 * t**4 - 60 * t**3 + 30 * t**2) / self.total_time
        
        # Interpolate position
        x_d = self.start_x + s * (self.goal_x - self.start_x)
        y_d = self.start_y + s * (self.goal_y - self.start_y)
        
        # Velocities
        x_dot = s_dot * (self.goal_x - self.start_x)
        y_dot = s_dot * (self.goal_y - self.start_y)
        
        return x_d, y_d, x_dot, y_dot
        
    def publish_trajectory(self):
        if self.trajectory_completed:
            # Keep publishing the final position
            x_d, y_d, x_dot, y_dot = self.generate_trajectory(self.total_time)
        else:
            current_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
            x_d, y_d, x_dot, y_dot = self.generate_trajectory(current_time)

            # DEBUG: Print what we're publishing
            self.get_logger().info(f'Publishing: x={x_d:.2f}, y={y_d:.2f}, x_dot={x_dot:.2f}')

                        
            if not self.trajectory_completed:
                self.get_logger().info(f'Time: {current_time:.1f}s, Target: ({x_d:.2f}, {y_d:.2f})', throttle_duration_sec=2.0)
        
        # Publish desired state
        msg = Point()
        msg.x = float(x_d)      # Desired x position
        msg.y = float(y_d)      # Desired y position
        msg.z = float(x_dot)    # Desired x velocity
        
        self.traj_pub.publish(msg)
        self.get_logger().info('Message published!', throttle_duration_sec=1.0)  # DEBUG

        if self.trajectory_completed:
            self.get_logger().info('Trajectory completed!')
            self.traj_timer.cancel()  # Stop publishing


def main(args=None):
    rclpy.init(args=args)
    trajectory_generator = TrajectoryGenerator()
    
    try:
        rclpy.spin(trajectory_generator)
    except KeyboardInterrupt:
        pass
    finally:
        trajectory_generator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

'''

#!/usr/bin/env python3

"""
Trajectory Generator for TurtleBot3 Simulation with Sine Wave Motion

TRAJECTORY TYPE: Sine wave trajectory for smooth oscillatory motion
INTERPOLATION: Trigonometric functions for continuous position, velocity, acceleration
OUTPUT: Point messages containing (x_desired, y_desired, x_velocity_desired)
FEATURES: Multi-robot namespace support, smooth motion profiles, continuous operation

MATHEMATICAL BASIS:
- Sine function: y(t) = amplitude * sin(2π * frequency * t)
- Linear forward motion: x(t) = forward_speed * t
- Provides continuous position, velocity, and acceleration
- Natural oscillatory motion for controller testing

TRAJECTORY PARAMETERS:
- Start position: (start_x, start_y) - initial robot position
- Amplitude: Side-to-side oscillation magnitude
- Frequency: Oscillation rate in Hz
- Forward speed: Constant forward velocity
- Update rate: 50Hz for smooth simulation performance

MESSAGE FORMAT:
- Point.x: Desired x-position (meters)
- Point.y: Desired y-position (meters)
- Point.z: Desired x-velocity (m/s)

USE CASE: Gazebo simulation with continuous oscillatory motion for controller testing
"""

import rclpy
from rclpy.node import Node
import numpy as np

from geometry_msgs.msg import Point


class TrajectoryGenerator(Node):
    def __init__(self):
        super().__init__('trajectory_generator')
        
        # Get parameters - KEEP ALL EXISTING PARAMETERS
        self.declare_parameter('robot_namespace', 'burger1')
        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('goal_x', 1.0)  # Will be repurposed as amplitude hint
        self.declare_parameter('goal_y', 1.0)  # Will be repurposed as frequency hint
        self.declare_parameter('total_time', 10.0)  # Will be repurposed as forward_speed
        
        # NEW: Sine wave specific parameters
        self.declare_parameter('amplitude', 2.0)
        self.declare_parameter('frequency', 0.1)
        self.declare_parameter('forward_speed', 0.1)
        
        self.robot_namespace = self.get_parameter('robot_namespace').value
        self.start_x = self.get_parameter('start_x').value
        self.start_y = self.get_parameter('start_y').value
        
        # Use existing parameters or new ones for sine wave
        amplitude_param = self.get_parameter('amplitude').value
        frequency_param = self.get_parameter('frequency').value
        forward_speed_param = self.get_parameter('forward_speed').value
        
        # If new parameters are default, try to use existing ones as hints
        self.amplitude = amplitude_param if amplitude_param != 2.0 else abs(self.get_parameter('goal_y').value)
        self.frequency = frequency_param if frequency_param != 0.1 else abs(self.get_parameter('goal_x').value) / 20.0
        self.forward_speed = forward_speed_param if forward_speed_param != 0.1 else self.get_parameter('total_time').value / 10.0
        
        # Trajectory state - KEEP ALL EXISTING VARIABLES
        self.start_time = self.get_clock().now()
        self.trajectory_completed = False
        
        # Publisher - KEEP EXACTLY THE SAME
        self.traj_pub = self.create_publisher(
            Point, 
            f'/{self.robot_namespace}/pointB_desired_state', 
            10
        )
        
        # Trajectory timer (50 Hz) - KEEP EXACTLY THE SAME
        self.traj_timer = self.create_timer(0.02, self.publish_trajectory)  # 50Hz
        
        self.get_logger().info(f'Trajectory generator started for {self.robot_namespace}')
        self.get_logger().info(f'Sine wave: amplitude={self.amplitude:.2f}m, frequency={self.frequency:.2f}Hz, speed={self.forward_speed:.2f}m/s')
        self.get_logger().info(f'Starting from: ({self.start_x}, {self.start_y})')

    def generate_trajectory(self, current_time):
        """
        Generate sine wave trajectory:
        - x(t) = start_x + forward_speed * t
        - y(t) = start_y + amplitude * sin(2π * frequency * t)
        """
        # Calculate desired position using sine wave
        x_d = self.start_x + self.forward_speed * current_time
        y_d = self.start_y + self.amplitude * np.sin(2 * np.pi * self.frequency * current_time)
        
        # Calculate desired velocities (derivatives)
        x_dot = self.forward_speed
        y_dot = self.amplitude * 2 * np.pi * self.frequency * np.cos(2 * np.pi * self.frequency * current_time)
        
        # Sine wave never truly "completes" but we can stop after a long time
        # Using total_time parameter as maximum duration
        max_duration = self.get_parameter('total_time').value
        if current_time >= max_duration and max_duration > 0:
            self.trajectory_completed = True
            
        return x_d, y_d, x_dot, y_dot

    def publish_trajectory(self):
        """KEEP THE EXACT SAME PUBLISHING LOGIC"""
        if self.trajectory_completed:
            # For sine wave, just stop publishing when completed
            self.get_logger().info('Sine trajectory completed!')
            self.traj_timer.cancel()
            return
            
        current_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        x_d, y_d, x_dot, y_dot = self.generate_trajectory(current_time)

        # DEBUG: Print what we're publishing - KEEP THE SAME
        self.get_logger().info(f'Publishing: x={x_d:.2f}, y={y_d:.2f}, x_dot={x_dot:.2f}')

        # Log occasionally to reduce spam
        if int(current_time * 10) % 20 == 0:  # Every ~2 seconds
            self.get_logger().info(f'Time: {current_time:.1f}s, Target: ({x_d:.2f}, {y_d:.2f})')
        
        # Publish desired state - KEEP EXACTLY THE SAME
        msg = Point()
        msg.x = float(x_d)      # Desired x position
        msg.y = float(y_d)      # Desired y position  
        msg.z = float(x_dot)    # Desired x velocity
        
        self.traj_pub.publish(msg)
        self.get_logger().info('Message published!', throttle_duration_sec=1.0)  # DEBUG


def main(args=None):
    rclpy.init(args=args)
    trajectory_generator = TrajectoryGenerator()
    
    try:
        rclpy.spin(trajectory_generator)
    except KeyboardInterrupt:
        pass
    finally:
        trajectory_generator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
'''