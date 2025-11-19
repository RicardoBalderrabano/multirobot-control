
'''
#!/usr/bin/env python3

"""
Trajectory Generator for Real TurtleBot3 with Smooth Polynomial Motion

TRAJECTORY TYPE: 5th-order polynomial minimum-jerk trajectory
INTERPOLATION: Smooth step function for continuous position, velocity, acceleration
OUTPUT: Point messages containing (x_desired, y_desired, x_velocity_desired)
FEATURES: Multi-robot namespace support, velocity limiting, safety delays

MATHEMATICAL BASIS:
- 5th-order polynomial: s(t) = 6t⁵ - 15t⁴ + 10t³
- Provides continuous position, velocity, and acceleration
- Zero start/end velocity and acceleration for smooth motion
- Velocity saturation for real robot safety

SAFETY: Maximum velocity limits, 2-second startup delay, automatic completion
"""

import rclpy
from rclpy.node import Node
import numpy as np

from geometry_msgs.msg import Point


class TrajectoryGenerator(Node):
    def __init__(self):
        super().__init__('trajectory_generator')
        
        # Get parameters - REAL ROBOT SAFETY
        self.declare_parameter('robot_namespace', 'burger1')
        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('goal_x', 1.0)  # Reasonable distance for real robot
        self.declare_parameter('goal_y', 1.0)  # Straight line for initial testing
        self.declare_parameter('total_time', 15.0)  # Increased time for smoother motion
        self.declare_parameter('max_velocity', 0.1)  # Limit maximum velocity
        
        self.robot_namespace = self.get_parameter('robot_namespace').value
        self.start_x = self.get_parameter('start_x').value
        self.start_y = self.get_parameter('start_y').value
        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value
        self.total_time = self.get_parameter('total_time').value
        self.max_velocity = self.get_parameter('max_velocity').value
        
        # Trajectory state
        self.start_time = self.get_clock().now()
        self.trajectory_completed = False
        self.trajectory_started = False
        
        # Publisher
        self.traj_pub = self.create_publisher(
            Point, 
            f'/{self.robot_namespace}/pointB_desired_state', 
            10
        )
        
        # Trajectory timer (20 Hz - reduced for real robot stability)
        self.traj_timer = self.create_timer(0.05, self.publish_trajectory)  # 20Hz
        
        # Start delay to ensure robot is ready
        self.start_delay_timer = self.create_timer(2.0, self.start_trajectory)  # 2 second delay
        
        self.get_logger().info(f'Real Robot Trajectory Generator started for {self.robot_namespace}')
        self.get_logger().info(f'Moving from ({self.start_x}, {self.start_y}) to ({self.goal_x}, {self.goal_y}) in {self.total_time}s')
        
    def start_trajectory(self):
        """Start trajectory after delay to ensure robot is ready"""
        self.start_time = self.get_clock().now()
        self.trajectory_started = True
        self.start_delay_timer.cancel()
        self.get_logger().info(f'[{self.robot_namespace}] Starting trajectory execution')
        
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
        
        # Limit maximum velocity for real robot safety
        velocity_magnitude = np.sqrt(x_dot**2 + y_dot**2)
        if velocity_magnitude > self.max_velocity:
            scale = self.max_velocity / velocity_magnitude
            x_dot *= scale
            y_dot *= scale
        
        return x_d, y_d, x_dot, y_dot
        
    def publish_trajectory(self):
        if not self.trajectory_started:
            return
            
        if self.trajectory_completed:
            # Keep publishing the final position for a while, then stop
            x_d, y_d, x_dot, y_dot = self.generate_trajectory(self.total_time)
            
            # Stop publishing after 5 seconds at goal
            current_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
            if current_time > self.total_time + 5.0:
                self.get_logger().info(f'[{self.robot_namespace}] Trajectory completed - stopping publication')
                self.traj_timer.cancel()
                return
        else:
            current_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
            x_d, y_d, x_dot, y_dot = self.generate_trajectory(current_time)

            # Progress logging at reduced frequency
            if not self.trajectory_completed:
                self.get_logger().info(
                    f'[{self.robot_namespace}] Progress: {current_time:.1f}/{self.total_time:.1f}s, '
                    f'Target: ({x_d:.2f}, {y_d:.2f}), Vel: ({x_dot:.3f}, {y_dot:.3f})',
                    throttle_duration_sec=3.0  # Reduced logging frequency
                )
        
        # Publish desired state
        msg = Point()
        msg.x = float(x_d)      # Desired x position
        msg.y = float(y_d)      # Desired y position
        msg.z = float(x_dot)    # Desired x velocity
        
        self.traj_pub.publish(msg)

        if self.trajectory_completed and not hasattr(self, 'completion_reported'):
            self.get_logger().info(f'[{self.robot_namespace}] TRAJECTORY COMPLETED!')
            self.completion_reported = True

    def destroy_node(self):
        """Override to provide clean shutdown"""
        self.get_logger().info(f'[{self.robot_namespace}] Trajectory generator shutdown')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    trajectory_generator = TrajectoryGenerator()
    
    try:
        rclpy.spin(trajectory_generator)
    except KeyboardInterrupt:
        trajectory_generator.get_logger().info(f'[{trajectory_generator.robot_namespace}] Trajectory generator shutdown by user')
    finally:
        trajectory_generator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
'''

#!/usr/bin/env python3
"""
Trajectory Generator for Real TurtleBot3 with Smooth Polynomial Motion

IMPROVEMENTS:
- Keeps publishing final position indefinitely to prevent emergency stops
- Better error handling and shutdown
- Continuous operation for real robot testing
"""

import rclpy
from rclpy.node import Node
import numpy as np

from geometry_msgs.msg import Point


class TrajectoryGenerator(Node):
    def __init__(self):
        super().__init__('trajectory_generator')
        
        # Get parameters - REAL ROBOT SAFETY
        self.declare_parameter('robot_namespace', 'burger1')
        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('goal_x', 1.0)
        self.declare_parameter('goal_y', 0.0)  # Straight line for testing
        self.declare_parameter('total_time', 10.0)
        self.declare_parameter('max_velocity', 0.1)
        
        self.robot_namespace = self.get_parameter('robot_namespace').value
        self.start_x = self.get_parameter('start_x').value
        self.start_y = self.get_parameter('start_y').value
        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value
        self.total_time = self.get_parameter('total_time').value
        self.max_velocity = self.get_parameter('max_velocity').value
        
        # Trajectory state
        self.start_time = self.get_clock().now()
        self.trajectory_completed = False
        self.trajectory_started = False
        self.completion_reported = False
        
        # Publisher
        self.traj_pub = self.create_publisher(
            Point, 
            f'/{self.robot_namespace}/pointB_desired_state', 
            10
        )
        
        # Trajectory timer (20 Hz)
        self.traj_timer = self.create_timer(0.05, self.publish_trajectory)
        
        # Start delay to ensure robot is ready
        self.start_delay_timer = self.create_timer(2.0, self.start_trajectory)
        
        self.get_logger().info(f'Real Robot Trajectory Generator started for {self.robot_namespace}')
        self.get_logger().info(f'Moving from ({self.start_x}, {self.start_y}) to ({self.goal_x}, {self.goal_y}) in {self.total_time}s')
        
    def start_trajectory(self):
        """Start trajectory after delay to ensure robot is ready"""
        self.start_time = self.get_clock().now()
        self.trajectory_started = True
        self.start_delay_timer.cancel()
        self.get_logger().info(f'[{self.robot_namespace}] Starting trajectory execution')
        
    def generate_trajectory(self, current_time):
        # Normalized time (0 to 1)
        t = current_time / self.total_time
        
        if t >= 1.0:
            self.trajectory_completed = True
            t = 1.0
        
        # 5th order polynomial for smooth motion
        s = 6 * t**5 - 15 * t**4 + 10 * t**3
        s_dot = (30 * t**4 - 60 * t**3 + 30 * t**2) / self.total_time
        
        # Interpolate position
        x_d = self.start_x + s * (self.goal_x - self.start_x)
        y_d = self.start_y + s * (self.goal_y - self.start_y)
        
        # Velocities
        x_dot = s_dot * (self.goal_x - self.start_x)
        y_dot = s_dot * (self.goal_y - self.start_y)
        
        # Limit maximum velocity for real robot safety
        velocity_magnitude = np.sqrt(x_dot**2 + y_dot**2)
        if velocity_magnitude > self.max_velocity:
            scale = self.max_velocity / velocity_magnitude
            x_dot *= scale
            y_dot *= scale
        
        return x_d, y_d, x_dot, y_dot
        
    def publish_trajectory(self):
        if not self.trajectory_started:
            return
            
        current_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        
        if current_time > self.total_time:
            self.trajectory_completed = True
            # Keep publishing final position indefinitely
            x_d, y_d, x_dot, y_dot = self.generate_trajectory(self.total_time)
        else:
            x_d, y_d, x_dot, y_dot = self.generate_trajectory(current_time)

        # Progress logging at reduced frequency
        if not self.trajectory_completed:
            self.get_logger().info(
                f'[{self.robot_namespace}] Progress: {current_time:.1f}/{self.total_time:.1f}s, '
                f'Target: ({x_d:.2f}, {y_d:.2f}), Vel: ({x_dot:.3f}, {y_dot:.3f})',
                throttle_duration_sec=3.0
            )
        
        # Publish desired state
        msg = Point()
        msg.x = float(x_d)
        msg.y = float(y_d)
        msg.z = float(x_dot)
        
        try:
            self.traj_pub.publish(msg)
        except Exception as e:
            self.get_logger().debug(f'Publish failed: {e}')

        # Log completion once
        if self.trajectory_completed and not self.completion_reported:
            self.get_logger().info(f'[{self.robot_namespace}] TRAJECTORY COMPLETED - maintaining final position')
            self.completion_reported = True

    def destroy_node(self):
        """Clean shutdown"""
        self.get_logger().info(f'[{self.robot_namespace}] Trajectory generator shutdown')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    trajectory_generator = TrajectoryGenerator()
    
    try:
        rclpy.spin(trajectory_generator)
    except KeyboardInterrupt:
        trajectory_generator.get_logger().info(f'[{trajectory_generator.robot_namespace}] Trajectory generator shutdown by user')
    except Exception as e:
        trajectory_generator.get_logger().error(f'Trajectory generator error: {e}')
    finally:
        trajectory_generator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()