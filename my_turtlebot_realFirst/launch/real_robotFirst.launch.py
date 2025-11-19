
'''
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    # Define launch arguments for multiple robots
    robot1_namespace = DeclareLaunchArgument(
        'robot1_namespace',
        default_value='burger1',
        description='Namespace for robot 1'
    )
    
    robot2_namespace = DeclareLaunchArgument(
        'robot2_namespace', 
        default_value='burger2',
        description='Namespace for robot 2'
    )
    
    # Robot 1 trajectory generator
    robot1_trajectory = Node(
        package='my_turtlebot_realFirst',
        executable='trajectory_generator',
        output='screen',
        name='trajectory_generator_1',
        namespace=LaunchConfiguration('robot1_namespace'),
        parameters=[{
            'robot_namespace': LaunchConfiguration('robot1_namespace'),
            'start_x': 0.1,
            'start_y': 0.0,
            'goal_x': 1.0,    # Safe distance for real robot
            'goal_y': 0.0,    # Safe distance for real robot
            'total_time': 15.0,
            'max_velocity': 0.1
        }]
    )
    
    # Robot 2 trajectory generator (different path)
    robot2_trajectory = Node(
        package='my_turtlebot_real',
        executable='trajectory_generator',
        output='screen', 
        name='trajectory_generator_2',
        namespace=LaunchConfiguration('robot2_namespace'),
        parameters=[{
            'robot_namespace': LaunchConfiguration('robot2_namespace'),
            'start_x': 0.0,
            'start_y': 1.0,   # Different starting position
            'goal_x': 1.0,    # Safe distance for real robot
            'goal_y': 0.0,    # Safe distance for real robot  
            'total_time': 12.0,  # Different timing
            'max_velocity': 0.08  # Different speed limit
        }]
    )

    return LaunchDescription([
        robot1_namespace,
        robot2_namespace,
        robot1_trajectory,
        robot2_trajectory,
    ])'''
'''
# This launch file automates the complete startup of a TurtleBot3 real robot 
# control system by launching two coordinated ROS2 nodes.

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    # Define launch arguments
    robot_namespace = DeclareLaunchArgument(
        'robot_namespace',
        default_value='burger1',
        description='Namespace for the robot'
    )
    
    # Single robot nodes
    trajectory_node = Node(
        package='my_turtlebot_realFirst',
        executable='trajectory_generatorFirst',
        output='screen',
        name='trajectory_generator',
        namespace=LaunchConfiguration('robot_namespace'),
        parameters=[{
            'robot_namespace': LaunchConfiguration('robot_namespace'),
            'start_x': 0.1,
            'start_y': 0.0,
            'goal_x': 2.0,
            'goal_y': 0.0,
            'total_time': 20.0,
            'max_velocity': 0.1
        }]
    )
    
    controller_node = Node(
        package='my_turtlebot_realFirst',
        executable='controller',
        output='screen',
        name='controller',
        namespace=LaunchConfiguration('robot_namespace'),
        parameters=[{
            'robot_namespace': LaunchConfiguration('robot_namespace')
        }]
    )

    return LaunchDescription([
        robot_namespace,
        trajectory_node,
        controller_node,
    ])
'''
#!/usr/bin/env python3

"""
Multi-Robot TurtleBot3 Real Robot Launch File

PURPOSE: Launches a complete multi-robot control system for real TurtleBot3 robots
FEATURES: 
  - Supports multiple real TurtleBot3 robots with proper namespace isolation
  - Provides trajectory generation and I/O linearization control for autonomous navigation
  - Safety features including velocity limits and emergency stop

CONTROL SYSTEM:
  - Input/Output Linearization: Mathematical exact linearization via feedback
  - Point B Control: Controls point at offset b from robot center for stability
  - Smooth Trajectories: 5th-order polynomial for continuous motion

SAFETY: Velocity limits, trajectory timeout, emergency stop capability
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    # Define launch arguments
    robot_namespace = DeclareLaunchArgument(
        'robot_namespace',
        default_value='burger1',
        description='Namespace for the robot'
    )
    
    # Single robot nodes - REAL TURTLEBOT3
    trajectory_node = Node(
        package='my_turtlebot_realFirst',
        executable='trajectory_generatorFirst',
        output='screen',
        name='trajectory_generator',
        namespace=LaunchConfiguration('robot_namespace'),
        parameters=[{
            'robot_namespace': LaunchConfiguration('robot_namespace'),
            'start_x': 0.0,
            'start_y': 0.0,
            'goal_x': 1.0,
            'goal_y': 1.0,
            'total_time': 10.0,
            'max_velocity': 0.1
        }]
    )
    
    controller_node = Node(
        package='my_turtlebot_realFirst',
        executable='controller',
        output='screen',
        name='controller',
        namespace=LaunchConfiguration('robot_namespace'),
        parameters=[{
            'robot_namespace': LaunchConfiguration('robot_namespace'),
            'Kx': 2.0,
            'Ky': 2.0,
            'b': 0.1,
            'max_linear_vel': 0.15,
            'max_angular_vel': 0.8
        }]
    )

    return LaunchDescription([
        robot_namespace,
        trajectory_node,
        controller_node,
    ])