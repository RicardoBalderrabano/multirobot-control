from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # --- IMPORTANT: Change this to your actual ROS 2 package name ---
    # Based on your previous terminal paths, it looks like it is:
    PACKAGE_NAME = 'my_turtlebot_realFirst'

    return LaunchDescription([
        # 1. Launch the Global Transformer Node
        Node(
            package=PACKAGE_NAME,
            executable='multi_global_transformer', # Name defined in your setup.py
            name='multi_global_transformer',
            output='screen',
            parameters=[{'use_sim_time': True}]    # <--- THE MAGIC LINE
        ),

        # 2. Launch the Kalman Observer Node
        Node(
            package=PACKAGE_NAME,
            executable='kalman_observer',          # Name defined in your setup.py
            name='kalman_observer',
            output='screen',
            parameters=[{'use_sim_time': True}]    # <--- THE MAGIC LINE
        )
    ])
