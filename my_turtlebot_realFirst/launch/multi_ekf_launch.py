import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('my_turtlebot_realFirst')
    ekf_config_path = os.path.join(pkg_share, 'config', 'ekf_template.yaml')

    # List your robot names here
    robot_names = ['tb1', 'tb3']
    nodes = []

    for robot in robot_names:
        nodes.append(
            Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_filter_node',
                namespace=robot,
                output='screen',
                parameters=[ekf_config_path, {
                    'odom_frame': f'{robot}/odom',
                    'base_link_frame': f'{robot}/base_footprint',
                    'world_frame': f'{robot}/odom',
                }],
                # Remap global topics to namespaced ones
                remappings=[
                    ('/odom', f'/{robot}/odom'),
                    ('/imu', f'/{robot}/imu'),
                    ('/set_pose', f'/{robot}/set_pose'),
                ]
            )
        )

    return LaunchDescription(nodes)
