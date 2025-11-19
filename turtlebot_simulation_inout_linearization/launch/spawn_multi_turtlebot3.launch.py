#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    # Get model from environment variable
    TURTLEBOT3_MODEL = os.environ.get('TURTLEBOT3_MODEL', 'burger')
    model_folder = 'turtlebot3_' + TURTLEBOT3_MODEL

    urdf_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'models',
        model_folder,
        'model.sdf'
    )

    # Launch configuration variables
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    robot_name = LaunchConfiguration('robot_name', default=TURTLEBOT3_MODEL)
    namespace = LaunchConfiguration('namespace', default='')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Declare arguments
    declare_x_position_cmd = DeclareLaunchArgument(
        'x_pose', default_value='0.0', description='Specify x position of the robot')
    declare_y_position_cmd = DeclareLaunchArgument(
        'y_pose', default_value='0.0', description='Specify y position of the robot')
    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name', default_value=TURTLEBOT3_MODEL, description='Specify name of the robot')
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='', description='Namespace for the robot')
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation time')

    # Bridge params (namespace-aware)
    bridge_params = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'params',
        model_folder + '_bridge.yaml'
    )

    # Group all nodes under a namespace
    group = GroupAction([
        PushRosNamespace(namespace),

        # Spawn robot in Gazebo
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', robot_name,
                '-file', urdf_path,
                '-x', x_pose,
                '-y', y_pose,
                '-z', '0.01'
            ],
            output='screen',
        ),

        # Bridge Gazebo topics
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '--ros-args', '-p', f'config_file:={bridge_params}',
            ],
            output='screen',
        ),

        # Optional image bridge for non-burger models
        Node(
            package='ros_gz_image',
            executable='image_bridge',
            arguments=['/camera/image_raw'],
            output='screen',
            condition=IfCondition(
                PythonExpression([f"'{TURTLEBOT3_MODEL}' != 'burger'"])
            ),
        ),
    ])

    ld = LaunchDescription()
    ld.add_action(declare_x_position_cmd)
    ld.add_action(declare_y_position_cmd)
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(group)

    return ld