#!/usr/bin/env python3

import os
import yaml
import tempfile

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import GroupAction

from ament_index_python.packages import get_package_share_directory


def _make_namespaced_sdf(original_sdf_path, namespace):
    with open(original_sdf_path, 'r') as f:
        sdf_content = f.read()

    replacements = {
        '<odom_topic>odom</odom_topic>': f'<odom_topic>/model/{namespace}/odometry</odom_topic>',
        '<frame_id>odom</frame_id>': f'<odom_frame>odom</odom_frame>',
        '<child_frame_id>base_footprint</child_frame_id>': f'<robot_base_frame>base_footprint</robot_base_frame>',
        '<topic>cmd_vel</topic>': f'<topic>/model/{namespace}/cmd_vel</topic>'
    }
    
    for old, new in replacements.items():
        sdf_content = sdf_content.replace(old, new)

    sdf_content = sdf_content.replace('__model__', namespace)

    # INJECT GROUND TRUTH POSE PLUGIN
    pose_plugin = f"""
    <plugin filename="gz-sim-pose-publisher-system" name="gz::sim::systems::PosePublisher">
        <publish_link_pose>false</publish_link_pose>
        <publish_sensor_pose>false</publish_sensor_pose>
        <publish_collision_pose>false</publish_collision_pose>
        <publish_visual_pose>false</publish_visual_pose>
        <publish_nested_model_pose>true</publish_nested_model_pose>
        <use_pose_vector_msg>false</use_pose_vector_msg>
        <update_frequency>20</update_frequency>
    </plugin>
    </model>"""
    
    if '</model>' in sdf_content:
        sdf_content = sdf_content.replace('</model>', pose_plugin)

    tmp_sdf = tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='.sdf')
    tmp_sdf.write(sdf_content)
    tmp_sdf.close()
    return tmp_sdf.name


def _make_namespaced_bridge_yaml(original_yaml_path, namespace):
    with open(original_yaml_path, 'r') as f:
        bridge_config = yaml.safe_load(f)

    namespaced_config = []
    for entry in bridge_config:
        new_entry = entry.copy()

        if 'ros_topic_name' in new_entry:
            ros_topic = new_entry['ros_topic_name'].lstrip('/')
            new_entry['ros_topic_name'] = f"/{namespace}/{ros_topic}"

        if 'gz_topic_name' in new_entry:
            gz_topic = new_entry['gz_topic_name']
            if gz_topic == "cmd_vel":
                new_entry['gz_topic_name'] = f"/model/{namespace}/cmd_vel"
                new_entry['direction'] = 'BIDIRECTIONAL'
            elif gz_topic == "odom":
                new_entry['gz_topic_name'] = f"/model/{namespace}/odometry"
            elif gz_topic == "tf":
                new_entry['gz_topic_name'] = f"/world/default/pose/info"
            elif gz_topic == "joint_states":
                new_entry['gz_topic_name'] = f"/world/default/model/{namespace}/joint_state"
            elif gz_topic == "imu":
                new_entry['gz_topic_name'] = f"/{namespace}/imu"
            elif gz_topic == "scan":
                new_entry['gz_topic_name'] = f"/{namespace}/scan"

        if 'ros_topic_name' in new_entry and 'cmd_vel' in new_entry['ros_topic_name']:
            if 'ros_type_name' in new_entry:
                new_entry['ros_type_name'] = 'geometry_msgs/msg/Twist'

        namespaced_config.append(new_entry)

    # ADD GROUND TRUTH BRIDGE ENTRY
    ground_truth_entry = {
        'ros_topic_name': f'/{namespace}/ground_truth_pose',
        'gz_topic_name': f'/model/{namespace}/pose',
        'gz_type_name': 'gz.msgs.Pose',
        'ros_type_name': 'geometry_msgs/msg/PoseStamped',
        'direction': 'GZ_TO_ROS'
    }
    namespaced_config.append(ground_truth_entry)

    tmp_file = tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='.yaml')
    yaml.dump(namespaced_config, tmp_file)
    tmp_file.close()
    return tmp_file.name


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    try:
        tb3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    except Exception:
        tb3_gazebo_dir = os.path.join(os.path.expanduser('~'), 'turtlebot3_ws/install/turtlebot3_gazebo/share/turtlebot3_gazebo')

    try:
        ros_gz_sim_dir = get_package_share_directory('ros_gz_sim')
    except Exception:
        ros_gz_sim_dir = get_package_share_directory('ros_gz_sim')

    world_file = os.path.join(tb3_gazebo_dir, 'worlds', 'empty_world.world')
    turtlebot_model_dir = os.path.join(tb3_gazebo_dir, 'models', 'turtlebot3_burger')
    model_sdf_path = os.path.join(turtlebot_model_dir, 'model.sdf')
    bridge_yaml_path = os.path.join(tb3_gazebo_dir, 'params', 'turtlebot3_burger_bridge.yaml')

    # UPDATED: 4 Robots, clustered tightly near the center
    robots = [
        {'name': 'tb1', 'x':  0.5, 'y':  0.0},
        {'name': 'tb2', 'x': -0.5, 'y':  0.0},
        {'name': 'tb3', 'x':  0.0, 'y':  0.5},
        {'name': 'tb7', 'x':  0.0, 'y': -0.5},
    ]

    ld = LaunchDescription()

    gzserver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r -s -v4 {world_file}',
            'on_exit_shutdown': 'true'
        }.items()
    )
    ld.add_action(gzserver_launch)

    gzclient_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-g -v4'}.items()
    )
    ld.add_action(gzclient_launch)

    # --- Spawn robots AND Controllers ---
    for robot in robots:
        ns = robot['name']

        bridge_yaml_namespaced = _make_namespaced_bridge_yaml(bridge_yaml_path, ns)
        namespaced_sdf = _make_namespaced_sdf(model_sdf_path, ns)

        robot_group = GroupAction([
            PushRosNamespace(ns),

            # 1. Spawn Robot
            Node(
                package='ros_gz_sim',
                executable='create',
                name=f'{ns}_create',
                arguments=[
                    '-name', ns,
                    '-file', namespaced_sdf,
                    '-x', str(robot['x']),
                    '-y', str(robot['y']),
                    '-z', '0.01'
                ],
                output='screen'
            ),

            # 2. Bridge
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name=f'{ns}_bridge',
                parameters=[{'config_file': bridge_yaml_namespaced}],
                output='screen'
            ),
            
            # 3. SWARM CONTROLLER NODE
            Node(
                package='turtlebot_simulation_inout_linearization',
                executable='distributed_controller_consensus_collision',
                name=f'{ns}_controller',
                # DELETED namespace=ns here to prevent double namespace (/tb1/tb1)
                parameters=[{
                    'robot_namespace': ns,
                    'pose_topic': f'/{ns}/ground_truth_pose',
                    'sim_mode': False,            # Let Mock EKF handle neighborhood topics
                    'use_twist_msg': True,        # Forces pure Twist for Gazebo bridge compatibility
                    'control_mode': 'dynamic_trajectory',
                    'traj_type': 'circle',
                    'alpha': 1.0,
                    'beta': 0.25,
                    'rho': 0.6,
                    'traj_center_x': 0.0,
                    'traj_center_y': 0.0,
                    'traj_radius': 0.6,
                    'traj_w': 0.12,
                    'm11': 0.36,
                    'm22': 0.10,
                    'is_rotating': True,
                    'ellipse_alpha': 0.0,
                    'b': 0.1,
                    'max_v': 0.18,
                    'max_w': 1.5
                }],
                output='screen'
            ),

            # 4. MOCK EKF OBSERVER NODE
            Node(
                package='turtlebot_simulation_inout_linearization',
                executable='mock_ekf_node',
                name=f'{ns}_mock_ekf',
                # DELETED namespace=ns here to prevent double namespace (/tb1/tb1)
                parameters=[{'observer_namespace': ns}],
                output='screen'
            )
        ])
        ld.add_action(robot_group)

    return ld