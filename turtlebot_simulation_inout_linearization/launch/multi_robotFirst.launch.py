#!/usr/bin/env python3

"""
Multi-Robot TurtleBot3 Gazebo Simulation Launch File

PURPOSE: Launches a complete multi-robot simulation environment with ROS 2 control
FEATURES: 
  - Spawns multiple TurtleBot3 robots in Gazebo with proper namespace isolation
  - Sets up ROS-Gazebo bridge for each robot with topic remapping
  - Provides trajectory generation and control for autonomous navigation
  - Supports input/output linearization control with point offset

KEY COMPONENTS:
1. GAZEBO SIMULATION: Empty world with multiple TurtleBot3 robots
2. TOPIC BRIDGING: Dynamic ROS-Gazebo topic mapping for each namespace
3. ROBOT SPAWNING: Automated robot placement with modified SDF models
4. CONTROL SYSTEM: I/O linearization controller with smooth trajectory generation

ROBOT CONFIGURATION:
  - burger1: Controlled robot with trajectory following (0,0)
  - burger2: Additional robot for multi-agent testing (10,10) 
  - burger3: Additional robot for multi-agent testing (-10,-10)

TOPIC MAPPING STRATEGY:
  - ROS: /burger1/cmd_vel → Gazebo: /model/burger1/cmd_vel
  - ROS: /burger1/odom ← Gazebo: /model/burger1/odometry
  - Each robot gets isolated topic namespace to prevent conflicts

CONTROL SYSTEM:
  - Input/Output Linearization: Mathematical exact linearization via feedback
  - Point B Control: Controls point at offset b from robot center for stability
  - Smooth Trajectories: 5th-order polynomial for continuous motion

USAGE:
  ros2 launch your_package multi_robot_simulation.launch.py

DEBUG FEATURES: Extensive console output for bridge configuration and SDF modifications
"""
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
    """
    Create a temporary SDF file with proper topic remapping for all plugins.
    """
    with open(original_sdf_path, 'r') as f:
        sdf_content = f.read()

    # CRITICAL FIX: Replace ALL odometry-related configurations
    replacements = {
        '<odom_topic>odom</odom_topic>': f'<odom_topic>/model/{namespace}/odometry</odom_topic>',
        '<frame_id>odom</frame_id>': f'<odom_frame>odom</odom_frame>',
        '<child_frame_id>base_footprint</child_frame_id>': f'<robot_base_frame>base_footprint</robot_base_frame>',
        '<topic>cmd_vel</topic>': f'<topic>/model/{namespace}/cmd_vel</topic>'
    }
    
    for old, new in replacements.items():
        sdf_content = sdf_content.replace(old, new)

    # Also replace model name
    sdf_content = sdf_content.replace('__model__', namespace)

    # DEBUG: Print the fixed diff_drive plugin section
    print(f"=== CHECKING FIXED DIFFDRIVE PLUGIN FOR {namespace} ===")
    start = sdf_content.find('<plugin filename="gz-sim-diff-drive-system"')
    if start != -1:
        end = sdf_content.find('</plugin>', start) + len('</plugin>')
        plugin_section = sdf_content[start:end]
        print("DIFFDRIVE PLUGIN CONTENT:")
        print(plugin_section)
        
        # Check if our fixes were applied
        if f'/model/{namespace}/odometry' in plugin_section:
            print(f"ODOMETRY TOPIC SUCCESSFULLY SET TO: /model/{namespace}/odometry")
        else:
            print("ODOMETRY TOPIC NOT SET CORRECTLY")
            
        if f'/model/{namespace}/cmd_vel' in plugin_section:
            print(f"CMD_VEL TOPIC SUCCESSFULLY SET TO: /model/{namespace}/cmd_vel")
        else:
            print("CMD_VEL TOPIC NOT SET CORRECTLY")
    else:
        print("DIFFDRIVE PLUGIN NOT FOUND")
    print("=========================================")

    tmp_sdf = tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='.sdf')
    tmp_sdf.write(sdf_content)
    tmp_sdf.close()
    
    print(f"=== MODIFIED SDF SAVED TO: {tmp_sdf.name} ===")
    return tmp_sdf.name

def _make_namespaced_bridge_yaml(original_yaml_path, namespace):
    with open(original_yaml_path, 'r') as f:
        bridge_config = yaml.safe_load(f)

    namespaced_config = []
    for entry in bridge_config:
        new_entry = entry.copy()

        # Prefix ROS topic with namespace
        if 'ros_topic_name' in new_entry:
            ros_topic = new_entry['ros_topic_name'].lstrip('/')
            new_entry['ros_topic_name'] = f"/{namespace}/{ros_topic}"

        # Map Gazebo topics correctly
        if 'gz_topic_name' in new_entry:
            gz_topic = new_entry['gz_topic_name']
            
            if gz_topic == "cmd_vel":
                new_entry['gz_topic_name'] = f"/model/{namespace}/cmd_vel"
                # CRITICAL: Ensure cmd_vel is bidirectional
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

        # Ensure cmd_vel uses Twist (not TwistStamped)
        if 'ros_topic_name' in new_entry and 'cmd_vel' in new_entry['ros_topic_name']:
            if 'ros_type_name' in new_entry:
                new_entry['ros_type_name'] = 'geometry_msgs/msg/Twist'

        namespaced_config.append(new_entry)

    # Debug output
    print(f"=== FINAL BRIDGE CONFIG FOR {namespace} ===")
    for entry in namespaced_config:
        direction = entry.get('direction', 'NOT_SET')
        print(f"ROS: {entry.get('ros_topic_name')} -> GZ: {entry.get('gz_topic_name')} (Dir: {direction})")
    print("=========================================")

    tmp_file = tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='.yaml')
    yaml.dump(namespaced_config, tmp_file)
    tmp_file.close()
    return tmp_file.name

def generate_launch_description():
    # --- Launch configs ---
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # --- Package paths ---
    try:
        tb3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    except PackageNotFoundError:
        # Fallback to your workspace installation
        tb3_gazebo_dir = os.path.join(os.path.expanduser('~'), 'turtlebot3_ws/install/turtlebot3_gazebo/share/turtlebot3_gazebo')

    try:
        ros_gz_sim_dir = get_package_share_directory('ros_gz_sim')
    except PackageNotFoundError:
        # For ROS 2 Jazzy, it might be named differently
        ros_gz_sim_dir = get_package_share_directory('ros_gz_sim')
    # --- World file ---
    world_file = os.path.join(tb3_gazebo_dir, 'worlds', 'empty_world.world')

    # --- TurtleBot3 model ---
    turtlebot_model_dir = os.path.join(tb3_gazebo_dir, 'models', 'turtlebot3_burger')
    model_sdf_path = os.path.join(turtlebot_model_dir, 'model.sdf')

    # --- Original bridge YAML ---
    bridge_yaml_path = os.path.join(tb3_gazebo_dir, 'params', 'turtlebot3_burger_bridge.yaml')

    # --- Robots definition ---
    robots = [
        {'name': 'burger1', 'x': 0.0, 'y': 0.0},
        {'name': 'burger2', 'x': 10.0, 'y': 10.0},
        {'name': 'burger3', 'x': -10.0, 'y': -10.0},
    ]

    ld = LaunchDescription()

    # --- Gazebo server ---
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

    # --- Gazebo client ---
    gzclient_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-g -v4'}.items()
    )
    ld.add_action(gzclient_launch)

    # --- Spawn robots ---
    for robot in robots:
        ns = robot['name']

        # Patch bridge YAML
        bridge_yaml_namespaced = _make_namespaced_bridge_yaml(bridge_yaml_path, ns)

        # Patch SDF for this robot (only topic, plugin name stays same)
        namespaced_sdf = _make_namespaced_sdf(model_sdf_path, ns)

        robot_group = GroupAction([
            PushRosNamespace(ns),

            # Spawn robot in Gazebo with patched SDF
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

            # ROS-Gazebo bridge
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name=f'{ns}_bridge',
                parameters=[{'config_file': bridge_yaml_namespaced}],
                output='screen'
            ),
        ])
        ld.add_action(robot_group)

        # --- Add controller for burger1 ---
    ld.add_action(Node(
        package='turtlebot_simulation_inout_linearization',
        executable='sim_controller',
        name='burger1_controller',
        namespace='burger1',
        parameters=[{
            'robot_namespace': 'burger1',
            'Kx': 2.0, 
            'Ky': 2.0,
            'b': 0.2, 
        }],
        output='screen'
    ))

    ld.add_action(Node(
        package='turtlebot_simulation_inout_linearization',
        executable='sim_trajectory_generator',
        name='burger1_trajectory',
        namespace='burger1',
        parameters=[{
            'robot_namespace': 'burger1',
            'start_x': 0.0,
            'start_y': 0.0,
            'goal_x': 1.0,
            'goal_y': 1.0,
            'total_time': 15.0,
        }],
        output='screen'
    ))

    return ld

