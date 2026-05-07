#!/usr/bin/env python3

import os
import yaml
import tempfile
import time
import math

# --- IMPORTS FOR DIGITAL TWIN SNIFFER ---
import rclpy
from rclpy.node import Node as RclpyNode
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
# --------------------------------------------

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import GroupAction

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, OpaqueFunction
# ==============================================================================
# DIGITAL TWIN INITIALIZATION (OPTITRACK SNIFFER)
# ==============================================================================
def euler_yaw_from_quaternion(x, y, z, w):
    """Calculates Yaw from Quaternion without needing tf_transformations dependency."""
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    return math.atan2(t3, t4)

def get_real_robot_poses(robot_names, timeout=3.0):
    """Spins a temporary node to fetch real OptiTrack poses before spawning."""
    rclpy.init()
    node = RclpyNode('initial_pose_sniffer')
    
    poses = {name: None for name in robot_names}
    
    # Dynamic callback generator for each robot
    def make_cb(name):
        def cb(msg):
            if poses[name] is None:
                q = msg.pose.orientation
                yaw = euler_yaw_from_quaternion(q.x, q.y, q.z, q.w)
                poses[name] = {'x': msg.pose.position.x, 'y': msg.pose.position.y, 'yaw': yaw}
        return cb
        
    # --- QOS FIX APPLIED HERE ---
    qos_best_effort = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=1
    )
    
    subs = []
    for name in robot_names:
        subs.append(node.create_subscription(PoseStamped, f'/{name}/pose', make_cb(name), qos_best_effort))
        
    print("\n--- 📡 SYNCING DIGITAL TWIN WITH OPTITRACK ---")
    print(f"Waiting up to {timeout} seconds for real poses...")
    
    start_time = time.time()
    while time.time() - start_time < timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
        if all(p is not None for p in poses.values()):
            break
            
    node.destroy_node()
    rclpy.shutdown()
    
    # Fallback dictionary in case OptiTrack is off or a robot is missing
    defaults = {
        'tb1': {'x':  -0.75, 'y':  -1.8, 'yaw': 1.57},
        'tb2': {'x': -0.75, 'y':  -2.2, 'yaw': 1.57},
        'tb3': {'x':  0.75, 'y':  -1.8, 'yaw': 1.57},
        'tb7': {'x':  0.75, 'y': -2.2, 'yaw': 1.57}
    }
    
    results = []
    for name in robot_names:
        if poses[name] is not None:
            print(f"✅ Found {name.upper()}: x={poses[name]['x']:.2f}, y={poses[name]['y']:.2f}, yaw={poses[name]['yaw']:.2f}")
            results.append({'name': name, 'x': poses[name]['x'], 'y': poses[name]['y'], 'yaw': poses[name]['yaw']})
        else:
            print(f"⚠️ Timeout for {name.upper()}. Using default fallback cluster.")
            results.append({'name': name, 'x': defaults[name]['x'], 'y': defaults[name]['y'], 'yaw': defaults[name]['yaw']})
            
    print("----------------------------------------------\n")
    return results
# ==============================================================================


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
    # 1. DECLARE LAUNCH ARGUMENTS
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    control_mode_arg = DeclareLaunchArgument('control_mode', default_value='tangent_mapping')
    traj_type_arg = DeclareLaunchArgument('traj_type', default_value='circle')
    use_groups_arg = DeclareLaunchArgument('use_groups', default_value='false')

    # ==========================================================
    # THE MISSING PATH DEFINITIONS (Must be before launch_setup)
    # ==========================================================
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
    
    # THESE ARE THE VARIABLES IT WAS LOOKING FOR:
    model_sdf_path = os.path.join(turtlebot_model_dir, 'model.sdf')
    bridge_yaml_path = os.path.join(tb3_gazebo_dir, 'params', 'turtlebot3_burger_bridge.yaml')
    # ==========================================================

    robot_names_list = ['tb1', 'tb2', 'tb3', 'tb7']
    robots = get_real_robot_poses(robot_names_list)

    ld = LaunchDescription()
    
    ld.add_action(use_sim_time_arg)
    ld.add_action(control_mode_arg)
    ld.add_action(traj_type_arg)
    ld.add_action(use_groups_arg)

    # Launch Gazebo
    gzserver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r -s -v4 {world_file}', 'on_exit_shutdown': 'true'}.items()
    )
    ld.add_action(gzserver_launch)

    gzclient_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-g -v4'}.items()
    )
    ld.add_action(gzclient_launch)

    # 2. THE OPAQUE FUNCTION
    def launch_setup(context, *args, **kwargs):
        use_groups = LaunchConfiguration('use_groups').perform(context).lower() == 'true'
        current_control_mode = LaunchConfiguration('control_mode').perform(context)
        current_traj_type = LaunchConfiguration('traj_type').perform(context)
        
        spawn_actions = []

        for robot in robots:
            ns = robot['name']

            # Now it can safely access these variables from the outer scope!
            bridge_yaml_namespaced = _make_namespaced_bridge_yaml(bridge_yaml_path, ns)
            namespaced_sdf = _make_namespaced_sdf(model_sdf_path, ns)

            # =========================================================
            # DYNAMIC GROUP & OFFSET LOGIC
            # =========================================================
            if use_groups:
                if ns in ['tb1', 'tb2']:
                    assigned_group = ['tb1', 'tb2']
                    assigned_center_x = -0.75   # <-- Shift Gamma 1 UP by 1 meter
                elif ns in ['tb3', 'tb7']:    # (Fixed tb4 to tb7 to match your robots)
                    assigned_group = ['tb3', 'tb7']
                    assigned_center_x = 0.75  # <-- Shift Gamma 2 DOWN by 1 meter
                else:
                    assigned_group = [ns]
                    assigned_center_y = 0.0
            else:
                assigned_group = robot_names_list 
                assigned_center_y = 0.0
                assigned_center_x = 0.0  
            # =========================================================

            robot_group = GroupAction([
                PushRosNamespace(ns),

                Node(
                    package='ros_gz_sim',
                    executable='create',
                    name=f'{ns}_create',
                    arguments=[
                        '-name', ns, '-file', namespaced_sdf,
                        '-x', str(robot['x']), '-y', str(robot['y']),
                        '-z', '0.01', '-Y', str(robot['yaw'])
                    ],
                    output='screen'
                ),

                Node(
                    package='ros_gz_bridge',
                    executable='parameter_bridge',
                    name=f'{ns}_bridge',
                    parameters=[{'config_file': bridge_yaml_namespaced}],
                    output='screen'
                ),
                
                Node(
                    package='turtlebot_simulation_inout_linearization',
                    executable='distributed_controller_consensus_collision',
                    name=f'{ns}_controller',
                    parameters=[{
                        'robot_namespace': ns,
                        'pose_topic': f'/{ns}/ground_truth_pose',
                        'sim_mode': False,            
                        'use_twist_msg': True,        
                        
                        'control_mode': current_control_mode,  
                        'traj_type': current_traj_type,
                        
                        'traj_amplitude': 0.6,           
                        'traj_frequency': 1.5,           
                        'traj_v_x': 0.10,                
                        
                        'alpha': 1.0, 'beta': 0.25, 'rho': 1.0,
                        'goal_x': 0.0, 'goal_y': 0.0,
                        'traj_center_x': assigned_center_x, 'traj_center_y': -1.8,
                        'traj_radius': 1.0, 'traj_w': 0.08,
                        'm11': 1.0, 'm22': 1.0, 'is_rotating': False,
                        'ellipse_alpha': 0.0, 'b': 0.1,
                        'max_v': 0.18, 'max_w': 1.5,
                        'K': 0.8, 'y_bar': 1.0, 'safe_dist': 0.2,
                        
                        'group_members': assigned_group 
                    }],
                    output='screen'
                ),

                Node(
                    package='turtlebot_simulation_inout_linearization',
                    executable='mock_ekf_node',
                    name=f'{ns}_mock_ekf',
                    parameters=[{'observer_namespace': ns}],
                    output='screen'
                )
            ])
            spawn_actions.append(robot_group)

        return spawn_actions

    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld