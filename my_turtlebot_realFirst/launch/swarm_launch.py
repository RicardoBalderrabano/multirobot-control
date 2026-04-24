import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Define the robots in your swarm
    robot_namespaces = ['tb1', 'tb3', 'tb7']
    
    # MODIFICATION 1: Put your actual package name here
    my_package_name = 'my_turtlebot_realFirst' # <--- CHANGE THIS 

    # Path to your shared YAML parameter file
    config_file = os.path.join(
        get_package_share_directory(my_package_name),
        'config',
        'swarm_params.yaml'
    )

    nodes = []

    # Dynamically generate a node for each robot
    for ns in robot_namespaces:
        nodes.append(
            Node(
                package=my_package_name, 
                
                # MODIFICATION 2: Update to your new script name (no .py)
                executable='distributed_controller_consensus_collision', 
                
                # MODIFICATION 3: Give the running node a clean name
                name='distributed_controller',
                
                namespace=ns,
                output='screen',
                parameters=[
                    config_file,  # Load the shared Alpha/Beta gains
                    {'robot_namespace': ns},          
                    {'pose_topic': f'/{ns}/pose'} # <-- Make sure this matches the parameter name in your python script!
                ]
            )
        )

    return LaunchDescription(nodes)
