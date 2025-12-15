
# from launch import LaunchDescription
# from launch_ros.actions import Node
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration

# def generate_launch_description():
    
#     # Define launch arguments
#     robot_namespace = DeclareLaunchArgument(
#         'robot_namespace',
#         default_value='burger1',
#         description='Namespace for the robot'
#     )
    
#     # # Single robot nodes - REAL TURTLEBOT3
#     # trajectory_node = Node(
#     #     package='my_turtlebot_realFirst',
#     #     executable='trajectory_generatorFirst',
#     #     output='screen',
#     #     name='trajectory_generator',
#     #     namespace=LaunchConfiguration('robot_namespace'),
#     #     parameters=[{
#     #         'robot_namespace': LaunchConfiguration('robot_namespace'),
#     #         'start_x': 0.0,
#     #         'start_y': 0.0,
#     #         'goal_x': 1.0,
#     #         'goal_y': 1.0,
#     #         'total_time': 10.0,
#     #         'max_velocity': 0.1
#     #     }]
#     # )
    
#     controller_node = Node(
#         package='my_turtlebot_realFirst',
#         executable='controller',
#         output='screen',
#         name='controller',
#         namespace=LaunchConfiguration('robot_namespace'),
#         parameters=[{
#             'robot_namespace': LaunchConfiguration('robot_namespace'),
#             'Kx': 2.0,
#             'Ky': 2.0,
#             'b': 0.1,
#             'max_linear_vel': 0.15,
#             'max_angular_vel': 0.8
#         }]
#     )

#     return LaunchDescription([
#         robot_namespace,
#         #trajectory_node,
#         controller_node,
#     ])

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # --- CONFIGURATION ---
    # List the names of all your robots here. 
    # This is the only place you need to edit to add/remove robots.
    robot_list = ['tb1', 'tb3', 'tb6'] 
    
    # Controller Parameters (Common to all robots)
    common_params = {
        'Kx': 2.0,
        'Ky': 2.0,
        'b': 0.1,
        'max_v': 0.15,  # Note: matched to your python code variable name 'max_v'
        'max_w': 0.8,
        'goal_tolerance': 0.05
    }

    # List to hold all launch actions
    ld_actions = []

    # --- GENERATE NODES LOOP ---
    for robot_name in robot_list:
        
        # Define the OptiTrack topic based on your logic: "/tbX/pose"
        pose_topic_name = f'/{robot_name}/pose'
        
        # Create the node instance for this specific robot
        node = Node(
            package='my_turtlebot_realFirst',
            executable='controller',
            name='controller',
            namespace=robot_name, # Pushes node to /tbX/controller
            output='screen',
            emulate_tty=True,
            parameters=[
                common_params,
                {
                    # Specific overrides for this robot
                    'robot_namespace': robot_name,
                    'pose_topic': pose_topic_name
                }
            ]
        )
        
        ld_actions.append(node)

    return LaunchDescription(ld_actions)