"""
from setuptools import setup

package_name = 'my_turtlebot_realFirst'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Added your new launch file to the list below so ROS can find it
        ('share/' + package_name + '/launch', [
            'launch/real_robotFirst.launch.py',
            'launch/filter_playback.launch.py' 
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='TurtleBot3 real robot implementation',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller = my_turtlebot_realFirst.controller:main',
            'trajectory_generatorFirst = my_turtlebot_realFirst.trajectory_generatorFirst:main',
            'optitrack_bridge_node = my_turtlebot_realFirst.optitrack_bridge_node:main',
            'fleet_commander = my_turtlebot_realFirst.fleet_commander:main',
            'simple_tracker = my_turtlebot_realFirst.simple_tracker:main',
            # --- NEW NODES ADDED BELOW ---
            'multi_global_transformer = my_turtlebot_realFirst.global_pose:main',
            'kalman_observer = my_turtlebot_realFirst.kalmanObserver_v2:main',            
            'distributed_controller_consensus_collision = my_turtlebot_realFirst.distributed_controller_consensus_collision:main',
        ],
    },
)
"""

from setuptools import setup

package_name = 'my_turtlebot_realFirst'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Added your new launch file to the list below so ROS can find it
        ('share/' + package_name + '/launch', [
            'launch/real_robotFirst.launch.py',
            'launch/filter_playback.launch.py',
            'launch/swarm_launch.py'
        ]),
        ('share/' + package_name + '/config', [
            'config/swarm_params.yaml'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='TurtleBot3 real robot implementation',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller = my_turtlebot_realFirst.controller:main',
            'trajectory_generatorFirst = my_turtlebot_realFirst.trajectory_generatorFirst:main',
            'optitrack_bridge_node = my_turtlebot_realFirst.optitrack_bridge_node:main',
            'fleet_commander = my_turtlebot_realFirst.fleet_commander:main',
            'simple_tracker = my_turtlebot_realFirst.simple_tracker:main',
            # --- NEW NODES ADDED BELOW ---
            'multi_global_transformer = my_turtlebot_realFirst.global_pose:main',
            'kalman_observer = my_turtlebot_realFirst.kalmanObserver_v2:main',            
            'distributed_controller_consensus_collision = my_turtlebot_realFirst.distributed_controller_consensus_collision:main',
        ],
    },
)
