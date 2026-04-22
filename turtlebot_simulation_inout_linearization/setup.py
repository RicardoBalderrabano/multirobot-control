from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'turtlebot_simulation_inout_linearization'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # The line below automatically grabs ALL .launch.py files in the launch folder!
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Input-Output Linearization control for TurtleBot3 simulation',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sim_controller = turtlebot_simulation_inout_linearization.controller_node:main',
            'sim_trajectory_generator = turtlebot_simulation_inout_linearization.trajectory_generator:main',
            # ADDED: Your new distributed controller executable
            'distributed_controller_consensus_collision = turtlebot_simulation_inout_linearization.distributed_controller_consensus_collision:main',
            'mock_ekf_node = turtlebot_simulation_inout_linearization.mock_ekf_node:main',
        ],
    },
)