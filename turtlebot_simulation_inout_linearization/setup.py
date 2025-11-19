from setuptools import setup, find_packages

package_name = 'turtlebot_simulation_inout_linearization'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/multi_robotFirst.launch.py']),
        ('share/' + package_name + '/launch', ['launch/spawn_multi_turtlebot3.launch.py']),
        ('share/' + package_name + '/launch', ['launch/spawn_turtlebot3_with_name.launch.py']),
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
        ],
    },
)