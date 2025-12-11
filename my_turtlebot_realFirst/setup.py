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
        ('share/' + package_name + '/launch', ['launch/real_robotFirst.launch.py']),
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
        ],
    },
)