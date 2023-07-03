from setuptools import setup

package_name = 'drone_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gugafelds',
    maintainer_email='gustavo.tenoriobritodebarros@rub.de',
    description='Drone control for Gazebo Simulation in PX4-ROS2 Bridge',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'route_publisher = drone_control.route_publisher:main',
            'setpoint_publisher = drone_control.setpoint_publisher:main',
            'circle_flight = drone_control.circle_flight:main'
        ],
    },
)
