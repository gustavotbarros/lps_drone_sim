from setuptools import setup

package_name = 'drone_command'

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
    description='Drone command center for simulation in Gazebo with PX4-ROS2 Bridge',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control_gui = drone_command.control_gui:main'
        ],
    },
)
