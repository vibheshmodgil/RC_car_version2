from setuptools import find_packages, setup

package_name = 'car_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/bridge_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='RC Car project',
    maintainer_email='vibheshmodgil@gmail.com',
    description=(
        'ROS2 bridge node translating cmd_vel/estop/imu/scan to and from '
        "the DevKit's REST/WS API and the Pi's IMU/LiDAR drivers."
    ),
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bridge_node = car_bridge.bridge_node:main',
        ],
    },
)
