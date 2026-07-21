"""Launch the car_bridge node.

Usage:
    ros2 launch car_bridge bridge_launch.py
    ros2 launch car_bridge bridge_launch.py pi_src_path:=/home/pi/car/pi linear_scale:=200.0
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pi_src_path_arg = DeclareLaunchArgument(
        'pi_src_path', default_value='/home/pi/car/pi',
        description="Path to the repo's pi/ directory (put on sys.path so "
                    'esp32_link.py/imu.py/lidar.py can be imported unmodified).',
    )
    linear_scale_arg = DeclareLaunchArgument(
        'linear_scale', default_value='255.0',
        description='cmd_vel linear.x -> PWM scale (open-loop placeholder, not calibrated to m/s).',
    )
    angular_scale_arg = DeclareLaunchArgument(
        'angular_scale', default_value='150.0',
        description='cmd_vel angular.z -> PWM scale (open-loop placeholder, not calibrated to rad/s).',
    )

    bridge_node = Node(
        package='car_bridge',
        executable='bridge_node',
        name='car_bridge',
        output='screen',
        parameters=[{
            'pi_src_path': LaunchConfiguration('pi_src_path'),
            'linear_scale': LaunchConfiguration('linear_scale'),
            'angular_scale': LaunchConfiguration('angular_scale'),
        }],
    )

    return LaunchDescription([
        pi_src_path_arg,
        linear_scale_arg,
        angular_scale_arg,
        bridge_node,
    ])
