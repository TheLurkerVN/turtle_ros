from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='main_module',
            executable='keymodule',
            name='keymodule'),
        Node(
            package='main_module',
            executable='launchmodule',
            name='launchmodule'),
    ])