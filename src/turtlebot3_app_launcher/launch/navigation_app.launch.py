from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import ThisLaunchFileDir

import os
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('turtlebot3_app_launcher')
    
    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(
            pkg_dir,
            'map',
            'test_map.yaml'))
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    param_file_name = 'burger.yaml'
    param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            get_package_share_directory('turtlebot3_app_launcher'),
            'config',
            param_file_name))

    rviz_config_dir = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'rviz',
        'nav2_default_view.rviz')
    
    return LaunchDescription([
        Node(
            package='pose_module',
            executable='posemodule',
            name='posemodule'),
        Node(
            package='main_module',
            executable='testthread',
            name='testthread'),
        Node(
            package='main_module',
            executable='navmodule',
            name='navmodule'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_dir, 'nav2_launch.launch.py')),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'params_file': param_dir}.items(),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
        
    ])