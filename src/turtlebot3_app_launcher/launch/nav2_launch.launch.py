import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    SetEnvironmentVariable,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ParameterFile, ComposableNode
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    launch_dir = get_package_share_directory('turtlebot3_app_launcher')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    autostart = LaunchConfiguration('autostart', default = 'true')
    params_file = LaunchConfiguration('params_file', default = 
        os.path.join(launch_dir, 'config','burger.yaml'))
    map_dir = LaunchConfiguration('map', default = 
        os.path.join(launch_dir, 'map', 'map_default.yaml'))
    
    lifecycle_nodes = [
        'map_server', 
        'amcl',
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',]
    
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]
    
    param_substitutions_map = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_dir}
    configured_params_map = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            param_rewrites=param_substitutions_map,
            convert_types=True),
        allow_substs=True)
    
    param_substitutions= {
        'use_sim_time': use_sim_time,
        'autostart': autostart}
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            param_rewrites=param_substitutions,
            convert_types=True),
        allow_substs=True)
    
    container = ComposableNodeContainer(
        name='custom_nav2_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_isolated',
        parameters=[configured_params_map],
            composable_node_descriptions=[
                ComposableNode(
                    package='nav2_map_server',
                    plugin='nav2_map_server::MapServer',
                    name='map_server',
                    parameters=[configured_params_map],
                    remappings=remappings),
                ComposableNode(
                    package='nav2_amcl',
                    plugin='nav2_amcl::AmclNode',
                    name='amcl',
                    parameters=[configured_params_map],
                    remappings=remappings),
                ComposableNode(
                    package='nav2_controller',
                    plugin='nav2_controller::ControllerServer',
                    name='controller_server',
                    parameters=[configured_params],
                    remappings=remappings),
                ComposableNode(
                    package='nav2_planner',
                    plugin='nav2_planner::PlannerServer',
                    name='planner_server',
                    parameters=[configured_params],
                    remappings=remappings),
                ComposableNode(
                    package='nav2_behaviors',
                    plugin='behavior_server::BehaviorServer',
                    name='behavior_server',
                    parameters=[configured_params],
                    remappings=remappings),
                ComposableNode(
                    package='nav2_bt_navigator',
                    plugin='nav2_bt_navigator::BtNavigator',
                    name='bt_navigator',
                    parameters=[configured_params],
                    remappings=remappings),
                ComposableNode(
                    package='nav2_lifecycle_manager',
                    plugin='nav2_lifecycle_manager::LifecycleManager',
                    name='lifecycle_manager_navigation',
                    parameters=[{'use_sim_time': use_sim_time,
                                'autostart': autostart,
                                'node_names': lifecycle_nodes}]),
                ],
            output='screen',
    )

    return LaunchDescription([
        SetEnvironmentVariable(
            'RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Automatically startup the nav2 stack'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(launch_dir, 'config', 'burger.yaml'),
            description='Full path to the ROS2 parameters file to use for all launched nodes'),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(launch_dir, 'map', 'map_default.yaml'),
            description='Full path to map yaml file to load'),

        container,

        
    ])
    