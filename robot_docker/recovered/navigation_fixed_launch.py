import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    namespece = LaunchConfiguration('namespece', default='')
    map_yaml_path = LaunchConfiguration(
        'maps', default='/root/Broom.yaml')
    nav2_param_path = LaunchConfiguration(
        'params_file', default='/root/dwb_nav_params_fixed.yaml')

    scan_fix_process = ExecuteProcess(
        cmd=['python3', '/root/scan_time_fix.py'],
        output='screen'
    )

    odom_relay_process = ExecuteProcess(
        cmd=['python3', '/root/odom_relay.py'],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value=use_sim_time),
        DeclareLaunchArgument('namespece', default_value=namespece),
        DeclareLaunchArgument('maps', default_value=map_yaml_path),
        DeclareLaunchArgument('params_file', default_value=nav2_param_path),

        scan_fix_process,
        odom_relay_process,

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [nav2_bringup_dir, '/launch', '/bringup_launch.py']),
            launch_arguments={
                'map': map_yaml_path,
                'use_sim_time': use_sim_time,
                'namespece': namespece,
                'params_file': nav2_param_path}.items(),
        ),
        Node(
            package='yahboomcar_nav',
            executable='stop_car'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_footprint_to_base_link',
            arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_base_laser',
            arguments=['-0.0046412', '0', '0.094079', '0', '0', '0', 'base_link', 'laser_frame']
        ),
    ])
