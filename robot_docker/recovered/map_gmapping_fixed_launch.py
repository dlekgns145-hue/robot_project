from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    scan_fix_process = ExecuteProcess(
        cmd=['python3', '/root/scan_time_fix.py'],
        output='screen'
    )

    slam_gmapping_node = Node(
        package='slam_gmapping',
        executable='slam_gmapping',
        output='screen',
        parameters=[os.path.join(
            get_package_share_directory("slam_gmapping"), "params", "slam_gmapping.yaml"
        )],
        remappings=[('/scan', '/scan_fixed')]
    )

    # base_footprint <-> base_link 연결 (gmapping 내부가 base_link를 고정적으로 찾음)
    base_footprint_to_base_link_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_footprint_to_base_link',
        arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link']
    )

    # base_link -> laser_frame (원래 launch 파일이 쓰던 방식으로 복귀)
    base_link_to_laser_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_base_laser',
        arguments=['-0.0046412', '0', '0.094079', '0', '0', '0', 'base_link', 'laser_frame']
    )

    return LaunchDescription([
        scan_fix_process,
        slam_gmapping_node,
        base_footprint_to_base_link_tf_node,
        base_link_to_laser_tf_node
    ])
