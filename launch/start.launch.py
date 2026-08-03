"""Launch the integrated Perception, Follow Me, or Navigation runner.

Examples:
    ros2 launch robot_project start.launch.py mode:=perception
    ros2 launch robot_project start.launch.py mode:=follow linear_speed:=0.3
    ros2 launch robot_project start.launch.py mode:=navigation goal_x:=1.0 goal_y:=0.5
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument(
            'mode', default_value='follow',
            description='perception, follow, or navigation',
        ),
        DeclareLaunchArgument(
            'linear_speed', default_value='0.2',
            description='Follow Me forward speed',
        ),
        DeclareLaunchArgument('camera_topic', default_value='/camera/image_raw'),
        DeclareLaunchArgument('model_path', default_value='perception/best.pt'),
        DeclareLaunchArgument('goal_x', default_value='1.0'),
        DeclareLaunchArgument('goal_y', default_value='0.5'),
        DeclareLaunchArgument('goal_yaw', default_value='0.0'),
    ]

    integrated_node = Node(
        package='robot_project',
        executable='integrated_main',
        output='screen',
        arguments=[
            '--mode', LaunchConfiguration('mode'),
            '--ros-args',
            '-p', ['linear_speed:=', LaunchConfiguration('linear_speed')],
            '-p', ['camera_topic:=', LaunchConfiguration('camera_topic')],
            '-p', ['model_path:=', LaunchConfiguration('model_path')],
            '-p', ['goal_x:=', LaunchConfiguration('goal_x')],
            '-p', ['goal_y:=', LaunchConfiguration('goal_y')],
            '-p', ['goal_yaw:=', LaunchConfiguration('goal_yaw')],
        ],
    )

    return LaunchDescription([*arguments, integrated_node])
