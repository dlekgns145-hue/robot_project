"""
전체 실행용 launch 파일 (start.launch.py)
-----------------------------------------
현재는 1순위 목표(카메라 -> YOLO -> Follow Me)만 켜도록 구성되어 있음.
SLAM / Navigation 노드는 준비되면 아래 주석을 해제해서 추가하면 됨.

실행 방법:
    ros2 launch robot_project start.launch.py
    ros2 launch robot_project start.launch.py linear_speed:=0.4   # 속도 바꿔서 테스트 (STEP1)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    linear_speed_arg = DeclareLaunchArgument(
        'linear_speed', default_value='0.2',
        description='Follow Me 전진 속도 (STEP1: 이 값을 바꿔가며 테스트)',
    )

    return LaunchDescription([
        linear_speed_arg,

        # TODO: 카메라 드라이버 노드 - 실제 사용 중인 카메라 패키지에 맞게 수정
        # (예시: v4l2_camera 사용 시)
        # Node(
        #     package='v4l2_camera',
        #     executable='v4l2_camera_node',
        #     name='camera_node',
        #     output='screen',
        # ),

        Node(
            package='robot_project',
            executable='detect',
            name='yolo_detect_node',
            output='screen',
        ),

        Node(
            package='robot_project',
            executable='follow_person',
            name='follow_person_node',
            output='screen',
            parameters=[{
                'linear_speed': LaunchConfiguration('linear_speed'),
            }],
        ),

        # TODO: Navigation(STEP 3) 준비되면 주석 해제
        # Node(
        #     package='robot_project',
        #     executable='nav',
        #     name='nav_goal_node',
        #     output='screen',
        # ),
    ])
