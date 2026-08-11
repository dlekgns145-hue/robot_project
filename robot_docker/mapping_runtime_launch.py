"""mapping_runtime_launch.py - 자율 SLAM 매핑 런타임 ROS 2 Launch 파일
-----------------------------------------------------------------------
[매핑 기능 커스텀/뜯어고치기 안내]
이 launch 파일은 자율 매핑을 수행하기 위해 아래 노드들을 통합 실행합니다:

1. scan_time_fix.py:
   - 센서 타임스탬프 보정 및 노이즈/자기반사(Self-reflection) 필터링
   - /scan -> /scan_fixed (내비게이션용) 및 /scan_slam (SLAM용 4m 컷오프)

2. odom_relay.py:
   - 오도메트리 데이터 검증 및 /tf_nav 분리 리레이 (NaN/Inf 튀는 현상 방지)

3. slam_toolbox (async_slam_toolbox_node):
   - 2D LiDAR 데이터 기반 실시간 SLAM 점유 격자 지도(/map) 생성 노드

4. Nav2 (navigation_launch.py):
   - 경로 계획(Planner) 및 추종(Controller) 알고리즘 (DWB Planner)
   - 매핑 전용 파라미터 재설정 (yaw_goal_tolerance 3.14로 완화하여 목표 방향 맞춤 부담 감소)

5. map_saver (nav2_map_server):
   - 자율 탐색 완료 시 지도를 파일(*.pgm, *.yaml)로 저장하는 서버 서비스

6. autonomous_mapping.py:
   - 경계선(Frontier) 탐색 알고리즘을 수행하는 메인 조정자 노드
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from nav2_common.launch import RewrittenYaml



def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")
    runtime_dir = "/opt/robot-control/navigation"
    use_sim_time = LaunchConfiguration("use_sim_time")
    slam_params_path = LaunchConfiguration("slam_params_file")
    nav_params_path = LaunchConfiguration("nav_params_file")
    exploration_enabled = LaunchConfiguration("exploration_enabled")
    map_output = LaunchConfiguration("map_output")
    mapping_maximum_runtime = LaunchConfiguration("mapping_maximum_runtime")
    mapping_maximum_radius = LaunchConfiguration("mapping_maximum_radius")
    mapping_goal_progress_timeout = LaunchConfiguration(
        "mapping_goal_progress_timeout"
    )
    mapping_maximum_map_age = LaunchConfiguration("mapping_maximum_map_age")
    mapping_minimum_save_known_area = LaunchConfiguration(
        "mapping_minimum_save_known_area"
    )
    mapping_minimum_save_free_area = LaunchConfiguration(
        "mapping_minimum_save_free_area"
    )
    # Frontier exploration cares about reaching the free cell, not finishing
    # at an exact heading.  Keep normal saved-map navigation at its stricter
    # yaw tolerance and relax it only inside the mapping runtime.
    mapping_nav_params = RewrittenYaml(
        source_file=nav_params_path,
        root_key="",
        param_rewrites={
            "yaw_goal_tolerance": "3.14",
            # Mapping goals are deliberately placed on safe known-free cells.
            # A 0.5 m planner tolerance plus a 0.25 m goal tolerance allowed
            # Nav2 to report success while the robot was still about 0.7 m
            # from the frontier, causing an endless no-motion retry loop.
            "planner_server.ros__parameters.GridBased.tolerance": "0.10",
            "controller_server.ros__parameters.general_goal_checker.xy_goal_tolerance": "0.15",
            "controller_server.ros__parameters.FollowPath.xy_goal_tolerance": "0.15",
            # Mapping and exploration are deliberately LiDAR-only.  Camera
            # imagery may be georeferenced later, but it must never affect the
            # occupancy grid or the path used to create it.
            "observation_sources": "scan",
        },
        convert_types=True,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "slam_params_file",
                default_value=f"{runtime_dir}/mapping_slam_params.yaml",
            ),
            DeclareLaunchArgument(
                "nav_params_file",
                default_value=f"{runtime_dir}/dwb_nav_params_fixed.yaml",
            ),
            DeclareLaunchArgument("exploration_enabled", default_value="false"),
            DeclareLaunchArgument(
                "map_output",
                default_value="/opt/robot-control/maps/orchard_map",
            ),
            DeclareLaunchArgument(
                "mapping_maximum_runtime", default_value="900.0"
            ),
            DeclareLaunchArgument(
                "mapping_maximum_radius", default_value="8.0"
            ),
            DeclareLaunchArgument(
                "mapping_goal_progress_timeout", default_value="55.0"
            ),
            DeclareLaunchArgument(
                "mapping_maximum_map_age", default_value="8.0"
            ),
            DeclareLaunchArgument(
                "mapping_minimum_save_known_area", default_value="1.0"
            ),
            DeclareLaunchArgument(
                "mapping_minimum_save_free_area", default_value="0.5"
            ),
            GroupAction(
                [
                    SetRemap(src="/tf", dst="/tf_nav"),
                    # Nav2 may run on the robot or an external compute server.
                    # In both cases its command is only a short-lived proposal;
                    # robot_cmd_bridge performs the final local safety check.
                    SetRemap(src="/cmd_vel", dst="/cmd_vel_server"),
                    ExecuteProcess(
                        cmd=[
                            "python3",
                            f"{runtime_dir}/scan_time_fix.py",
                            "--ros-args",
                            "-r",
                            "__node:=navigation_scan_filter",
                            "-p",
                            "output_topic:=/scan_fixed",
                        ],
                        output="screen",
                    ),
                    ExecuteProcess(
                        cmd=[
                            "python3",
                            f"{runtime_dir}/scan_time_fix.py",
                            "--ros-args",
                            "-r",
                            "__node:=slam_scan_filter",
                            "-p",
                            "output_topic:=/scan_slam",
                            "-p",
                            "max_publish_hz:=5.0",
                            "-p",
                            "spatial_filter_radius:=2",
                            "-p",
                            "spatial_tolerance:=0.12",
                            "-p",
                            "temporal_window:=3",
                            "-p",
                            "temporal_minimum_hits:=2",
                            "-p",
                            "temporal_tolerance:=0.15",
                            "-p",
                            "maximum_range_m:=4.0",
                        ],
                        output="screen",
                    ),
                    ExecuteProcess(
                        cmd=[
                            "python3",
                            f"{runtime_dir}/odom_relay.py",
                            "--ros-args",
                            "-r",
                            "/tf:=/tf_nav",
                        ],
                        output="screen",
                    ),
                    Node(
                        package="slam_toolbox",
                        executable="async_slam_toolbox_node",
                        name="slam_toolbox",
                        parameters=[
                            slam_params_path,
                            {"use_sim_time": use_sim_time},
                        ],
                        remappings=[("/tf", "/tf_nav")],
                        output="screen",
                    ),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            [nav2_bringup_dir, "/launch/navigation_launch.py"]
                        ),
                        launch_arguments={
                            "use_sim_time": use_sim_time,
                            "params_file": mapping_nav_params,
                            "autostart": "true",
                            "use_composition": "False",
                        }.items(),
                    ),
                    Node(
                        package="nav2_map_server",
                        executable="map_saver_server",
                        name="map_saver",
                        parameters=[nav_params_path, {"use_sim_time": use_sim_time}],
                        output="screen",
                    ),
                    Node(
                        package="nav2_lifecycle_manager",
                        executable="lifecycle_manager",
                        name="lifecycle_manager_mapping_save",
                        parameters=[
                            {"use_sim_time": use_sim_time},
                            {"autostart": True},
                            {"node_names": ["map_saver"]},
                        ],
                        output="screen",
                    ),
                    Node(
                        package="tf2_ros",
                        executable="static_transform_publisher",
                        name="base_footprint_to_base_link",
                        arguments=[
                            "0",
                            "0",
                            "0",
                            "0",
                            "0",
                            "0",
                            "base_footprint",
                            "base_link",
                        ],
                    ),
                    Node(
                        package="tf2_ros",
                        executable="static_transform_publisher",
                        name="base_link_to_base_laser",
                        arguments=[
                            "-0.0046412",
                            "0",
                            "0.094079",
                            "0",
                            "0",
                            "0",
                            "base_link",
                            "laser_frame",
                        ],
                    ),
                    ExecuteProcess(
                        cmd=[
                            "python3",
                            f"{runtime_dir}/autonomous_mapping.py",
                            "--ros-args",
                            "-r",
                            "/tf:=/tf_nav",
                            "-p",
                            ["start_enabled:=", exploration_enabled],
                            "-p",
                            ["map_output:=", map_output],
                            "-p",
                            ["maximum_runtime:=", mapping_maximum_runtime],
                            "-p",
                            [
                                "maximum_exploration_radius:=",
                                mapping_maximum_radius,
                            ],
                            "-p",
                            [
                                "goal_progress_timeout:=",
                                mapping_goal_progress_timeout,
                            ],
                            "-p",
                            ["maximum_map_age:=", mapping_maximum_map_age],
                            "-p",
                            [
                                "minimum_save_known_area_m2:=",
                                mapping_minimum_save_known_area,
                            ],
                            "-p",
                            [
                                "minimum_save_free_area_m2:=",
                                mapping_minimum_save_free_area,
                            ],
                        ],
                        output="screen",
                    ),
                ]
            ),
        ]
    )
