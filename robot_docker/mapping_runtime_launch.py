"""Start autonomous SLAM exploration on the finite navigation sensor pipeline.

The vendor base and EKF publish stale or non-finite transforms on ``/tf``.
Mapping therefore shares Nav2's isolated ``/tf_nav`` channel. Exploration is
disabled at launch and starts only through ``/autonomous_mapping/start``.
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
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
    camera_url = LaunchConfiguration("camera_url")
    camera_guard_enabled = LaunchConfiguration("camera_guard_enabled")
    texture_source_top_fraction = LaunchConfiguration(
        "texture_source_top_fraction"
    )
    texture_near_m = LaunchConfiguration("texture_near_m")
    texture_far_m = LaunchConfiguration("texture_far_m")
    texture_near_width_m = LaunchConfiguration("texture_near_width_m")
    texture_far_width_m = LaunchConfiguration("texture_far_width_m")
    mapping_maximum_runtime = LaunchConfiguration("mapping_maximum_runtime")
    mapping_maximum_radius = LaunchConfiguration("mapping_maximum_radius")
    # Frontier exploration cares about reaching the free cell, not finishing
    # at an exact heading.  Keep normal saved-map navigation at its stricter
    # yaw tolerance and relax it only inside the mapping runtime.
    mapping_nav_params = RewrittenYaml(
        source_file=nav_params_path,
        root_key="",
        param_rewrites={"yaw_goal_tolerance": "3.14"},
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
                "camera_url",
                default_value="http://127.0.0.1:8080/stream.mjpg",
            ),
            DeclareLaunchArgument("camera_guard_enabled", default_value="true"),
            DeclareLaunchArgument(
                "texture_source_top_fraction", default_value="0.50"
            ),
            DeclareLaunchArgument("texture_near_m", default_value="0.18"),
            DeclareLaunchArgument("texture_far_m", default_value="2.0"),
            DeclareLaunchArgument(
                "texture_near_width_m", default_value="0.85"
            ),
            DeclareLaunchArgument(
                "texture_far_width_m", default_value="1.8"
            ),
            DeclareLaunchArgument(
                "mapping_maximum_runtime", default_value="900.0"
            ),
            DeclareLaunchArgument(
                "mapping_maximum_radius", default_value="8.0"
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
                        ],
                        output="screen",
                    ),
                    ExecuteProcess(
                        cmd=[
                            "python3",
                            f"{runtime_dir}/camera_obstacle_guard.py",
                            "--ros-args",
                            "-p",
                            ["camera_url:=", camera_url],
                        ],
                        output="screen",
                        condition=IfCondition(camera_guard_enabled),
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
                        ],
                        output="screen",
                    ),
                    ExecuteProcess(
                        cmd=[
                            "python3",
                            f"{runtime_dir}/map_texture_recorder.py",
                            "--ros-args",
                            "-r",
                            "/tf:=/tf_nav",
                            "-p",
                            ["camera_url:=", camera_url],
                            "-p",
                            ["projection_near_m:=", texture_near_m],
                            "-p",
                            ["projection_far_m:=", texture_far_m],
                            "-p",
                            [
                                "projection_near_width_m:=",
                                texture_near_width_m,
                            ],
                            "-p",
                            [
                                "projection_far_width_m:=",
                                texture_far_width_m,
                            ],
                            "-p",
                            [
                                "projection_source_top_fraction:=",
                                texture_source_top_fraction,
                            ],
                        ],
                        output="screen",
                    ),
                ]
            ),
        ]
    )
