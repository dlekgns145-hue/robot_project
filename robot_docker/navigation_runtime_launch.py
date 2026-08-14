"""Run the recovered Yahboom Nav2 stack without duplicate TF/cmd_vel nodes.

The base Compose service already publishes robot_description and the static
base_footprint -> base_link transform.  The vendor navigation launch adds the
same transform again and starts StopCarNode, so this launch intentionally does
neither.  Runtime assets are installed under /opt/robot-control/navigation.
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


def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")
    runtime_dir = "/opt/robot-control/navigation"
    use_sim_time = LaunchConfiguration("use_sim_time")
    map_yaml_path = LaunchConfiguration("map")
    params_path = LaunchConfiguration("params_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "map", default_value="/opt/robot-control/maps/orchard_map.yaml"
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=f"{runtime_dir}/dwb_nav_params_fixed.yaml",
            ),
            GroupAction(
                [
                    # The vendor EKF and odom_relay both publish odom ->
                    # base_footprint.  Isolate the finite relay, AMCL and Nav2
                    # listeners from the conflicting vendor transform.  The
                    # Docker images patch Nav2's internal /tf remap to preserve
                    # this scope through the included bringup launch.
                    SetRemap(src="/tf", dst="/tf_nav"),
                    # Final motor authority remains on the robot safety bridge.
                    SetRemap(src="/cmd_vel", dst="/cmd_vel_server"),
                    ExecuteProcess(
                        cmd=[
                            "python3",
                            f"{runtime_dir}/scan_time_fix.py",
                            "--ros-args",
                            "-p",
                            # Align scans to the latest /odom_nav TF sample.
                            "timestamp_delay_seconds:=0.0",
                        ],
                        output="screen",
                    ),
                    ExecuteProcess(
                        cmd=[
                            "python3",
                            f"{runtime_dir}/camera_obstacle_guard.py",
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
                    # The image patches Nav2's stock launch so smoothed and
                    # recovery velocities publish directly to
                    # /cmd_vel_server. A /cmd_vel relay would also consume the
                    # robot bridge's final output and feed it back into itself.
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            [nav2_bringup_dir, "/launch/bringup_launch.py"]
                        ),
                        launch_arguments={
                            "map": map_yaml_path,
                            "use_sim_time": use_sim_time,
                            "params_file": params_path,
                            "autostart": "true",
                            "use_composition": "False",
                        }.items(),
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
                ]
            ),
        ]
    )
