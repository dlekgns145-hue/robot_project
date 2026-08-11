"""Launch camera BEV, global visual mapper and the manual saver endpoint."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description() -> LaunchDescription:
    default_parameters = PathJoinSubstitution(
        [FindPackageShare("orchard_mapper"), "config", "mapper.yaml"]
    )
    parameters = LaunchConfiguration("params_file")
    tf_topic = LaunchConfiguration("tf_topic")
    use_sim_time = LaunchConfiguration("use_sim_time")
    common = {
        "parameters": [parameters, {"use_sim_time": use_sim_time}],
        "output": "screen",
    }
    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=default_parameters),
            DeclareLaunchArgument("tf_topic", default_value="/tf"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            Node(
                package="orchard_mapper",
                executable="camera_bev_node",
                name="camera_bev",
                **common,
            ),
            Node(
                package="orchard_mapper",
                executable="global_visual_mapper",
                name="orchard_visual_mapper",
                remappings=[("/tf", tf_topic)],
                **common,
            ),
            Node(
                package="orchard_mapper",
                executable="visual_map_saver",
                name="visual_map_saver",
                **common,
            ),
        ]
    )
