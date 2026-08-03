#!/usr/bin/env python3
"""Single ROS 2 entry point for Perception, Follow Me, and Navigation.

Examples::

    ros2 run robot_project integrated_main --mode perception
    ros2 run robot_project integrated_main --mode follow --ros-args \
        -p camera_topic:=/camera/image_raw -p model_path:=/absolute/best.pt
    ros2 run robot_project integrated_main --mode navigation --ros-args \
        -p goal_x:=1.0 -p goal_y:=0.5 -p goal_yaw:=0.0

Follow mode starts Perception and FollowPerson together in one executor.
Navigation is intentionally a separate mode because both Follow and Nav2 can
control robot motion.
"""

from __future__ import annotations

import argparse
import sys
import threading
import time
from collections.abc import Sequence

from robot_project.runtime import VALID_MODES, components_for_mode


def _arguments(args: Sequence[str] | None) -> tuple[argparse.Namespace, list[str]]:
    parser = argparse.ArgumentParser(description="Robot Project integrated runner")
    parser.add_argument(
        "--mode",
        choices=VALID_MODES,
        default="follow",
        help="perception only, perception+follow, or navigation",
    )
    parser.add_argument(
        "--control-stdin",
        action="store_true",
        help="accept a 'stop' line on stdin (used by the desktop GUI)",
    )
    parsed, ros_args = parser.parse_known_args(args)
    return parsed, ros_args


def main(args: Sequence[str] | None = None) -> None:
    parsed, ros_args = _arguments(args)

    # Keep ROS imports lazy. This lets mode validation and unit tests run on the
    # macOS GUI machine where ROS 2 is not installed.
    import rclpy
    from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor

    rclpy.init(args=ros_args)
    executor = MultiThreadedExecutor()
    nodes = []
    navigation_node = None

    try:
        for component in components_for_mode(parsed.mode):
            if component == "perception":
                from robot_project.perception.detect import YoloDetectNode

                node = YoloDetectNode()
            elif component == "follow":
                from robot_project.follow.follow_person import FollowPersonNode

                node = FollowPersonNode()
            else:
                from robot_project.navigation.nav import NavGoalNode

                node = NavGoalNode()
                navigation_node = node
            nodes.append(node)
            executor.add_node(node)

        if navigation_node is not None and not navigation_node.send_configured_goal():
            return

        names = ", ".join(node.get_name() for node in nodes)
        nodes[0].get_logger().info(
            f"integrated mode={parsed.mode} started: {names}"
        )
        if parsed.control_stdin:
            def watch_stdin() -> None:
                for line in sys.stdin:
                    if line.strip().lower() != "stop":
                        continue
                    if navigation_node is not None:
                        navigation_node.cancel_goal()
                        time.sleep(0.75)
                    if rclpy.ok():
                        rclpy.shutdown()
                    return

            threading.Thread(target=watch_stdin, daemon=True).start()
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        for node in reversed(nodes):
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
