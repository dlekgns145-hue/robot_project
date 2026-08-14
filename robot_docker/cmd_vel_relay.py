#!/usr/bin/env python3
"""Relay Nav2's smoothed /cmd_vel to /cmd_vel_server.

nav2_bringup's navigation_launch.py hardcodes velocity_smoother's output
remap to the literal topic name "cmd_vel" internally (controller_server ->
cmd_vel_nav -> velocity_smoother -> cmd_vel). The GroupAction-level
SetRemap("/cmd_vel", "/cmd_vel_server") in mapping_runtime_launch.py does not
reach across that IncludeLaunchDescription boundary, so Nav2's real velocity
commands were stopping at /cmd_vel and never reaching /cmd_vel_server, which
is the only topic the robot's Zenoh bridge relays to the robot -- the robot
never moved even though the controller was computing valid, non-zero
velocities.
"""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class CmdVelRelay(Node):
    def __init__(self) -> None:
        super().__init__("cmd_vel_server_relay")
        self.publisher = self.create_publisher(Twist, "/cmd_vel_server", 10)
        self.subscription = self.create_subscription(
            Twist, "/cmd_vel", self.callback, 10
        )

    def callback(self, message: Twist) -> None:
        self.publisher.publish(message)


def main() -> None:
    rclpy.init()
    node = CmdVelRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
