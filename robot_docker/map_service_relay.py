#!/usr/bin/env python3
"""Republish slam_toolbox's current map for the autonomous explorer."""

import rclpy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import UInt16


class MapServiceRelay(Node):
    def __init__(self) -> None:
        super().__init__("map_service_relay")
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        self.publisher = self.create_publisher(
            OccupancyGrid, "/autonomous_mapping/map_refresh", qos
        )
        self.battery_publisher = self.create_publisher(
            UInt16, "/autonomous_mapping/battery_refresh", 10
        )
        self.battery_subscription = self.create_subscription(
            UInt16, "/battery", self.battery_publisher.publish, 10
        )
        self.client = self.create_client(GetMap, "/slam_toolbox/dynamic_map")
        self.pending = None
        self.timer = self.create_timer(1.0, self._request_map)

    def _request_map(self) -> None:
        if self.pending is not None or not self.client.service_is_ready():
            return
        future = self.client.call_async(GetMap.Request())
        self.pending = future
        future.add_done_callback(self._publish_map)

    def _publish_map(self, future) -> None:
        self.pending = None
        try:
            message = future.result().map
        except Exception as error:
            self.get_logger().warn(f"dynamic map request failed: {error}")
            return
        if int(message.info.width) <= 0 or int(message.info.height) <= 0:
            return
        self.publisher.publish(message)


def main() -> None:
    rclpy.init()
    node = MapServiceRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
