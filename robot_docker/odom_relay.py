#!/usr/bin/env python3
"""Publish fresh, finite wheel odometry for navigation.

The controller's ``/odom_raw`` clock can lag behind the ROS host clock and the
vendor EKF may publish NaNs when its IMU input is invalid.  This relay keeps the
navigation odometry on a separate topic and owns the corresponding finite
``odom -> base_footprint`` transform.
"""

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


def odometry_is_finite(message: Odometry) -> bool:
    pose = message.pose.pose
    twist = message.twist.twist
    values = (
        pose.position.x,
        pose.position.y,
        pose.position.z,
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
        twist.linear.x,
        twist.linear.y,
        twist.linear.z,
        twist.angular.x,
        twist.angular.y,
        twist.angular.z,
    )
    return all(math.isfinite(value) for value in values)


class OdomRelayNode(Node):
    def __init__(self) -> None:
        super().__init__("navigation_odom_relay")
        self.declare_parameter("input_topic", "/odom_raw")
        self.declare_parameter("output_topic", "/odom_nav")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self._invalid_count = 0

        self.publisher = self.create_publisher(Odometry, self.output_topic, 10)
        self.subscription = self.create_subscription(
            Odometry, self.input_topic, self.callback, 10
        )
        self.tf_broadcaster = TransformBroadcaster(self)
        self.get_logger().info(
            f"navigation odom relay: {self.input_topic} -> {self.output_topic} "
            f"({self.odom_frame} -> {self.base_frame})"
        )

    def callback(self, message: Odometry) -> None:
        if not odometry_is_finite(message):
            self._invalid_count += 1
            if self._invalid_count == 1 or self._invalid_count % 50 == 0:
                self.get_logger().error(
                    f"discarding non-finite odometry (count={self._invalid_count})"
                )
            return

        now = self.get_clock().now().to_msg()
        message.header.stamp = now
        message.header.frame_id = self.odom_frame
        message.child_frame_id = self.base_frame
        self.publisher.publish(message)

        transform = TransformStamped()
        transform.header.stamp = now
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = message.pose.pose.position.x
        transform.transform.translation.y = message.pose.pose.position.y
        transform.transform.translation.z = message.pose.pose.position.z
        transform.transform.rotation = message.pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)


def main() -> None:
    rclpy.init()
    node = OdomRelayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
