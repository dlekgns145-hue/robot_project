#!/usr/bin/env python3
"""Publish a timestamped AMCL initial pose from the command line."""

import argparse
import math
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--x", type=float, default=0.0)
    parser.add_argument("--y", type=float, default=0.0)
    parser.add_argument("--yaw", type=float, default=0.0)
    parser.add_argument("--position-variance", type=float, default=0.25)
    parser.add_argument("--yaw-variance", type=float, default=0.0685)
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = Node("initial_pose_cli")
    publisher = node.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)

    deadline = time.monotonic() + 5.0
    while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    message = PoseWithCovarianceStamped()
    message.header.frame_id = "map"
    message.pose.pose.position.x = args.x
    message.pose.pose.position.y = args.y
    message.pose.pose.orientation.z = math.sin(args.yaw / 2.0)
    message.pose.pose.orientation.w = math.cos(args.yaw / 2.0)
    message.pose.covariance[0] = args.position_variance
    message.pose.covariance[7] = args.position_variance
    message.pose.covariance[35] = args.yaw_variance

    for _ in range(5):
        message.header.stamp = node.get_clock().now().to_msg()
        publisher.publish(message)
        rclpy.spin_once(node, timeout_sec=0.1)
        time.sleep(0.1)

    node.get_logger().info(
        f"published initial pose: x={args.x:.3f}, y={args.y:.3f}, yaw={args.yaw:.3f}"
    )
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
