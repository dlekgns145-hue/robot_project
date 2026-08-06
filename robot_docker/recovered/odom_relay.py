#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class OdomRelayNode(Node):
    def __init__(self):
        super().__init__('odom_relay_node')
        self.pub = self.create_publisher(Odometry, '/odom', 10)
        self.sub = self.create_subscription(Odometry, '/odom_raw', self.callback, 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.get_logger().info('odom_relay_node 시작됨: /odom_raw -> /odom (+ TF, 시각 보정)')

    def callback(self, msg):
        # 타임스탬프를 현재 시각으로 보정 (ESP32 원본 타임스탬프가 부정확하므로)
        now = self.get_clock().now().to_msg()
        msg.header.stamp = now

        # /odom 토픽 재발행
        self.pub.publish(msg)

        # odom -> base_footprint TF 발행
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = OdomRelayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
