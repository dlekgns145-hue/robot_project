#!/usr/bin/env python3
"""
scan_time_fix.py - LiDAR(/scan) 타임스탬프를 현재 시각으로 보정 + 로봇 자체 오탐지 각도 필터링
                    후 /scan_fixed 로 다시 발행하는 중계 노드.
------------------------------------------------------------------
1) 타임스탬프 보정: ESP32(micro-ROS)에서 오는 /scan 메시지의 header.stamp가
   실제 현재 시각보다 부정확하여 amcl/gmapping이 tf를 못 찾는 문제 해결.
2) 각도 필터링: 로봇 자체 부품(프레임/브라켓 등)을 라이다가 근접 오탐지하는
   특정 각도 구간을 무효화(range_max로 대체)하여 costmap 오염을 방지.
   대상 구간(도, 0~359 인덱스 기준): 5~30도, 155~220도
   필요시 EXCLUDE_RANGES 값을 조정하세요.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

# 로봇 자체 오탐지로 확인된 각도 구간 (인덱스, 1도 간격 기준)
EXCLUDE_RANGES = [
    (5, 30),
    (155, 220),
]

class ScanTimeFixNode(Node):
    def __init__(self):
        super().__init__('scan_time_fix_node')
        self.pub = self.create_publisher(LaserScan, '/scan_fixed', 10)
        self.sub = self.create_subscription(LaserScan, '/scan', self.callback, 10)
        self.get_logger().info(
            'scan_time_fix_node 시작됨: /scan -> /scan_fixed (시각 보정 + 자체 오탐지 각도 필터링)'
        )

    def callback(self, msg: LaserScan):
        msg.header.stamp = self.get_clock().now().to_msg()

        n = len(msg.ranges)
        ranges = list(msg.ranges)
        for start, end in EXCLUDE_RANGES:
            for i in range(max(0, start), min(n, end + 1)):
                ranges[i] = msg.range_max + 1.0  # 유효 범위 밖으로 처리 -> 무시됨
        # range_min보다 작거나 0으로 찍힌(측정 실패) 값도 무효 처리
        for i in range(n):
            if ranges[i] < msg.range_min:
                ranges[i] = msg.range_max + 1.0
        msg.ranges = ranges

        self.pub.publish(msg)

def main():
    rclpy.init()
    node = ScanTimeFixNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
