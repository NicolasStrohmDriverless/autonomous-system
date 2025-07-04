#!/usr/bin/env python3
"""Monitor /vehicle/desired_speed and shutdown when idle."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class IdleMonitorNode(Node):
    """Shutdown system if desired speed stays 0 for a period."""

    def __init__(self, timeout_sec: float = 10.0, threshold: float = 1e-3):
        super().__init__('idle_monitor_node')
        self.timeout_sec = float(timeout_sec)
        self.threshold = float(threshold)
        self.last_moving_time = self.get_clock().now()
        self.create_subscription(
            Float32,
            '/vehicle/desired_speed',
            self.speed_callback,
            10,
        )
        self.create_timer(1.0, self.check_timeout)

    def speed_callback(self, msg: Float32):
        speed = float(msg.data)
        if abs(speed) > self.threshold:
            self.last_moving_time = self.get_clock().now()

    def check_timeout(self):
        elapsed = (
            self.get_clock().now() - self.last_moving_time
        ).nanoseconds * 1e-9
        if elapsed >= self.timeout_sec:
            self.get_logger().info(
                f'No movement detected for {self.timeout_sec} s, returning to selection.'
            )
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = IdleMonitorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
