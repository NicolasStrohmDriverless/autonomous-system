#!/usr/bin/env python3
"""Node adapting actual speed to desired speed."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

ACCELERATION = 9.81  # m/s^2
MAX_BRAKE_BAR = 14.0  # each bar equals 1 m/s^2


class SpeedControlNode(Node):
    def __init__(self) -> None:
        super().__init__('speed_control_node')
        self.actual_speed = 0.0
        self.desired_speed = None
        self.last_time = self.get_clock().now()

        self.create_subscription(Float32, '/vehicle/desired_speed', self.desired_speed_cb, 10)
        self.speed_pub = self.create_publisher(Float32, '/vehicle/speed', 10)
        self.timer = self.create_timer(0.05, self.update)

    def desired_speed_cb(self, msg: Float32) -> None:
        self.desired_speed = float(msg.data)

    def update(self) -> None:
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        if self.desired_speed is None:
            return

        if self.actual_speed < self.desired_speed:
            self.actual_speed = min(self.desired_speed, self.actual_speed + ACCELERATION * dt)
        elif self.actual_speed > self.desired_speed:
            diff = self.actual_speed - self.desired_speed
            brake_force = min(diff, MAX_BRAKE_BAR)
            self.get_logger().info(f'asb active: {brake_force:.1f} bar')
            self.actual_speed = max(self.desired_speed, self.actual_speed - brake_force * dt)

        if self.actual_speed < 0.0:
            self.actual_speed = 0.0

        self.speed_pub.publish(Float32(data=float(self.actual_speed)))


def main(args=None):
    rclpy.init(args=args)
    node = SpeedControlNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
