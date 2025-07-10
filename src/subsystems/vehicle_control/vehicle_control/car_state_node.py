#!/usr/bin/env python3
"""Simple car state estimation node."""
import math
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose2D

MAX_BRAKEFORCE = -10.0  # m/s^2 (negative)

class CarStateNode(Node):
    def __init__(self):
        super().__init__('car_state_node')
        self.declare_parameter('max_brakeforce', MAX_BRAKEFORCE)
        self.max_brakeforce = float(self.get_parameter('max_brakeforce').value)

        self.pos_x = 0.0
        self.pos_y = 0.0
        self.yaw = 0.0
        self.speed = 0.0
        self.desired_speed: Optional[float] = None
        self.steering_angle = 0.0
        self.desired_angle: Optional[float] = None
        self.last_time = self.get_clock().now()

        self.create_subscription(Float32, '/vehicle/speed', self.speed_cb, 10)
        self.create_subscription(Float32, '/vehicle/steering_angle', self.angle_cb, 10)
        self.create_subscription(Float32, '/vehicle/desired_speed', self.desired_speed_cb, 10)
        self.create_subscription(Float32, '/path_to_y_axis_angle', self.desired_angle_cb, 10)
        self.state_pub = self.create_publisher(Pose2D, '/vehicle/car_state', 10)
        self.timer = self.create_timer(0.05, self.update)

    def speed_cb(self, msg: Float32):
        self.speed = float(msg.data)

    def angle_cb(self, msg: Float32):
        self.steering_angle = float(msg.data)

    def desired_speed_cb(self, msg: Float32):
        self.desired_speed = float(msg.data)

    def desired_angle_cb(self, msg: Float32):
        self.desired_angle = float(msg.data)

    def update(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        # simple kinematic update
        self.pos_x += math.sin(math.radians(self.yaw)) * self.speed * dt
        self.pos_y += math.cos(math.radians(self.yaw)) * self.speed * dt
        self.yaw += self.steering_angle * dt

        pose = Pose2D(x=float(self.pos_x), y=float(self.pos_y), theta=float(self.yaw))
        self.state_pub.publish(pose)

        if self.desired_speed is not None:
            if self.desired_speed - self.speed < self.max_brakeforce:
                self.get_logger().warn('EBS triggered due to speed difference')
        if self.desired_angle is not None:
            diff = abs(self.desired_angle - self.steering_angle)
            if diff > 45.0:
                self.get_logger().warn('EBS triggered due to steering difference')

def main(args=None):
    rclpy.init(args=args)
    node = CarStateNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
