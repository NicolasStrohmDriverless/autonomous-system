#!/usr/bin/env python3
"""Simple car state estimation node."""
import math
from typing import Optional

import numpy as np
import cv2
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import Image

ACCELERATION = 9.81  # m/s^2
MAX_BRAKE_BAR = 14.0  # 1 bar = -1 m/s^2
MAX_STEERING_SPEED = 40.0  # deg/s


def draw_pressure_gauge(pressure: float, max_pressure: float,
                        width: int = 180, height: int = 90) -> np.ndarray:
    """Return an image visualizing ``pressure`` as a semicircular gauge.

    The gauge resembles the tachometer of a car.
    """
    img = np.ones((height, width, 3), dtype=np.uint8) * 255

    center = (width // 2, height - 10)
    radius = min(width // 2 - 10, height - 20)

    # draw outer semicircle
    cv2.ellipse(img, center, (radius, radius), 0, 180, 360, (0, 0, 0), 2)

    # draw tick marks
    for i in range(11):
        tick_angle = 180 + (i / 10.0) * 180
        a = math.radians(tick_angle)
        x1 = int(center[0] + (radius - 5) * math.cos(a))
        y1 = int(center[1] + (radius - 5) * math.sin(a))
        x2 = int(center[0] + radius * math.cos(a))
        y2 = int(center[1] + radius * math.sin(a))
        cv2.line(img, (x1, y1), (x2, y2), (0, 0, 0), 2)

    frac = 0.0 if max_pressure <= 0 else max(0.0, min(1.0, pressure / max_pressure))

    # needle position
    needle_angle = 180 + frac * 180
    a = math.radians(needle_angle)
    x = int(center[0] + (radius - 10) * math.cos(a))
    y = int(center[1] + (radius - 10) * math.sin(a))
    cv2.line(img, center, (x, y), (0, 0, 255), 2)

    cv2.putText(
        img,
        f"{pressure:.0f} bar",
        (10, 20),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (0, 0, 0),
        1,
    )
    return img

class CarStateNode(Node):
    def __init__(self):
        super().__init__('car_state_node')

        self.pos_x = 0.0
        self.pos_y = 0.0
        self.yaw = 0.0
        self.speed = 0.0
        self.desired_speed: Optional[float] = None
        self.steering_angle = 0.0
        self.desired_angle: Optional[float] = None
        self.last_time = self.get_clock().now()

        self.create_subscription(Float32, '/vehicle/desired_speed', self.desired_speed_cb, 10)
        self.create_subscription(Float32, '/path_to_y_axis_angle', self.desired_angle_cb, 10)
        self.state_pub = self.create_publisher(Pose2D, '/vehicle/car_state', 10)
        self.speed_pub = self.create_publisher(Float32, '/vehicle/actual_speed', 10)
        self.steering_pub = self.create_publisher(Float32, '/vehicle/actual_steering', 10)
        self.brake_pub = self.create_publisher(Float32, '/vehicle/asb_pressure', 10)
        self.brake_image_pub = self.create_publisher(Image, '/path_status/brake_image', 1)
        self.bridge = CvBridge()
        self.timer = self.create_timer(0.05, self.update)

    def desired_speed_cb(self, msg: Float32):
        self.desired_speed = float(msg.data)

    def desired_angle_cb(self, msg: Float32):
        self.desired_angle = float(msg.data)

    def update(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        brake = 0.0

        # adjust speed towards desired speed
        if self.desired_speed is not None:
            if self.speed < self.desired_speed:
                self.speed += ACCELERATION * dt
                if self.speed > self.desired_speed:
                    self.speed = self.desired_speed
            elif self.speed > self.desired_speed:
                diff = self.speed - self.desired_speed
                brake = min(MAX_BRAKE_BAR, diff)
                self.speed -= brake * dt
                if self.speed < self.desired_speed:
                    self.speed = self.desired_speed

        # adjust steering angle towards desired angle
        if self.desired_angle is not None:
            diff = self.desired_angle - self.steering_angle
            max_change = MAX_STEERING_SPEED * dt
            if diff > max_change:
                diff = max_change
            elif diff < -max_change:
                diff = -max_change
            self.steering_angle += diff

        # simple kinematic update of pose
        self.pos_x += math.sin(math.radians(self.yaw)) * self.speed * dt
        self.pos_y += math.cos(math.radians(self.yaw)) * self.speed * dt
        self.yaw += self.steering_angle * dt

        pose = Pose2D(x=float(self.pos_x), y=float(self.pos_y), theta=float(self.yaw))
        self.state_pub.publish(pose)
        self.speed_pub.publish(Float32(data=float(self.speed)))
        self.steering_pub.publish(Float32(data=float(self.steering_angle)))
        self.brake_pub.publish(Float32(data=float(brake)))

        img = draw_pressure_gauge(brake, MAX_BRAKE_BAR)
        img_msg = self.bridge.cv2_to_imgmsg(img, 'bgr8')
        img_msg.header.stamp = now.to_msg()
        self.brake_image_pub.publish(img_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CarStateNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
