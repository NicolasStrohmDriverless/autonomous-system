#!/usr/bin/env python3
"""Simple car state estimation node."""
import math
from typing import Optional

import numpy as np
import cv2
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import Image
from oak_cone_detect_interfaces.msg import PathPrediction

ACCELERATION = 9.81  # m/s^2
DECELERATION = 9.81  # m/s^2 used when slowing down without braking
MAX_BRAKE_BAR = 14.0  # 1 bar = -1 m/s^2
MAX_STEERING_ANGLE = 30.0  # deg
STEERING_TRANSLATION = 15.0  # ratio
MAX_YAW_ACCEL = 180.0  # deg/s^2
# Step distance used for path predictions
PREDICTION_INTERVAL = 0.1
# PID gains for steering control
STEERING_KP = 2.5
STEERING_KI = 0.1
STEERING_KD = 0.0
# PID gains for speed control
SPEED_KP = 1.2
SPEED_KI = 0.05
SPEED_KD = 0.0
# PID gains for brake pressure control
BRAKE_KP = 2.0
BRAKE_KI = 0.1
BRAKE_KD = 0.0
# Lenkwinkelgeschwindigkeit
# θ_dot = R * v × i ≈ STEERING_TRANSLATION * v


def draw_pressure_gauge(
    pressure: float, max_pressure: float, width: int = 180, height: int = 90
) -> np.ndarray:
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

    if max_pressure <= 0:
        frac = 0.0
    else:
        frac = max(0.0, min(1.0, pressure / max_pressure))

    # needle position
    needle_angle = 180 + frac * 180
    a = math.radians(needle_angle)
    x = int(center[0] + (radius - 10) * math.cos(a))
    y = int(center[1] + (radius - 10) * math.sin(a))
    cv2.line(img, center, (x, y), (0, 0, 255), 2)

    text = f"{pressure:.2f} bar"
    text_size, _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
    text_pos = (center[0] - text_size[0] // 2, height - 5)
    cv2.putText(
        img,
        text,
        text_pos,
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (0, 0, 0),
        1,
    )
    return img


class PIDController:
    """Simple PID controller."""

    def __init__(self, kp: float, ki: float, kd: float) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0.0
        self.last_error = 0.0

    def reset(self) -> None:
        self.integral = 0.0
        self.last_error = 0.0

    def update(self, setpoint: float, measurement: float, dt: float) -> float:
        error = setpoint - measurement
        self.integral += error * dt
        derivative = (error - self.last_error) / dt if dt > 0 else 0.0
        self.last_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative


class CarStateNode(Node):
    def __init__(self):
        super().__init__("car_state_node")

        self.pos_x = 0.0
        self.pos_y = 0.0
        self.yaw = 0.0
        self.yaw_rate = 0.0
        self.speed = 0.0
        self.desired_speed: Optional[float] = None
        self.steering_angle = 0.0
        self.desired_angle: Optional[float] = None
        self.predicted_speeds = []
        self.predicted_angles = []
        self.prediction_idx = 0
        self.steering_pid = PIDController(
            STEERING_KP,
            STEERING_KI,
            STEERING_KD,
        )
        self.speed_pid = PIDController(SPEED_KP, SPEED_KI, SPEED_KD)
        self.brake_pid = PIDController(BRAKE_KP, BRAKE_KI, BRAKE_KD)
        self.last_time = self.get_clock().now()
        self.ebs_active = False
        self.frozen_angle: Optional[float] = None

        self.declare_parameter("max_yaw_accel", MAX_YAW_ACCEL)
        self.max_yaw_accel = float(self.get_parameter("max_yaw_accel").value)

        self.create_subscription(
            Float32, "/vehicle/desired_speed", self.desired_speed_cb, 10
        )
        self.create_subscription(
            Float32, "/path_to_y_axis_angle", self.desired_angle_cb, 10
        )
        self.create_subscription(
            PathPrediction, "/prediction", self.prediction_cb, 10
        )
        self.state_pub = self.create_publisher(
            Pose2D, "/vehicle/car_state", 10
        )
        self.speed_pub = self.create_publisher(
            Float32, "/vehicle/actual_speed", 10
        )
        self.steering_pub = self.create_publisher(
            Float32, "/vehicle/actual_steering", 10
        )
        self.brake_pub = self.create_publisher(
            Float32, "/vehicle/asb_pressure", 10
        )
        self.asb_cmd = 0.0
        self.create_subscription(
            Float32,
            "/vehicle/asb_pressure_cmd",
            self.asb_cmd_cb,
            10,
        )
        self.brake_image_pub = self.create_publisher(
            Image, "/path_status/brake_image", 1
        )
        self.create_subscription(Bool, "/system/ebs_active", self.ebs_active_cb, 10)
        self.bridge = CvBridge()
        self.timer = self.create_timer(0.05, self.update)
        self.get_logger().info('CarStateNode started')

    def desired_speed_cb(self, msg: Float32):
        self.desired_speed = float(msg.data)

    def desired_angle_cb(self, msg: Float32):
        self.desired_angle = float(msg.data)

    def prediction_cb(self, msg: PathPrediction):
        self.predicted_speeds = list(msg.speeds)
        self.predicted_angles = list(msg.steering_angles)

        # estimate how far the vehicle has travelled since the prediction was
        # generated and fast-forward the index accordingly
        now = self.get_clock().now()
        dt = (now - msg.header.stamp).nanoseconds * 1e-9
        travelled = self.speed * dt
        skip = int(travelled / PREDICTION_INTERVAL)
        self.prediction_idx = min(max(skip, 0), len(self.predicted_speeds))

    def asb_cmd_cb(self, msg: Float32):
        self.asb_cmd = float(msg.data)

    def ebs_active_cb(self, msg: Bool) -> None:
        active = bool(msg.data)
        if active and not self.ebs_active:
            self.frozen_angle = self.steering_angle
        self.ebs_active = active

    def update(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        cmd_brake = self.asb_cmd
        self.asb_cmd = 0.0
        brake = 0.0

        if self.ebs_active:
            self.desired_speed = 0.0
            if self.frozen_angle is not None:
                self.desired_angle = self.frozen_angle
            brake = MAX_BRAKE_BAR
        else:
            if self.prediction_idx < len(self.predicted_speeds):
                self.desired_speed = self.predicted_speeds[self.prediction_idx]
                if self.predicted_angles:
                    self.desired_angle = self.predicted_angles[self.prediction_idx]
                self.prediction_idx += 1

        # adjust speed towards desired speed using separate PID controllers
        if self.desired_speed is not None:
            if self.desired_speed > 0.0:
                control = self.speed_pid.update(
                    self.desired_speed,
                    self.speed,
                    dt,
                )
                if control > 0.0:
                    accel = min(control, ACCELERATION * dt)
                    self.speed += accel
                    if self.speed > self.desired_speed:
                        self.speed = self.desired_speed
                elif control < 0.0:
                    decel = min(-control, DECELERATION * dt)
                    self.speed -= decel
                    if self.speed < self.desired_speed:
                        self.speed = self.desired_speed
            else:
                if self.speed > 0.0:
                    diff = self.speed
                    brake = min(MAX_BRAKE_BAR, diff)
                    brake_cmd = self.brake_pid.update(0.0, self.speed, dt)
                    brake = min(MAX_BRAKE_BAR, max(brake, brake_cmd))
                if abs(self.speed) < 1e-2:
                    self.speed = 0.0

        # apply external brake command
        brake = max(brake, cmd_brake)
        if brake > 0.0:
            self.speed -= brake * dt
            if self.speed < 0.0:
                self.speed = 0.0

        # adjust steering angle using PID controller
        if self.desired_angle is not None:
            control = self.steering_pid.update(
                self.desired_angle, self.steering_angle, dt
            )
            max_change = STEERING_TRANSLATION * abs(self.speed) * dt
            if control > max_change:
                control = max_change
            elif control < -max_change:
                control = -max_change
            self.steering_angle += control
            self.steering_angle = max(
                -MAX_STEERING_ANGLE,
                min(MAX_STEERING_ANGLE, self.steering_angle),
            )

        # simple kinematic update of pose
        self.pos_x += math.sin(math.radians(self.yaw)) * self.speed * dt
        self.pos_y += math.cos(math.radians(self.yaw)) * self.speed * dt

        desired_yaw_rate = self.steering_angle * self.speed
        diff_rate = desired_yaw_rate - self.yaw_rate
        max_rate_change = self.max_yaw_accel * dt
        if diff_rate > max_rate_change:
            diff_rate = max_rate_change
        elif diff_rate < -max_rate_change:
            diff_rate = -max_rate_change
        self.yaw_rate += diff_rate
        self.yaw += self.yaw_rate * dt

        pose = Pose2D(
            x=float(self.pos_x),
            y=float(self.pos_y),
            theta=float(self.yaw),
        )
        self.state_pub.publish(pose)
        self.speed_pub.publish(Float32(data=float(self.speed)))
        self.steering_pub.publish(Float32(data=float(self.steering_angle)))
        if brake > 0.0 and (
            self.desired_speed is not None and abs(self.desired_speed) > 1e-3
        ):
            self.get_logger().warn(
                'Brake engaged while desired speed is non-zero'
            )

        self.brake_pub.publish(Float32(data=float(brake)))

        img = draw_pressure_gauge(brake, MAX_BRAKE_BAR)
        img_msg = self.bridge.cv2_to_imgmsg(img, "bgr8")
        img_msg.header.stamp = now.to_msg()
        self.brake_image_pub.publish(img_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CarStateNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
