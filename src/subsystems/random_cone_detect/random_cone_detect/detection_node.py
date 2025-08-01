#!/usr/bin/env python3
"""Publish simulated cone detections in the vehicle coordinate frame.

Cones are loaded from the track publisher and transformed so that the
vehicle pose (:msg:`Pose2D`) represents the origin. By default, cones
whose lateral position lies between ``-15`` and ``15`` meters are not
published, effectively hiding objects directly in front of the vehicle
within that range.
"""

from __future__ import annotations

import math
from typing import List
import argparse
import random

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float32

from vehicle_control.forward_feedback import (
    kinematic_step_deg,
)

from oak_cone_detect_interfaces.msg import (
    Cone2D,
    ConeArray2D,
    ConeArray3D,
    Cone3D,
)


# Global toggle to add random noise simulating camera shake
CAMERA_SHAKE = True

# Global toggle to add distance dependent noise
DISTANCE_NOISE = True

# Maximum positional deviation at 30m distance in meters
MAX_DISTORTION_30M = 0.01

# Minimum distance driven before updating the orientation used for detection
DELAY_DISTANCE = 0.1  # meters
WHEEL_BASE = 1.9  # m
# Default prediction framerate used until an update is received
DEFAULT_PREDICTION_FPS = 30.0


class DetectionNode(Node):
    """Publish cones based on ground truth positions."""

    def __init__(self, publish_all: bool = False) -> None:
        super().__init__("cone_detection_node")
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.pub = self.create_publisher(
            ConeArray3D, "/cone_detections_3d", qos
        )
        track_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            ConeArray2D, "/track/cones", self.track_cb, track_qos
        )
        self.create_subscription(
            Pose2D, "/vehicle/car_state", self.state_cb, 10
        )
        self.create_subscription(
            Float32, "/vehicle/desired_speed", self.speed_cb, 10
        )
        self.create_subscription(
            Float32, "/vehicle/steering_angle", self.angle_cb, 10
        )
        self.create_subscription(
            Float32, "/path_inference_fps", self.fps_cb, 10
        )
        self.cones: list[Cone2D] = []
        self.state = Pose2D()
        self.pred_x = 0.0
        self.pred_y = 0.0
        self.pred_yaw = 0.0
        self.desired_speed = 0.0
        self.desired_angle = 0.0
        self.prediction_fps = DEFAULT_PREDICTION_FPS
        self.delayed_yaw = 0.0
        self._yaw_update_pos: tuple[float, float] | None = None
        self.publish_all = self.declare_parameter(
            "publish_all", publish_all
        ).value
        self.timer = self.create_timer(0.001, self.publish_visible)
        self.pred_timer = self.create_timer(
            1.0 / self.prediction_fps, self.update_prediction
        )
        self.get_logger().info('DetectionNode started')

    # ------------------------------------------------------------------
    def track_cb(self, msg: ConeArray2D) -> None:
        self.cones = list(msg.cones)

    # ------------------------------------------------------------------
    def state_cb(self, msg: Pose2D) -> None:
        if self._yaw_update_pos is None:
            self._yaw_update_pos = (float(msg.x), float(msg.y))
            self.delayed_yaw = float(msg.theta)
            self.pred_x = float(msg.x)
            self.pred_y = float(msg.y)
            self.pred_yaw = float(msg.theta)
        else:
            dist = math.hypot(
                float(msg.x) - self._yaw_update_pos[0],
                float(msg.y) - self._yaw_update_pos[1],
            )
            if dist >= DELAY_DISTANCE:
                self.delayed_yaw = float(msg.theta)
                self._yaw_update_pos = (float(msg.x), float(msg.y))
        self.state = msg

    # ------------------------------------------------------------------
    def speed_cb(self, msg: Float32) -> None:
        self.desired_speed = float(msg.data)

    # ------------------------------------------------------------------
    def angle_cb(self, msg: Float32) -> None:
        self.desired_angle = float(msg.data)

    # ------------------------------------------------------------------
    def fps_cb(self, msg: Float32) -> None:
        fps = float(msg.data)
        if fps <= 0:
            return
        self.prediction_fps = fps
        self.pred_timer.timer_period_ns = int(1e9 / fps)

    # ------------------------------------------------------------------
    def update_prediction(self) -> None:
        dt = 1.0 / self.prediction_fps if self.prediction_fps > 0 else 0.0
        self.pred_x, self.pred_y, self.pred_yaw = kinematic_step_deg(
            self.pred_x,
            self.pred_y,
            self.pred_yaw,
            self.desired_speed,
            self.desired_angle,
            dt,
            WHEEL_BASE,
        )
        if self._yaw_update_pos is None:
            self._yaw_update_pos = (self.pred_x, self.pred_y)
            self.delayed_yaw = self.pred_yaw
        else:
            dist = math.hypot(
                self.pred_x - self._yaw_update_pos[0],
                self.pred_y - self._yaw_update_pos[1],
            )
            if dist >= DELAY_DISTANCE:
                self.delayed_yaw = self.pred_yaw
                self._yaw_update_pos = (self.pred_x, self.pred_y)

    # ------------------------------------------------------------------
    def publish_visible(self) -> None:
        if not self.cones:
            return
        px = self.pred_x
        py = self.pred_y
        yaw = - math.radians(float(self.delayed_yaw))
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        out = ConeArray3D()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "map"

        for c in self.cones:
            dx = float(c.x) - px
            dy = float(c.y) - py
            # transform to vehicle coordinate frame
            local_x = cos_yaw * dx + sin_yaw * dy
            local_y = -sin_yaw * dx + cos_yaw * dy

            dist = math.hypot(local_x, local_y)

            if DISTANCE_NOISE:
                noise = min(dist / 30.0, 1.0) * MAX_DISTORTION_30M
                local_x += random.uniform(-noise, noise)
                local_y += random.uniform(-noise, noise)

            if CAMERA_SHAKE:
                # Add small random jitter to simulate a shaky camera
                local_x += random.uniform(-0.05, 0.05)
                local_y += random.uniform(-0.05, 0.05)

            if not self.publish_all:
                if 30.0 < local_y < 0.0:
                    continue
                if -15.0 > local_x > 15.0:
                    continue

            new_c = Cone3D(
                id=c.id,
                label=c.label,
                conf=c.conf,
                # Output cones in vehicle coordinates (z is forward axis)
                x=local_x,
                y=0.0,
                z=local_y,
                color=c.color,
            )
            out.cones.append(new_c)

        if out.cones:
            self.pub.publish(out)


def main(args: List[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="Cone detection node")
    parser.add_argument(
        "--publish-all",
        action="store_true",
        help="publish all cones without visibility filtering",
    )
    parsed_args = parser.parse_args(args)

    rclpy.init(args=None)
    node = DetectionNode(publish_all=parsed_args.publish_all)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
