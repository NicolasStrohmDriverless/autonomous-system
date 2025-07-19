#!/usr/bin/env python3
"""Publish simulated cone detections in the vehicle coordinate frame.

Cones are loaded from the track publisher and transformed so that the
vehicle pose (:msg:`Pose2D`) represents the origin. Only cones within a
semicircle in front of the vehicle (30 m radius and ±15 m lateral
span) are published.
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

from oak_cone_detect_interfaces.msg import Cone2D, ConeArray2D, ConeArray3D, Cone3D


# Global toggle to add random noise simulating camera shake
CAMERA_SHAKE = False

# Global toggle to add distance dependent noise
DISTANCE_NOISE = False

# Maximum positional deviation at 30m distance in meters
MAX_DISTORTION_30M = 1.0

# Minimum distance driven before updating the orientation used for detection
DELAY_DISTANCE = 0.5  # meters


class DetectionNode(Node):
    """Publish cones based on ground truth positions."""

    def __init__(self, publish_all: bool = False) -> None:
        super().__init__("cone_detection_node")
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.pub = self.create_publisher(ConeArray3D, "/cone_detections_3d", qos)
        track_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(ConeArray2D, "/track/cones", self.track_cb, track_qos)
        self.create_subscription(Pose2D, "/vehicle/car_state", self.state_cb, 10)
        self.cones: list[Cone2D] = []
        self.state = Pose2D()
        self.delayed_yaw = 0.0
        self._yaw_update_pos: tuple[float, float] | None = None
        self.publish_all = self.declare_parameter("publish_all", publish_all).value
        self.timer = self.create_timer(0.1, self.publish_visible)
        self.get_logger().info('DetectionNode started')

    # ------------------------------------------------------------------
    def track_cb(self, msg: ConeArray2D) -> None:
        self.cones = list(msg.cones)

    # ------------------------------------------------------------------
    def state_cb(self, msg: Pose2D) -> None:
        if self._yaw_update_pos is None:
            self._yaw_update_pos = (float(msg.x), float(msg.y))
            self.delayed_yaw = float(msg.theta)
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
    def publish_visible(self) -> None:
        if not self.cones:
            return
        px = float(self.state.x)
        py = float(self.state.y)
        yaw = math.radians(float(self.delayed_yaw))
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
                dist = math.hypot(local_x, local_y)
                if dist > 30.0:
                    continue
                if local_y < 0.0:
                    continue
                if abs(local_x) > 15.0:
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
