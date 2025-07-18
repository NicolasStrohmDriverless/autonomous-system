#!/usr/bin/env python3
"""Simple cone detection node using pre-defined cone positions."""

from __future__ import annotations

import math
from typing import List
import argparse

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

from oak_cone_detect_interfaces.msg import Cone2D, ConeArray2D, ConeArray3D, Cone3D


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
        self.speed = 0.0
        self.publish_all = self.declare_parameter("publish_all", publish_all).value
        self.timer = self.create_timer(0.1, self.publish_visible)

        self.create_subscription(Float32, "/vehicle/speed", self.speed_cb, 10)

    # ------------------------------------------------------------------
    def track_cb(self, msg: ConeArray2D) -> None:
        self.cones = list(msg.cones)

    # ------------------------------------------------------------------
    def state_cb(self, msg: Pose2D) -> None:
        self.state = msg

    # ------------------------------------------------------------------
    def speed_cb(self, msg: Float32) -> None:
        self.speed = float(msg.data)

    # ------------------------------------------------------------------
    def publish_visible(self) -> None:
        if not self.cones:
            return
        px = float(self.state.x)
        py = float(self.state.y)
        yaw = math.radians(float(self.state.theta))
        dir_x = math.cos(yaw)
        dir_y = math.sin(yaw)

        out = ConeArray3D()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "map"

        max_dist = 30.0 + self.speed
        for c in self.cones:
            if not self.publish_all:
                dx = float(c.x) - px
                dy = float(c.y) - py
                dist = math.hypot(dx, dy)
                if dist > max_dist:
                    continue
                if dx * dir_x + dy * dir_y < 0:
                    continue
            new_c = Cone3D(
                id=c.id,
                label=c.label,
                conf=c.conf,
                # Track publisher provides 2D positions. Transform to (x, 0, y)
                # so downstream nodes treat ``z`` as the forward axis.
                x=c.x,
                y=0.0,
                z=c.y,
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
