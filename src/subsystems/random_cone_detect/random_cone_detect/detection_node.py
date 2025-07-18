#!/usr/bin/env python3
"""Simple cone detection node using pre-defined cone positions."""

from __future__ import annotations

import math
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Pose2D

from oak_cone_detect_interfaces.msg import ConeArray3D, Cone3D


class DetectionNode(Node):
    """Publish cones that are located in front of the vehicle."""

    def __init__(self) -> None:
        super().__init__("cone_detection_node")
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.pub = self.create_publisher(ConeArray3D, "/cone_detections_3d", qos)
        self.create_subscription(ConeArray3D, "/track/cones", self.track_cb, 10)
        self.create_subscription(Pose2D, "/vehicle/car_state", self.state_cb, 10)
        self.cones: list[Cone3D] = []
        self.state = Pose2D()
        self.timer = self.create_timer(0.1, self.publish_visible)

    # ------------------------------------------------------------------
    def track_cb(self, msg: ConeArray3D) -> None:
        self.cones = list(msg.cones)

    # ------------------------------------------------------------------
    def state_cb(self, msg: Pose2D) -> None:
        self.state = msg

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

        for c in self.cones:
            dx = float(c.x) - px
            dy = float(c.y) - py
            dist = math.hypot(dx, dy)
            if dist > 30.0:
                continue
            if dx * dir_x + dy * dir_y < 0:
                continue
            new_c = Cone3D(
                id=c.id,
                label=c.label,
                conf=c.conf,
                x=c.x,
                y=c.y,
                z=c.z,
                color=c.color,
            )
            out.cones.append(new_c)

        if out.cones:
            self.pub.publish(out)


def main(args: List[str] | None = None) -> None:
    rclpy.init(args=args)
    node = DetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
