#!/usr/bin/env python3
"""Publish predefined tracks and a scaled image of the cones."""

from __future__ import annotations
import random
from pathlib import Path
from typing import List

import cv2
import numpy as np
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)
from sensor_msgs.msg import Image

from oak_cone_detect_interfaces.msg import Cone2D, ConeArray2D
from random_cone_detect.utils import load_yaml_track


class TrackPublisher(Node):
    """Load a track file and publish cone positions and an image."""

    FS_TRACKS = [
        "FSE23.yaml",
        "FSE24.yaml",
        "FSG23.yaml",
        "FSG24.yaml",
        "FSS22_V1.yaml",
        "FSS22_V2.yaml",
    ]

    def __init__(self, mode: str = "autox") -> None:
        super().__init__("track_publisher")
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.cone_pub = self.create_publisher(ConeArray2D, "/track/cones", qos)
        self.image_pub = self.create_publisher(Image, "/vehicle/car_image", 10)
        self.bridge = CvBridge()
        self.mode = mode
        # cache track so we don't change it between publishes
        self.left, self.right, self.centerline, self.start = self._load_track()
        self.publish_track()

    # ------------------------------------------------------------------
    def _track_file(self) -> Path:
        track_dir = Path(__file__).resolve().parents[1] / "tracks"
        if self.mode == "accel":
            return track_dir / "acceleration.yaml"
        if self.mode == "skidpad":
            return track_dir / "skidpad.yaml"
        return track_dir / random.choice(self.FS_TRACKS)

    # ------------------------------------------------------------------
    def _load_track(
        self,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        path = self._track_file()
        self._track_path = path
        return load_yaml_track(str(path))

    # ------------------------------------------------------------------
    def publish_track(self) -> None:
        left = self.left
        right = self.right
        start = self.start
        path = getattr(self, "_track_path", Path("generated"))

        msg = ConeArray2D()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        color_counts = {"blue": 0, "yellow": 0, "orange": 0}

        for i, (x, y, col) in enumerate(left):
            c = Cone2D(
                id=f"L{i}",
                label="left",
                conf=1.0,
                x=float(x),
                y=float(y),
                color=str(col),
            )
            msg.cones.append(c)
            if str(col) in color_counts:
                color_counts[str(col)] += 1
        off = len(msg.cones)
        for i, (x, y, col) in enumerate(right):
            c = Cone2D(
                id=f"R{i}",
                label="right",
                conf=1.0,
                x=float(x),
                y=float(y),
                color=str(col),
            )
            msg.cones.append(c)
            if str(col) in color_counts:
                color_counts[str(col)] += 1
        for i, (x, y, col) in enumerate(start, start=off + len(right)):
            c = Cone2D(
                id=f"S{i}",
                label="start",
                conf=1.0,
                x=float(x),
                y=float(y),
                color=str(col),
            )
            msg.cones.append(c)
            if str(col) in color_counts:
                color_counts[str(col)] += 1

        self.cone_pub.publish(msg)
        self.publish_image(left, right, start)
        count = len(msg.cones)
        log_msg = (
            f"Published {count} cones from {path.name} in mode {self.mode}. "
            f"blue={color_counts['blue']} "
            f"yellow={color_counts['yellow']} "
            f"orange={color_counts['orange']}"
        )
        self.get_logger().info(log_msg)

    # ------------------------------------------------------------------
    def publish_image(
        self, left: np.ndarray, right: np.ndarray, start: np.ndarray
    ) -> None:
        cones = np.vstack(
            [
                left[:, :2].astype(float),
                right[:, :2].astype(float),
            ]
        )
        if cones.size == 0:
            return
        min_x, max_x = cones[:, 0].min(), cones[:, 0].max()
        min_y, max_y = cones[:, 1].min(), cones[:, 1].max()
        span = max(max_x - min_x, max_y - min_y, 1e-3)
        scale = 180.0 / span
        off_x = (200 - (max_x - min_x) * scale) / 2 - min_x * scale
        off_y = (200 - (max_y - min_y) * scale) / 2 - min_y * scale

        img = np.zeros((200, 200, 4), dtype=np.uint8)

        colors = {
            "blue": (255, 0, 0, 255),
            "yellow": (0, 255, 255, 255),
            "orange": (0, 165, 255, 255),
            "red": (0, 0, 255, 255),
        }

        def draw(cones_arr: np.ndarray) -> None:
            for x, y, col in cones_arr:
                xi = int(x * scale + off_x)
                yi = int(200 - (y * scale + off_y))
                color = colors.get(str(col), (255, 255, 255, 255))
                cv2.circle(img, (xi, yi), 3, color, -1)

        draw(left)
        draw(right)
        draw(start)

        msg = self.bridge.cv2_to_imgmsg(img, encoding="rgba8")
        msg.header.stamp = self.get_clock().now().to_msg()
        self.image_pub.publish(msg)


def main(args: List[str] | None = None) -> None:
    rclpy.init(args=args)
    node = TrackPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
