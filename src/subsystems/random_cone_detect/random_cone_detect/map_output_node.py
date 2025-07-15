#!/usr/bin/env python3
"""Publish a simple map image based on the car state."""

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

MAP_SIZE = 100
CAR_WIDTH = 2.0
CAR_LENGTH = 4.0


class MapOutputNode(Node):
    """Maintain a tiny map and publish it as an image."""

    def __init__(self):
        super().__init__("map_output_node")
        self.map = np.zeros((MAP_SIZE, MAP_SIZE, 3), dtype=np.uint8)
        self.origin = MAP_SIZE // 2
        self.bridge = CvBridge()
        self.create_subscription(Pose2D, "/vehicle/car_state", self.state_cb, 10)
        self.image_pub = self.create_publisher(Image, "/mapping/image", 1)

    def state_cb(self, msg: Pose2D) -> None:
        ix = int(msg.x) + self.origin
        iy = int(msg.y) + self.origin
        w = int(CAR_WIDTH)
        l = int(CAR_LENGTH)
        x0 = max(ix - w // 2, 0)
        x1 = min(ix + w // 2, MAP_SIZE - 1)
        y0 = max(iy - l // 2, 0)
        y1 = min(iy + l // 2, MAP_SIZE - 1)
        self.map[y0 : y1 + 1, x0 : x1 + 1] = [255, 255, 255]
        img_msg = self.bridge.cv2_to_imgmsg(self.map, "bgr8")
        img_msg.header.stamp = self.get_clock().now().to_msg()
        self.image_pub.publish(img_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MapOutputNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
