#!/usr/bin/env python3
"""Publish a map image showing detected track cones and the current car position."""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import Image
from visualization_msgs.msg import MarkerArray
from oak_cone_detect_interfaces.msg import ConeArray3D
from cv_bridge import CvBridge

MAP_SIZE = 1000
SCALE = 10  # 1 dm per pixel
CAR_WIDTH_PX = 2
CAR_LENGTH_PX = 4

# simple BGR colors for drawing additional objects
CONE_COLORS = {
    "blue": (255, 0, 0),
    "yellow": (0, 255, 255),
    "orange": (0, 165, 255),
    "red": (0, 0, 255),
}


class MapOutputNode(Node):
    """Maintain a tiny map and publish it as an image."""

    def __init__(self):
        super().__init__("map_output_node")
        self.map = np.zeros((MAP_SIZE, MAP_SIZE, 3), dtype=np.uint8)
        self.origin = MAP_SIZE // 2
        self.bridge = CvBridge()
        self.pos_x = 0.0
        self.pos_y = 0.0
        # store seen cones so the map keeps detected track information
        self.cones = []

        self.create_subscription(Pose2D, "/vehicle/car_state", self.state_cb, 10)
        cone_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(
            ConeArray3D, "/cone_detections_3d", self.cone_cb, cone_qos
        )
        self.create_subscription(
            MarkerArray, "/best_path_marker", self.path_cb, 10
        )
        self.image_pub = self.create_publisher(Image, "/mapping/image", 1)
        self.track_image_pub = self.create_publisher(Image, "/track/image", 1)
        self.create_timer(1.0, self.publish_map)

        # draw initial car position at the origin so a map is available even
        # before any state updates arrive
        self.draw_car(self.pos_x, self.pos_y)
        self.publish_map()

    def draw_car(self, x: float, y: float) -> None:
        ix, iy = self.world_to_map(x, y)
        w = CAR_WIDTH_PX
        l = CAR_LENGTH_PX
        x0 = max(int(ix - w / 2), 0)
        x1 = min(int(ix + w / 2 - 1), MAP_SIZE - 1)
        y0 = max(int(iy - l / 2), 0)
        y1 = min(int(iy + l / 2 - 1), MAP_SIZE - 1)
        self.map[y0 : y1 + 1, x0 : x1 + 1] = [0, 0, 255]

    def world_to_map(self, x: float, y: float) -> tuple[int, int]:
        ix = int(round(x * SCALE)) + self.origin
        iy = int(round(y * SCALE)) + self.origin
        return ix, iy

    def draw_cones(self) -> None:
        for x, y, color in self.cones:
            ix, iy = self.world_to_map(x, y)
            if 0 <= ix < MAP_SIZE and 0 <= iy < MAP_SIZE:
                self.map[iy, ix] = CONE_COLORS.get(color, (255, 0, 255))


    def state_cb(self, msg: Pose2D) -> None:
        self.pos_x = float(msg.x)
        self.pos_y = float(msg.y)
        self.publish_map()

    def cone_cb(self, msg: ConeArray3D) -> None:
        # extend the stored list of cones so detections remain visible
        self.cones.extend((c.x, c.y, c.color) for c in msg.cones)
        self.publish_map()

    def path_cb(self, msg: MarkerArray) -> None:
        """Ignore path updates but trigger a map refresh."""

        self.publish_map()

    def publish_map(self) -> None:
        """Render the current map image and publish it."""

        # start with a blank map so only the current state is shown
        self.map[:] = 0

        # draw static track elements and the current car position
        self.draw_cones()
        self.draw_car(self.pos_x, self.pos_y)

        img_msg = self.bridge.cv2_to_imgmsg(self.map, "bgr8")
        img_msg.header.stamp = self.get_clock().now().to_msg()
        self.image_pub.publish(img_msg)
        self.track_image_pub.publish(img_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MapOutputNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
