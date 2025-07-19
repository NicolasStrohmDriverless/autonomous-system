#!/usr/bin/env python3
"""Simple mapping node maintaining a 400x400 grid.

The map persists the driven path and drawn cones. Each pixel represents
half a meter in the real world.
"""
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Header
from oak_cone_detect_interfaces.msg import ConeArray3D
import math
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

MAP_SIZE = 400
PIXELS_PER_METER = 2.0  # 0.5 m per pixel
CAR_WIDTH_PX = 2
CAR_LENGTH_PX = 4


class MappingNode(Node):
    def __init__(self):
        super().__init__("mapping_node")
        self.map = np.zeros((MAP_SIZE, MAP_SIZE, 3), dtype=np.uint8)
        self.origin = MAP_SIZE // 2
        self.last_state = Pose2D()
        self.colors = {
            "blue": (0, 0, 255),
            "yellow": (255, 255, 0),
            "orange": (255, 165, 0),
            "red": (255, 0, 0),
        }
        self.create_subscription(Pose2D, "/vehicle/car_state", self.state_cb, 10)
        self.create_subscription(ConeArray3D, "/cone_detections_3d", self.cone_cb, 10)
        self.map_pub = self.create_publisher(Header, "/mapping/update", 10)
        self.image_pub = self.create_publisher(Image, "/mapping/image", 1)
        self.bridge = CvBridge()

    def state_cb(self, msg: Pose2D):
        # store state and draw rectangle at current position
        self.last_state = msg
        ix = int(round(msg.x * PIXELS_PER_METER)) + self.origin
        iy = int(round(msg.y * PIXELS_PER_METER)) + self.origin
        w = CAR_WIDTH_PX
        length = CAR_LENGTH_PX
        x0 = max(int(ix - w / 2), 0)
        x1 = min(x0 + w - 1, MAP_SIZE - 1)
        y0 = max(int(iy - length / 2), 0)
        y1 = min(y0 + length - 1, MAP_SIZE - 1)
        self.map[y0 : y1 + 1, x0 : x1 + 1] = [255, 0, 0]
        self.publish_map()

    def cone_cb(self, msg: ConeArray3D):
        yaw = math.radians(float(self.last_state.theta))
        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)
        px = float(self.last_state.x)
        py = float(self.last_state.y)
        for c in msg.cones:
            local_x = float(c.x)
            local_z = float(c.z)
            gx = px + cos_y * local_x - sin_y * local_z
            gy = py + sin_y * local_x + cos_y * local_z
            ix = int(round(gx * PIXELS_PER_METER)) + self.origin
            iy = int(round(gy * PIXELS_PER_METER)) + self.origin
            if 0 <= ix < MAP_SIZE and 0 <= iy < MAP_SIZE:
                color = self.colors.get(str(c.color), (255, 255, 255))
                self.map[iy, ix] = color
        self.publish_map()

    def publish_map(self) -> None:
        self.map_pub.publish(Header())
        img_msg = self.bridge.cv2_to_imgmsg(self.map, "rgb8")
        img_msg.header.stamp = self.get_clock().now().to_msg()
        self.image_pub.publish(img_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MappingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
