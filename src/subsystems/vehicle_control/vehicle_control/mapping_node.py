#!/usr/bin/env python3
"""Simple mapping node maintaining a 100x100 grid."""
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

MAP_SIZE = 100
CAR_WIDTH = 2.0
CAR_LENGTH = 4.0

class MappingNode(Node):
    def __init__(self):
        super().__init__('mapping_node')
        self.map = np.zeros((MAP_SIZE, MAP_SIZE, 3), dtype=np.uint8)
        self.origin = MAP_SIZE // 2
        self.create_subscription(Pose2D, '/vehicle/car_state', self.state_cb, 10)
        self.map_pub = self.create_publisher(Header, '/mapping/update', 10)
        self.image_pub = self.create_publisher(Image, '/mapping/image', 1)
        self.bridge = CvBridge()

    def state_cb(self, msg: Pose2D):
        # place simple rectangle for car position
        ix = int(msg.x) + self.origin
        iy = int(msg.y) + self.origin
        w = int(CAR_WIDTH)
        length = int(CAR_LENGTH)
        x0 = max(ix - w // 2, 0)
        x1 = min(ix + w // 2, MAP_SIZE - 1)
        y0 = max(iy - length // 2, 0)
        y1 = min(iy + length // 2, MAP_SIZE - 1)
        self.map[y0:y1+1, x0:x1+1] = [255, 255, 255]
        self.map_pub.publish(Header())
        img_msg = self.bridge.cv2_to_imgmsg(self.map, 'rgb8')
        img_msg.header.stamp = self.get_clock().now().to_msg()
        self.image_pub.publish(img_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MappingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
