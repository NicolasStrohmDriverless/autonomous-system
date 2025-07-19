#!/usr/bin/env python3
"""Vehicle control visualization node publishing car position as an image."""
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import Image

IMAGE_SIZE = 200
CAR_WIDTH_PX = 2
CAR_LENGTH_PX = 4
ORIGIN = IMAGE_SIZE // 2

class VehicleControllNode(Node):
    def __init__(self):
        super().__init__('vehicle_controll_node')
        self.position = Pose2D()
        self.create_subscription(Pose2D, '/vehicle/car_state', self.state_cb, 10)
        self.image_pub = self.create_publisher(Image, '/vehicle/car_image', 10)
        self.timer = self.create_timer(0.1, self.publish_image)
        self.get_logger().info('VehicleControllNode started')

    def state_cb(self, msg: Pose2D):
        self.position = msg

    def publish_image(self):
        img = np.zeros((IMAGE_SIZE, IMAGE_SIZE, 3), dtype=np.uint8)
        ix = int(self.position.x) + ORIGIN
        iy = int(self.position.y) + ORIGIN
        x0 = max(ix - CAR_WIDTH_PX // 2, 0)
        x1 = min(ix + CAR_WIDTH_PX // 2, IMAGE_SIZE - 1)
        y0 = max(iy - CAR_LENGTH_PX // 2, 0)
        y1 = min(iy + CAR_LENGTH_PX // 2, IMAGE_SIZE - 1)
        img[y0:y1+1, x0:x1+1] = [255, 0, 0]

        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.height = IMAGE_SIZE
        msg.width = IMAGE_SIZE
        msg.encoding = 'rgb8'
        msg.is_bigendian = False
        msg.step = IMAGE_SIZE * 3
        msg.data = img.tobytes()
        self.image_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = VehicleControllNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
