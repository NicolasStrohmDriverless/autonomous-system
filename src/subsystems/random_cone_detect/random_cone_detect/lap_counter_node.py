#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2

class LapCounterNode(Node):
    def __init__(self, max_laps=2):
        super().__init__('lap_counter_node')
        self.max_laps = int(max_laps)
        self.lap = 0
        self.bridge = CvBridge()
        self.create_subscription(Int32, '/lap_count', self.lap_callback, 10)
        self.image_pub = self.create_publisher(Image, '/lap_counter/image', 1)
        self.timer = self.create_timer(0.5, self.publish_image)
        self.get_logger().info('LapCounterNode started')

    def lap_callback(self, msg: Int32):
        self.lap = int(msg.data)

    def publish_image(self):
        img = np.ones((30, 120, 3), dtype=np.uint8) * 255
        text = f"{self.lap}/{self.max_laps}"
        cv2.putText(img, text, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,0), 2)
        msg = self.bridge.cv2_to_imgmsg(img, 'bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        self.image_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LapCounterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
