#!/usr/bin/env python3
"""Safety watchdog node watching the main watchdog."""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2


class SafetyWatchdogNode(Node):
    def __init__(self, watched_nodes):
        super().__init__('safety_watchdog_node')
        self.watched_nodes = list(watched_nodes)
        self.declare_parameter('check_period', 1.0)
        period = float(self.get_parameter('check_period').value)
        self.timer = self.create_timer(period, self.check_nodes)
        self.status = {name: True for name in self.watched_nodes}
        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, '/safety_watchdog/image', 1)
        self.get_logger().info(
            f'Watching nodes: {", ".join(self.watched_nodes)}')

    def check_nodes(self):
        alive_nodes = set(self.get_node_names())
        for name in self.watched_nodes:
            alive = name in alive_nodes
            self.status[name] = alive
            if not alive:
                self.get_logger().warn(f'Node "{name}" is not alive!')
        self.publish_image()

    def publish_image(self):
        row_h = 30
        width = 250
        img = np.ones((row_h * len(self.watched_nodes), width, 3), dtype=np.uint8) * 255
        for i, name in enumerate(self.watched_nodes):
            center = (width - 20, i * row_h + row_h // 2)
            color = (0, 255, 0) if self.status.get(name, False) else (0, 0, 255)
            cv2.circle(img, center, 10, color, -1)
            cv2.putText(img, name, (10, i * row_h + row_h // 2 + 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
        msg = self.bridge.cv2_to_imgmsg(img, 'bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        self.image_pub.publish(msg)


def main():
    rclpy.init()
    node = SafetyWatchdogNode([])
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
