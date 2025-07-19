#!/usr/bin/env python3
"""Placeholder SLAM node storing detections and path."""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import Header
from oak_cone_detect_interfaces.msg import ConeArray3D
from geometry_msgs.msg import Point

class SlamNode(Node):
    def __init__(self):
        super().__init__('slam_node')
        self.cones = []
        self.path = []
        cone_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(ConeArray3D, '/cone_detections_3d', self.cone_cb, cone_qos)
        self.create_subscription(Point, '/path_point', self.path_cb, 10)
        self.map_pub = self.create_publisher(Header, '/slam/update', 10)
        self.get_logger().info('SlamNode started')

    def cone_cb(self, msg: ConeArray3D):
        self.cones.extend([(c.x, c.y, c.color) for c in msg.cones])
        self.map_pub.publish(Header())

    def path_cb(self, msg: Point):
        self.path.append((msg.x, msg.y))
        self.map_pub.publish(Header())


def main(args=None):
    rclpy.init(args=args)
    node = SlamNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
