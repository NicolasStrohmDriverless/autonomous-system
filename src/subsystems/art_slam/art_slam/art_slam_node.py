#!/usr/bin/env python3

import math
import numpy as np
from collections import deque

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Point
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray

from oak_cone_detect_interfaces.msg import ConeArray3D, Cone3D


class ArtSlamNode(Node):
    """Very simple SLAM example for cones."""

    def __init__(self):
        super().__init__('art_slam_node')

        self.declare_parameter('publish_range', 30.0)
        self.publish_range = float(self.get_parameter('publish_range').value)

        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_yaw = 0.0

        self.cone_map = {}
        self.paths = []
        self.current_path = []

        self.sub_cones = self.create_subscription(
            ConeArray3D,
            '/cone_detections_3d',
            self.cone_callback,
            10)

        self.sub_imu = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            10)

        self.map_pub = self.create_publisher(MarkerArray, '/art_slam/map', 10)
        self.range_pub = self.create_publisher(MarkerArray, '/art_slam/range_map', 10)
        self.path_pub = self.create_publisher(MarkerArray, '/art_slam/paths', 10)

    def imu_callback(self, msg: Imu):
        q = msg.orientation
        # yaw from quaternion
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.pose_yaw = math.atan2(siny_cosp, cosy_cosp)

    def cone_callback(self, msg: ConeArray3D):
        # transform cones to map frame
        for c in msg.cones:
            gx = (math.cos(self.pose_yaw) * c.x - math.sin(self.pose_yaw) * c.y) + self.pose_x
            gy = (math.sin(self.pose_yaw) * c.x + math.cos(self.pose_yaw) * c.y) + self.pose_y
            self.cone_map[c.id] = (gx, gy, c.color)

        self.current_path = self.compute_path()
        if self.current_path:
            self.paths.append(list(self.current_path))

        self.publish_markers(msg.header)

    def compute_path(self):
        # naive midpoint path between blue and yellow cones sorted by y
        left = []
        right = []
        for cid, (x, y, color) in self.cone_map.items():
            if color == 'blue':
                left.append((x, y))
            elif color == 'yellow':
                right.append((x, y))
        left.sort(key=lambda p: p[1])
        right.sort(key=lambda p: p[1])

        path = []
        for lx, ly in left:
            cand = [r for r in right if abs(r[1] - ly) < 1.0]
            if not cand:
                continue
            rx, ry = min(cand, key=lambda p: abs(p[1] - ly))
            mx = (lx + rx) / 2.0
            my = (ly + ry) / 2.0
            path.append((mx, my))
        path.sort(key=lambda p: p[1])
        return path

    def publish_markers(self, header: Header):
        all_markers = MarkerArray()
        range_markers = MarkerArray()
        path_markers = MarkerArray()

        clear = Marker()
        clear.action = Marker.DELETEALL
        all_markers.markers.append(clear)
        range_markers.markers.append(clear)
        path_markers.markers.append(clear)

        idx = 0
        for cid, (x, y, color) in self.cone_map.items():
            m = Marker()
            m.header = header
            m.ns = 'cones'
            m.id = idx
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position = Point(x=float(x), y=float(y), z=0.0)
            m.scale.x = 0.23
            m.scale.y = 0.23
            m.scale.z = 0.3
            if color == 'blue':
                m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 0.0, 1.0, 1.0
            elif color == 'yellow':
                m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 1.0, 0.0, 1.0
            else:
                m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.5, 0.0, 1.0
            all_markers.markers.append(m)
            if 0.0 <= y <= self.publish_range:
                range_markers.markers.append(m)
            idx += 1

        # path markers
        pid = 0
        for path in self.paths:
            if len(path) < 2:
                continue
            line = Marker()
            line.header = header
            line.ns = 'paths'
            line.id = pid
            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.scale.x = 0.05
            line.color.r = 0.0
            line.color.g = 1.0
            line.color.b = 0.0
            line.color.a = 1.0
            line.points = [Point(x=float(x), y=float(y), z=0.0) for x, y in path]
            path_markers.markers.append(line)
            pid += 1

        self.map_pub.publish(all_markers)
        self.range_pub.publish(range_markers)
        self.path_pub.publish(path_markers)


def main(args=None):
    rclpy.init(args=args)
    node = ArtSlamNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
