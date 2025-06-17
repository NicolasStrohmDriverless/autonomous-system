#!/usr/bin/env python3

import math
import numpy as np
import struct
from collections import deque
from scipy.spatial import cKDTree

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, PointCloud2, PointField
from geometry_msgs.msg import Point
from std_msgs.msg import Header, Bool
from visualization_msgs.msg import Marker, MarkerArray

from oak_cone_detect_interfaces.msg import ConeArray3D, Cone3D


class ArtSlamNode(Node):
    """Very simple SLAM example for cones."""

    def __init__(self):
        super().__init__('art_slam_node')

        self.declare_parameter('publish_range', 30.0)
        self.declare_parameter('cluster_radius', 0.3)
        self.publish_range = float(self.get_parameter('publish_range').value)
        self.cluster_radius = float(self.get_parameter('cluster_radius').value)

        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_yaw = 0.0
        self.vel_x = 0.0
        self.vel_y = 0.0
        self.pose_z = 0.0
        self.vel_z = 0.0
        self.last_imu_time = None

        self.last_path = []
        self.ideal_path = []

        self.cone_map = {}
        self.paths = []
        self.current_path = []

        self.lap_done = False
        self.create_subscription(Bool, '/lap_completed', self.lap_callback, 10)

        self.sub_cones = self.create_subscription(
            ConeArray3D,
            '/cone_detections_3d',
            self.cone_callback,
            10)

        self.sub_imu = self.create_subscription(
            Imu,
            '/sensor/imu',
            self.imu_callback,
            10)

        self.map_pub = self.create_publisher(MarkerArray, '/art_slam/map', 10)
        self.range_pub = self.create_publisher(MarkerArray, '/art_slam/range_map', 10)
        self.path_pub = self.create_publisher(MarkerArray, '/art_slam/paths', 10)
        self.cloud_pub = self.create_publisher(PointCloud2, '/art_slam/cone_cloud', 10)

        # Confirm initialization
        self.get_logger().info('ArtSlamNode started')

    def lap_callback(self, msg: Bool):
        if msg.data and not self.lap_done:
            self.get_logger().info('Lap completed: publishing stored map.')
            self.lap_done = True

    def imu_callback(self, msg: Imu):
        """Update pose using only angular_velocity.y and linear_acceleration.z."""

        now = self.get_clock().now().nanoseconds / 1e9
        if self.last_imu_time is None:
            dt = 0.0
        else:
            dt = now - self.last_imu_time
        self.last_imu_time = now

        self.pose_yaw += msg.angular_velocity.y * dt

        az = msg.linear_acceleration.z
        self.vel_z += az * dt
        self.pose_z += self.vel_z * dt

    def cone_callback(self, msg: ConeArray3D):
        # transform cones to map frame and cluster duplicates
        for c in msg.cones:
            gx = (math.cos(self.pose_yaw) * c.x - math.sin(self.pose_yaw) * c.y) + self.pose_x
            gy = (math.sin(self.pose_yaw) * c.x + math.cos(self.pose_yaw) * c.y) + self.pose_y

            # update by id or cluster with nearby cones of same color
            if c.id in self.cone_map:
                self.cone_map[c.id] = (gx, gy, c.color)
                continue

            merged = False
            for cid, (px, py, col) in self.cone_map.items():
                if col == c.color and (px - gx) ** 2 + (py - gy) ** 2 <= self.cluster_radius ** 2:
                    mx = (px + gx) / 2.0
                    my = (py + gy) / 2.0
                    self.cone_map[cid] = (mx, my, col)
                    merged = True
                    break

            if not merged:
                self.cone_map[c.id] = (gx, gy, c.color)

        new_path = self.compute_path()
        if new_path:
            self.update_ideal_path(new_path)
            self.paths.append(list(self.ideal_path))
            self.current_path = list(self.ideal_path)
        else:
            self.current_path = []

        if self.lap_done:
            self.publish_markers(msg.header)

    def compute_path(self):
        # use KD-tree to efficiently find matching cones
        left = []
        right = []
        for cid, (x, y, color) in self.cone_map.items():
            if color == 'blue':
                left.append((x, y))
            elif color == 'yellow':
                right.append((x, y))

        if not left or not right:
            return []

        left.sort(key=lambda p: p[1])
        right_arr = np.array(right)
        tree = cKDTree(right_arr)

        path = []
        for lx, ly in left:
            idxs = tree.query_ball_point([lx, ly], r=1.0)
            if not idxs:
                continue
            best_idx = min(idxs, key=lambda i: abs(right_arr[i][1] - ly))
            rx, ry = right_arr[best_idx]
            mx = (lx + rx) / 2.0
            my = (ly + ry) / 2.0
            path.append((mx, my))

        path.sort(key=lambda p: p[1])
        return path

    def update_ideal_path(self, new_path, alpha: float = 0.8):
        """Blend new_path with last ideal path."""
        if not self.ideal_path:
            self.ideal_path = list(new_path)
            self.last_path = list(new_path)
            return

        m = min(len(new_path), len(self.ideal_path))
        updated = []
        for i in range(m):
            ox, oy = self.ideal_path[i]
            nx, ny = new_path[i]
            x = alpha * ox + (1 - alpha) * nx
            y = alpha * oy + (1 - alpha) * ny
            updated.append((x, y))

        if len(new_path) > m:
            updated.extend(new_path[m:])
        else:
            updated.extend(self.ideal_path[m:])

        self.ideal_path = updated
        self.last_path = list(new_path)

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
        self.publish_cloud(header)

    def publish_cloud(self, header: Header):
        pc = PointCloud2()
        pc.header = header
        pc.height = 1
        pc.width = len(self.cone_map)
        pc.is_dense = True
        pc.is_bigendian = False
        pc.point_step = 16
        pc.row_step = pc.point_step * pc.width
        pc.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        data = bytearray()
        for x, y, color in self.cone_map.values():
            intensity = 0.0
            if color == 'yellow':
                intensity = 1.0
            elif color not in ('blue', 'yellow'):
                intensity = 2.0
            data.extend(struct.pack('ffff', float(x), float(y), 0.0, intensity))

        pc.data = bytes(data)
        self.cloud_pub.publish(pc)


def main(args=None):
    rclpy.init(args=args)
    node = ArtSlamNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
