#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
import math


def euler_from_quaternion(quat):
    """Convert quaternion to Euler angles.

    Args:
        quat (list or tuple): [x, y, z, w]

    Returns:
        tuple: roll, pitch, yaw in radians
    """
    x, y, z, w = quat
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = max(min(t2, 1.0), -1.0)
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z
from sensor_msgs.msg import Imu


class PathVizNode(Node):
    def __init__(self):
        super().__init__('path_viz_node')
        self.declare_parameter('world_frame', 'odom')
        self.declare_parameter('camera_frame', 'oak-d-base-frame')

        self.world_frame = self.get_parameter('world_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value

        self.marker_pub = self.create_publisher(MarkerArray, '/camera_path_markers', 10)

        # Subscribe to calibrated IMU data
        self.sub_imu = self.create_subscription(
            Imu,
            '/sensor/imu',
            self.imu_callback,
            10)

        # Internal state for simple dead reckoning
        self.position = [0.0, 0.0, 0.0]
        self.velocity = [0.0, 0.0, 0.0]
        self.orientation = None
        self.last_imu_time = None
        self.path_points = []

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('PathVizNode started')

    def timer_callback(self):
        if self.orientation is None:
            return

        p = Point(x=float(self.position[0]),
                   y=float(self.position[1]),
                   z=float(self.position[2]))
        self.path_points.append(p)

        arr = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)

        cube = Marker()
        cube.header.stamp = self.get_clock().now().to_msg()
        cube.header.frame_id = self.world_frame
        cube.ns = 'camera'
        cube.id = 0
        cube.type = Marker.CUBE
        cube.action = Marker.ADD
        cube.pose.position.x = float(self.position[0])
        cube.pose.position.y = float(self.position[1])
        cube.pose.position.z = float(self.position[2])
        cube.pose.orientation = self.orientation
        cube.scale.x = 0.1
        cube.scale.y = 0.1
        cube.scale.z = 0.1
        cube.color.r = 0.0
        cube.color.g = 0.0
        cube.color.b = 1.0
        cube.color.a = 1.0
        arr.markers.append(cube)

        line = Marker()
        line.header.stamp = self.get_clock().now().to_msg()
        line.header.frame_id = self.world_frame
        line.ns = 'path'
        line.id = 1
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = 0.05
        line.color.r = 1.0
        line.color.g = 0.0
        line.color.b = 0.0
        line.color.a = 1.0
        line.points = self.path_points
        arr.markers.append(line)

        self.marker_pub.publish(arr)

    def imu_callback(self, msg: Imu):
        now = self.get_clock().now().nanoseconds / 1e9
        if self.last_imu_time is None:
            self.last_imu_time = now
            self.orientation = msg.orientation
            return

        dt = now - self.last_imu_time
        self.last_imu_time = now

        self.orientation = msg.orientation

        q = msg.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z

        global_ax = ax * math.cos(yaw) - ay * math.sin(yaw)
        global_ay = ax * math.sin(yaw) + ay * math.cos(yaw)

        self.velocity[0] += global_ax * dt
        self.velocity[1] += global_ay * dt
        self.velocity[2] += az * dt

        self.position[0] += self.velocity[0] * dt
        self.position[1] += self.velocity[1] * dt
        self.position[2] += self.velocity[2] * dt


def main(args=None):
    rclpy.init(args=args)
    node = PathVizNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
