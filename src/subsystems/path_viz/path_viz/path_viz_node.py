#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener
import tf2_ros
from rclpy.time import Time


class PathVizNode(Node):
    def __init__(self):
        super().__init__('path_viz_node')
        self.declare_parameter('world_frame', 'odom')
        self.declare_parameter('camera_frame', 'oak-d-base-frame')

        self.world_frame = self.get_parameter('world_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.marker_pub = self.create_publisher(MarkerArray, '/camera_path_markers', 10)
        self.path_points = []

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('PathVizNode started')

    def timer_callback(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                self.world_frame,
                self.camera_frame,
                Time())
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f'Transform not available: {e}')
            return

        p = Point(
            x=trans.transform.translation.x,
            y=trans.transform.translation.y,
            z=0.0,
        )
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
        cube.pose.position.x = trans.transform.translation.x
        cube.pose.position.y = trans.transform.translation.y
        cube.pose.position.z = trans.transform.translation.z
        cube.pose.orientation = trans.transform.rotation
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


def main(args=None):
    rclpy.init(args=args)
    node = PathVizNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
