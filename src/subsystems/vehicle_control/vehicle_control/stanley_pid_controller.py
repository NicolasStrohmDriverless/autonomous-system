#!/usr/bin/env python3
"""Stanley lateral controller with feedforward speed and curvature control."""
import math
from typing import List, Optional

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from tf_transformations import euler_from_quaternion


class StanleyPidController(Node):
    """Node implementing Stanley steering with feedforward speed control."""

    def __init__(self) -> None:
        super().__init__("stanley_pid_controller")
        self.declare_parameter("k", 1.0)
        self.declare_parameter("k_soft", 1e-6)
        self.declare_parameter("desired_speed", 1.0)
        self.declare_parameter("max_steering", math.radians(30.0))
        self.declare_parameter("wheel_base", 0.3)
        self.path_x: List[float] = []
        self.path_y: List[float] = []
        self.path_yaw: List[float] = []
        self.path_curvature: List[float] = []
        self.x: Optional[float] = None
        self.y: Optional[float] = None
        self.yaw: Optional[float] = None
        self.speed: float = 0.0
        self.create_subscription(Path, "/path", self.path_cb, 10)
        self.create_subscription(Odometry, "/odom", self.odom_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info("StanleyPidController started")

    def path_cb(self, msg: Path) -> None:
        poses = msg.poses
        if not poses:
            return
        self.path_x = [p.pose.position.x for p in poses]
        self.path_y = [p.pose.position.y for p in poses]
        self.path_yaw = []
        for i in range(1, len(poses)):
            dx = self.path_x[i] - self.path_x[i - 1]
            dy = self.path_y[i] - self.path_y[i - 1]
            self.path_yaw.append(math.atan2(dy, dx))
        self.path_yaw.append(self.path_yaw[-1])
        self.path_curvature = [0.0]
        for i in range(1, len(self.path_x) - 1):
            x1, y1 = self.path_x[i - 1], self.path_y[i - 1]
            x2, y2 = self.path_x[i], self.path_y[i]
            x3, y3 = self.path_x[i + 1], self.path_y[i + 1]
            dx1, dy1 = x2 - x1, y2 - y1
            dx2, dy2 = x3 - x2, y3 - y2
            ds1 = math.hypot(dx1, dy1)
            ds2 = math.hypot(dx2, dy2)
            yaw1 = math.atan2(dy1, dx1)
            yaw2 = math.atan2(dy2, dx2)
            dtheta = math.atan2(math.sin(yaw2 - yaw1), math.cos(yaw2 - yaw1))
            ds = (ds1 + ds2) / 2.0
            self.path_curvature.append(dtheta / ds if ds > 0.0 else 0.0)
        self.path_curvature.append(self.path_curvature[-1])

    def odom_cb(self, msg: Odometry) -> None:
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.yaw = yaw
        self.speed = msg.twist.twist.linear.x

    def control_loop(self) -> None:
        if self.x is None or not self.path_x:
            return
        # nearest path index
        dists = [
            (self.x - px) ** 2 + (self.y - py) ** 2
            for px, py in zip(self.path_x, self.path_y)
        ]
        target_idx = int(np.argmin(dists))
        path_yaw = self.path_yaw[target_idx]
        heading_error = path_yaw - (self.yaw or 0.0)
        heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))
        dx = self.x - self.path_x[target_idx]
        dy = self.y - self.path_y[target_idx]
        # sign by cross product
        cross = math.sin(path_yaw) * dx - math.cos(path_yaw) * dy
        cte = math.copysign(math.sqrt(dx * dx + dy * dy), cross)
        v = self.speed
        k = self.get_parameter("k").value
        k_soft = self.get_parameter("k_soft").value
        steer = heading_error + math.atan2(k * cte, v + k_soft)
        wheel_base = self.get_parameter("wheel_base").value
        curvature = self.path_curvature[target_idx]
        steer += math.atan(wheel_base * curvature)
        max_steer = self.get_parameter("max_steering").value
        steer = max(-max_steer, min(max_steer, steer))

        cmd = Twist()
        cmd.linear.x = float(self.get_parameter("desired_speed").value)
        cmd.angular.z = float(steer)
        self.cmd_pub.publish(cmd)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StanleyPidController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
