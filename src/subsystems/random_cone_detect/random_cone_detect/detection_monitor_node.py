#!/usr/bin/env python3
"""Monitor cone detections and trigger EBS on failure."""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from oak_cone_detect_interfaces.msg import ConeArray3D


class DetectionMonitorNode(Node):
    """Check that cone detections are published regularly."""

    def __init__(self, on_failure=None, timeout_cycles: int = 3) -> None:
        super().__init__('detection_monitor_node')
        self.on_failure = on_failure
        self.timeout_cycles = int(timeout_cycles)
        self.miss_count = 0
        self.started = False
        self.last_msg_time = self.get_clock().now()
        cone_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(
            ConeArray3D,
            '/cone_detections_3d',
            self.detection_cb,
            cone_qos,
        )
        self.timer = self.create_timer(0.1, self.check_timeout)

    def detection_cb(self, msg: ConeArray3D) -> None:
        self.started = True
        self.last_msg_time = self.get_clock().now()
        self.miss_count = 0

    def check_timeout(self) -> None:
        if not self.started:
            return
        elapsed = (self.get_clock().now() - self.last_msg_time).nanoseconds * 1e-9
        if elapsed >= 0.1:
            self.miss_count += 1
            self.last_msg_time = self.get_clock().now()
            if self.miss_count >= self.timeout_cycles:
                self.get_logger().warn(
                    'No cone detections received, triggering EBS')
                if self.on_failure is not None:
                    try:
                        self.on_failure()
                    finally:
                        pass
                self.miss_count = 0

