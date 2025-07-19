#!/usr/bin/env python3
"""Node publishing EBS activation state and auto-resetting after a timeout."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class EbsActiveNode(Node):
    """Publish a bool flag indicating whether the emergency brake is active."""

    def __init__(self):
        super().__init__('ebs_active_node')
        self.pub_active = self.create_publisher(Bool, '/system/ebs_active', 10)
        self.pub_ready = self.create_publisher(Bool, '/system/ebs_ready', 10)
        self.active = False
        self.ready = False
        self.reset_timer = None
        self.publish_state()
        self.get_logger().info('EbsActiveNode started')

    def announce_ready(self):
        """Publish a readiness flag without activating the EBS."""
        if not self.ready:
            self.ready = True
            self.get_logger().info('EBS ready to activate')
            self.publish_state()

    def publish_state(self):
        """Publish the current active state."""
        self.pub_active.publish(Bool(data=bool(self.active)))
        self.pub_ready.publish(Bool(data=bool(self.ready)))

    def trigger(self, duration: float = 10.0):
        """Activate EBS for ``duration`` seconds."""
        if self.active:
            return
        self.active = True
        self.ready = False
        self.get_logger().warn('EBS triggered')
        self.publish_state()
        if self.reset_timer is not None:
            self.reset_timer.cancel()
        self.reset_timer = self.create_timer(duration, self.deactivate)

    def shutdown(self) -> None:
        """Cancel pending timers before node destruction."""
        if self.reset_timer is not None:
            try:
                self.reset_timer.cancel()
            finally:
                try:
                    self.destroy_timer(self.reset_timer)
                except Exception:
                    pass
                self.reset_timer = None

    def deactivate(self):
        if not self.active:
            return
        self.active = False
        self.ready = False
        self.get_logger().info('EBS deactivated')
        self.publish_state()
        if self.reset_timer is not None:
            try:
                self.reset_timer.cancel()
            finally:
                try:
                    self.destroy_timer(self.reset_timer)
                except Exception:
                    pass
                self.reset_timer = None


def main(args=None):
    rclpy.init(args=args)
    node = EbsActiveNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
