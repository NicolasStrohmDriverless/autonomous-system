#!/usr/bin/env python3
"""Simple keyboard based manual control node.

This node adjusts desired speed and steering angle based on how long the
keys ``W``/``A``/``S``/``D`` are pressed.  The longer a key is held the
stronger the effect.  Results are published to the standard vehicle control
interfaces so that :class:`vehicle_control.car_state_node.CarStateNode`
updates the actual speed and steering angle accordingly.
"""
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

MAX_SPEED = 10.0  # m/s
MAX_STEERING = 30.0  # degrees
SPEED_RATE = 2.0  # m/s per second key held
ANGLE_RATE = 60.0  # deg per second key held


class ManualControlNode(Node):
    def __init__(self):
        super().__init__("manual_control_node")
        self.speed_pub = self.create_publisher(Float32, "/vehicle/desired_speed", 10)
        self.angle_pub = self.create_publisher(Float32, "/path_to_y_axis_angle", 10)
        self.current_speed = 0.0
        self.current_angle = 0.0
        self.lock = threading.Lock()
        self.input_thread = threading.Thread(target=self._input_loop, daemon=True)
        self.input_thread.start()
        self.get_logger().info("ManualControlNode started")

    def _publish(self) -> None:
        with self.lock:
            self.speed_pub.publish(Float32(data=float(self.current_speed)))
            self.angle_pub.publish(Float32(data=float(self.current_angle)))

    def _input_loop(self) -> None:
        import select
        import sys
        import termios
        import tty

        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        tty.setcbreak(fd)
        last = {}
        try:
            while rclpy.ok():
                r, _, _ = select.select([sys.stdin], [], [], 0.1)
                now = time.time()
                if r:
                    ch = sys.stdin.read(1).lower()
                    dt = now - last.get(ch, now)
                    with self.lock:
                        if ch == "w":
                            self.current_speed = min(
                                MAX_SPEED, self.current_speed + dt * SPEED_RATE
                            )
                        elif ch == "s":
                            self.current_speed = max(
                                0.0, self.current_speed - dt * SPEED_RATE
                            )
                        elif ch == "a":
                            self.current_angle = max(
                                -MAX_STEERING,
                                self.current_angle - dt * ANGLE_RATE,
                            )
                        elif ch == "d":
                            self.current_angle = min(
                                MAX_STEERING,
                                self.current_angle + dt * ANGLE_RATE,
                            )
                        elif ch == "q":
                            # emergency stop
                            self.current_speed = 0.0
                    last[ch] = now
                    self._publish()
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)


def main(args=None):
    rclpy.init(args=args)
    node = ManualControlNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
