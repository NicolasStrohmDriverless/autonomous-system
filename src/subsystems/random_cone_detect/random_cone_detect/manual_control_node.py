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
from pynput import keyboard
from rclpy.node import Node
from std_msgs.msg import Bool, Float32

MAX_SPEED = 10.0  # m/s
MAX_STEERING = 30.0  # degrees
SPEED_RATE = 2.0  # m/s per second key held
ANGLE_RATE = 60.0  # deg per second key held


class ManualControlNode(Node):
    def __init__(self):
        super().__init__("manual_control_node")
        self.speed_pub = self.create_publisher(Float32, "/vehicle/desired_speed", 10)
        self.angle_pub = self.create_publisher(Float32, "/path_to_y_axis_angle", 10)
        self.ebs_pub = self.create_publisher(Bool, "/system/ebs_active", 10)
        self.current_speed = 0.0
        self.current_angle = 0.0
        self.lock = threading.Lock()
        self.pressed: set[str] = set()
        self.listener = keyboard.Listener(
            on_press=self._on_press, on_release=self._on_release
        )
        self.listener.start()
        self.input_thread = threading.Thread(target=self._update_loop, daemon=True)
        self.input_thread.start()
        self.get_logger().info("ManualControlNode started")

    def _publish(self) -> None:
        with self.lock:
            self.speed_pub.publish(Float32(data=float(self.current_speed)))
            self.angle_pub.publish(Float32(data=float(self.current_angle)))

    def _on_press(self, key) -> None:
        try:
            ch = key.char.lower()
        except AttributeError:
            return
        with self.lock:
            if ch == "q":
                self.pressed.clear()
                self.current_speed = 0.0
                self.ebs_pub.publish(Bool(data=True))
                self._publish()
            else:
                self.pressed.add(ch)

    def _on_release(self, key) -> None:
        try:
            ch = key.char.lower()
        except AttributeError:
            return
        with self.lock:
            self.pressed.discard(ch)

    def _update_loop(self) -> None:
        last = time.time()
        while rclpy.ok():
            now = time.time()
            dt = now - last
            last = now
            with self.lock:
                if "w" in self.pressed:
                    self.current_speed = min(
                        MAX_SPEED, self.current_speed + dt * SPEED_RATE
                    )
                if "s" in self.pressed:
                    self.current_speed = max(
                        0.0, self.current_speed - dt * SPEED_RATE
                    )
                if "a" in self.pressed:
                    self.current_angle = max(
                        -MAX_STEERING,
                        self.current_angle - dt * ANGLE_RATE,
                    )
                if "d" in self.pressed:
                    self.current_angle = min(
                        MAX_STEERING,
                        self.current_angle + dt * ANGLE_RATE,
                    )
            self._publish()
            time.sleep(0.1)


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
