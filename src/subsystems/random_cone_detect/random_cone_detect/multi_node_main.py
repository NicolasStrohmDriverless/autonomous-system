#!/usr/bin/env python3
"""Mission launcher for ``random_cone_detect``.

Previously this script only started a few helper nodes and delegated the
vehicle control stack to :mod:`control_main`.  To simplify running the system
in different configurations the launcher now creates all required nodes
directly.  The old watchdog node has been removed and its functionality is
handled internally here.  The safety watchdog node is still spawned
separately.
"""

import argparse
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2

from random_cone_detect.lap_counter_node import LapCounterNode
from random_cone_detect.detection_monitor_node import DetectionMonitorNode
from random_cone_detect.track_publisher import TrackPublisher
from random_cone_detect.detection_node import DetectionNode
from random_cone_detect.safety_watchdog_node import SafetyWatchdogNode
from pathfinding.pathfinding_node import PathNode
from vehicle_control.mapping_node import MappingNode
from vehicle_control.slam_node import SlamNode
from vehicle_control.car_state_node import CarStateNode
from ebs_active.ebs_active_node import EbsActiveNode
from std_msgs.msg import Float32

MODES = ["accel", "skidpad", "autox", "endu"]


class MultiWatchdogNode(Node):
    """Internal watchdog checking that important nodes stay alive."""

    def __init__(self, watched_nodes, on_failure=None):
        super().__init__("multi_watchdog_node")
        self.watched_nodes = list(watched_nodes)
        self.status = {name: True for name in self.watched_nodes}
        self.miss_count = {name: 0 for name in self.watched_nodes}
        self.on_failure = on_failure
        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, "/watchdog/image", 1)
        self.timer = self.create_timer(0.25, self.check_nodes)
        self.nodes_to_kill = []
        self.exec = None
        self.input_thread = None
        self.get_logger().info('MultiWatchdogNode started')

    def _input_loop(self) -> None:
        import sys
        import select
        import termios
        import tty
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        tty.setcbreak(fd)
        try:
            while rclpy.ok():
                r, _, _ = select.select([sys.stdin], [], [], 0.1)
                if r:
                    ch = sys.stdin.read(1)
                    if ch.lower() == "e":
                        self._kill_random_node()
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

    def _kill_random_node(self) -> None:
        import random
        candidates = [n for n in self.nodes_to_kill if hasattr(n, "destroy_node")]
        if not candidates:
            self.get_logger().warn("No nodes available to close")
            return
        node = random.choice(candidates)
        name = node.get_name()
        self.get_logger().warn(f"Closing node {name} via keyboard trigger")
        if self.exec is not None:
            try:
                self.exec.remove_node(node)
            except Exception:
                pass
        if hasattr(node, "shutdown"):
            try:
                node.shutdown()
            except Exception:
                pass
        node.destroy_node()

    def set_managed_nodes(self, nodes, executor):
        self.nodes_to_kill = [n for n in nodes if n is not self]
        self.exec = executor
        if self.input_thread is None:
            self.input_thread = threading.Thread(
                target=self._input_loop, daemon=True
            )
            self.input_thread.start()

    def check_nodes(self) -> None:
        alive = set(self.get_node_names())
        for name in self.watched_nodes:
            if name in alive:
                self.status[name] = True
                self.miss_count[name] = 0
            else:
                self.status[name] = False
                self.miss_count[name] += 1
                if self.miss_count[name] >= 3:
                    if self.on_failure is not None:
                        try:
                            self.on_failure()
                        finally:
                            pass
                    self.miss_count[name] = 0
        self.publish_image()

    def publish_image(self) -> None:
        row_h = 30
        width = 250
        img = np.ones((row_h * len(self.watched_nodes), width, 3), dtype=np.uint8) * 255
        for i, name in enumerate(self.watched_nodes):
            center = (width - 20, i * row_h + row_h // 2)
            color = (0, 255, 0) if self.status.get(name, False) else (0, 0, 255)
            cv2.circle(img, center, 10, color, -1)
            cv2.putText(
                img,
                name,
                (10, i * row_h + row_h // 2 + 5),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 0, 0),
                1,
            )
        msg = self.bridge.cv2_to_imgmsg(img, "bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        self.image_pub.publish(msg)


class SystemUsageNode(Node):
    """Publish system CPU/GPU load and temperature."""

    def __init__(self):
        super().__init__('system_usage_node')
        self.cpu_pub = self.create_publisher(Float32, '/system/cpu_load', 10)
        self.gpu_pub = self.create_publisher(Float32, '/system/gpu_load', 10)
        self.cpu_temp_pub = self.create_publisher(Float32, '/system/cpu_temp', 10)
        self.gpu_temp_pub = self.create_publisher(Float32, '/system/gpu_temp', 10)
        self.create_timer(1.0, self.publish_usage)
        self.get_logger().info('SystemUsageNode started')

    def publish_usage(self) -> None:
        import psutil

        cpu_percent = psutil.cpu_percent(interval=None)
        self.cpu_pub.publish(Float32(data=float(cpu_percent)))

        cpu_temp = 0.0
        try:
            temps = psutil.sensors_temperatures()
            for label in ('cpu-thermal', 'cpu_thermal', 'coretemp'):
                if label in temps and temps[label]:
                    cpu_temp = float(temps[label][0].current)
                    break
        except Exception:
            cpu_temp = 0.0
        self.cpu_temp_pub.publish(Float32(data=float(cpu_temp)))

        try:
            import subprocess as sp
            result = sp.check_output(
                ['nvidia-smi', '--query-gpu=utilization.gpu', '--format=csv,noheader,nounits'],
                encoding='utf-8'
            )
            usage = float(result.strip().split('\n')[0])
        except Exception:
            usage = 0.0
        self.gpu_pub.publish(Float32(data=float(usage)))

        gpu_temp = 0.0
        try:
            import subprocess as sp
            result = sp.check_output(
                ['nvidia-smi', '--query-gpu=temperature.gpu', '--format=csv,noheader,nounits'],
                encoding='utf-8'
            )
            gpu_temp = float(result.strip().split('\n')[0])
        except Exception:
            gpu_temp = 0.0
        self.gpu_temp_pub.publish(Float32(data=float(gpu_temp)))


def run_mode(
    mode: str, executor: MultiThreadedExecutor, auto_start: bool = False
) -> None:
    if auto_start:
        ready_ok = True
    else:
        ready = input("Ich bin ready, darf ich fahren? [J/Enter] ").strip().lower()
        # Only start when the user explicitly confirms.  Just pressing enter
        # should no longer be interpreted as approval.
        ready_ok = ready in ("j", "ja", "yes", "y")

    if not ready_ok:
        return

    stop_event = threading.Event()
    ebs_node = EbsActiveNode()

    def abort() -> None:
        """Trigger the emergency brake and stop the current mission."""
        ebs_node.trigger()  # keep active until manually reset
        stop_event.set()

    watchdog = MultiWatchdogNode(
            [
                "safety_watchdog_node",
                "ebs_active_node",
                "slam_node",
                "mapping_node",
                "midpoint_path_node",
            ],
            on_failure=abort,
        )
    safety_watchdog = SafetyWatchdogNode(["multi_watchdog_node"], on_failure=abort)
    nodes = [
        watchdog,
        safety_watchdog,
        TrackPublisher(mode=mode),
        DetectionNode(publish_all=True),
        PathNode(start_offset=0.0),
        MappingNode(),
        SlamNode(),
        SystemUsageNode(),
        CarStateNode(),
        LapCounterNode(
            max_laps=
            1 if mode == "accel" else (4 if mode == "skidpad" else (22 if mode == "endu" else 2))
        ),
        DetectionMonitorNode(on_failure=abort),
        ebs_node,
    ]
    watchdog.set_managed_nodes(nodes, executor)
    for n in nodes:
        executor.add_node(n)

    try:
        while rclpy.ok() and not stop_event.is_set():
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass

    if stop_event.is_set():
        print("EBS event triggered, aborting mission.")
        ebs_node.trigger()
        time.sleep(5.0)
        while True:
            try:
                ans = input(
                    "Zur\u00fcck zur Missionsauswahl? (Y/Enter f\u00fcr Ja) "
                ).strip().lower()
            except EOFError:
                ans = "y"
            if ans in ("", "y", "yes", "j", "ja"):
                break
            time.sleep(5.0)
        stop_event.clear()

    for n in nodes:
        if hasattr(n, "shutdown"):
            try:
                n.shutdown()
            except Exception:
                pass
        executor.remove_node(n)
        n.destroy_node()


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description="Mission launcher")
    parser.add_argument("--mode", choices=MODES, help="preselect mission mode")
    parser.add_argument(
        "--auto-start",
        action="store_true",
        help="skip the ready prompt and start immediately",
    )
    args = parser.parse_args(argv)

    rclpy.init()
    executor = MultiThreadedExecutor()
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        while True:
            if args.mode:
                mode = args.mode
            else:
                try:
                    inp = (
                        input(
                            "Modus wählen ('accel' = Acceleration Track, 'skidpad' = Skidpad, 'autox' = Autocross Track, 'endu' = Endurance) [autox]: "
                        )
                        .strip()
                        .lower()
                    )
                except EOFError:
                    break
                mode = inp if inp in MODES else "autox"

            run_mode(mode, executor, auto_start=args.auto_start)

            if args.mode:
                break
    finally:
        rclpy.shutdown()
        spin_thread.join()


if __name__ == "__main__":
    main()
