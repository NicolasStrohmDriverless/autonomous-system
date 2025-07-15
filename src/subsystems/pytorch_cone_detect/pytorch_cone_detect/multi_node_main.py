#!/usr/bin/env python3
"""Mission launcher for pytorch_cone_detect.

This launcher starts the PyTorch-based detection pipeline and delegates vehicle
control to ``control_main.py``.  After that process exits the mission menu is
shown again.
"""

import subprocess
import sys
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor

from pytorch_cone_detect.detection_node import DepthAIDriver
from depth_tracking.depth_node import DepthTrackingNode
from path_viz.path_viz_node import PathVizNode
from art_slam.art_slam_node import ArtSlamNode
from imu_viz.imu_viz_node import ImuVizNode
from std_msgs.msg import Float32
from rclpy.node import Node


class SystemUsageNode(Node):
    def __init__(self):
        super().__init__('system_usage_node')
        self.cpu_pub = self.create_publisher(Float32, '/system/cpu_load', 10)
        self.gpu_pub = self.create_publisher(Float32, '/system/gpu_load', 10)
        self.create_timer(1.0, self.publish_usage)

    def publish_usage(self):
        import psutil
        cpu_percent = psutil.cpu_percent(interval=None)
        self.cpu_pub.publish(Float32(data=float(cpu_percent)))
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


MODES = ["accel", "autox", "endu"]

CONTROL_MODULE = "vehicle_control.control_main"


def run_mode(mode: str, executor: MultiThreadedExecutor) -> None:
    nodes = [
        DepthAIDriver(),
        DepthTrackingNode(),
        PathVizNode(),
        ArtSlamNode(),
        ImuVizNode(),
        SystemUsageNode(),
    ]
    for n in nodes:
        executor.add_node(n)

    ready = input("Ich bin ready, darf ich fahren? [J/Enter] ").strip().lower()
    if ready not in ("", "j", "ja", "yes", "y"):
        for n in nodes:
            executor.remove_node(n)
            n.destroy_node()
        return

    proc = subprocess.Popen([sys.executable, "-m", CONTROL_MODULE])
    try:
        while rclpy.ok() and proc.poll() is None:
            time.sleep(0.1)
    except KeyboardInterrupt:
        proc.terminate()
    proc.wait()

    for n in nodes:
        if hasattr(n, 'shutdown'):
            try:
                n.shutdown()
            except Exception:
                pass
        executor.remove_node(n)
        n.destroy_node()


def main() -> None:
    rclpy.init()
    executor = MultiThreadedExecutor()
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        while True:
            try:
                inp = input(
                    "Modus wählen ('accel' = Acceleration Track, 'autox' = Autocross Track, 'endu' = Endurance) [autox]: "
                ).strip().lower()
            except EOFError:
                break
            mode = inp if inp in MODES else 'autox'
            run_mode(mode, executor)
    finally:
        rclpy.shutdown()
        spin_thread.join()


if __name__ == "__main__":
    main()
