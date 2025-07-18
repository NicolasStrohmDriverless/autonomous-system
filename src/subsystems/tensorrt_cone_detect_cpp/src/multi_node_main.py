#!/usr/bin/env python3
"""Mission launcher for tensorrt_cone_detect_cpp.

This launcher starts the C++ TensorRT detection node alongside some helper
nodes and delegates vehicle control to ``control_main.py``.  After that process
exits the mission menu is shown again.
"""

import argparse
import subprocess
import sys
import threading
import time
import os

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float32
from ament_index_python.packages import get_package_share_directory

from depth_tracking.depth_node import DepthTrackingNode
from path_viz.path_viz_node import PathVizNode
from art_slam.art_slam_node import ArtSlamNode
from imu_viz.imu_viz_node import ImuVizNode
from vehicle_control.car_state_node import CarStateNode


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


def run_mode(mode: str, executor: MultiThreadedExecutor, auto_start: bool = False) -> None:
    pkg_share = get_package_share_directory('tensorrt_cone_detect_cpp')
    model_path = os.path.join(pkg_share, 'resource', 'v11n_416x416.onnx')
    detection_proc = subprocess.Popen([
        'ros2', 'run', 'tensorrt_cone_detect_cpp', 'detection_node',
        '--ros-args', '-p', f'onnx_path:={model_path}'
    ])

    nodes = [
        DepthTrackingNode(),
        PathVizNode(),
        ArtSlamNode(),
        ImuVizNode(),
        SystemUsageNode(),
        CarStateNode(),
    ]
    for n in nodes:
        executor.add_node(n)

    if auto_start:
        ready_ok = True
    else:
        ready = input("Ich bin ready, darf ich fahren? [J/Enter] ").strip().lower()
        ready_ok = ready in ("", "j", "ja", "yes", "y")

    if not ready_ok:
        for n in nodes:
            executor.remove_node(n)
            n.destroy_node()
        detection_proc.terminate()
        detection_proc.wait()
        return

    proc = subprocess.Popen([sys.executable, "-m", CONTROL_MODULE])
    try:
        while rclpy.ok() and proc.poll() is None:
            time.sleep(0.1)
    except KeyboardInterrupt:
        proc.terminate()
    proc.wait()

    detection_proc.terminate()
    detection_proc.wait()

    for n in nodes:
        if hasattr(n, 'shutdown'):
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
                            "Modus wählen ('accel' = Acceleration Track, 'autox' = Autocross Track, 'endu' = Endurance) [autox]: "
                        )
                        .strip()
                        .lower()
                    )
                except EOFError:
                    break
                mode = inp if inp in MODES else 'autox'

            run_mode(mode, executor, auto_start=args.auto_start)

            if args.mode:
                break
    finally:
        rclpy.shutdown()
        spin_thread.join()


if __name__ == "__main__":
    main()
