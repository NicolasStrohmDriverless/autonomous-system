#!/usr/bin/env python3
"""Mission launcher for array_cone_detect.

Besides running the detection and visualization nodes from this package it
delegates vehicle control to ``control_main.py``.  After that script exits (for
example due to an EBS event) the mission selection menu is shown again.
"""

import subprocess
import sys
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor

from array_cone_detect.detection_node import ConeArrayPublisher
from pathfinding.pathfinding_node import PathNode
from path_viz.path_viz_node import PathVizNode
from art_slam.art_slam_node import ArtSlamNode


MODES = ["accel", "autox", "endu"]

CONTROL_SCRIPT = (
    "/home/strohmo/autonomous-system/src/subsystems/"
    "vehicle_control/vehicle_control/control_main.py"
)


def run_mode(mode: str, executor: MultiThreadedExecutor) -> None:
    """Start detection nodes and run the vehicle control script."""

    mission = ConeArrayPublisher()
    path = PathNode()
    viz = PathVizNode()
    slam = ArtSlamNode()

    nodes = [mission, path, viz, slam]
    for n in nodes:
        executor.add_node(n)

    ready = input("darf ich starten? [J/Enter] ").strip().lower()
    if ready not in ("", "j", "ja", "yes", "y"):
        for n in nodes:
            executor.remove_node(n)
            n.destroy_node()
        return

    proc = subprocess.Popen([sys.executable, CONTROL_SCRIPT])
    try:
        while rclpy.ok() and proc.poll() is None:
            time.sleep(0.1)
    except KeyboardInterrupt:
        proc.terminate()
    proc.wait()

    for n in nodes:
        if hasattr(n, "shutdown"):
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
            mode = inp if inp in MODES else "autox"
            run_mode(mode, executor)
    finally:
        rclpy.shutdown()
        spin_thread.join()


if __name__ == "__main__":
    main()
