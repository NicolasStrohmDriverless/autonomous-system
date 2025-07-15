#!/usr/bin/env python3
"""Mission launcher for array_cone_detect.

Besides running the detection and visualization nodes from this package it
delegates vehicle control to ``control_main.py``.  After that script exits (for
example due to an EBS event) the mission selection menu is shown again.
"""

import argparse
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

CONTROL_MODULE = "vehicle_control.control_main"


def run_mode(mode: str, executor: MultiThreadedExecutor, auto_start: bool = False) -> None:
    """Start detection nodes and run the vehicle control script."""

    mission = ConeArrayPublisher()
    path = PathNode()
    viz = PathVizNode()
    slam = ArtSlamNode()

    nodes = [mission, path, viz, slam]
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
        return

    proc = subprocess.Popen([sys.executable, "-m", CONTROL_MODULE])
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
                mode = inp if inp in MODES else "autox"

            run_mode(mode, executor, auto_start=args.auto_start)

            if args.mode:
                break
    finally:
        rclpy.shutdown()
        spin_thread.join()


if __name__ == "__main__":
    main()
