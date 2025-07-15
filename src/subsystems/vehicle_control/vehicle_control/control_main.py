#!/usr/bin/env python3
"""Interactive control script to select discipline and run track generation.

This module can be used interactively or be called with command line
arguments to pre-select the mission mode.  When a ``--mode`` is supplied the
script will run exactly one mission without further prompts for the mode.
"""
import argparse
import threading

import rclpy
from rclpy.executors import MultiThreadedExecutor

from random_cone_detect.track_publisher import TrackPublisher
from random_cone_detect.detection_node import ConeArrayPublisher
from pathfinding.pathfinding_node import PathNode
from vehicle_control.mapping_node import MappingNode
from vehicle_control.slam_node import SlamNode
from random_cone_detect.watchdog_node import WatchdogNode
from random_cone_detect.safety_watchdog_node import SafetyWatchdogNode

MODES = ["accel", "autox", "endu"]


class Control:
    def __init__(self, executor: MultiThreadedExecutor):
        self.executor = executor
        self.nodes = []
        self.stop_event = threading.Event()

    def start(self, mode: str):
        self.nodes = [
            WatchdogNode(["mapping_node", "slam_node"]),
            SafetyWatchdogNode(["watchdog_node"]),
            TrackPublisher(),
            ConeArrayPublisher(mode=mode, max_laps=1),
            PathNode(),
            MappingNode(),
            SlamNode(),
        ]
        for n in self.nodes:
            self.executor.add_node(n)

    def stop(self):
        for n in self.nodes:
            if hasattr(n, "shutdown"):
                try:
                    n.shutdown()
                except Exception:
                    pass
            self.executor.remove_node(n)
            n.destroy_node()
        self.nodes = []


def main(argv=None):
    """Run the vehicle control stack.

    When called without arguments this function behaves exactly like before and
    presents an interactive menu for selecting the mission mode.  If a ``--mode``
    argument is supplied the script will run once using that mode and exit after
    completion.  The ``--auto-start`` flag can be used to skip the ready prompt.
    """

    parser = argparse.ArgumentParser(description="Vehicle control entrypoint")
    parser.add_argument("--mode", choices=MODES, help="preselect mission mode")
    parser.add_argument(
        "--auto-start",
        action="store_true",
        help="skip the ready prompt and start immediately",
    )
    args = parser.parse_args(argv)

    rclpy.init()
    executor = MultiThreadedExecutor()
    control = Control(executor)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        while True:
            if args.mode:
                mode = args.mode
            else:
                mode_inp = (
                    input("Disziplin wählen (accel, autox, endu) [autox]: ")
                    .strip()
                    .lower()
                )
                mode = mode_inp if mode_inp in MODES else "autox"

            control.start(mode)

            if args.auto_start:
                ready_ok = True
            else:
                ready = input("darf ich starten? [J/Enter] ").strip().lower()
                ready_ok = ready in ("", "j", "ja", "yes", "y")

            if not ready_ok:
                control.stop()
                if args.mode:
                    break
                else:
                    continue

            print("System läuft. Mit [q] oder [Space] stoppen")
            try:
                while True:
                    ch = input()
                    if ch.strip().lower() in ("q", ""):
                        print("EBS aktiv")
                        break
            except KeyboardInterrupt:
                print("EBS aktiv")

            control.stop()
            if args.mode:
                break
    finally:
        rclpy.shutdown()
        spin_thread.join()


if __name__ == "__main__":
    main()
