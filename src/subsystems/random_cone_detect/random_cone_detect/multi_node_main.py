#!/usr/bin/env python3
"""Mission launcher for random_cone_detect.

Additional monitoring nodes run here while vehicle control including the random
track generator is executed via ``control_main.py``.  After that script exits the
mission menu appears again.
"""

import subprocess
import sys
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor

from random_cone_detect.lap_counter_node import LapCounterNode
from random_cone_detect.idle_monitor_node import IdleMonitorNode
from random_cone_detect.map_output_node import MapOutputNode
from ebs_active.ebs_active_node import EbsActiveNode

MODES = ["accel", "autox", "endu"]

CONTROL_MODULE = "vehicle_control.control_main"


def run_mode(mode: str, executor: MultiThreadedExecutor) -> None:
    stop_event = threading.Event()
    ebs_node = EbsActiveNode()
    nodes = [
        LapCounterNode(
            max_laps=1 if mode == "accel" else (22 if mode == "endu" else 2)
        ),
        IdleMonitorNode(on_timeout=stop_event.set),
        MapOutputNode(),
        ebs_node,
    ]
    for n in nodes:
        executor.add_node(n)

    # Vehicle control (which also generates the random track) is started with
    # the preselected mode.  The control script will ask for the ready prompt
    # itself.
    proc = subprocess.Popen(
        [
            sys.executable,
            "-m",
            CONTROL_MODULE,
            "--mode",
            mode,
        ]
    )
    try:
        while rclpy.ok() and proc.poll() is None and not stop_event.is_set():
            time.sleep(0.1)
    except KeyboardInterrupt:
        proc.terminate()

    if stop_event.is_set() and proc.poll() is None:
        ebs_node.announce_ready()
        proc.terminate()
    proc.wait()

    if stop_event.is_set():
        ebs_node.trigger()
        time.sleep(10.0)
        stop_event.clear()

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
            run_mode(mode, executor)
    finally:
        rclpy.shutdown()
        spin_thread.join()


if __name__ == "__main__":
    main()
