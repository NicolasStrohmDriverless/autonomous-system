#!/usr/bin/env python3
"""Interactive control script to select discipline and run track generation."""
import threading

import rclpy
from rclpy.executors import MultiThreadedExecutor

from random_cone_detect.track_publisher import TrackPublisher
from random_cone_detect.detection_node import ConeArrayPublisher
from pathfinding.pathfinding_node import PathNode
from vehicle_control.car_state_node import CarStateNode
from vehicle_control.mapping_node import MappingNode
from vehicle_control.slam_node import SlamNode
from random_cone_detect.watchdog_node import WatchdogNode
from random_cone_detect.safety_watchdog_node import SafetyWatchdogNode

MODES = ['accel', 'autox', 'endu']

class Control:
    def __init__(self, executor: MultiThreadedExecutor):
        self.executor = executor
        self.nodes = []
        self.stop_event = threading.Event()

    def start(self, mode: str):
        self.nodes = [
            WatchdogNode(['car_state_node', 'mapping_node', 'slam_node']),
            SafetyWatchdogNode(['watchdog_node']),
            TrackPublisher(),
            ConeArrayPublisher(mode=mode, max_laps=1),
            PathNode(),
            CarStateNode(),
            MappingNode(),
            SlamNode(),
        ]
        for n in self.nodes:
            self.executor.add_node(n)

    def stop(self):
        for n in self.nodes:
            if hasattr(n, 'shutdown'):
                try:
                    n.shutdown()
                except Exception:
                    pass
            self.executor.remove_node(n)
            n.destroy_node()
        self.nodes = []


def main():
    rclpy.init()
    executor = MultiThreadedExecutor()
    control = Control(executor)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        while True:
            mode = input("Disziplin wählen (accel, autox, endu) [autox]: ").strip().lower()
            if mode not in MODES:
                mode = 'autox'
            control.start(mode)
            ready = input("darf ich starten? [J/Enter] ").strip().lower()
            if ready not in ('', 'j', 'ja', 'yes', 'y'):
                control.stop()
                continue
            print("System läuft. Mit [q] oder [Space] stoppen")
            try:
                while True:
                    ch = input()
                    if ch.strip().lower() in ('q', ''):
                        print('EBS aktiv')
                        break
            except KeyboardInterrupt:
                print('EBS aktiv')
            control.stop()
    finally:
        rclpy.shutdown()
        spin_thread.join()

if __name__ == '__main__':
    main()
