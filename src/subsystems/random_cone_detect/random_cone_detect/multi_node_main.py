#!/usr/bin/env python3
import subprocess
import sys

import psutil
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float32

# Deine ROS-Nodes
from random_cone_detect.track_publisher import TrackPublisher
from random_cone_detect.detection_node import ConeArrayPublisher
from pathfinding.pathfinding_node import PathNode
from art_slam.art_slam_node import ArtSlamNode
from random_cone_detect.watchdog_node import WatchdogNode
from random_cone_detect.safety_watchdog_node import SafetyWatchdogNode
from random_cone_detect.lap_counter_node import LapCounterNode
from random_cone_detect.idle_monitor_node import IdleMonitorNode
from std_msgs.msg import Int32


class CompletionWatcher(Node):
    """Watch /lap_count and /lap_max to stop the executor when the track is done."""

    def __init__(self):
        super().__init__('completion_watcher')
        self.lap = 0
        self.max_laps = 1
        self.create_subscription(Int32, '/lap_count', self.lap_cb, 10)
        self.create_subscription(Int32, '/lap_max', self.max_cb, 10)

    def lap_cb(self, msg: Int32):
        self.lap = int(msg.data)
        if self.lap >= self.max_laps:
            self.get_logger().info('Track finished, shutting down')
            rclpy.shutdown()

    def max_cb(self, msg: Int32):
        self.max_laps = int(msg.data)

class SystemUsageNode(Node):
    def __init__(self):
        super().__init__('system_usage_node')
        self.cpu_pub = self.create_publisher(Float32, '/system/cpu_load', 10)
        self.gpu_pub = self.create_publisher(Float32, '/system/gpu_load', 10)
        # alle 1s
        self.create_timer(1.0, self.publish_usage)

    def publish_usage(self):
        # CPU-Auslastung
        cpu_percent = psutil.cpu_percent(interval=None)
        cpu_msg = Float32(data=float(cpu_percent))
        self.cpu_pub.publish(cpu_msg)

        # GPU-Auslastung (NVIDIA)
        gpu_percent = self.get_gpu_usage()
        gpu_msg = Float32(data=float(gpu_percent))
        self.gpu_pub.publish(gpu_msg)

    def get_gpu_usage(self) -> float:
        try:
            result = subprocess.check_output(
                ['nvidia-smi', '--query-gpu=utilization.gpu', '--format=csv,noheader,nounits'],
                encoding='utf-8'
            )
            # erster Eintrag
            usage = float(result.strip().split('\n')[0])
            return usage
        except Exception as e:
            self.get_logger().warn(f"GPU usage read failed: {e}")
            return 0.0


def run_mode(mode: str):
    rclpy.init()
    executor = MultiThreadedExecutor()

    nodes = []
    if mode == "accel":
        nodes = [
            ConeArrayPublisher(mode="accel", max_laps=1),
            PathNode(),
            ArtSlamNode(),
            LapCounterNode(max_laps=1),
            SystemUsageNode()
        ]
    elif mode == "endu":
        nodes = [
            TrackPublisher(),
            ConeArrayPublisher(mode="autox", max_laps=22),
            PathNode(),
            ArtSlamNode(),
            LapCounterNode(max_laps=22),
            SystemUsageNode()
        ]
    else:  # autox
        nodes = [
            TrackPublisher(),
            ConeArrayPublisher(mode="autox", max_laps=2),
            PathNode(),
            ArtSlamNode(),
            LapCounterNode(max_laps=2),
            SystemUsageNode()
        ]

    watchdog = WatchdogNode(
        [n.get_name() for n in nodes] + ['safety_watchdog_node', 'idle_monitor_node']
    )
    safety_watchdog = SafetyWatchdogNode([watchdog.get_name()])
    nodes.extend([watchdog, safety_watchdog, CompletionWatcher(), IdleMonitorNode()])

    for node in nodes:
        executor.add_node(node)

    try:
        print(">> System läuft. Mit [Strg+C] beenden.")
        executor.spin()
    finally:
        for node in nodes:
            if hasattr(node, 'shutdown'):
                try:
                    node.shutdown()
                except Exception:
                    pass
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main():
    while True:
        try:
            inp = input(
                "Modus wählen ('accel' = Acceleration Track, 'autox' = Autocross Track, 'endu' = Endurance) [autox]: "
            ).strip().lower()
        except EOFError:
            break
        mode = inp if inp in ["accel", "autox", "endu"] else "autox"
        run_mode(mode)

if __name__ == '__main__':
    main()
