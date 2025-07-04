#!/usr/bin/env python3
import subprocess
import threading
import time
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

    def __init__(self, on_complete=None):
        super().__init__('completion_watcher')
        self.lap = 0
        self.max_laps = 1
        self.on_complete = on_complete
        self.create_subscription(Int32, '/lap_count', self.lap_cb, 10)
        self.create_subscription(Int32, '/lap_max', self.max_cb, 10)

    def lap_cb(self, msg: Int32):
        self.lap = int(msg.data)
        if self.lap >= self.max_laps:
            self.get_logger().info('Track finished, shutting down')
            if self.on_complete is not None:
                try:
                    self.on_complete()
                finally:
                    pass
            else:
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


def run_mode(mode: str, executor: MultiThreadedExecutor, stop_event: threading.Event):
    """Start all nodes for the selected mode.

    CompletionWatcher and IdleMonitorNode are started first so they receive
    initial lap information from the publishers."""
    nodes = [
        CompletionWatcher(on_complete=stop_event.set),
        IdleMonitorNode(on_timeout=stop_event.set),
    ]

    if mode == "accel":
        nodes.extend([
            ConeArrayPublisher(mode="accel", max_laps=1),
            PathNode(),
            ArtSlamNode(),
            LapCounterNode(max_laps=1),
        ])
    elif mode == "endu":
        nodes.extend([
            TrackPublisher(),
            ConeArrayPublisher(mode="autox", max_laps=22),
            PathNode(),
            ArtSlamNode(),
            LapCounterNode(max_laps=22),
        ])
    else:  # autox
        nodes.extend([
            TrackPublisher(),
            ConeArrayPublisher(mode="autox", max_laps=2),
            PathNode(),
            ArtSlamNode(),
            LapCounterNode(max_laps=2),
        ])

    for node in nodes:
        executor.add_node(node)

    print(">> System läuft. Mit [Strg+C] beenden.")
    try:
        while not stop_event.is_set():
            time.sleep(0.1)
    except KeyboardInterrupt:
        stop_event.set()

    for node in nodes:
        if hasattr(node, 'shutdown'):
            try:
                node.shutdown()
            except Exception:
                pass
        executor.remove_node(node)
        node.destroy_node()
    stop_event.clear()


WATCHED_NODES = [
    'watchdog_node',
    'safety_watchdog_node',
    'system_usage_node',
    'track_generator_node',
    'cone_array_publisher',
    'midpoint_path_node',
    'art_slam_node',
    'lap_counter_node',
    'idle_monitor_node',
]


def spin_loop(executor: MultiThreadedExecutor, running: threading.Event):
    while running.is_set():
        executor.spin_once(timeout_sec=0.1)


def main():
    rclpy.init()
    executor = MultiThreadedExecutor()

    running = threading.Event()
    running.set()
    spin_thread = threading.Thread(target=spin_loop, args=(executor, running))
    spin_thread.start()

    watchdog = WatchdogNode(WATCHED_NODES)
    safety_watchdog = SafetyWatchdogNode([watchdog.get_name()])
    system_node = SystemUsageNode()

    for n in [watchdog, safety_watchdog, system_node]:
        executor.add_node(n)

    try:
        while True:
            try:
                inp = input(
                    "Modus wählen ('accel' = Acceleration Track, 'autox' = Autocross Track, 'endu' = Endurance) [autox]: "
                ).strip().lower()
            except EOFError:
                break
            mode = inp if inp in ["accel", "autox", "endu"] else "autox"
            stop_event = threading.Event()
            run_mode(mode, executor, stop_event)
    finally:
        running.clear()
        spin_thread.join()
        for n in [watchdog, safety_watchdog, system_node]:
            if hasattr(n, 'shutdown'):
                try:
                    n.shutdown()
                except Exception:
                    pass
            executor.remove_node(n)
            n.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
