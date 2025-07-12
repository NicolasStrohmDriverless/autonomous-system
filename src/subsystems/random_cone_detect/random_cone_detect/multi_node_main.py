#!/usr/bin/env python3
import subprocess
import threading
import time
import random
import psutil
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float32

# Deine ROS-Nodes
from random_cone_detect.track_publisher import TrackPublisher
from random_cone_detect.detection_node import ConeArrayPublisher
from pathfinding.pathfinding_node import PathNode
from vehicle_control.car_state_node import CarStateNode
from vehicle_control.mapping_node import MappingNode
from vehicle_control.slam_node import SlamNode
from random_cone_detect.watchdog_node import WatchdogNode
from random_cone_detect.safety_watchdog_node import SafetyWatchdogNode
from random_cone_detect.lap_counter_node import LapCounterNode
from random_cone_detect.idle_monitor_node import IdleMonitorNode
from std_msgs.msg import Int32


class CompletionWatcher(Node):
    """Watch /lap_count and /lap_max to notify when the track is done."""

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
            self.get_logger().info('Track finished')
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


class MissionCheckNode(Node):
    """Placeholder node for initial mission checks."""

    def __init__(self):
        super().__init__('mission_check_node')
        # Just log a message once on startup
        self.get_logger().info('Mission check started')


def run_mode(mode: str, executor: MultiThreadedExecutor):
    """Run the system in three sequential phases."""

    # Use a single random seed so that all nodes operate on the same track
    seed = random.randrange(2**32 - 1)

    # Phase 1: mission check and track generation
    mission_nodes = [MissionCheckNode()]
    if mode != "accel":
        mission_nodes.append(TrackPublisher(seed=seed))
    for n in mission_nodes:
        executor.add_node(n)
    input("Mission Check abgeschlossen? [Enter]")
    for n in mission_nodes:
        executor.remove_node(n)
        n.destroy_node()

    # Phase 2: start remaining nodes and wait for confirmation
    stop_event = threading.Event()
    nodes = [
        CompletionWatcher(on_complete=stop_event.set),
        IdleMonitorNode(on_timeout=stop_event.set),
        ConeArrayPublisher(
            mode="accel" if mode == "accel" else "autox",
            max_laps=1 if mode == "accel" else (22 if mode == "endu" else 2),
            seed=seed if mode != "accel" else None,
        ),
        PathNode(),
        CarStateNode(),
        MappingNode(),
        SlamNode(),
        LapCounterNode(max_laps=1 if mode == "accel" else (22 if mode == "endu" else 2)),
    ]
    for n in nodes:
        executor.add_node(n)

    ready = input("darf ich starten? [J/Enter] ").strip().lower()
    if ready not in ('', 'j', 'ja', 'yes', 'y'):
        for n in nodes:
            executor.remove_node(n)
            n.destroy_node()
        return

    print(">> System läuft. Mit [Strg+C] beenden.")
    try:
        while not stop_event.is_set():
            time.sleep(0.1)
    except KeyboardInterrupt:
        stop_event.set()

    for n in nodes:
        if hasattr(n, 'shutdown'):
            try:
                n.shutdown()
            except Exception:
                pass
        executor.remove_node(n)
        n.destroy_node()


WATCHED_NODES = [
    'watchdog_node',
    'safety_watchdog_node',
    'system_usage_node',
    'cone_array_publisher',
    'midpoint_path_node',
    'car_state_node',
    'mapping_node',
    'slam_node',
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
            run_mode(mode, executor)
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
