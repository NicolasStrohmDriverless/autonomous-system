import os
import rclpy
import time
import threading
import subprocess
from std_msgs.msg import Float32
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from tensorrt_cone_detect.detection_node import DepthAIDriver
from depth_tracking.depth_node import DepthTrackingNode
from pathfinding.pathfinding_node import PathNode
from imu_viz.imu_viz_node import ImuVizNode
from art_slam.art_slam_node import ArtSlamNode
from path_viz.path_viz_node import PathVizNode
from tensorrt_cone_detect.watchdog_node import WatchdogNode
from tensorrt_cone_detect.safety_watchdog_node import SafetyWatchdogNode

import psutil
import shutil
import re


class MissionCheckNode(Node):
    """Placeholder node for initial mission checks."""

    def __init__(self):
        super().__init__('mission_check_node')
        self.get_logger().info('Mission check started')

class SystemUsageNode(Node):
    def __init__(self):
        super().__init__('system_usage_node')
        self.cpu_pub = self.create_publisher(Float32, '/system/cpu_load', 10)
        self.gpu_pub = self.create_publisher(Float32, '/system/gpu_load', 10)
        # alle 1s
        self.create_timer(1.0, self.publish_usage)

        # Confirm initialization
        self.get_logger().info('SystemUsageNode started')

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

def main():
    rclpy.init()
    executor = MultiThreadedExecutor()
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    mission = MissionCheckNode()
    executor.add_node(mission)
    input("Mission Check abgeschlossen? [Enter]")
    executor.remove_node(mission)
    mission.destroy_node()

    nodes = [
        DepthTrackingNode(),
        PathNode(),
        PathVizNode(),
        ArtSlamNode(),
        ImuVizNode(),
        SystemUsageNode(),
        DepthAIDriver(),
    ]
    watchdog = WatchdogNode([n.get_name() for n in nodes] + ['safety_watchdog_node'])
    safety_watchdog = SafetyWatchdogNode([watchdog.get_name()])
    nodes.extend([watchdog, safety_watchdog])
    rclpy.logging.get_logger('multi_node_main').info(
        'Using SORT-based tracking in detection node')
    for n in nodes:
        executor.add_node(n)
        rclpy.logging.get_logger('multi_node_main').info(
            f'{n.get_name()} added to executor')
    rclpy.logging.get_logger('multi_node_main').info('All nodes started')

    ready = input("darf ich starten? [J/Enter] ").strip().lower()
    if ready not in ('', 'j', 'ja', 'yes', 'y'):
        for n in nodes:
            executor.remove_node(n)
            n.destroy_node()
        rclpy.shutdown()
        spin_thread.join()
        return

    print(">> System läuft. Mit [Strg+C] beenden.")
    try:
        while rclpy.ok():
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        for n in nodes:
            if hasattr(n, 'shutdown'):
                try:
                    n.shutdown()
                except Exception:
                    pass
            executor.remove_node(n)
            n.destroy_node()
        rclpy.shutdown()
        spin_thread.join()

if __name__ == '__main__':
    main()
