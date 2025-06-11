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

def main():
    # Interaktive Textabfrage
    inp = input(
        "Modus wählen ('accel' = Acceleration Track, 'autox' = Autocross Track, 'endu' = Endurance) [autox]: "
    ).strip().lower()
    mode = inp if inp in ["accel", "autox", "endu"] else "autox"

    rclpy.init()
    executor = MultiThreadedExecutor()

    nodes = []
    if mode == "accel":
        nodes = [
            ConeArrayPublisher(mode="accel", max_laps=1),
            PathNode(),
            SystemUsageNode()
        ]
    elif mode == "endu":
        nodes = [
            TrackPublisher(),
            ConeArrayPublisher(mode="autox", max_laps=22),
            PathNode(),
            SystemUsageNode()
        ]
    else:  # autox
        nodes = [
            TrackPublisher(),
            ConeArrayPublisher(mode="autox", max_laps=2),
            PathNode(),
            SystemUsageNode()
        ]

    for node in nodes:
        executor.add_node(node)

    try:
        print(">> System läuft. Mit [Strg+C] beenden.")
        executor.spin()
    finally:
        for node in nodes:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
