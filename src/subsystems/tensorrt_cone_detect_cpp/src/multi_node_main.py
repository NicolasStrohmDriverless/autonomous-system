#!/usr/bin/env python3
import subprocess
import os
import psutil
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float32
from ament_index_python.packages import get_package_share_directory

from depth_tracking.depth_node import DepthTrackingNode
from pathfinding.pathfinding_node import PathNode
from imu_viz.imu_viz_node import ImuVizNode
from art_slam.art_slam_node import ArtSlamNode

class SystemUsageNode(Node):
    def __init__(self):
        super().__init__('system_usage_node')
        self.cpu_pub = self.create_publisher(Float32, '/system/cpu_load', 10)
        self.gpu_pub = self.create_publisher(Float32, '/system/gpu_load', 10)
        self.create_timer(1.0, self.publish_usage)

    def publish_usage(self):
        cpu_percent = psutil.cpu_percent(interval=None)
        self.cpu_pub.publish(Float32(data=float(cpu_percent)))
        gpu_percent = self.get_gpu_usage()
        self.gpu_pub.publish(Float32(data=float(gpu_percent)))

    def get_gpu_usage(self) -> float:
        try:
            result = subprocess.check_output(
                ['nvidia-smi', '--query-gpu=utilization.gpu', '--format=csv,noheader,nounits'],
                encoding='utf-8'
            )
            return float(result.strip().split('\n')[0])
        except Exception as e:
            self.get_logger().warn(f"GPU usage read failed: {e}")
            return 0.0


def main():
    rclpy.init()
    pkg_share = get_package_share_directory('tensorrt_cone_detect_cpp')
    model_path = os.path.join(pkg_share, 'resource', 'v11n_416x416.onnx')
    detection_proc = subprocess.Popen([
        'ros2', 'run', 'tensorrt_cone_detect_cpp', 'detection_node',
        '--ros-args', '-p', f'onnx_path:={model_path}'
    ])
    nodes = [
        DepthTrackingNode(),
        PathNode(),
        ArtSlamNode(),
        ImuVizNode(),
        SystemUsageNode(),
    ]
    rclpy.logging.get_logger('multi_node_main').info(
        'Using SORT-based tracking in detection node')
    executor = MultiThreadedExecutor(num_threads=len(nodes))
    for node in nodes:
        executor.add_node(node)
    try:
        executor.spin()
    finally:
        for node in nodes:
            node.destroy_node()
        detection_proc.terminate()
        detection_proc.wait()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
