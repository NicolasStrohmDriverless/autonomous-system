import os
import rclpy
from rclpy.executors import MultiThreadedExecutor

from array_cone_detect.detection_node import ConeArrayPublisher
from pathfinding.pathfinding_node import PathNode
from art_slam.art_slam_node import ArtSlamNode

def main():
    rclpy.init()
    nodes = [
        ConeArrayPublisher(),
        PathNode(),
        ArtSlamNode(),
    ]
    executor = MultiThreadedExecutor(num_threads=len(nodes))
    for node in nodes:
        executor.add_node(node)
    try:
        executor.spin()
    finally:
        for node in nodes:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
