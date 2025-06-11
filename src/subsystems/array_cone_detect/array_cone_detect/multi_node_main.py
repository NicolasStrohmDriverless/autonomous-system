import os
import rclpy
from rclpy.executors import MultiThreadedExecutor

from array_cone_detect.detection_node import ConeArrayPublisher
from array_cone_detect.pathfinding_node import ConePathNode

def main():
    rclpy.init()
    nodes = [
        ConeArrayPublisher(),
        ConePathNode(),
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
