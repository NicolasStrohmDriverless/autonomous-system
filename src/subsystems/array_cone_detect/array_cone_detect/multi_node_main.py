import os
import rclpy
from rclpy.executors import MultiThreadedExecutor

from array_cone_detect.detection_node import ConeArrayPublisher
from pathfinding.pathfinding_node import PathNode
from art_slam.art_slam_node import ArtSlamNode
from path_viz.path_viz_node import PathVizNode
from array_cone_detect.watchdog_node import WatchdogNode
from array_cone_detect.safety_watchdog_node import SafetyWatchdogNode

def main():
    rclpy.init()
    nodes = [
        ConeArrayPublisher(),
        PathNode(),
        PathVizNode(),
        ArtSlamNode(),
    ]
    watchdog = WatchdogNode([n.get_name() for n in nodes] + ['safety_watchdog_node'])
    safety_watchdog = SafetyWatchdogNode([watchdog.get_name()])
    nodes.extend([watchdog, safety_watchdog])
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
