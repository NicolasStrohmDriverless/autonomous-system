import os
import threading
import time
import rclpy
from rclpy.executors import MultiThreadedExecutor

from array_cone_detect.detection_node import ConeArrayPublisher
from pathfinding.pathfinding_node import PathNode
from art_slam.art_slam_node import ArtSlamNode
from path_viz.path_viz_node import PathVizNode
from array_cone_detect.watchdog_node import WatchdogNode
from array_cone_detect.safety_watchdog_node import SafetyWatchdogNode


class MissionCheckNode(rclpy.node.Node):
    """Placeholder node for initial mission checks."""

    def __init__(self):
        super().__init__('mission_check_node')
        self.get_logger().info('Mission check started')

def main():
    rclpy.init()
    executor = MultiThreadedExecutor()
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    # Phase 1: mission check
    mission = MissionCheckNode()
    executor.add_node(mission)
    input("Mission Check abgeschlossen? [Enter]")
    executor.remove_node(mission)
    mission.destroy_node()

    # Phase 2: start remaining nodes and ask for confirmation
    nodes = [
        ConeArrayPublisher(),
        PathNode(),
        PathVizNode(),
        ArtSlamNode(),
    ]
    watchdog = WatchdogNode([n.get_name() for n in nodes] + ['safety_watchdog_node'])
    safety_watchdog = SafetyWatchdogNode([watchdog.get_name()])
    nodes.extend([watchdog, safety_watchdog])
    for n in nodes:
        executor.add_node(n)

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
