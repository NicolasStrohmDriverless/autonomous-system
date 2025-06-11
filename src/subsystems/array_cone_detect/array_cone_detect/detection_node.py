import rclpy
from rclpy.node import Node
from oak_cone_detect_interfaces.msg import ConeArray3D, Cone3D
from std_msgs.msg import Header
import json
import os

class ConeArrayPublisher(Node):
    def __init__(self):
        super().__init__('cone_array_publisher')

        # Parameter für Umschaltintervall (in Sekunden)
        self.declare_parameter('switch_interval', 10.0)
        self.switch_interval = self.get_parameter('switch_interval').get_parameter_value().double_value

        # Publisher für ConeArray3D
        self.publisher = self.create_publisher(ConeArray3D, '/cone_detections_3d', 10)

        # Lade die beiden Kegellisten
        self.cone_sets = []
        self.load_cone_sets()

        self.current_index = 0

        # Timer zum Umschalten der Kegellisten
        self.timer = self.create_timer(self.switch_interval, self.publish_cones)

    def load_cone_sets(self):
        # Pfade zu den JSON-Dateien
        file1 = '/home/strohmo/autonomous-system/src/subsystems/array_cone_detect/resource/cone_set1.json'
        file2 = '/home/strohmo/autonomous-system/src/subsystems/array_cone_detect/resource/cone_set2.json'

        # Lade die JSON-Dateien
        with open(file1, 'r') as f:
            data1 = json.load(f)
        with open(file2, 'r') as f:
            data2 = json.load(f)

        # Konvertiere die Daten in ConeArray3D-Nachrichten
        self.cone_sets.append(self.create_cone_array(data1))
        self.cone_sets.append(self.create_cone_array(data2))

    def create_cone_array(self, data):
        cone_array = ConeArray3D()
        cone_array.header = Header()
        cone_array.header.frame_id = 'map'

        for obj in data['objects']:
            cone = Cone3D()
            cone.id = obj['id']
            cone.label = ''
            cone.conf = 1.0
            cone.x = obj['X_mm'] / 1000.0  # Umrechnung mm -> m
            cone.y = obj['Y_mm'] / 1000.0
            cone.z = obj['Z_cm'] / 100.0   # Umrechnung cm -> m
            cone.color = obj['color']
            cone_array.cones.append(cone)

        return cone_array

    def publish_cones(self):
        # Aktualisiere den Zeitstempel
        msg = self.cone_sets[self.current_index]
        msg.header.stamp = self.get_clock().now().to_msg()

        # Veröffentliche die aktuelle Kegelliste
        self.publisher.publish(msg)
        self.get_logger().info(f'Published cone set {self.current_index + 1}')

        # Wechsle zur nächsten Kegelliste
        self.current_index = (self.current_index + 1) % len(self.cone_sets)

def main(args=None):
    rclpy.init(args=args)
    node = ConeArrayPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
