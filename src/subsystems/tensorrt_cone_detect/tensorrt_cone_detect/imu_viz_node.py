import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32

class ImuVizNode(Node):
    def __init__(self):
        super().__init__('imu_viz')
        # Publisher für weitergeleitete IMU-Messages
        self.pub = self.create_publisher(Imu, '/sensor/imu', 10)
        # Publisher für FPS der IMU-Callbacks
        self.pub_fps = self.create_publisher(Float32, '/imu_fps', 10)
        # Subscription auf rohe IMU-Daten
        self.sub = self.create_subscription(Imu, '/camera/imu_raw', self.cb, 50)

        # Initialisierung für FPS-Berechnung
        self.last_time = None

        # Confirm initialization
        self.get_logger().info('ImuVizNode started')

    def cb(self, msg: Imu):
        now = self.get_clock().now()
        # FPS nur berechnen, wenn wir schon einen vorherigen Zeitstempel haben
        if self.last_time is not None:
            dt = (now - self.last_time).nanoseconds * 1e-9
            fps = 1.0 / dt if dt > 0 else 0.0

            # FPS nur publizieren, wenn jemand zuhört
            if self.pub_fps.get_subscription_count() > 0:
                fps_msg = Float32()
                fps_msg.data = float(fps)
                self.pub_fps.publish(fps_msg)

        # Zeitstempel für das nächste Intervall merken
        self.last_time = now

        # Rohdaten weiterleiten, wenn Abonnenten vorhanden
        if self.pub.get_subscription_count() > 0:
            self.pub.publish(msg)

def main():
    rclpy.init()
    node = ImuVizNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
