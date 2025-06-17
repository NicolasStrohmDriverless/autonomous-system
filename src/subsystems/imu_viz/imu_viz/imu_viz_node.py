#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
import numpy as np  # Nur zur Mittelwertberechnung

class ImuVizNode(Node):
    def __init__(self):
        super().__init__('imu_viz')

        # Publisher für weitergeleitete (kalibrierte & geglättete) IMU-Messages
        self.pub = self.create_publisher(Imu, '/sensor/imu', 10)
        # Publisher für FPS der IMU-Callbacks
        self.pub_fps = self.create_publisher(Float32, '/imu_fps', 10)
        # Subscription auf rohe IMU-Daten
        self.sub = self.create_subscription(Imu, '/camera/imu_raw', self.cb, 50)

        # Zeitpunkt des Starts
        self.start_time = self.get_clock().now()

        # Dauer der Kalibrierungsphase (Sekunden)
        self.calibration_duration = 5.0

        # Buffers zum Speichern aller IMU-Rohwerte in den ersten 5 Sekunden
        self._accel_buffer = []   # Liste von [ax, ay, az]
        self._gyro_buffer  = []   # Liste von [gx, gy, gz]

        # Nachdem die Kalibrierungsphase vorbei ist, werden diese Bias-Werte fest gesetzt
        self.bias_accel = [0.0, 0.0, 0.0]
        self.bias_gyro  = [0.0, 0.0, 0.0]
        self.is_calibrated = False

        # Schwellenwert für Änderungen relativ zum vorherigen IMU-Wert
        self.fluctuation_threshold = 0.05

        # Für FPS-Berechnung
        self.last_time = None

        # Vorherige ungefilterte Werte speichern, um Änderungen zu bestimmen
        self.prev_raw_accel = None
        self.prev_raw_gyro = None

        self.get_logger().info('ImuVizNode gestartet, sammle Rohdaten für Kalibrierung (5 Sekunden)...')

    def _compute_mean(self, data_list):
        """
        Berechnet den arithmetischen Mittelwert einer Liste von 3-Komponenten-Vektoren.
        data_list: z.B. [[x1,y1,z1], [x2,y2,z2], ..., [xN,yN,zN]]
        Rückgabe: [mean_x, mean_y, mean_z]
        """
        n = len(data_list)
        if n == 0:
            return [0.0, 0.0, 0.0]
        sum_x = sum(v[0] for v in data_list)
        sum_y = sum(v[1] for v in data_list)
        sum_z = sum(v[2] for v in data_list)
        return [sum_x / n, sum_y / n, sum_z / n]

    def cb(self, msg: Imu):
        now = self.get_clock().now()

        # 1) FPS-Berechnung (unabhängig von Kalibrierung)
        if self.last_time is not None:
            dt = (now - self.last_time).nanoseconds * 1e-9
            fps = 1.0 / dt if dt > 0 else 0.0

            if self.pub_fps.get_subscription_count() > 0:
                fps_msg = Float32()
                fps_msg.data = float(fps)
                self.pub_fps.publish(fps_msg)
        self.last_time = now

        # 2) Prüfen, ob wir uns noch in der Kalibrierungsphase befinden
        elapsed = (now - self.start_time).nanoseconds * 1e-9

        if not self.is_calibrated:
            if elapsed <= self.calibration_duration:
                # Noch in den ersten 5 Sekunden: Rohdaten sammeln
                ax = msg.linear_acceleration.x
                ay = msg.linear_acceleration.y
                az = msg.linear_acceleration.z
                self._accel_buffer.append([ax, ay, az])

                gx = msg.angular_velocity.x
                gy = msg.angular_velocity.y
                gz = msg.angular_velocity.z
                self._gyro_buffer.append([gx, gy, gz])

                # In der Kalibrierungsphase noch keine Ausgabe
                return
            else:
                # ----- Kalibrierungsphase ist vorbei: Bias-Werte berechnen -----
                self.bias_accel = self._compute_mean(self._accel_buffer)
                self.bias_gyro  = self._compute_mean(self._gyro_buffer)
                self.is_calibrated = True

                self.get_logger().info(
                    f'Kalibrierung beendet. '
                    f'Bias Accel = {self.bias_accel}, '
                    f'Bias Gyro = {self.bias_gyro}'
                )
                # Jetzt fällt noch die aktuelle Nachricht in die kalibrierte Verarbeitung:
                # (Wir fahren fort, ohne return.)

        # 3) Ab hier: self.is_calibrated == True, wir korrigieren und glätten jede eingehende Nachricht
        # Erzeuge neues Imu-Msg-Objekt für die kalibrierten/glätteten Daten
        calibrated_msg = Imu()
        calibrated_msg.header = msg.header  # Original-Header mit Zeitstempel beibehalten

        # Orientierung übernehmen (unverändert), da wir nur accel/gyro korrigieren:
        calibrated_msg.orientation = msg.orientation
        calibrated_msg.orientation_covariance = msg.orientation_covariance

        # ----- Lineare Beschleunigung korrigieren und glätten -----
        ax_raw = msg.linear_acceleration.x
        ay_raw = msg.linear_acceleration.y
        az_raw = msg.linear_acceleration.z

        # Bias subtrahieren
        ax_corr = ax_raw - self.bias_accel[0]
        ay_corr = ay_raw - self.bias_accel[1]
        az_corr = az_raw - self.bias_accel[2]

        # Nur Änderungen über dem Schwellenwert weitergeben
        def threshold_current(curr_value, prev_value):
            if abs(curr_value - prev_value) < self.fluctuation_threshold:
                return 0.0
            return curr_value

        if self.prev_raw_accel is None:
            self.prev_raw_accel = [ax_corr, ay_corr, az_corr]

        ax_out = threshold_current(ax_corr, self.prev_raw_accel[0])
        ay_out = threshold_current(ay_corr, self.prev_raw_accel[1])
        az_out = threshold_current(az_corr, self.prev_raw_accel[2])

        calibrated_msg.linear_acceleration.x = ax_out
        calibrated_msg.linear_acceleration.y = ay_out
        calibrated_msg.linear_acceleration.z = az_out
        calibrated_msg.linear_acceleration_covariance = msg.linear_acceleration_covariance

        # Rohwerte für nächste Iteration speichern
        self.prev_raw_accel = [ax_corr, ay_corr, az_corr]

        # ----- Winkelgeschwindigkeit korrigieren und glätten -----
        gx_raw = msg.angular_velocity.x
        gy_raw = msg.angular_velocity.y
        gz_raw = msg.angular_velocity.z

        gx_corr = gx_raw - self.bias_gyro[0]
        gy_corr = gy_raw - self.bias_gyro[1]
        gz_corr = gz_raw - self.bias_gyro[2]

        if self.prev_raw_gyro is None:
            self.prev_raw_gyro = [gx_corr, gy_corr, gz_corr]

        gx_out = threshold_current(gx_corr, self.prev_raw_gyro[0])
        gy_out = threshold_current(gy_corr, self.prev_raw_gyro[1])
        gz_out = threshold_current(gz_corr, self.prev_raw_gyro[2])

        calibrated_msg.angular_velocity.x = gx_out
        calibrated_msg.angular_velocity.y = gy_out
        calibrated_msg.angular_velocity.z = gz_out
        calibrated_msg.angular_velocity_covariance = msg.angular_velocity_covariance

        self.prev_raw_gyro = [gx_corr, gy_corr, gz_corr]

        # 4) Publizieren, falls Abonnenten existieren
        if self.pub.get_subscription_count() > 0:
            self.pub.publish(calibrated_msg)


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
