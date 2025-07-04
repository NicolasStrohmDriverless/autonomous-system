#!/usr/bin/env python3
import math
import random
import numpy as np
from scipy.spatial import cKDTree

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)
from std_msgs.msg import Header, Float32, Int32

from oak_cone_detect_interfaces.msg import ConeArray3D, Cone3D
from random_cone_detect.track_publisher import TrackGenerator, Mode

# Globale Schalter
ENABLE_NOISE     = False
ENABLE_DEVIATION = True
MAX_DEVIATION = 1.5

# Accel/Brake Werte
ACCEL_A = 9.81         # m/s² (Beschleunigung)
BRAKE_A = -14.715      # m/s² (Bremsen, negativ!)

# Maximale Fahrgeschwindigkeit f\xc3\xbc\r die Simulation
MAX_SPEED = 5.0  # [m/s]

# Track-Parameter
Y_FINISH = 75.0        # Wo Beschleunigen aufhört, Bremsen beginnt
Y_STOP   = 150.0       # Ende der Strecke

qos = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
    durability=QoSDurabilityPolicy.VOLATILE
)

def generate_accel_track():
    y_drive_end = Y_FINISH
    y_stop_end  = Y_STOP
    track_x_left = -1.5
    track_x_right = 1.5
    y_spacing = 3.0
    stop_side_spacing = 1.0
    stop_rear_spacing = 1.0

    centerline = [(0.0, y) for y in np.arange(0.0, y_stop_end + 0.1, y_spacing)]

    cones_l = []
    cones_r = []

    # Kegel bis zum Ende der Beschleunigungsstrecke (blau/gelb)
    for y in np.arange(0.0, y_drive_end + 0.1, y_spacing):
        cones_l.append([track_x_left, y, 'blue'])
        cones_r.append([track_x_right, y, 'yellow'])

    # Stopp-Bereich (orange)
    for y in np.arange(y_drive_end, y_stop_end + 0.1, stop_side_spacing):
        cones_l.append([track_x_left, y, 'orange'])
        cones_r.append([track_x_right, y, 'orange'])

    # Hinten quer Kegel (nur orange, nach Regelwerk)
    for x in np.arange(track_x_left, track_x_right + 0.1, stop_rear_spacing):
        cones_l.append([x, y_stop_end, 'orange'])  # als Extra-Liste (Kegel auf Linie hinten)

    # Startkegel orange an Start
    cones_l.append([track_x_left, 0.0, 'orange'])
    cones_r.append([track_x_right, 0.0, 'orange'])

    return np.array(cones_l, dtype=object), np.array(cones_r, dtype=object), np.array(centerline, dtype=float)

class ConeArrayPublisher(Node):
    def __init__(self, mode="autox", max_laps=2):
        super().__init__('cone_array_publisher')
        self.pub = self.create_publisher(ConeArray3D, '/cone_detections_3d', qos)
        self.mode = mode
        self.max_laps = max_laps
        self.lap = 0

        if self.mode == "accel":
            cones_l, cones_r, centerline = generate_accel_track()
            self.is_accel = True
        else:
            cones_l, cones_r, centerline = TrackGenerator(
                n_points=50, n_regions=10,
                min_bound=0.0, max_bound=100.0,
                mode=Mode.RANDOM
            ).create_track()
            # cones_l, cones_r als (N,2), also ohne Farbe
            # -> erweitere um Farben für Konsistenz
            cones_l = np.array([[x, y, 'blue'] for x, y in cones_l], dtype=object)
            cones_r = np.array([[x, y, 'yellow'] for x, y in cones_r], dtype=object)
            self.is_accel = False

        self.cones_left  = cones_l
        self.cones_right = cones_r
        self.centerline  = centerline

        seg = np.linalg.norm(np.diff(self.centerline, axis=0), axis=1)
        self.cumlen    = np.insert(np.cumsum(seg), 0, 0.0)
        self.total_len = float(self.cumlen[-1])

        self.update_rate = 20.0   # Hz
        self.window_length = 30.0

        # State für beide Modi
        self.timer = self.create_timer(1.0/self.update_rate, self.publish_cones)

        # Geteilter Status für Geschwindigkeit und Lenkwinkel
        self.speed_pub = self.create_publisher(Float32, '/vehicle/desired_speed', 10)
        self.speed_sub = self.create_subscription(Float32, '/vehicle/desired_speed', self.speed_callback, 10)
        self.angle_pub = self.create_publisher(Float32, '/vehicle/steering_angle', 10)
        self.angle_sub = self.create_subscription(Float32, '/vehicle/steering_angle', self.angle_callback, 10)
        self.external_speed = None
        self.external_angle = None

        # Lap count publisher
        self.lap_pub = self.create_publisher(Int32, '/lap_count', 10)
        self.max_pub = self.create_publisher(Int32, '/lap_max', 1)
        self.max_pub.publish(Int32(data=int(self.max_laps)))

        # maximale Geschwindigkeit als Parameter
        self.declare_parameter('max_speed', MAX_SPEED)
        self.max_speed = float(self.get_parameter('max_speed').value)

        # Accel-Mode States
        self.distance_traveled = 0.0  # Strecke entlang y
        self.v = 0.0                  # Momentangeschwindigkeit [m/s]
        self.time_accel = 0.0         # Zeit seit Start (nur für accel-mode)
        self.phase = "accel"          # "accel" oder "brake"

        # Autox/Endu-Mode States
        # Geschwindigkeit orientiert sich an der maximal zulässigen Geschwindigkeit
        self.speed = self.max_speed

        # Parameter für Geschwindigkeitanpassung
        self.lookahead = 2.0  # [m] Abstand zur Vorhersage des Lenkwinkels

        self.get_logger().info(
            f'ConeArrayPublisher startet: Track {self.total_len:.1f} m, Mode: {mode}, Max Laps: {self.max_laps}'
        )

    def _orientation_at(self, dist: float) -> float:
        """Rückgabe der Bahnausrichtung (Yaw) an Position dist [m]"""
        d = dist % self.total_len
        idx = np.searchsorted(self.cumlen, d, side='right')
        if idx == 0:
            p0, p1 = self.centerline[0], self.centerline[1]
        elif idx >= len(self.cumlen):
            p0, p1 = self.centerline[-2], self.centerline[-1]
        else:
            p0, p1 = self.centerline[idx-1], self.centerline[idx]
        v = p1 - p0
        return math.atan2(v[0], v[1])

    def _steer_angle(self, dist: float) -> float:
        """Berechnet den Lenkwinkel anhand der Streckengeometrie"""
        phi_now = self._orientation_at(dist)
        phi_future = self._orientation_at(dist + self.lookahead)
        return math.atan2(math.sin(phi_future - phi_now), math.cos(phi_future - phi_now))

    def speed_callback(self, msg: Float32):
        self.external_speed = float(msg.data)

    def angle_callback(self, msg: Float32):
        self.external_angle = float(msg.data)

    def publish_cones(self):
        dt = 1.0/self.update_rate
        # publish current lap count continuously
        self.lap_pub.publish(Int32(data=int(self.lap)))

        if self.is_accel:
            # --- Beschleunigung und Bremsen ---
            if self.phase == "accel":
                self.distance_traveled += self.v * dt + 0.5 * ACCEL_A * dt * dt
                self.v += ACCEL_A * dt
                self.time_accel += dt
                if self.distance_traveled >= Y_FINISH:
                    self.phase = "brake"
                    self.get_logger().info(
                        f'>> Wechsle zu BREMSEN bei y={self.distance_traveled:.2f}, v={self.v:.2f} m/s'
                    )
            elif self.phase == "brake":
                self.distance_traveled += self.v * dt + 0.5 * BRAKE_A * dt * dt
                self.v += BRAKE_A * dt
                if self.v < 0.0:
                    self.v = 0.0
                if self.distance_traveled >= Y_STOP or self.v == 0.0:
                    self.get_logger().info(
                        f'>> Fahrzeug steht! y={self.distance_traveled:.2f}'
                    )
                    self.lap += 1
                    self.lap_pub.publish(Int32(data=int(self.lap)))
                    if self.lap >= self.max_laps:
                        self.get_logger().info(
                            f'>> Alle Runden gefahren ({self.max_laps}). Stoppe Publisher.'
                        )
                        self.timer.cancel()
                        return
                    # Reset für die nächste Runde (hier nicht nötig, da nur eine Runde bei accel)
            current_speed = self.v
            angle = self.external_angle if self.external_angle is not None else 0.0
        else:
            angle = (
                self.external_angle
                if self.external_angle is not None
                else math.degrees(self._steer_angle(self.distance_traveled))
            )
            factor = max(0.3, 1.0 - abs(angle) / 90.0)
            base_speed = self.max_speed * factor
            self.speed = (
                self.external_speed if self.external_speed is not None else base_speed
            )
            self.distance_traveled += self.speed * dt
            current_speed = self.speed
            if self.distance_traveled >= self.total_len:
                self.lap += 1
                self.lap_pub.publish(Int32(data=int(self.lap)))
                self.get_logger().info(f'Runde {self.lap} beendet.')
                if self.lap >= self.max_laps:
                    self.get_logger().info(
                        f'Maximale Runden erreicht ({self.max_laps}) – stoppe Publisher.'
                    )
                    self.timer.cancel()
                    return
                else:
                    self.distance_traveled = 0.0  # Für neue Runde zurücksetzen

        # Sichtbare Kegel im aktuellen Fenster bestimmen
        d0 = self.distance_traveled
        d1 = d0 + self.window_length

        tree = cKDTree(self.centerline)
        def visible(cones):
            if cones.shape[0] == 0:
                return cones
            dists, idxs = tree.query(cones[:, :2].astype(float))
            d_along = self.cumlen[idxs]
            if d1 <= self.total_len:
                mask = (d_along >= d0) & (d_along <= d1)
            else:
                mask = (d_along >= d0) | (d_along <= (d1 - self.total_len))
            return cones[mask]

        vis_l = visible(self.cones_left)
        vis_r = visible(self.cones_right)

        idx = np.searchsorted(self.cumlen, d0, side='right')

        # default transform using centerline orientation
        if idx == 0:
            origin = self.centerline[0]
            v      = self.centerline[1] - origin
        elif idx >= len(self.cumlen):
            origin = self.centerline[-1]
            v      = origin - self.centerline[-2]
        else:
            t0, t1 = self.cumlen[idx-1], self.cumlen[idx]
            p0, p1 = self.centerline[idx-1], self.centerline[idx]
            frac   = (d0 - t0)/(t1 - t0)
            origin = p0 + frac*(p1 - p0)
            v      = p1 - p0

        phi = math.atan2(v[0], v[1])
        scale = 1.0

        # align track so that nearest cones are at x=±1.5, y=0
        if vis_l.shape[0] > 0 and vis_r.shape[0] > 0:
            vis_l_xy = vis_l[:, :2].astype(float)
            vis_r_xy = vis_r[:, :2].astype(float)
            l_near = vis_l_xy[np.argmin(np.linalg.norm(vis_l_xy - origin, axis=1))]
            r_near = vis_r_xy[np.argmin(np.linalg.norm(vis_r_xy - origin, axis=1))]
            origin = (l_near + r_near) / 2.0
            lr_vec = r_near - l_near
            width = np.linalg.norm(lr_vec)
            if width > 0:
                scale = 3.0 / width
            phi = math.atan2(lr_vec[1], lr_vec[0]) + math.pi / 2.0

        if self.external_angle is not None:
            phi = math.radians(self.external_angle)

        c_, s = math.cos(phi), math.sin(phi)
        R = np.array([[ c_, -s],
                      [ s,  c_]])

        vis_l_xy = vis_l[:, :2].astype(float)
        vis_l_colors = vis_l[:, 2]
        vis_l_tf = ((vis_l_xy - origin) @ R.T) * scale

        vis_r_xy = vis_r[:, :2].astype(float)
        vis_r_colors = vis_r[:, 2]
        vis_r_tf = ((vis_r_xy - origin) @ R.T) * scale

        # move track backwards by the commanded speed to simulate vehicle motion
        advance = -(self.external_speed or 0.0) * dt
        vis_l_tf[:, 1] += advance
        vis_r_tf[:, 1] += advance

        # ignore cones that moved behind the origin
        mask_l = vis_l_tf[:, 1] >= 0
        vis_l_tf = vis_l_tf[mask_l]
        vis_l_colors = vis_l_colors[mask_l]

        mask_r = vis_r_tf[:, 1] >= 0
        vis_r_tf = vis_r_tf[mask_r]
        vis_r_colors = vis_r_colors[mask_r]

        msg = ConeArray3D()
        msg.header = Header()
        msg.header.frame_id = 'map'
        msg.header.stamp    = self.get_clock().now().to_msg()

        def apply_deviation(x, y):
            if not ENABLE_DEVIATION or y <= 0:
                return x, y
            factor = (math.exp(y/self.window_length) - 1) / (math.e - 1)
            dev = MAX_DEVIATION * factor
            return x + np.random.uniform(-dev, dev), y + np.random.uniform(-dev, dev)

        for i, ((x, y), color) in enumerate(zip(vis_l_tf, vis_l_colors)):
            x_, y_ = apply_deviation(x, y)
            c = Cone3D(
                id=f'L{i}', label='left', conf=1.0,
                x=float(x_), y=0.0, z=float(y_),
                color=str(color)
            )
            msg.cones.append(c)

        for i, ((x, y), color) in enumerate(zip(vis_r_tf, vis_r_colors)):
            x_, y_ = apply_deviation(x, y)
            c = Cone3D(
                id=f'R{i}', label='right', conf=1.0,
                x=float(x_), y=0.0, z=float(y_),
                color=str(color)
            )
            msg.cones.append(c)

        self.pub.publish(msg)
        # Aktuelle Geschwindigkeit und Lenkwinkel publizieren
        if self.speed_pub.get_subscription_count() > 0:
            self.speed_pub.publish(Float32(data=float(current_speed)))
        if self.angle_pub.get_subscription_count() > 0:
            self.angle_pub.publish(Float32(data=float(angle)))

def main(args=None):
    rclpy.init(args=args)
    import sys
    mode = "autox"
    max_laps = 2
    # Modus und Runden je nach argv (oder interaktiv, wenn du willst)
    if len(sys.argv) > 1:
        mode_arg = sys.argv[1].lower()
        if mode_arg == "accel":
            mode = "accel"
            max_laps = 1
        elif mode_arg == "endu":
            mode = "autox"
            max_laps = 22
        else:
            mode = "autox"
            max_laps = 2
    node = ConeArrayPublisher(mode=mode, max_laps=max_laps)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
