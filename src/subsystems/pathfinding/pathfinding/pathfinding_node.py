#!/usr/bin/env python3
import math
import random
import time

import numpy as np
from scipy.spatial import Delaunay
from std_msgs.msg import ColorRGBA, Float32
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)
from oak_cone_detect_interfaces.msg import ConeArray3D

# QoS nur für den Subscriber: Best Effort, volatile, depth=1
qos_sub = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
    durability=QoSDurabilityPolicy.VOLATILE
)

# Pfad- und Marker-Parameter
PATH_LENGTH             = 20.0    # nun 20 m
MAX_ANGLE               = 40.0
FIRST_STEP_MAX_ANGLE    = 50.0
MIDPOINT_MARKER_SCALE   = 0.1
DEFAULT_CONE_SCALE      = (0.228, 0.228, 0.325)
LARGE_ORANGE_CONE_SCALE = (0.285, 0.285, 0.505)
CONE_POSITION_SCALE     = (1.0, 1.0, 1.0)
COLOR_MAP = {
    'blue':   [0.0, 0.0, 1.0, 1.0],
    'yellow': [1.0, 1.0, 0.0, 1.0],
    'orange': [1.0, 0.5, 0.0, 1.0],
    'red':    [1.0, 0.0, 0.0, 1.0],
}
MIDPOINT_COLOR    = [0.5, 0.5, 0.5, 1.0]
SIDE_CHECK_RADIUS = 2.0
MAX_STEP_DIST     = 2.0
GEGENCHECK        = 1
FALLBACK_DIST     = 1.0  # Mindestpfadlänge, falls keine Punkte gefunden werden
SMOOTH_ALPHA      = 0.4   # Faktor für exponentielle Glättung des Pfades

# Neuer Winkel-Filter: größere Fenster, höhere Toleranz und EMA-Glättung
ANGLE_WINDOW        = 7     # Anzahl der letzten Winkel für Median
ANGLE_JUMP_THRESH   = 30.0  # Maximal erlaubter Sprung in °
ANGLE_SMOOTH_ALPHA  = 0.3   # Faktor für exponentielle Glättung des Winkels
MAX_SPEED          = 5.0    # Maximale Geschwindigkeit in m/s

def smooth_path(path, alpha=SMOOTH_ALPHA):
    """Exponentielle Glättung eines Pfades."""
    if not path:
        return []
    smoothed = [np.array(path[0])]
    for p in path[1:]:
        smoothed.append(alpha * np.array(p) + (1 - alpha) * smoothed[-1])
    return [tuple(p) for p in smoothed]


class PathNode(Node):
    def __init__(self):
        super().__init__('midpoint_path_node')

        # Subscriber mit Best-Effort QoS
        self.sub = self.create_subscription(
            ConeArray3D,
            '/cone_detections_3d',
            self.callback,
            qos_profile=qos_sub
        )

        # Publisher für Kegel-, Pfad- und Winkel-Marker
        self.marker_pub = self.create_publisher(MarkerArray, '/cone_markers', 10)
        self.path_pub   = self.create_publisher(MarkerArray, '/best_path_marker', 10)
        self.fps_pub    = self.create_publisher(Float32,     '/path_inference_fps', 10)
        self.angle_pub       = self.create_publisher(Float32, '/path_to_y_axis_angle', 10)
        self.angle_image_pub = self.create_publisher(Image,   '/path_status/angle_image', 1)
        self.speed_image_pub = self.create_publisher(Image,   '/path_status/speed_image', 1)
        self.track_image_pub = self.create_publisher(Image,   '/path_status/track_image', 1)

        self.declare_parameter('max_speed', MAX_SPEED)
        self.max_speed = float(self.get_parameter('max_speed').value)

        # Geschwindigkeit aus IMU (optional)
        self.speed = None
        self.create_subscription(Float32, '/vehicle/speed', self.speed_callback, 10)

        # Geteilter Status mit anderen Nodes
        self.speed_cmd_pub = self.create_publisher(Float32, '/vehicle/desired_speed', 10)
        self.speed_cmd_sub = self.create_subscription(Float32, '/vehicle/desired_speed', self.speed_cmd_callback, 10)
        self.angle_shared_pub = self.create_publisher(Float32, '/vehicle/steering_angle', 10)
        self.angle_shared_sub = self.create_subscription(Float32, '/vehicle/steering_angle', self.angle_shared_callback, 10)
        self.desired_speed = None
        self.shared_angle = None
        # used to ignore speed command messages originating from this node
        self._ignore_next_speed_msg = False

        self.bridge = CvBridge()

        # Confirm initialization
        self.get_logger().info('PathNode started')

        # interne Puffer
        self._frame_times    = []
        self.prev_bg         = None
        self.prev_or         = None
        self._angle_buffer   = []
        self._angle_smoothed = None

    def speed_callback(self, msg: Float32):
        self.speed = float(msg.data)

    def speed_cmd_callback(self, msg: Float32):
        if self._ignore_next_speed_msg:
            # ignore messages resulting from this node's own publication
            self._ignore_next_speed_msg = False
            return
        self.desired_speed = float(msg.data)

    def angle_shared_callback(self, msg: Float32):
        self.shared_angle = float(msg.data)

    def callback(self, msg: ConeArray3D):
        t0 = time.time()

        # 1) Kegel sammeln
        cones = {col: [] for col in COLOR_MAP}
        for c in msg.cones:
            if c.y < 0:
                continue
            if c.color not in cones:
                # ignore unknown colors
                continue
            p = np.array([c.x, c.z, 0]) * np.array(CONE_POSITION_SCALE)
            cones[c.color].append(p)

        # 2) Kegel-Marker
        markers = MarkerArray()
        clear = Marker(); clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        for col, pts in cones.items():
            for idx, p in enumerate(pts):
                m = Marker()
                m.header = msg.header
                m.ns     = f'cone_{col}'
                m.id     = hash(col) % 1000 + idx
                m.type   = Marker.CYLINDER
                m.action = Marker.ADD
                m.pose.position = Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))
                scale = LARGE_ORANGE_CONE_SCALE if col=='orange' else DEFAULT_CONE_SCALE
                m.scale.x, m.scale.y, m.scale.z = scale
                r, g, b, a = COLOR_MAP[col]
                m.color = ColorRGBA(r=r, g=g, b=b, a=a)
                markers.markers.append(m)

        # 3) Mittelpunkte via Delaunay
        pts2d, cols2d = [], []
        for col, pts in cones.items():
            for p in pts:
                pts2d.append(p[:2])
                cols2d.append(col)

        mids_bg, mids_or = [], []
        if len(pts2d) >= 2:
            try:
                tri = Delaunay(np.array(pts2d))
                for s in tri.simplices:
                    for i in range(3):
                        a, b = s[i], s[(i+1)%3]
                        cs = {cols2d[a], cols2d[b]}
                        mid = tuple(((np.array(pts2d[a]) + np.array(pts2d[b]))/2).round(4))
                        if mid[1] <= 0:
                            continue
                        if cs == {'blue','yellow'}:
                            mids_bg.append(mid)
                        if cs == {'orange'}:
                            mids_or.append(mid)
            except:
                pass

        # Erstes Sortieren, bevor Zusatzpunkte ergänzt werden
        mids_bg = sorted(set(mids_bg), key=lambda x: x[0])
        mids_or = sorted(set(mids_or), key=lambda x: x[0])

        # Zusätzliche Mittelpunkte zwischen Ursprung und erstem blauen
        # sowie erstem gelben Kegel erzeugen. Dadurch existieren gerade
        # zu Beginn mehr valide Punkte zwischen links/blau und rechts/gelb.
        if cones['blue'] and cones['yellow']:
            # naheliegendste Kegel bestimmen (Distanz zum Ursprung)
            first_blue = min(cones['blue'], key=lambda p: np.linalg.norm(p[:2]))
            first_yellow = min(cones['yellow'], key=lambda p: np.linalg.norm(p[:2]))
            mid_first = (first_blue[:2] + first_yellow[:2]) / 2
            # mehrere Punkte auf der Strecke Ursprung -> Mittelpunkt einfügen
            steps = 4
            for i in range(1, steps + 1):
                frac = i / (steps + 1)
                extra = tuple(np.round(mid_first * frac, 4))
                if extra[1] > 0:
                    mids_bg.append(extra)

        mids_bg = sorted(set(mids_bg), key=lambda x: x[0])

        # Mittelpunkte markieren
        for idx,(mx,my) in enumerate(mids_bg):
            m = Marker(); m.header=msg.header; m.ns='midpoints_bg'
            m.id=10000+idx; m.type=Marker.SPHERE; m.action=Marker.ADD
            m.pose.position=Point(x=float(mx),y=float(my),z=0.0)
            m.scale.x=m.scale.y=m.scale.z=MIDPOINT_MARKER_SCALE
            r,g,b,a=MIDPOINT_COLOR; m.color=ColorRGBA(r=r,g=g,b=b,a=a)
            markers.markers.append(m)
        for idx,(mx,my) in enumerate(mids_or):
            m = Marker(); m.header=msg.header; m.ns='midpoints_or'
            m.id=20000+idx; m.type=Marker.SPHERE; m.action=Marker.ADD
            m.pose.position=Point(x=float(mx),y=float(my),z=0.0)
            m.scale.x=m.scale.y=m.scale.z=MIDPOINT_MARKER_SCALE
            m.color=ColorRGBA(r=1.0,g=0.6,b=0.3,a=1.0)
            markers.markers.append(m)

        # Mittelpunkte markieren
        for idx, (mx, my) in enumerate(mids_bg):
            m = Marker()
            m.header = msg.header
            m.ns     = 'midpoints_bg'
            m.id     = 10000 + idx
            m.type   = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position = Point(x=float(mx), y=float(my), z=0.0)
            m.scale.x = m.scale.y = m.scale.z = MIDPOINT_MARKER_SCALE
            r, g, b, a = MIDPOINT_COLOR
            m.color = ColorRGBA(r=r, g=g, b=b, a=a)
            markers.markers.append(m)
        for idx, (mx, my) in enumerate(mids_or):
            m = Marker()
            m.header = msg.header
            m.ns     = 'midpoints_or'
            m.id     = 20000 + idx
            m.type   = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position = Point(x=float(mx), y=float(my), z=0.0)
            m.scale.x = m.scale.y = m.scale.z = MIDPOINT_MARKER_SCALE
            m.color   = ColorRGBA(r=1.0, g=0.6, b=0.3, a=1.0)
            markers.markers.append(m)

        # --- 4) Pfadfindung (Greedy) mit Inertia & Extrapolation ---
        blue_pts   = np.array([p[:2] for p in cones['blue']])   if cones['blue']   else np.empty((0,2))
        yellow_pts = np.array([p[:2] for p in cones['yellow']]) if cones['yellow'] else np.empty((0,2))
        orange_pts = np.array([p[:2] for p in cones['orange']]) if cones['orange'] else np.empty((0,2))

        # Start-Vektor aus vorherigem Pfad (Inertia)
        if self.prev_bg and len(self.prev_bg) >= 2:
            vec = np.array(self.prev_bg[-1]) - np.array(self.prev_bg[-2])
            v0  = vec / np.linalg.norm(vec)
        else:
            v0 = np.array([0.0, 1.0])

        def check_side_bg(mid, prev, nxt):
            v = np.array(nxt) - np.array(prev)
            if np.linalg.norm(v) < 1e-3: return False
            v = v / np.linalg.norm(v)
            ln, rn = np.array([-v[1], v[0]]), np.array([v[1], -v[0]])
            def sides(pts):
                if pts.shape[0] == 0: return False, False
                d    = pts - mid
                dist = np.linalg.norm(d, axis=1)
                pl   = np.dot(d, ln); pr = np.dot(d, rn)
                return np.any((pl>0)&(dist<SIDE_CHECK_RADIUS)), np.any((pr>0)&(dist<SIDE_CHECK_RADIUS))
            lb, rb = sides(blue_pts)
            ly, ry = sides(yellow_pts)
            return (lb and ry or ly and rb) and not (lb and rb) and not (ly and ry)

        def check_side_or(mid, prev, nxt):
            v = np.array(nxt) - np.array(prev)
            if np.linalg.norm(v) < 1e-3: return False
            v = v / np.linalg.norm(v)
            ln, rn = np.array([-v[1], v[0]]), np.array([v[1], -v[0]])
            if orange_pts.shape[0] == 0: return False
            d    = orange_pts - mid
            dist = np.linalg.norm(d, axis=1)
            pl   = np.dot(d, ln); pr   = np.dot(d, rn)
            return np.any((pl>0)&(dist<SIDE_CHECK_RADIUS)) and np.any((pr>0)&(dist<SIDE_CHECK_RADIUS))

        def find_greedy_path(mids, check_fn, start_pt, last_vec, max_len, max_step=MAX_STEP_DIST):
            if not mids:
                # Fallback: ohne erkannte Mittelpunkte einen kurzen Pfad nach vorn erzeugen
                fallback_len = min(max_len, FALLBACK_DIST)
                fallback_pt = tuple((np.array(start_pt) + last_vec * fallback_len).tolist())
                return [tuple(start_pt), fallback_pt], fallback_len, last_vec
            arr  = np.array(mids)
            path = [tuple(start_pt)]
            used = set()
            last = np.array(start_pt)
            total = 0.0
            y_axis = np.array([0.0,1.0])

            while total < max_len:
                dists = np.linalg.norm(arr - last, axis=1)
                cands = [
                    (i, mids[i], dists[i]) for i in range(len(mids))
                    if (i not in used
                        and dists[i] > 0.1
                        and dists[i] <= max_step
                        and total + dists[i] <= max_len
                        and mids[i][1] > 0)
                ]
                if len(path) == 1:
                    cands = [
                        (i, p, d) for (i, p, d) in cands
                        if abs(np.degrees(np.arccos(
                            np.clip(
                                np.dot((np.array(p)-last)/np.linalg.norm(np.array(p)-last), y_axis),
                                -1, 1
                            )
                        ))) <= FIRST_STEP_MAX_ANGLE
                    ]
                if not cands:
                    break

                # Greedy-Auswahl: minimaler Abstand
                best, best_d = None, None
                for i, p, d in cands:
                    v = np.array(p) - last
                    n = np.linalg.norm(v)
                    if n < 1e-3:
                        continue
                    dirv = v / n
                    if len(path) > 1:
                        ang = np.degrees(np.arccos(np.clip(np.dot(last_vec, dirv), -1, 1)))
                        if ang > MAX_ANGLE:
                            continue
                    if not check_fn(p, last, p):
                        continue
                    if best is None or d < best_d:
                        best, best_d = (i, p, dirv, d), d

                if best is None:
                    break

                # Korrektes Entpacken des besten Kandidaten
                idx, pt, dirv, dist = best
                path.append(pt)
                used.add(idx)
                last = np.array(pt)
                last_vec = dirv
                total += dist

            return path, total, last_vec

        best_bg, best_or = [], []
        max_pts, best_len, abort = 0, 0.0, ""

        # nur eine Konfiguration (GEGENCHECK = 1)
        for _ in range(GEGENCHECK):
            p_bg, l_bg, v1 = find_greedy_path(mids_bg, check_side_bg, (0,0), v0, PATH_LENGTH)
            l_or_max = PATH_LENGTH - l_bg
            p_or, l_or = [], 0.0
            if l_or_max > 0 and len(p_bg) > 1:
                p_or, l_or, _ = find_greedy_path(mids_or, check_side_or, p_bg[-1], v1, l_or_max)

            pts_count = len(p_bg) + len(p_or)
            total_len = l_bg + l_or

            # Auswahl: zuerst mehr Punkte, dann längere Strecke
            if (pts_count > max_pts) or (pts_count == max_pts and total_len > best_len):
                max_pts  = pts_count
                best_len = total_len
                best_bg  = p_bg
                best_or  = p_or
                abort = "" if best_len >= PATH_LENGTH else f"Nur {best_len:.2f}m erreicht ({max_pts} Punkte)."

        # Extrapolation, damit immer genau PATH_LENGTH erreicht wird
        combined = best_bg + best_or
        if len(combined) >= 2 and best_len < PATH_LENGTH:
            prev_pt = np.array(combined[-2])
            last_pt = np.array(combined[-1])
            dir_vec = last_pt - prev_pt
            dir_vec /= np.linalg.norm(dir_vec)
            missing = PATH_LENGTH - best_len
            ext_pt  = tuple((last_pt + dir_vec * missing).tolist())
            if best_or:
                best_or.append(ext_pt)
            else:
                best_bg.append(ext_pt)

        # 5) Pfad glätten
        best_bg = smooth_path(best_bg)
        best_or = smooth_path(best_or)

        # 6) Winkel berechnen, filtern und publizieren
        if len(best_bg) >= 2:
            v = np.array(best_bg[1])
            raw_angle = float(np.degrees(np.arctan2(v[0], v[1])))

            # Ausreißerprüfung
            if self._angle_buffer and abs(raw_angle - self._angle_buffer[-1]) > ANGLE_JUMP_THRESH:
                self.get_logger().warn(
                    f"Ausreißer erkannt: Δ{(raw_angle - self._angle_buffer[-1]):.1f}° > {ANGLE_JUMP_THRESH}°"
                )
            else:
                # Puffer aktualisieren
                self._angle_buffer.append(raw_angle)
                if len(self._angle_buffer) > ANGLE_WINDOW:
                    self._angle_buffer.pop(0)

            # Median-Filter
            filtered = float(np.median(self._angle_buffer)) if self._angle_buffer else raw_angle

            # EMA-Glättung
            if self._angle_smoothed is None:
                self._angle_smoothed = filtered
            else:
                self._angle_smoothed = (
                    ANGLE_SMOOTH_ALPHA * self._angle_smoothed +
                    (1 - ANGLE_SMOOTH_ALPHA) * filtered
                )

            # Publizieren
            angle_msg = Float32(data=self._angle_smoothed)
            self.angle_pub.publish(angle_msg)
            if self.angle_shared_pub.get_subscription_count() > 0:
                self.angle_shared_pub.publish(angle_msg)
            self.get_logger().info(f"Gefilterter Winkel: {self._angle_smoothed:.2f}°")

        # Pfad in MarkerArray
        path_markers = MarkerArray()
        clr = Marker(); clr.action = Marker.DELETEALL
        clr.header = msg.header; clr.ns='best_path'; clr.id=40000
        path_markers.markers.append(clr)

        combined = best_bg + best_or
        if len(combined) >= 2:
            pts = [Point(x=float(x), y=float(y), z=0.0) for x, y in combined]
            n_bg = len(best_bg)
            m = Marker()
            m.header = msg.header
            m.ns     = 'best_path'
            m.id     = 30000
            m.type   = Marker.LINE_STRIP
            m.action = Marker.ADD
            m.scale.x = 0.08
            m.points  = pts
            m.colors  = []
            for i in range(len(pts)):
                if i < n_bg:
                    m.colors.append(ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0))
                else:
                    m.colors.append(ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0))
            path_markers.markers.append(m)

            # Track also as simple image
            pts_np = np.array(combined)
            min_x, max_x = pts_np[:,0].min(), pts_np[:,0].max()
            min_y, max_y = pts_np[:,1].min(), pts_np[:,1].max()
            span = max(max_x - min_x, max_y - min_y, 1e-3)
            scale = 360.0 / span
            offset_x = (400 - (max_x - min_x) * scale) / 2 - min_x * scale
            offset_y = (400 - (max_y - min_y) * scale) / 2 - min_y * scale
            track_img = np.ones((400, 400, 3), dtype=np.uint8) * 255
            for a, b in zip(pts_np[:-1], pts_np[1:]):
                ax = int(a[0] * scale + offset_x)
                ay = int(400 - (a[1] * scale + offset_y))
                bx = int(b[0] * scale + offset_x)
                by = int(400 - (b[1] * scale + offset_y))
                cv2.line(track_img, (ax, ay), (bx, by), (0, 0, 0), 1)
            for x, y in pts_np:
                xi = int(x * scale + offset_x)
                yi = int(400 - (y * scale + offset_y))
                cv2.circle(track_img, (xi, yi), 3, (0, 0, 255), -1)
            track_msg = self.bridge.cv2_to_imgmsg(track_img, 'bgr8')
            track_msg.header.stamp = msg.header.stamp
            self.track_image_pub.publish(track_msg)

        # Geschwindigkeit anhand Pfadlänge und Lenkwinkel berechnen
        if len(combined) >= 2:
            path_len = sum(
                np.linalg.norm(np.array(b) - np.array(a))
                for a, b in zip(combined[:-1], combined[1:])
            )
        else:
            path_len = 0.0

        angle_val = float(self._angle_smoothed) if self._angle_smoothed is not None else 0.0
        length_factor = min(1.0, path_len / PATH_LENGTH)
        angle_factor = max(0.0, 1.0 - abs(angle_val) / 90.0)
        speed_est = self.max_speed * length_factor * angle_factor
        speed = speed_est
        if self.desired_speed is not None:
            speed = min(self.desired_speed, self.max_speed)
        if self.speed is not None:
            speed = min(speed, self.speed)

        angle_img = np.ones((30, 180, 3), dtype=np.uint8) * 255
        cv2.putText(angle_img, f"{angle_val:.1f} Grad", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)
        angle_msg = self.bridge.cv2_to_imgmsg(angle_img, 'bgr8')
        angle_msg.header.stamp = msg.header.stamp
        self.angle_image_pub.publish(angle_msg)

        speed_img = np.ones((30, 180, 3), dtype=np.uint8) * 255
        cv2.putText(speed_img, f"{speed:.2f} m/s", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)
        speed_msg = self.bridge.cv2_to_imgmsg(speed_img, 'bgr8')
        speed_msg.header.stamp = msg.header.stamp
        self.speed_image_pub.publish(speed_msg)
        if self.speed_cmd_pub.get_subscription_count() > 0:
            self._ignore_next_speed_msg = True
            self.speed_cmd_pub.publish(Float32(data=float(speed)))

        # 8) FPS berechnen & publizieren
        dt = time.time() - t0
        self._frame_times.append(dt)
        if len(self._frame_times) > 10:
            self._frame_times.pop(0)
        avg = sum(self._frame_times) / len(self._frame_times)
        self.fps_pub.publish(Float32(data=(1.0/avg if avg>0 else 0.0)))

        # 9) Warnung bei Kürze
        if abort:
            self.get_logger().warn(f"Pfad < {PATH_LENGTH}m! Grund: {abort}")

        # abschließend Marker-Arrays senden (nicht entfernen)
        self.marker_pub.publish(markers)
        self.path_pub.publish(path_markers)


def main(args=None):
    rclpy.init(args=args)
    node = PathNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()