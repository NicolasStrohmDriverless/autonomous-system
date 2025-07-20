#!/usr/bin/env python3
import math
import random
import time
import os

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
from oak_cone_detect_interfaces.msg import ConeArray3D, PathPrediction

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
    'orange': [1.0, 0.5, 0.0, 1.0]
}
MIDPOINT_COLOR       = [0.5, 0.5, 0.5, 1.0]
PATH_MIDPOINT_COLOR = [0.0, 1.0, 0.0, 1.0]
SIDE_CHECK_RADIUS = 2.0
MAX_STEP_DIST     = 2.0
GEGENCHECK        = 1
SMOOTH_ALPHA      = 0.4   # Faktor für exponentielle Glättung des Pfades

# Neuer Winkel-Filter: größere Fenster, höhere Toleranz und EMA-Glättung
ANGLE_WINDOW        = 7     # Anzahl der letzten Winkel für Median
ANGLE_JUMP_THRESH   = 30.0  # Maximal erlaubter Sprung in °
ANGLE_SMOOTH_ALPHA  = 0.3   # Faktor für exponentielle Glättung des Winkels

# Zusätzliche Parameter für Geschwindigkeits- und Lenkwinkelanzeige
SPEED_PATH_LENGTH   = 30.0
ANGLE_SPEED_DIVISOR = 45.0
MAX_SPEED           = 5.0
PREDICTION_INTERVAL = 0.5
MAX_STEERING_ANGLE  = 30.0
STEERING_RATIO      = 15.0

def _load_wheel_image() -> tuple[str, 'np.ndarray']:
    path = os.path.join(os.path.dirname(__file__), "..", "resource", "f1_wheel.jpg")
    if not os.path.isfile(path):
        try:
            from ament_index_python.packages import get_package_share_directory

            share = get_package_share_directory("pathfinding")
            alt = os.path.join(share, "resource", "f1_wheel.jpg")
            if os.path.isfile(alt):
                path = alt
        except Exception:
            pass
    return path, cv2.imread(path)


WHEEL_IMAGE_PATH, _WHEEL_IMAGE = _load_wheel_image()


def predict_speed_angle(path, max_speed, step=PREDICTION_INTERVAL):
    """Return lists of speeds and steering angles along ``path``."""
    if len(path) < 2:
        return [], []

    cum = [0.0]
    for a, b in zip(path[:-1], path[1:]):
        cum.append(cum[-1] + float(np.linalg.norm(np.array(b) - np.array(a))))

    total = cum[-1]
    speeds = []
    angles = []
    idx = 0
    dist = 0.0
    while dist <= total and idx < len(path) - 1:
        while idx < len(cum) - 1 and cum[idx + 1] < dist:
            idx += 1
        if idx >= len(path) - 1:
            break
        p0 = np.array(path[idx])
        p1 = np.array(path[idx + 1])
        vec = p1 - p0
        if np.linalg.norm(vec) > 1e-6:
            angle = float(np.degrees(np.arctan2(vec[0], vec[1])))
        else:
            angle = 0.0
        remain = total - dist
        speed = max_speed * (
            (1 - abs(angle) / ANGLE_SPEED_DIVISOR + remain / SPEED_PATH_LENGTH) / 2.0
        )
        speed = min(max(speed, 0.0), max_speed)
        speeds.append(float(speed))
        angles.append(float(angle))
        dist += step

    return speeds, angles


def draw_speed_gauge(
    speed: float, max_speed: float, width: int = 180, height: int = 90
) -> np.ndarray:
    """Return an image visualizing ``speed`` as a semicircular gauge."""
    img = np.ones((height, width, 3), dtype=np.uint8) * 255
    center = (width // 2, height - 10)
    radius = min(width // 2 - 10, height - 20)
    cv2.ellipse(img, center, (radius, radius), 0, 180, 360, (0, 0, 0), 2)
    for i in range(11):
        tick_angle = 180 + (i / 10.0) * 180
        a = math.radians(tick_angle)
        x1 = int(center[0] + (radius - 5) * math.cos(a))
        y1 = int(center[1] + (radius - 5) * math.sin(a))
        x2 = int(center[0] + radius * math.cos(a))
        y2 = int(center[1] + radius * math.sin(a))
        cv2.line(img, (x1, y1), (x2, y2), (0, 0, 0), 2)
    frac = 0.0 if max_speed <= 0 else max(0.0, min(1.0, speed / max_speed))
    needle_angle = 180 + frac * 180
    a = math.radians(needle_angle)
    x = int(center[0] + (radius - 10) * math.cos(a))
    y = int(center[1] + (radius - 10) * math.sin(a))
    cv2.line(img, center, (x, y), (0, 0, 255), 2)
    text = f"{speed:.2f} m/s"
    text_size, _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
    text_pos = (center[0] - text_size[0] // 2, height - 5)
    cv2.putText(img, text, text_pos, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
    return img


def draw_steering_wheel(
    angle: float, max_angle: float, ratio: float, size: int = 180
) -> np.ndarray:
    """Return an image visualizing ``angle`` as a steering wheel."""
    angle = max(-max_angle, min(max_angle, angle))
    if _WHEEL_IMAGE is not None:
        img = cv2.resize(_WHEEL_IMAGE, (size, size))
        rot_deg = angle * ratio
        center = (size // 2, size // 2)
        M = cv2.getRotationMatrix2D(center, -rot_deg, 1.0)
        img = cv2.warpAffine(
            img,
            M,
            (size, size),
            flags=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=(255, 255, 255),
        )
    else:
        img = np.ones((size, size, 3), dtype=np.uint8) * 255
    text = f"{angle:.1f} Grad"
    text_size, _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
    text_pos = (size // 2 - text_size[0] // 2, size - 5)
    cv2.putText(img, text, text_pos, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
    return img

def smooth_path(path, alpha=SMOOTH_ALPHA):
    """Exponentielle Glättung eines Pfades."""
    if not path:
        return []
    smoothed = [np.array(path[0])]
    for p in path[1:]:
        smoothed.append(alpha * np.array(p) + (1 - alpha) * smoothed[-1])
    return [tuple(p) for p in smoothed]


class PathNode(Node):
    def __init__(self, start_offset: float = 0.0):
        super().__init__("midpoint_path_node")
        self.start_offset = float(start_offset)

        # Subscriber mit Best-Effort QoS
        self.sub = self.create_subscription(
            ConeArray3D, "/cone_detections_3d", self.callback, qos_profile=qos_sub
        )

        # Publisher für Kegel-, Pfad- und Winkel-Marker
        self.marker_pub = self.create_publisher(MarkerArray, "/cone_markers", 10)
        self.path_pub = self.create_publisher(MarkerArray, "/best_path_marker", 10)
        self.fps_pub = self.create_publisher(Float32, "/path_inference_fps", 10)
        self.angle_pub = self.create_publisher(Float32, "/path_to_y_axis_angle", 10)
        self.angle_image_pub = self.create_publisher(Image, "/path_status/angle_image", 1)
        self.speed_image_pub = self.create_publisher(Image, "/path_status/speed_image", 1)
        self.prediction_pub = self.create_publisher(PathPrediction, "/prediction", 10)
        self.track_image_pub = self.create_publisher(Image, "/path_status/track_image", 1)
        self.track_global_pub = self.create_publisher(Image, "/track/image", 1)

        self.declare_parameter("max_speed", MAX_SPEED)
        self.max_speed = float(self.get_parameter("max_speed").value)

        self.speed = None
        self.create_subscription(Float32, "/vehicle/actual_speed", self.speed_callback, 10)

        self.actual_steering = 0.0
        self.create_subscription(Float32, "/vehicle/actual_steering", self.actual_steering_cb, 10)

        self.speed_cmd_pub = self.create_publisher(Float32, "/vehicle/desired_speed", 10)
        self.speed_cmd_sub = self.create_subscription(Float32, "/vehicle/desired_speed", self.speed_cmd_callback, 10)
        self.angle_shared_pub = self.create_publisher(Float32, "/vehicle/steering_angle", 10)
        self.angle_shared_sub = self.create_subscription(Float32, "/vehicle/steering_angle", self.angle_shared_callback, 10)
        self.desired_speed = None
        self.shared_angle = None
        self._ignore_next_speed_msg = False

        self.bridge = CvBridge()
        self.get_logger().info('PathNode started')

        # interne Puffer
        self._frame_times = []
        self.prev_bg = None
        self.prev_or = None
        self._angle_buffer = []
        self._angle_smoothed = None
        self.current_bg = []
        self.current_or = []
        self.current_len = 0.0
        self.green_len = 0.0
        self.dist_since_update = 0.0
        self.last_time = time.time()
        self.last_speed = 0.0
        self.last_angle = 0.0
        self.stop_braked = False
        self.prev_cones = {}
        self.path_id_counter = 0
        self.current_path_id = 0
        self.longest_bg = []
        self.longest_or = []
        self.longest_len = 0.0
        self.midpoint_best_path = []

    def speed_callback(self, msg: Float32):
        self.speed = float(msg.data)

    def speed_cmd_callback(self, msg: Float32):
        if self._ignore_next_speed_msg:
            self._ignore_next_speed_msg = False
            return
        self.desired_speed = float(msg.data)

    def angle_shared_callback(self, msg: Float32):
        self.shared_angle = float(msg.data)

    def actual_steering_cb(self, msg: Float32):
        self.actual_steering = float(msg.data)

    def callback(self, msg: ConeArray3D):
        t0 = time.time()

        # 1) Kegel sammeln
        cones = {col: [] for col in COLOR_MAP}
        for c in msg.cones:
            if c.y < 0:
                continue
            if c.color not in cones:
                continue
            p = np.array([c.x, c.y, c.z]) * np.array(CONE_POSITION_SCALE)
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

        mids_bg = sorted(set(mids_bg), key=lambda x: x[0])
        mids_or = sorted(set(mids_or), key=lambda x: x[0])

        # wähle mindestens die 5 nächstgelegenen Mittelpunkte zum Koordinatenursprung
        all_mids = [(m, 'bg') for m in mids_bg] + [(m, 'or') for m in mids_or]
        all_mids.sort(key=lambda mc: math.hypot(mc[0][0], mc[0][1]))
        filtered = [
            mc for mc in all_mids
            if (
                math.hypot(mc[0][0], mc[0][1]) <= PATH_LENGTH and
                abs(math.degrees(math.atan2(mc[0][0], mc[0][1]))) <= MAX_ANGLE
            )
        ]
        if len(filtered) < 5:
            filtered = all_mids[:5]
        mids_bg = [m for m, c in filtered if c == 'bg']
        mids_or = [m for m, c in filtered if c == 'or']

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
                return [], 0.0, last_vec
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
            p_bg, l_bg, v1 = find_greedy_path(mids_bg, check_side_bg, (0, self.start_offset), v0, PATH_LENGTH)
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

        used_mids = set(best_bg + best_or)

        # 5) Pfad glätten
        best_bg = smooth_path(best_bg)
        best_or = smooth_path(best_or)

        # Mittelpunkte markieren (grün, wenn Teil des Pfades)
        for idx, (mx, my) in enumerate(mids_bg):
            m = Marker()
            m.header = msg.header
            m.ns = 'midpoints_bg'
            m.id = 10000 + idx
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position = Point(x=float(mx), y=float(my), z=0.0)
            m.scale.x = m.scale.y = m.scale.z = MIDPOINT_MARKER_SCALE
            color = PATH_MIDPOINT_COLOR if (mx, my) in used_mids else MIDPOINT_COLOR
            r, g, b, a = color
            m.color = ColorRGBA(r=r, g=g, b=b, a=a)
            markers.markers.append(m)
        for idx, (mx, my) in enumerate(mids_or):
            m = Marker()
            m.header = msg.header
            m.ns = 'midpoints_or'
            m.id = 20000 + idx
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position = Point(x=float(mx), y=float(my), z=0.0)
            m.scale.x = m.scale.y = m.scale.z = MIDPOINT_MARKER_SCALE
            color = PATH_MIDPOINT_COLOR if (mx, my) in used_mids else MIDPOINT_COLOR
            r, g, b, a = color
            m.color = ColorRGBA(r=r, g=g, b=b, a=a)
            markers.markers.append(m)

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

        # Publisher
        self.marker_pub.publish(markers)
        self.path_pub.publish(path_markers)

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

        # abschließend Marker-Arrays senden
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