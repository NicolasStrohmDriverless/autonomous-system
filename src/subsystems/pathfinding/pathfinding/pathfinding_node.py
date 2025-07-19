#!/usr/bin/env python3
import math
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
from oak_cone_detect_interfaces.msg import ConeArray3D

# QoS nur für den Subscriber: Best Effort, volatile, depth=1
qos_sub = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
    durability=QoSDurabilityPolicy.VOLATILE,
)

# Pfad- und Marker-Parameter
PATH_LENGTH = 20.0  # nun 20 m
SPEED_PATH_LENGTH = 30.0  # Pfadlänge für Geschwindigkeitsberechnung
ANGLE_SPEED_DIVISOR = 45.0  # Winkel-Referenz für Geschwindigkeitsberechnung
MAX_ANGLE = 90.0
FIRST_STEP_MAX_ANGLE = 50.0
MIDPOINT_MARKER_SCALE = 0.1
DEFAULT_CONE_SCALE = (0.228, 0.228, 0.325)
LARGE_ORANGE_CONE_SCALE = (0.285, 0.285, 0.505)
CONE_POSITION_SCALE = (1.0, 1.0, 1.0)
MAX_MARKER_Y = 30.0
COLOR_MAP = {
    "blue": [0.0, 0.0, 1.0, 1.0],
    "yellow": [1.0, 1.0, 0.0, 1.0],
    "orange": [1.0, 0.5, 0.0, 1.0],
    "red": [1.0, 0.0, 0.0, 1.0],
}
MIDPOINT_COLOR = [0.5, 0.5, 0.5, 1.0]
SIDE_CHECK_RADIUS = 2.0
MAX_STEP_DIST = 2.0
GEGENCHECK = 1
FALLBACK_DIST = 1.0  # Mindestpfadlänge, falls keine Punkte gefunden werden
SMOOTH_ALPHA = 0.4  # Faktor für exponentielle Glättung des Pfades

# Neuer Winkel-Filter: größere Fenster, höhere Toleranz und EMA-Glättung
ANGLE_WINDOW = 7  # Anzahl der letzten Winkel für Median
ANGLE_JUMP_THRESH = 30.0  # Maximal erlaubter Sprung in °
ANGLE_SMOOTH_ALPHA = 0.3  # Faktor für exponentielle Glättung des Winkels
MAX_SPEED = 5.0  # Maximale Geschwindigkeit in m/s

# Lenkrad-Anzeige
MAX_STEERING_ANGLE = 30.0  # maximale Lenkwinkelanzeige in Grad
STEERING_RATIO = 15.0  # Übersetzung Lenkrad zu Rad
# Lenkwinkelgeschwindigkeit ~ θ_dot = R * v × i

# Use a custom steering wheel image if available
WHEEL_IMAGE_PATH = os.path.join(
    os.path.dirname(__file__), "..", "resource", "f1_wheel.jpg"
)
_WHEEL_IMAGE = cv2.imread(WHEEL_IMAGE_PATH)
# ``_WHEEL_IMAGE`` is ``None`` if the file does not exist.


def smooth_path(path, alpha=SMOOTH_ALPHA):
    """Exponentielle Glättung eines Pfades."""
    if not path:
        return []
    smoothed = [np.array(path[0])]
    for p in path[1:]:
        smoothed.append(alpha * np.array(p) + (1 - alpha) * smoothed[-1])
    return [tuple(p) for p in smoothed]


def transform_path(path, prev_angle, curr_angle, distance):
    """Rotate and translate a path according to vehicle movement."""
    if not path:
        return []

    delta = math.radians(curr_angle - prev_angle)
    cos_a, sin_a = math.cos(delta), math.sin(delta)
    dir_prev = np.array(
        [
            math.sin(math.radians(prev_angle)),
            math.cos(math.radians(prev_angle)),
        ]
    )
    translation = -distance * dir_prev

    transformed = []
    for x, y in path:
        xr = cos_a * x - sin_a * y
        yr = sin_a * x + cos_a * y
        transformed.append((float(xr + translation[0]), float(yr + translation[1])))
    return transformed


def crop_path(path, colors, distance):
    """Remove the traveled distance from the start of the path."""
    pts = list(path)
    cols = list(colors)
    d = float(distance)
    while d > 0 and len(pts) >= 2:
        p0 = np.array(pts[0])
        p1 = np.array(pts[1])
        seg_len = float(np.linalg.norm(p1 - p0))
        if seg_len < 1e-6:
            pts.pop(0)
            cols.pop(0)
            continue
        if d >= seg_len:
            pts.pop(0)
            cols.pop(0)
            d -= seg_len
        else:
            new_p0 = p0 + (p1 - p0) * (d / seg_len)
            pts[0] = tuple(new_p0.tolist())
            d = 0.0
    return pts, cols


def path_length(pts):
    if len(pts) < 2:
        return 0.0
    return sum(
        np.linalg.norm(np.array(b) - np.array(a)) for a, b in zip(pts[:-1], pts[1:])
    )


def draw_speed_gauge(
    speed: float, max_speed: float, width: int = 180, height: int = 90
) -> np.ndarray:
    """Return an image visualizing ``speed`` as a semicircular gauge.

    The design resembles the tachometer of a car.
    """
    img = np.ones((height, width, 3), dtype=np.uint8) * 255

    center = (width // 2, height - 10)
    radius = min(width // 2 - 10, height - 20)

    # draw outer semicircle
    cv2.ellipse(img, center, (radius, radius), 0, 180, 360, (0, 0, 0), 2)

    # draw tick marks
    for i in range(11):
        tick_angle = 180 + (i / 10.0) * 180
        a = math.radians(tick_angle)
        x1 = int(center[0] + (radius - 5) * math.cos(a))
        y1 = int(center[1] + (radius - 5) * math.sin(a))
        x2 = int(center[0] + radius * math.cos(a))
        y2 = int(center[1] + radius * math.sin(a))
        cv2.line(img, (x1, y1), (x2, y2), (0, 0, 0), 2)

    frac = 0.0 if max_speed <= 0 else max(0.0, min(1.0, speed / max_speed))

    # needle position
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


def draw_angle_gauge(
    angle: float, max_angle: float, width: int = 180, height: int = 90
) -> np.ndarray:
    """Return an image visualizing ``angle`` as a semicircular gauge.

    The design resembles the tachometer of a car and replaces the degree
    symbol with the word "Grad".
    """
    img = np.ones((height, width, 3), dtype=np.uint8) * 255

    center = (width // 2, height - 10)
    radius = min(width // 2 - 10, height - 20)

    # draw outer semicircle
    cv2.ellipse(img, center, (radius, radius), 0, 180, 360, (0, 0, 0), 2)

    # draw tick marks
    for i in range(-5, 6):
        tick_angle = 270 + (i / 5.0) * 90
        a = math.radians(tick_angle)
        x1 = int(center[0] + (radius - 5) * math.cos(a))
        y1 = int(center[1] + (radius - 5) * math.sin(a))
        x2 = int(center[0] + radius * math.cos(a))
        y2 = int(center[1] + radius * math.sin(a))
        cv2.line(img, (x1, y1), (x2, y2), (0, 0, 0), 2)

    if max_angle > 0:
        frac = max(-1.0, min(1.0, angle / max_angle))
    else:
        frac = 0.0

    # needle position
    needle_angle = 270 + frac * 90
    a = math.radians(needle_angle)
    x = int(center[0] + (radius - 10) * math.cos(a))
    y = int(center[1] + (radius - 10) * math.sin(a))
    cv2.line(img, center, (x, y), (0, 0, 255), 2)

    text = f"{angle:.1f} Grad"
    text_size, _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
    text_pos = (center[0] - text_size[0] // 2, height - 5)
    cv2.putText(img, text, text_pos, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
    return img


def draw_steering_wheel(
    angle: float, max_angle: float, ratio: float, size: int = 180
) -> np.ndarray:
    """Return an image visualizing ``angle`` as a steering wheel.

    If the image at :data:`WHEEL_IMAGE_PATH` is available it will be rotated
    according to ``angle`` and used as the wheel graphic. Otherwise a blank
    image is returned. ``ratio`` specifies the conversion from wheel angle to
    steering wheel rotation.
    """

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
        self.angle_image_pub = self.create_publisher(
            Image, "/path_status/angle_image", 1
        )
        self.speed_image_pub = self.create_publisher(
            Image, "/path_status/speed_image", 1
        )
        # Publishes the track image for status monitoring and for generic UI
        # components. Keep the original topic for backwards compatibility.
        self.track_image_pub = self.create_publisher(
            Image,
            "/path_status/track_image",
            1,
        )
        # Additional topic with a generic name for convenience
        self.track_global_pub = self.create_publisher(Image, "/track/image", 1)

        self.declare_parameter("max_speed", MAX_SPEED)
        self.max_speed = float(self.get_parameter("max_speed").value)

        # Geschwindigkeit des Fahrzeugs
        self.speed = None
        self.create_subscription(
            Float32, "/vehicle/actual_speed", self.speed_callback, 10
        )

        # Geteilter Status mit anderen Nodes
        self.speed_cmd_pub = self.create_publisher(
            Float32, "/vehicle/desired_speed", 10
        )
        self.speed_cmd_sub = self.create_subscription(
            Float32, "/vehicle/desired_speed", self.speed_cmd_callback, 10
        )
        self.angle_shared_pub = self.create_publisher(
            Float32, "/vehicle/steering_angle", 10
        )
        self.angle_shared_sub = self.create_subscription(
            Float32, "/vehicle/steering_angle", self.angle_shared_callback, 10
        )
        self.desired_speed = None
        self.shared_angle = None
        # used to ignore speed command messages originating from this node
        self._ignore_next_speed_msg = False

        self.bridge = CvBridge()

        # Confirm initialization (logger disabled)

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
        # longest path tracking
        self.longest_bg = []
        self.longest_or = []
        self.longest_len = 0.0

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

    def _update_longest_path(
        self, prev_angle, curr_angle, move_dist, avg_dx, avg_dy, matches
    ):
        """Transform and crop the stored longest path."""
        self.longest_bg = transform_path(
            self.longest_bg, prev_angle, curr_angle, move_dist
        )
        self.longest_or = transform_path(
            self.longest_or, prev_angle, curr_angle, move_dist
        )
        if matches > 0:
            self.longest_bg = [(x + avg_dx, y + avg_dy) for x, y in self.longest_bg]
            self.longest_or = [(x + avg_dx, y + avg_dy) for x, y in self.longest_or]
        colors = ["bg"] * len(self.longest_bg) + ["or"] * len(self.longest_or)
        combined, colors = crop_path(
            self.longest_bg + self.longest_or, colors, move_dist
        )
        self.longest_bg = [p for p, c in zip(combined, colors) if c == "bg"]
        self.longest_or = [p for p, c in zip(combined, colors) if c == "or"]
        self.longest_len = path_length(combined)

    def callback(self, msg: ConeArray3D):
        t0 = time.time()
        now = t0
        dt = now - self.last_time
        self.last_time = now
        move_dist = abs(self.last_speed) * dt
        prev_angle = self.last_angle
        self.dist_since_update += move_dist

        # 1) Kegel sammeln
        cones = {col: [] for col in COLOR_MAP}
        for c in msg.cones:
            if c.z < 0 or c.z > MAX_MARKER_Y:
                continue
            if c.color not in cones:
                # ignore unknown colors
                continue
            p = np.array([c.x, c.z, 0]) * np.array(CONE_POSITION_SCALE)
            cones[c.color].append(p)

        cone_positions = {c.id: np.array([c.x, c.z]) for c in msg.cones}
        dx_sum = dy_sum = 0.0
        matches = 0
        for cid, pos in cone_positions.items():
            if cid in self.prev_cones:
                prev = self.prev_cones[cid]
                dx_sum += pos[0] - prev[0]
                dy_sum += pos[1] - prev[1]
                matches += 1
        avg_dx = dx_sum / matches if matches > 0 else 0.0
        avg_dy = dy_sum / matches if matches > 0 else 0.0

        # 2) Kegel-Marker
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        for col, pts in cones.items():
            for idx, p in enumerate(pts):
                m = Marker()
                m.header = msg.header
                m.ns = f"cone_{col}"
                m.id = hash(col) % 1000 + idx
                m.type = Marker.CYLINDER
                m.action = Marker.ADD
                m.pose.position = Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))
                scale = (
                    LARGE_ORANGE_CONE_SCALE if col == "orange" else DEFAULT_CONE_SCALE
                )
                m.scale.x, m.scale.y, m.scale.z = scale
                r, g, b, a = COLOR_MAP[col]
                m.color = ColorRGBA(r=r, g=g, b=b, a=a)
                markers.markers.append(m)

        # Verbindungslinien zwischen Kegeln nicht mehr zeichnen
        # Ursprünglich wurden hier orange, blaue und gelbe Kegel
        # mittels LINE_STRIP-Markern verbunden. Diese Marker wurden
        # entfernt, sodass nur noch der Hauptpfad dargestellt wird.

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
                        a, b = s[i], s[(i + 1) % 3]
                        cs = {cols2d[a], cols2d[b]}
                        mid = tuple(
                            ((np.array(pts2d[a]) + np.array(pts2d[b])) / 2).round(4)
                        )
                        if mid[1] <= 0 or mid[1] > MAX_MARKER_Y:
                            continue
                        vec = np.array(pts2d[b]) - np.array(pts2d[a])
                        ang = abs(np.degrees(np.arctan2(vec[0], vec[1])))
                        if ang > MAX_ANGLE:
                            continue
                        if cs == {"blue", "yellow"}:
                            mids_bg.append(mid)
                        if cs == {"orange"}:
                            mids_or.append(mid)
            except:
                pass

        # Erstes Sortieren, bevor Zusatzpunkte ergänzt werden
        mids_bg = [m for m in mids_bg if 0.0 < m[1] < MAX_MARKER_Y]
        mids_bg = sorted(set(mids_bg), key=lambda x: x[0])
        mids_or = [m for m in mids_or if 0.0 < m[1] < MAX_MARKER_Y]
        mids_or = sorted(set(mids_or), key=lambda x: x[0])

        start_pt = (0.0, self.start_offset)
        # Zusätzliche Mittelpunkte zwischen Ursprung und erstem blauen
        # sowie erstem gelben Kegel erzeugen. Der Startpunkt kann einen
        # Y-Versatz besitzen, es werden lediglich Zwischenpunkte eingefügt,
        # damit zu Beginn mehr valide Punkte existieren.
        if cones["blue"] and cones["yellow"]:
            # naheliegendste Kegel bestimmen (Distanz zum Ursprung)
            first_blue = min(cones["blue"], key=lambda p: np.linalg.norm(p[:2]))
            first_yellow = min(cones["yellow"], key=lambda p: np.linalg.norm(p[:2]))
            mid_first = (first_blue[:2] + first_yellow[:2]) / 2
            # mehrere Punkte auf der Strecke Ursprung -> Mittelpunkt einfügen
            steps = 4
            for i in range(1, steps + 1):
                frac = i / (steps + 1)
                extra = tuple(np.round(mid_first * frac, 4))
                if extra[1] > 0:
                    mids_bg.append(extra)

        mids_bg = [m for m in mids_bg if 0.0 < m[1] < MAX_MARKER_Y]
        mids_bg = sorted(set(mids_bg), key=lambda x: x[0])
        mids_or = [m for m in mids_or if 0.0 < m[1] < MAX_MARKER_Y]
        mids_or = sorted(set(mids_or), key=lambda x: x[0])

        # Mittelpunkte markieren
        for idx, (mx, my) in enumerate(mids_bg):
            if not (0.0 < my < MAX_MARKER_Y):
                continue
            m = Marker()
            m.header = msg.header
            m.ns = "midpoints_bg"
            m.id = 10000 + idx
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position = Point(x=float(mx), y=float(my), z=0.0)
            m.scale.x = m.scale.y = m.scale.z = MIDPOINT_MARKER_SCALE
            r, g, b, a = MIDPOINT_COLOR
            m.color = ColorRGBA(r=r, g=g, b=b, a=a)
            markers.markers.append(m)
        for idx, (mx, my) in enumerate(mids_or):
            if not (0.0 < my < MAX_MARKER_Y):
                continue
            m = Marker()
            m.header = msg.header
            m.ns = "midpoints_or"
            m.id = 20000 + idx
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position = Point(x=float(mx), y=float(my), z=0.0)
            m.scale.x = m.scale.y = m.scale.z = MIDPOINT_MARKER_SCALE
            m.color = ColorRGBA(r=1.0, g=0.6, b=0.3, a=1.0)
            markers.markers.append(m)

        # Mittelpunkte markieren
        for idx, (mx, my) in enumerate(mids_bg):
            if not (0.0 < my < MAX_MARKER_Y):
                continue
            m = Marker()
            m.header = msg.header
            m.ns = "midpoints_bg"
            m.id = 10000 + idx
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position = Point(x=float(mx), y=float(my), z=0.0)
            m.scale.x = m.scale.y = m.scale.z = MIDPOINT_MARKER_SCALE
            r, g, b, a = MIDPOINT_COLOR
            m.color = ColorRGBA(r=r, g=g, b=b, a=a)
            markers.markers.append(m)
        for idx, (mx, my) in enumerate(mids_or):
            if not (0.0 < my < MAX_MARKER_Y):
                continue
            m = Marker()
            m.header = msg.header
            m.ns = "midpoints_or"
            m.id = 20000 + idx
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position = Point(x=float(mx), y=float(my), z=0.0)
            m.scale.x = m.scale.y = m.scale.z = MIDPOINT_MARKER_SCALE
            m.color = ColorRGBA(r=1.0, g=0.6, b=0.3, a=1.0)
            markers.markers.append(m)

        # Verbindungslinien zwischen Mittelpunkten nicht mehr zeichnen
        # Zuvor wurden hier Midpoints zu einem Pfad verbunden. Diese
        # Marker werden jetzt weggelassen, damit lediglich der Hauptpfad
        # sichtbar bleibt.

        # --- 4) Pfadfindung (Greedy) mit Inertia & Extrapolation ---
        blue_pts = (
            np.array([p[:2] for p in cones["blue"]])
            if cones["blue"]
            else np.empty((0, 2))
        )
        yellow_pts = (
            np.array([p[:2] for p in cones["yellow"]])
            if cones["yellow"]
            else np.empty((0, 2))
        )
        orange_pts = (
            np.array([p[:2] for p in cones["orange"]])
            if cones["orange"]
            else np.empty((0, 2))
        )

        # Start-Vektor aus vorherigem Pfad (Inertia)
        if self.prev_bg and len(self.prev_bg) >= 2:
            vec = np.array(self.prev_bg[-1]) - np.array(self.prev_bg[-2])
            v0 = vec / np.linalg.norm(vec)
        else:
            v0 = np.array([0.0, 1.0])

        def check_side_bg(mid, prev, nxt):
            v = np.array(nxt) - np.array(prev)
            if np.linalg.norm(v) < 1e-3:
                return False
            v = v / np.linalg.norm(v)
            ln, rn = np.array([-v[1], v[0]]), np.array([v[1], -v[0]])

            def sides(pts):
                if pts.shape[0] == 0:
                    return False, False
                d = pts - mid
                dist = np.linalg.norm(d, axis=1)
                pl = np.dot(d, ln)
                pr = np.dot(d, rn)
                return np.any((pl > 0) & (dist < SIDE_CHECK_RADIUS)), np.any(
                    (pr > 0) & (dist < SIDE_CHECK_RADIUS)
                )

            lb, rb = sides(blue_pts)
            ly, ry = sides(yellow_pts)
            return (lb and ry or ly and rb) and not (lb and rb) and not (ly and ry)

        def check_side_or(mid, prev, nxt):
            v = np.array(nxt) - np.array(prev)
            if np.linalg.norm(v) < 1e-3:
                return False
            v = v / np.linalg.norm(v)
            ln, rn = np.array([-v[1], v[0]]), np.array([v[1], -v[0]])
            if orange_pts.shape[0] == 0:
                return False
            d = orange_pts - mid
            dist = np.linalg.norm(d, axis=1)
            pl = np.dot(d, ln)
            pr = np.dot(d, rn)
            return np.any((pl > 0) & (dist < SIDE_CHECK_RADIUS)) and np.any(
                (pr > 0) & (dist < SIDE_CHECK_RADIUS)
            )

        def find_greedy_path(
            mids, check_fn, start_pt, last_vec, max_len, max_step=MAX_STEP_DIST
        ):
            if not mids:
                # Keine Pfadgenerierung ohne erkannte Mittelpunkte
                return [], 0.0, last_vec
            arr = np.array(mids)
            path = [tuple(start_pt)]
            used = set()
            last = np.array(start_pt)
            total = 0.0
            y_axis = np.array([0.0, 1.0])

            while total < max_len:
                dists = np.linalg.norm(arr - last, axis=1)
                cands = [
                    (i, mids[i], dists[i])
                    for i in range(len(mids))
                    if (
                        i not in used
                        and dists[i] > 0.1
                        and dists[i] <= max_step
                        and total + dists[i] <= max_len
                        and mids[i][1] > 0
                    )
                ]
                if len(path) == 1:
                    cands = [
                        (i, p, d)
                        for (i, p, d) in cands
                        if abs(
                            np.degrees(
                                np.arccos(
                                    np.clip(
                                        np.dot(
                                            (np.array(p) - last)
                                            / np.linalg.norm(np.array(p) - last),
                                            y_axis,
                                        ),
                                        -1,
                                        1,
                                    )
                                )
                            )
                        )
                        <= FIRST_STEP_MAX_ANGLE
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
                        ang = np.degrees(
                            np.arccos(np.clip(np.dot(last_vec, dirv), -1, 1))
                        )
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
            p_bg, l_bg, v1 = find_greedy_path(
                mids_bg, check_side_bg, start_pt, v0, PATH_LENGTH
            )
            l_or_max = PATH_LENGTH - l_bg
            p_or, l_or = [], 0.0
            if l_or_max > 0 and len(p_bg) > 1:
                p_or, l_or, _ = find_greedy_path(
                    mids_or, check_side_or, p_bg[-1], v1, l_or_max
                )

            pts_count = len(p_bg) + len(p_or)
            total_len = l_bg + l_or

            # Auswahl: zuerst mehr Punkte, dann längere Strecke
            if (pts_count > max_pts) or (pts_count == max_pts and total_len > best_len):
                max_pts = pts_count
                best_len = total_len
                best_bg = p_bg
                best_or = p_or
                abort = (
                    ""
                    if best_len >= PATH_LENGTH
                    else f"Nur {best_len:.2f}m erreicht ({max_pts} Punkte)."
                )

        # Extrapolation, damit immer genau PATH_LENGTH erreicht wird
        combined = best_bg + best_or

        if len(combined) >= 2 and best_len < PATH_LENGTH:
            prev_pt = np.array(combined[-2])
            last_pt = np.array(combined[-1])
            dir_vec = last_pt - prev_pt
            norm = np.linalg.norm(dir_vec)
            if norm > 1e-6:
                dir_vec /= norm
                missing = PATH_LENGTH - best_len
                ext_pt = tuple((last_pt + dir_vec * missing).tolist())
                if best_or:
                    best_or.append(ext_pt)
                else:
                    best_bg.append(ext_pt)

        # 5) Pfad glätten
        best_bg = smooth_path(best_bg)
        best_or = smooth_path(best_or)
        cand_bg = best_bg
        cand_or = best_or

        # --- Pfadaktualisierung nach Längenvergleich ---
        cand_combined = cand_bg + cand_or
        frame_best_path = cand_combined
        cand_len = (
            sum(
                np.linalg.norm(np.array(b) - np.array(a))
                for a, b in zip(cand_combined[:-1], cand_combined[1:])
            )
            if len(cand_combined) >= 2
            else 0.0
        )
        cand_green = (
            sum(
                np.linalg.norm(np.array(b) - np.array(a))
                for a, b in zip(cand_bg[:-1], cand_bg[1:])
            )
            if len(cand_bg) >= 2
            else 0.0
        )

        accept = False
        if not self.current_bg:
            accept = True
        elif cand_len > (self.current_len - self.dist_since_update):
            accept = True

        if accept:
            self.current_bg = cand_bg
            self.current_or = cand_or
            self.current_len = cand_len
            self.green_len = cand_green
            self.dist_since_update = 0.0
            self.stop_braked = False
            self.path_id_counter += 1
            self.current_path_id = self.path_id_counter
            angle_curr = (
                self.shared_angle
                if self.shared_angle is not None
                else (self._angle_smoothed if self._angle_smoothed is not None else 0.0)
            )
            self._update_longest_path(
                prev_angle, angle_curr, move_dist, avg_dx, avg_dy, matches
            )
            if self.current_len > self.longest_len:
                self.longest_bg = list(self.current_bg)
                self.longest_or = list(self.current_or)
                self.longest_len = self.current_len
            self.last_angle = angle_curr
            best_bg = self.current_bg
            best_or = self.current_or
        else:
            angle_curr = (
                self.shared_angle
                if self.shared_angle is not None
                else (
                    self._angle_smoothed
                    if self._angle_smoothed is not None
                    else self.last_angle
                )
            )
            self.current_bg = transform_path(
                self.current_bg,
                self.last_angle,
                angle_curr,
                move_dist,
            )
            self.current_or = transform_path(
                self.current_or,
                self.last_angle,
                angle_curr,
                move_dist,
            )
            if matches > 0:
                self.current_bg = [(x + avg_dx, y + avg_dy) for x, y in self.current_bg]
                self.current_or = [(x + avg_dx, y + avg_dy) for x, y in self.current_or]

            colors = ["bg"] * len(self.current_bg) + ["or"] * len(self.current_or)
            combined, colors = crop_path(
                self.current_bg + self.current_or,
                colors,
                move_dist,
            )
            self.current_bg = [p for p, c in zip(combined, colors) if c == "bg"]
            self.current_or = [p for p, c in zip(combined, colors) if c == "or"]
            self.current_len = path_length(combined)
            self.green_len = path_length(self.current_bg)
            self.dist_since_update = 0.0
            self._update_longest_path(
                prev_angle, angle_curr, move_dist, avg_dx, avg_dy, matches
            )
            if self.current_len > self.longest_len:
                self.longest_bg = list(self.current_bg)
                self.longest_or = list(self.current_or)
                self.longest_len = self.current_len
            self.last_angle = angle_curr
            best_bg = self.current_bg
            best_or = self.current_or
        combined = best_bg + best_or

        # 6) Winkel berechnen, filtern und publizieren
        angle_source = frame_best_path if len(frame_best_path) >= 2 else best_bg
        if len(angle_source) >= 2:
            v = np.array(angle_source[1])
            raw_angle = float(np.degrees(np.arctan2(v[0], v[1])))

            # Ausreißerprüfung
            if (
                self._angle_buffer
                and abs(raw_angle - self._angle_buffer[-1]) > ANGLE_JUMP_THRESH
            ):
                pass  # logging disabled
            else:
                # Puffer aktualisieren
                self._angle_buffer.append(raw_angle)
                if len(self._angle_buffer) > ANGLE_WINDOW:
                    self._angle_buffer.pop(0)

            # Median-Filter
            filtered = (
                float(np.median(self._angle_buffer))
                if self._angle_buffer
                else raw_angle
            )

            # EMA-Glättung
            if self._angle_smoothed is None:
                self._angle_smoothed = filtered
            else:
                self._angle_smoothed = (
                    ANGLE_SMOOTH_ALPHA * self._angle_smoothed
                    + (1 - ANGLE_SMOOTH_ALPHA) * filtered
                )

            # Publizieren
            angle_msg = Float32(data=self._angle_smoothed)
            self.angle_pub.publish(angle_msg)
            if self.angle_shared_pub.get_subscription_count() > 0:
                self.angle_shared_pub.publish(angle_msg)

        # Pfad in MarkerArray
        path_markers = MarkerArray()
        clr = Marker()
        clr.action = Marker.DELETEALL
        clr.header = msg.header
        clr.ns = "best_path"
        clr.id = 40000
        path_markers.markers.append(clr)

        combined_display = [p for p in frame_best_path if 0.0 < p[1] < MAX_MARKER_Y]
        if self.current_len < 1.0:
            combined_display = self.longest_bg + self.longest_or
        combined_display = [p for p in combined_display if 0.0 < p[1] < MAX_MARKER_Y]
        if len(combined_display) >= 2:
            # keep path anchored at the origin
            pts = [Point(x=float(x), y=float(y), z=0.0) for x, y in combined_display]
            n_bg = len(cand_bg)
            m = Marker()
            m.header = msg.header
            m.ns = "best_path"
            m.id = 30000 + self.current_path_id
            m.type = Marker.LINE_STRIP
            m.action = Marker.ADD
            m.scale.x = 0.08
            m.points = pts
            m.colors = []
            for i in range(len(pts)):
                if i < n_bg:
                    m.colors.append(ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0))
                else:
                    m.colors.append(ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0))
            path_markers.markers.append(m)

            cand_clr = Marker()
            cand_clr.action = Marker.DELETEALL
            cand_clr.header = msg.header
            cand_clr.ns = "frame_best"
            cand_clr.id = 41000
            path_markers.markers.append(cand_clr)
            if len(frame_best_path) >= 2:
                cand_pts = [
                    Point(x=float(x), y=float(y), z=0.0)
                    for x, y in frame_best_path
                    if 0.0 < y < MAX_MARKER_Y
                ]
                cand_m = Marker()
                cand_m.header = msg.header
                cand_m.ns = "frame_best"
                cand_m.id = 31000
                cand_m.type = Marker.LINE_STRIP
                cand_m.action = Marker.ADD
                cand_m.scale.x = 0.04
                cand_m.points = cand_pts
                cand_m.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)
                path_markers.markers.append(cand_m)

            # Track also as simple image showing blue/yellow cones and lines
            blue_np = (
                np.array([p[:2] for p in cones["blue"]])
                if cones["blue"]
                else np.empty((0, 2))
            )
            yellow_np = (
                np.array([p[:2] for p in cones["yellow"]])
                if cones["yellow"]
                else np.empty((0, 2))
            )
            if blue_np.size or yellow_np.size:
                all_pts = np.vstack([a for a in (blue_np, yellow_np) if a.size])
                min_x, max_x = all_pts[:, 0].min(), all_pts[:, 0].max()
                min_y, max_y = all_pts[:, 1].min(), all_pts[:, 1].max()
                span = max(max_x - min_x, max_y - min_y, 1e-3)
                scale = 360.0 / span
                offset_x = (400 - (max_x - min_x) * scale) / 2 - min_x * scale
                offset_y = (400 - (max_y - min_y) * scale) / 2 - min_y * scale
                track_img = np.ones((400, 400, 3), dtype=np.uint8) * 255

                def draw_set(pts, color_line, color_point):
                    if len(pts) >= 2:
                        pts_sorted = sorted(pts, key=lambda p: p[1])
                        for a, b in zip(pts_sorted[:-1], pts_sorted[1:]):
                            ax = int(a[0] * scale + offset_x)
                            ay = int(400 - (a[1] * scale + offset_y))
                            bx = int(b[0] * scale + offset_x)
                            by = int(400 - (b[1] * scale + offset_y))
                            cv2.line(track_img, (ax, ay), (bx, by), color_line, 1)
                        pts_use = pts_sorted
                    else:
                        pts_use = pts
                    for x, y in pts_use:
                        xi = int(x * scale + offset_x)
                        yi = int(400 - (y * scale + offset_y))
                        cv2.circle(track_img, (xi, yi), 3, color_point, -1)

                draw_set(blue_np, (255, 0, 0), (255, 0, 0))  # blue in BGR
                draw_set(yellow_np, (0, 255, 255), (0, 255, 255))  # yellow in BGR

                track_msg = self.bridge.cv2_to_imgmsg(track_img, "bgr8")
                track_msg.header.stamp = msg.header.stamp
                self.track_image_pub.publish(track_msg)
                self.track_global_pub.publish(track_msg)

        # Geschwindigkeit anhand Pfadlänge und Lenkwinkel berechnen
        if len(combined) >= 2:
            path_len = sum(
                np.linalg.norm(np.array(b) - np.array(a))
                for a, b in zip(combined[:-1], combined[1:])
            )
        else:
            path_len = 0.0

        angle_val = (
            float(self._angle_smoothed) if self._angle_smoothed is not None else 0.0
        )
        if path_len > 0.0:
            speed = self.max_speed * (
                (
                    1
                    - abs(angle_val) / ANGLE_SPEED_DIVISOR
                    + path_len / SPEED_PATH_LENGTH
                )
                / 2.0
            )
        else:
            speed = 0.0
        speed = min(speed, self.max_speed)

        first_green = len(self.current_bg) >= 2
        if not first_green:
            speed = 0.0

        if (
            not self.stop_braked
            and self.dist_since_update >= self.green_len
            and self.green_len > 0
        ):
            speed = 0.0
            self.stop_braked = True

        # store newly calculated speed as desired speed
        self.desired_speed = speed

        angle_img = draw_steering_wheel(angle_val, MAX_STEERING_ANGLE, STEERING_RATIO)
        angle_msg = self.bridge.cv2_to_imgmsg(angle_img, "bgr8")
        angle_msg.header.stamp = msg.header.stamp
        self.angle_image_pub.publish(angle_msg)

        speed_img = draw_speed_gauge(speed, self.max_speed)
        speed_msg = self.bridge.cv2_to_imgmsg(speed_img, "bgr8")
        speed_msg.header.stamp = msg.header.stamp
        self.speed_image_pub.publish(speed_msg)
        if self.speed_cmd_pub.get_subscription_count() > 0:
            self._ignore_next_speed_msg = True
            self.speed_cmd_pub.publish(Float32(data=float(speed)))

        self.last_speed = speed

        # 8) FPS berechnen & publizieren
        dt = time.time() - t0
        self._frame_times.append(dt)
        if len(self._frame_times) > 10:
            self._frame_times.pop(0)
        avg = sum(self._frame_times) / len(self._frame_times)
        self.fps_pub.publish(Float32(data=(1.0 / avg if avg > 0 else 0.0)))

        # 9) Warnung bei Kürze (logger disabled)
        if abort:
            pass

        # abschließend Marker-Arrays senden (nicht entfernen)
        self.marker_pub.publish(markers)
        self.path_pub.publish(path_markers)

        self.prev_bg = list(self.current_bg)
        self.prev_or = list(self.current_or)
        self.prev_cones = cone_positions


def main(args=None):
    rclpy.init(args=args)
    node = PathNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
