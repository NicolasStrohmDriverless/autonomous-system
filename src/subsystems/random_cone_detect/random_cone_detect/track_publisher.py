#!/usr/bin/env python3
import math
import random
from pathlib import Path
import numpy as np
from scipy import signal, spatial, interpolate
from shapely.geometry import Point as ShapelyPoint, LineString, Polygon, MultiPolygon  # <--- MultiPolygon import
import cv2
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

# Message-Typen
from oak_cone_detect_interfaces.msg import ConeArray3D, Cone3D
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point as MsgPoint
from sensor_msgs.msg import Image

# Hilfsfunktionen (aus utils.py)
from random_cone_detect.utils import (
    closest_node,
    clockwise_sort,
    curvature,
    load_yaml_track,
)

# QoS für Cone-Publisher (Best Effort, volatile, depth=1)
qos_cone_pub = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
    durability=QoSDurabilityPolicy.VOLATILE
)
# QoS für Marker-Publisher (reliable, depth=10)
qos_marker_pub = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
    durability=QoSDurabilityPolicy.VOLATILE
)

# Globale Parameter f\xc3\xbcr die Trackgenerierung
MAX_CURVE_ANGLE_DEG = 90.0   # maximal erlaubter Kurvenwinkel
START_STRAIGHT_LEN = 3.0     # L\xc3\xa4nge des geraden Startst\xc3\xbccks
TRACK_WIDTH = 3.0            # Breite der Strecke

from enum import Enum
class Mode(Enum):
    RANDOM  = 1
    LINE    = 2
    NEAREST = 3
    ACCEL   = 4

class TrackGenerator:
    """
    Generiert einen zufälligen, geschlossenen Track via begrenzter Voronoi-Regionen,
    optional mit oder ohne Krümmungs-Pruning, liefert linke Kegel, rechte Kegel und Centerline.
    """
    def __init__(
        self,
        n_points,
        n_regions,
        min_bound,
        max_bound,
        mode: Mode,
        track_width=TRACK_WIDTH,
        curvature_threshold: float = 1.0 / 3.75,
        allow_sharp_turns: bool = False,
        max_turn_angle_deg: float = MAX_CURVE_ANGLE_DEG,
        seed: int | None = None,
    ):
        self._n_points = n_points
        self._n_regions = n_regions
        self._min_bound = min_bound
        self._max_bound = max_bound
        self._bounding_box = np.array([min_bound, max_bound, min_bound, max_bound])
        self._mode = mode
        self._track_width = track_width
        self._curvature_threshold = curvature_threshold
        self._allow_sharp_turns = allow_sharp_turns
        self._max_turn_angle_deg = max_turn_angle_deg
        # Independent random generators for reproducibility
        self._rng = np.random.default_rng(seed)
        self._rnd = random.Random(seed)
        if curvature_threshold is None:
            self._curvature_threshold = max(1e-3, math.radians(90 - max_turn_angle_deg/2))

    def bounded_voronoi(self, pts, bbox):
        def _mirror(boundary, axis):
            m = pts.copy()
            m[:,axis] = 2*boundary - m[:,axis]
            return m
        x_min, x_max, y_min, y_max = bbox
        pts_all = np.vstack([
            pts,
            _mirror(x_min,0), _mirror(x_max,0),
            _mirror(y_min,1), _mirror(y_max,1)
        ])
        vor = spatial.Voronoi(pts_all)
        vor.filtered_points = pts
        vor.filtered_regions = np.array(vor.regions, object)[vor.point_region[:len(pts)]]
        return vor

    def create_track(self):
        if self._mode == Mode.ACCEL:
            y = np.linspace(0.0, 100.0, int(100.0 / self._track_width) + 1)
            left = np.stack((-self._track_width/2 * np.ones_like(y), y), axis=1)
            right = np.stack((self._track_width/2 * np.ones_like(y), y), axis=1)
            center = np.stack((np.zeros_like(y), y), axis=1)
            return left, right, center

        pts = self._rng.uniform(self._min_bound, self._max_bound, (self._n_points, 2))
        vor = self.bounded_voronoi(pts, self._bounding_box)

        # wiederhole Auswahl, bis gültiger Track gefunden
        while True:
            # 1) Punkte auswählen
            if self._mode == Mode.RANDOM:
                i = self._rnd.randrange(len(pts))
                sel = [i] + [closest_node(pts[i], pts, k=j+1)
                             for j in range(self._n_regions-1)]
            elif self._mode == Mode.LINE:
                i = self._rnd.randrange(len(pts))
                ang = self._rnd.uniform(0, math.pi / 2)
                p = pts[i]
                d = self._max_bound / 2
                line = LineString([
                    (p[0]-d*math.cos(ang), p[1]-d*math.sin(ang)),
                    (p[0]+d*math.cos(ang), p[1]+d*math.sin(ang))
                ])
                dists = [ShapelyPoint(q).distance(line) for q in pts]
                sel = np.argpartition(dists, self._n_regions)[:self._n_regions]
            else:  # NEAREST
                sel = self._rng.choice(len(pts), self._n_regions, replace=False)

            regs = vor.filtered_regions[sel]
            verts = np.unique(
                np.vstack([vor.vertices[r] for r in regs]), axis=0
            )
            verts = np.vstack([clockwise_sort(verts), clockwise_sort(verts)[0]])

            # 2) Spline & optional Krümmungs-Pruning
            if not self._allow_sharp_turns:
                # klassisches Pruning, bis Krümmung akzeptabel
                while True:
                    tck, _ = interpolate.splprep([verts[:,0], verts[:,1]], s=0, per=True)
                    t = np.linspace(0,1,1000)
                    x, y   = interpolate.splev(t, tck, der=0)
                    dx, dy = interpolate.splev(t, tck, der=1)
                    ddx, ddy = interpolate.splev(t, tck, der=2)
                    k = curvature(dx, ddx, dy, ddy)
                    peaks, _ = signal.find_peaks(np.abs(k))
                    if peaks.size and np.abs(k[peaks]).max() > self._curvature_threshold:
                        # schärfste Stelle entfernen und neu versuchen
                        idx = peaks[np.abs(k[peaks]).argmax()]
                        vid = closest_node((x[idx], y[idx]), verts, k=0)
                        verts = np.delete(verts, vid, 0)
                        if not np.array_equal(verts[0], verts[-1]):
                            verts = np.vstack([verts, verts[0]])
                        continue
                    break
            else:
                # nur einmal splinen, keine Krümmungsprüfung
                tck, _ = interpolate.splprep([verts[:,0], verts[:,1]], s=0, per=True)
                t = np.linspace(0,1,1000)
                x, y = interpolate.splev(t, tck, der=0)

            poly = Polygon(zip(x, y))
            if poly.is_valid:
                break

        centerline = np.vstack((x, y)).T
        left  = poly.buffer(self._track_width / 2)
        right = poly.buffer(-self._track_width / 2)

        # --- Fix: MultiPolygon-Handling einbauen ---
        if isinstance(left, MultiPolygon):
            left = max(left.geoms, key=lambda g: g.area)
        if isinstance(right, MultiPolygon):
            right = max(right.geoms, key=lambda g: g.area)

        nL = int(np.ceil(left.length / self._track_width)) + 1
        nR = int(np.ceil(right.length / self._track_width)) + 1
        cones_left  = np.array([
            left.exterior.interpolate(i * left.length / nL).coords[0] for i in range(nL)
        ])
        cones_right = np.array([
            right.exterior.interpolate(i * right.length / nR).coords[0] for i in range(nR)
        ])

        # Align track so that the first left and right cones are located at
        # (-TRACK_WIDTH/2, 0) and (TRACK_WIDTH/2, 0) respectively.  This keeps the field of view
        # consistent and places the start line at the origin.
        start_left = cones_left[0]
        start_right = cones_right[0]
        midpoint = (start_left + start_right) / 2
        vec = start_right - start_left
        angle = math.atan2(vec[1], vec[0])
        rot = np.array([
            [math.cos(-angle), -math.sin(-angle)],
            [math.sin(-angle),  math.cos(-angle)],
        ])
        scale = 3.0 / np.linalg.norm(vec)

        def _transform(arr: np.ndarray) -> np.ndarray:
            arr = (arr - midpoint) @ rot.T
            return arr * scale

        cones_left = _transform(cones_left)
        cones_right = _transform(cones_right)
        centerline = _transform(centerline)

        # Startrichtung nach oben ausrichten
        dir_vec = centerline[1] - centerline[0]
        phi = math.atan2(dir_vec[1], dir_vec[0])
        rot2 = np.array([
            [math.cos(math.pi/2 - phi), -math.sin(math.pi/2 - phi)],
            [math.sin(math.pi/2 - phi),  math.cos(math.pi/2 - phi)],
        ])
        cones_left = cones_left @ rot2.T
        cones_right = cones_right @ rot2.T
        centerline = centerline @ rot2.T

        # Sicherstellen, dass die ersten Meter gerade verlaufen
        cones_left[0] = (-self._track_width/2, 0.0)
        cones_right[0] = (self._track_width/2, 0.0)
        centerline[0] = (0.0, 0.0)
        if len(centerline) > 1:
            cones_left[1] = (-self._track_width/2, START_STRAIGHT_LEN)
            cones_right[1] = (self._track_width/2, START_STRAIGHT_LEN)
            centerline[1] = (0.0, START_STRAIGHT_LEN)

        return cones_left, cones_right, centerline


class TrackPublisher(Node):
    def __init__(self, mode: str = "autox", seed: int | None = None):
        super().__init__('track_generator_node')
        # Publisher für Kegel (als ConeArray3D für PathNode)
        self.cone_pub  = self.create_publisher(ConeArray3D, '/cone_detections_3d', qos_cone_pub)
        # Publisher für Track-Marker (Visualisierung)
        self.track_pub = self.create_publisher(MarkerArray, '/track_markers', qos_marker_pub)
        # Publish generated track image on a debug topic to avoid confusion with
        # the actual driven track published by the pathfinding node
        self.image_pub = self.create_publisher(Image, '/track/generated_image', 1)
        self.bridge = CvBridge()
        self._seed = seed
        self._mode = mode

        self.get_logger().info('TrackPublisher startet und publiziert Track...')
        self.publish_track()

    def publish_track(self):
        track_dir = Path(__file__).resolve().parents[1] / "tracks"
        if self._mode == "accel":
            track_file = track_dir / "acceleration.yaml"
        elif self._mode == "skidpad":
            track_file = track_dir / "skidpad.yaml"
        else:
            fs_tracks = [
                "FSE23.yaml",
                "FSE24.yaml",
                "FSG23.yaml",
                "FSG24.yaml",
                "FSS22_V1.yaml",
                "FSS22_V2.yaml",
            ]
            track_file = track_dir / random.choice(fs_tracks)
        cones_left, cones_right, centerline, gates = load_yaml_track(str(track_file))

        # --- 1) ConeArray3D erzeugen und publizieren ---
        cone_msg = ConeArray3D()
        cone_msg.header.stamp = self.get_clock().now().to_msg()
        cone_msg.header.frame_id = 'map'
        # linke Kegel
        for i, (x, y, color) in enumerate(cones_left):
            c = Cone3D()
            c.id    = str(i)
            c.label = 'left'
            c.conf  = 1.0
            c.x, c.y, c.z = float(x), float(y), 0.0
            c.color = str(color)
            cone_msg.cones.append(c)
        # rechte Kegel
        off = len(cones_left)
        for i, (x, y, color) in enumerate(cones_right):
            c = Cone3D()
            c.id    = str(off + i)
            c.label = 'right'
            c.conf  = 1.0
            c.x, c.y, c.z = float(x), float(y), 0.0
            c.color = str(color)
            cone_msg.cones.append(c)
        # Startkegel aus den Zeitmess-Toren verwenden
        start_positions = [tuple(map(float, g[:2])) for g in gates[:2]]
        if len(start_positions) < 2:
            start_positions = [(-TRACK_WIDTH/2, 0.0), (TRACK_WIDTH/2, 0.0)]
        for idx, (sx, sy) in enumerate(start_positions, start=off + len(cones_right)):
            s = Cone3D()
            s.id    = f'start{idx}'
            s.label = 'start'
            s.conf  = 1.0
            s.x, s.y, s.z = float(sx), float(sy), 0.0
            s.color = 'orange'
            cone_msg.cones.append(s)

        self.cone_pub.publish(cone_msg)
        self.get_logger().info(f'Publiziert {len(cone_msg.cones)} Kegel auf /cone_detections_3d')

        # --- 2) Track als MarkerArray visualisieren ---
        m_arr = MarkerArray()

        # Centerline
        line = Marker()
        line.header = cone_msg.header
        line.ns     = 'track_centerline'
        line.id     = 0
        line.type   = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = 0.05
        line.color.r = 1.0
        line.color.a = 1.0
        for pt in centerline:
            p = MsgPoint(x=float(pt[0]), y=float(pt[1]), z=0.0)
            line.points.append(p)
        m_arr.markers.append(line)

        # Startlinie
        start = Marker()
        start.header = cone_msg.header
        start.ns     = 'track_start_line'
        start.id     = 1
        start.type   = Marker.LINE_LIST
        start.action = Marker.ADD
        start.scale.x = 0.1
        start.color.g = 1.0
        start.color.a = 1.0
        for x in [-TRACK_WIDTH/2, TRACK_WIDTH/2]:
            p = MsgPoint(x=x, y=0.0, z=0.0)
            start.points.append(p)
        m_arr.markers.append(start)

        self.track_pub.publish(m_arr)
        self.get_logger().info('Track-Marker auf /track_markers publiziert')

        # Track als Punktdiagramm als Bild senden
        # Zeichne Track farblich codiert als Bild
        pts = np.vstack([cones_left, cones_right])
        min_x, max_x = pts[:, 0].min(), pts[:, 0].max()
        min_y, max_y = pts[:, 1].min(), pts[:, 1].max()
        span = max(max_x - min_x, max_y - min_y, 1e-3)
        scale = 360.0 / span
        offset_x = (400 - (max_x - min_x) * scale) / 2 - min_x * scale
        offset_y = (400 - (max_y - min_y) * scale) / 2 - min_y * scale

        img = np.ones((400, 400, 3), dtype=np.uint8) * 255

        # linke Kegel blau einzeichnen
        for x, y, *_ in cones_left:
            xi = int(x * scale + offset_x)
            yi = int(400 - (y * scale + offset_y))
            cv2.circle(img, (xi, yi), 3, (255, 0, 0), -1)

        # rechte Kegel gelb einzeichnen
        for x, y, *_ in cones_right:
            xi = int(x * scale + offset_x)
            yi = int(400 - (y * scale + offset_y))
            cv2.circle(img, (xi, yi), 3, (0, 255, 255), -1)

        # Startkegel orange markieren
        for sx, sy in start_positions:
            xi = int(sx * scale + offset_x)
            yi = int(400 - (sy * scale + offset_y))
            cv2.circle(img, (xi, yi), 4, (0, 165, 255), -1)
        # Startlinie gr\u00fcn einzeichnen
        if len(start_positions) >= 2:
            sx1 = int(start_positions[0][0] * scale + offset_x)
            sy1 = int(400 - (start_positions[0][1] * scale + offset_y))
            sx2 = int(start_positions[1][0] * scale + offset_x)
            sy2 = int(400 - (start_positions[1][1] * scale + offset_y))
        else:
            sx1 = int(-TRACK_WIDTH/2 * scale + offset_x)
            sy1 = int(400 - (0.0 * scale + offset_y))
            sx2 = int(TRACK_WIDTH/2 * scale + offset_x)
            sy2 = sy1
        cv2.line(img, (sx1, sy1), (sx2, sy2), (0, 255, 0), 2)
        msg = self.bridge.cv2_to_imgmsg(img, 'bgr8')
        msg.header.stamp = cone_msg.header.stamp
        self.image_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TrackPublisher()
    # Nur eine kurze Spin-Iteration, da wir nur einmal publizieren
    rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
