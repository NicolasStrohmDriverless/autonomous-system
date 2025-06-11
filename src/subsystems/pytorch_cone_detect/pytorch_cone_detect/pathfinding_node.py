#!/usr/bin/env python3

import time
import random

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

# nur Imu aus sensor_msgs, Header aus std_msgs
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
from std_msgs.msg import ColorRGBA, Float32
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from oak_cone_detect_interfaces.msg import ConeArray3D

import numpy as np
from scipy.spatial import Delaunay
from scipy.spatial.transform import Rotation as R, Slerp

# QoS nur für den Subscriber: Best Effort, volatile, depth=1
qos_sub = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
    durability=QoSDurabilityPolicy.VOLATILE
)

# Pfad- und Marker-Parameter
PATH_LENGTH             = 20.0
MAX_ANGLE               = 30.0
FIRST_STEP_MAX_ANGLE    = 40.0
MIDPOINT_MARKER_SCALE   = 0.1
DEFAULT_CONE_SCALE      = (0.228, 0.228, 0.325)
LARGE_ORANGE_CONE_SCALE = (0.285, 0.285, 0.505)
CONE_POSITION_SCALE     = (1, 1, 1)
COLOR_MAP = {
    'blue':   [0.0, 0.0, 1.0, 1.0],
    'yellow': [1.0, 1.0, 0.0, 1.0],
    'orange': [1.0, 0.5, 0.0, 1.0]
}
MIDPOINT_COLOR    = [0.5, 0.5, 0.5, 1.0]
SIDE_CHECK_RADIUS = 2.0
MAX_STEP_DIST     = 2.0
GEGENCHECK        = 1

def quaternion_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    x = w1*x2 + x1*w2 + y1*z2 - z1*y2
    y = w1*y2 - x1*z2 + y1*w2 + z1*x2
    z = w1*z2 + x1*y2 - y1*x2 + z1*w2
    w = w1*w2 - x1*x2 - y1*y2 - z1*z2
    return np.array([x, y, z, w], dtype=float)

def rotation_matrix_from_quaternion(q: np.ndarray) -> np.ndarray:
    x, y, z, w = q
    n = np.linalg.norm(q)
    if n == 0:
        return np.eye(3)
    x /= n; y /= n; z /= n; w /= n
    return np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - z*w),       2*(x*z + y*w)],
        [2*(x*y + z*w),         1 - 2*(x*x + z*z),   2*(y*z - x*w)],
        [2*(x*z - y*w),         2*(y*z + x*w),       1 - 2*(x*x + y*y)]
    ], dtype=float)

class PathNode(Node):
    def __init__(self):
        super().__init__('midpoint_path_node')

        # Subscriber für 3D-Detektionen
        self.sub_det = self.create_subscription(
            ConeArray3D,
            '/cone_detections_3d',
            self.detection_cb,
            qos_profile=qos_sub
        )
        # Subscriber für IMU-Daten
        self.sub_imu = self.create_subscription(
            Imu,
            '/camera/imu_raw',
            self.imu_cb,
            50
        )

        # Marker-Publisher
        self.marker_pub = self.create_publisher(MarkerArray, '/cone_markers', 10)
        self.path_pub   = self.create_publisher(MarkerArray, '/best_path_marker', 10)
        self.fps_pub    = self.create_publisher(Float32, '/path_inference_fps', 10)

        # Für FPS-Messung
        self._frame_times = []

        # IMU-Status
        self.last_imu_time       = None
        self.cam_orientation_q   = np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
        self.global_rotation     = np.eye(3)
        self.acc_cam             = np.zeros(3, dtype=float)

        # Zwischengespeicherte letzte Detektionen
        self.last_cones_data = None  # List of tuples (x, y, z, color)
        self.last_header     = Header()

    def detection_cb(self, msg: ConeArray3D):
        # Wenn neue Detektionen ankommen, speichere sie und verarbeite direkt
        self.last_cones_data = [(c.x, c.y, c.z, c.color) for c in msg.cones]
        self.last_header     = msg.header
        self._handle_cones(self.last_cones_data, msg.header)

    def imu_cb(self, imu_msg: Imu):
        """
        Verarbeitet IMU-Daten und, falls keine neuen Detektionen vorliegen,
        verschiebt die alten Detektionen entsprechend der Kamera-Drehung.
        """
        cur_time = self.get_clock().now().nanoseconds * 1e-9
        # Erster IMU-Callback: Initialisierung
        if self.last_imu_time is None:
            self.last_imu_time = cur_time
            if imu_msg.orientation_covariance[0] != -1:
                q = imu_msg.orientation
                self.cam_orientation_q = np.array([q.x, q.y, q.z, q.w], dtype=float)
                self.cam_orientation_q /= np.linalg.norm(self.cam_orientation_q)
            self.acc_cam = np.array([
                imu_msg.linear_acceleration.x,
                imu_msg.linear_acceleration.y,
                imu_msg.linear_acceleration.z
            ], dtype=float)
            self.global_rotation = rotation_matrix_from_quaternion(self.cam_orientation_q)
            return

        dt = cur_time - self.last_imu_time
        self.last_imu_time = cur_time
        if dt <= 0:
            return

        # Quaternion-Delta aus Winkelgeschwindigkeit
        wx, wy, wz = (imu_msg.angular_velocity.x,
                      imu_msg.angular_velocity.y,
                      imu_msg.angular_velocity.z)
        omega = np.array([wx, wy, wz], dtype=float)
        omega_mag = np.linalg.norm(omega)
        if omega_mag > 1e-6:
            axis  = omega / omega_mag
            theta = omega_mag * dt
            dq    = R.from_rotvec(axis * theta).as_quat()
        else:
            dq = np.array([0.0, 0.0, 0.0, 1.0], dtype=float)

        # Kumulative Quaternion-Integration
        q_old = self.cam_orientation_q.copy()
        q_new = quaternion_multiply(dq, q_old)
        self.cam_orientation_q = q_new / np.linalg.norm(q_new)

        # Optional: Slerp mit IMU-Orientation, falls vorhanden
        if imu_msg.orientation_covariance[0] != -1:
            q_imu = imu_msg.orientation
            measured_q = np.array([q_imu.x, q_imu.y, q_imu.z, q_imu.w], dtype=float)
            measured_q /= np.linalg.norm(measured_q)
            rotations = R.from_quat([self.cam_orientation_q, measured_q])
            slerp     = Slerp([0.0, 1.0], rotations)
            r_interp  = slerp([0.02])[0]
            self.cam_orientation_q = r_interp.as_quat()

        # Alte und neue Rotationsmatrix
        rot_old = self.global_rotation.copy()
        self.global_rotation = rotation_matrix_from_quaternion(self.cam_orientation_q)

        # Aktualisiere lineare Beschleunigung (optional)
        self.acc_cam = np.array([
            imu_msg.linear_acceleration.x,
            imu_msg.linear_acceleration.y,
            imu_msg.linear_acceleration.z
        ], dtype=float)

        # Wenn keine neuen 3D-Detektionen angekommen sind, verschiebe die letzten
        if self.last_cones_data:
            rot_delta = self.global_rotation.T.dot(rot_old)
            updated = []
            for x, y, z, color in self.last_cones_data:
                p_old = np.array([x, y, z], dtype=float)
                p_new = rot_delta.dot(p_old)
                updated.append((p_new[0], p_new[1], p_new[2], color))
            self.last_cones_data = updated

            hdr = Header()
            hdr.stamp    = self.get_clock().now().to_msg()
            hdr.frame_id = self.last_header.frame_id

            self._handle_cones(self.last_cones_data, hdr)

    def _handle_cones(self, cones_data, header):
        """
        Kernfunktion zur Verarbeitung von 3D-Kegel-Positionen: 
        Markerdarstellung und Pfadfindung.
        """
        t0 = time.time()

        # --- 1) Kegel sammeln ---
        cones = {col: [] for col in COLOR_MAP}
        for x, y, z, color in cones_data:
            # nur Kegel vor der Kamera
            p = np.array([x, z, 0.0]) * np.array(CONE_POSITION_SCALE)
            cones[color].append(p)

        # --- 2) Marker für Kegel ---
        markers = MarkerArray()
        clear = Marker(); clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        for col, pts in cones.items():
            for idx, p in enumerate(pts):
                m = Marker()
                m.header = header
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

        # --- 3) Mittelpunkte via Delaunay ---
        pts2d, cols2d = [], []
        for col, pts in cones.items():
            for p in pts:
                pts2d.append(p[:2])
                cols2d.append(col)

        mids_bg, mids_or = [], []
        if len(pts2d) >= 2:
            try:
                tri = Delaunay(np.array(pts2d))
                for simplex in tri.simplices:
                    for i in range(3):
                        a, b = simplex[i], simplex[(i+1)%3]
                        cs = {cols2d[a], cols2d[b]}
                        mid = tuple(((np.array(pts2d[a]) + np.array(pts2d[b]))/2).round(4))
                        if mid[1] < 0:
                            continue
                        if cs == {'blue','yellow'}:
                            mids_bg.append(mid)
                        if cs == {'orange'}:
                            mids_or.append(mid)
            except:
                pass

        mids_bg = sorted(set(mids_bg), key=lambda x: x[0])
        mids_or = sorted(set(mids_or), key=lambda x: x[0])

        # Mittelpunkte als Spheres
        for idx,(mx,my) in enumerate(mids_bg):
            m = Marker()
            m.header = header; m.ns='midpoints_bg'; m.id=10000+idx
            m.type = Marker.SPHERE; m.action = Marker.ADD
            m.pose.position = Point(x=float(mx), y=float(my), z=0.0)
            m.scale.x = m.scale.y = m.scale.z = MIDPOINT_MARKER_SCALE
            r,g,b,a = MIDPOINT_COLOR
            m.color = ColorRGBA(r=r,g=g,b=b,a=a)
            markers.markers.append(m)
        for idx,(mx,my) in enumerate(mids_or):
            m = Marker()
            m.header = header; m.ns='midpoints_or'; m.id=20000+idx
            m.type = Marker.SPHERE; m.action = Marker.ADD
            m.pose.position = Point(x=float(mx), y=float(my), z=0.0)
            m.scale.x = m.scale.y = m.scale.z = MIDPOINT_MARKER_SCALE
            m.color = ColorRGBA(r=1.0, g=0.6, b=0.3, a=1.0)
            markers.markers.append(m)

        # --- 4) Pfadfindung ---
        blue_pts   = np.array([p[:2] for p in cones['blue']])   if cones['blue']   else np.empty((0,2))
        yellow_pts = np.array([p[:2] for p in cones['yellow']]) if cones['yellow'] else np.empty((0,2))
        orange_pts = np.array([p[:2] for p in cones['orange']]) if cones['orange'] else np.empty((0,2))

        def check_side_bg(mid, prev, nxt):
            v = np.array(nxt)-np.array(prev)
            if np.linalg.norm(v)<1e-3: return False
            v = v/np.linalg.norm(v)
            ln = np.array([-v[1], v[0]]); rn = np.array([v[1], -v[0]])
            def sides(pts):
                if pts.shape[0]==0: return False,False
                d = pts - mid; dist = np.linalg.norm(d,axis=1)
                pl = np.dot(d, ln); pr = np.dot(d, rn)
                return np.any((pl>0)&(dist<SIDE_CHECK_RADIUS)), np.any((pr>0)&(dist<SIDE_CHECK_RADIUS))
            lb, rb = sides(blue_pts); ly, ry = sides(yellow_pts)
            return (lb and ry or ly and rb) and not (lb and rb) and not (ly and ry)

        def check_side_or(mid, prev, nxt):
            v = np.array(nxt)-np.array(prev)
            if np.linalg.norm(v)<1e-3: return False
            v = v/np.linalg.norm(v)
            ln = np.array([-v[1], v[0]]); rn = np.array([v[1], -v[0]])
            if orange_pts.shape[0]==0: return False
            d = orange_pts - mid; dist = np.linalg.norm(d,axis=1)
            pl = np.dot(d,ln); pr = np.dot(d,rn)
            return np.any((pl>0)&(dist<SIDE_CHECK_RADIUS)) and np.any((pr>0)&(dist<SIDE_CHECK_RADIUS))

        def find_greedy_path(mids, check_fn, start_pt, last_vec, max_len):
            if not mids:
                return [], 0.0, last_vec
            arr = np.array(mids)
            path = [tuple(start_pt)]; used = set()
            last = np.array(start_pt); total = 0.0
            y_axis = np.array([0.0,1.0])
            while total < max_len:
                dists = np.linalg.norm(arr-last,axis=1)
                cands = [
                    (i,mids[i],d) for i,d in enumerate(dists)
                    if i not in used and 0.1<d<=MAX_STEP_DIST and total+d<=max_len and mids[i][1]>=0
                ]
                if len(path)==1:
                    cands = [
                        (i,p,d) for (i,p,d) in cands
                        if abs(np.degrees(
                            np.arccos(np.clip(
                                np.dot((np.array(p)-last)/np.linalg.norm(np.array(p)-last),
                                       y_axis),-1,1)
                            ))) <= FIRST_STEP_MAX_ANGLE
                    ]
                if not cands: break
                best = None; best_d = None
                for i,p,d in cands:
                    v = np.array(p)-last; n = np.linalg.norm(v)
                    if n<1e-3: continue
                    dirv = v/n
                    if len(path)>1:
                        ang = np.degrees(np.arccos(np.clip(np.dot(last_vec, dirv),-1,1)))
                        if ang>MAX_ANGLE: continue
                    if not check_fn(p, last, p): continue
                    if best is None or d<best_d:
                        best = (i,p,dirv,d); best_d = d
                if best is None: break
                i,p,dirv,d = best
                path.append(p); used.add(i)
                last = np.array(p); last_vec = dirv; total += d
            return path, total, last_vec

        # Versuche sowohl blau/gelb als auch orange
        best_bg, best_or = [], []
        max_pts, best_len, abort = 0, 0.0, ""
        for r in range(GEGENCHECK):
            ang = 0 if r==0 else random.uniform(0,2*np.pi)
            v0 = np.array([np.cos(ang), np.sin(ang)])
            p_bg, l_bg, v1 = find_greedy_path(mids_bg, check_side_bg, (0,0), v0, PATH_LENGTH)
            l_or_max = PATH_LENGTH - l_bg
            p_or, l_or = [], 0.0
            if l_or_max>0 and len(p_bg)>1:
                p_or, l_or, _ = find_greedy_path(mids_or, check_side_or, p_bg[-1], v1, l_or_max)
            pts_total = len(p_bg) + max(0, len(p_or)-1)
            if pts_total > max_pts:
                best_bg = p_bg
                best_or = p_or[1:] if len(p_or)>1 else []
                max_pts = pts_total
                best_len = l_bg + l_or
                if best_len < PATH_LENGTH:
                    if len(p_bg)<2:
                        abort = "Zu wenige Midpoints für blauen/gelben Pfad."
                    elif pts_total==0:
                        abort = "Kein gültiger Pfad gefunden."
                    else:
                        abort = f"Nur {best_len:.2f} m Pfadlänge erreicht."
                else:
                    abort = ""

        # --- 5) Pfad in MarkerArray ---
        path_markers = MarkerArray()
        clr = Marker(); clr.action = Marker.DELETEALL
        clr.header = header; clr.ns='best_path'; clr.id=40000
        path_markers.markers.append(clr)

        if len(best_bg) + len(best_or) >= 2:
            pts = [Point(x=float(x), y=float(y), z=0.0)
                   for (x,y) in (best_bg + best_or)]
            n_bg = len(best_bg)
            m = Marker()
            m.header = header; m.ns='best_path'; m.id=30000
            m.type = Marker.LINE_STRIP; m.action = Marker.ADD
            m.scale.x = 0.08; m.points = pts; m.colors = []
            for i in range(len(pts)):
                if i < n_bg:
                    m.colors.append(ColorRGBA(r=0.0,g=1.0,b=0.0,a=1.0))
                else:
                    m.colors.append(ColorRGBA(r=1.0,g=0.0,b=0.0,a=1.0))
            path_markers.markers.append(m)

        # --- Publish ---
        self.marker_pub.publish(markers)
        self.path_pub.publish(path_markers)

        # --- FPS ---
        dt = time.time() - t0
        self._frame_times.append(dt)
        if len(self._frame_times) > 10:
            self._frame_times.pop(0)
        avg = sum(self._frame_times) / len(self._frame_times)
        fps = Float32(data=(1.0/avg if avg>0 else 0.0))
        self.fps_pub.publish(fps)

        # Warnung, falls Pfad nicht lang genug
        if abort and best_len < PATH_LENGTH:
            self.get_logger().warn(f"Pfad < {PATH_LENGTH}m! Grund: {abort}")

def main(args=None):
    rclpy.init(args=args)
    node = PathNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
