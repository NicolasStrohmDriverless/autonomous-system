#!/usr/bin/env python3

import time
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from oak_cone_detect_interfaces.msg import ConeArray2D, ConeArray3D, Cone3D
from cv_bridge import CvBridge
import numpy as np
import cv2
from std_msgs.msg import Float32
from filterpy.kalman import KalmanFilter

# --- MAXIMALE ANZAHL GESPEICHERTER TRACKER ---
MAX_TRACKERS = 50

COLOR_PREFIX = {
    'blue': 'b',
    'orange': 'o',
    'red': 'r',
    'yellow': 'g'
}

# --- STEREOKAMERA-PARAMETER (für OAK-D) ---
BASELINE_M = 0.075         # Meter (75 mm)
FOCAL_LENGTH_PX = 870.0    # Pixel (laut OAK-D Datenblatt)
MIN_DISP = 0.1             # Minimalwert Disparität, sonst ungültig

IMG_WIDTH = 640
IMG_HEIGHT = 416
CAM_CX = IMG_WIDTH // 2    # = 320
CAM_CY = IMG_HEIGHT // 2   # = 208
FX = FOCAL_LENGTH_PX
FY = FOCAL_LENGTH_PX

# --- Radius (in Pixel) für Median-Tiefenfenster ---
SAMPLE_RADIUS = 10

class TrackPoint:
    _color_counters = {c: 0 for c in COLOR_PREFIX}

    def __init__(self, x: float, y: float, z: float, color: str):
        # Zustandsvektor: [x, y, z, vx, vy, vz]
        self.kf = KalmanFilter(dim_x=6, dim_z=3)
        # State transition und Messmatrix
        self.kf.F = np.eye(6)
        self.kf.H = np.zeros((3,6)); self.kf.H[:3,:3] = np.eye(3)
        self.kf.P *= 500.
        self.kf.R *= 5.
        # Prozessrauschen
        q_pos, q_vel = 0.01, 0.1
        Q = np.zeros((6,6))
        Q[0:3,0:3] = np.eye(3)*q_pos
        Q[3:6,3:6] = np.eye(3)*q_vel
        self.kf.Q = Q
        # Initialzustand
        self.kf.x = np.zeros((6,1))
        self.kf.x[:3,0] = [x, y, z]
        self.color = color
        TrackPoint._color_counters[color] += 1
        prefix = COLOR_PREFIX.get(color,'x')
        self.id = f"{prefix}{TrackPoint._color_counters[color]:04d}"
        self.time_since_update = 0

    def predict(self, dt: float):
        # Update F für dt
        self.kf.F[0,3] = dt
        self.kf.F[1,4] = dt
        self.kf.F[2,5] = dt
        self.kf.predict()
        self.time_since_update += 1
        # Verwerfe ungültige Zustände
        if self.kf.x[2,0] < 0:
            self.kf.x[2,0] = 0.01
        return self.kf.x[:3,0].copy()

    def update(self, x: float, y: float, z: float):
        self.kf.update(np.array([x, y, z]))
        self.time_since_update = 0

class DepthTrackingNode(Node):
    def __init__(self):
        super().__init__('depth_tracking_node')
        self.bridge = CvBridge()
        self.latest_detections = None
        self.last_measurements = []
        self.trackers = []
        self.mahalanobis_threshold = 9.21
        self.last_disp_time = time.time()
        self.fps = 0.0

        # Subscriptions
        self.create_subscription(Image, '/camera/depth/disparity_raw', self.disparity_cb, 5)
        self.create_subscription(ConeArray2D, '/cone_detections_2d', self.detections_cb, 5)

        # Publishers
        self.pub_depth_overlay = self.create_publisher(Image, '/camera/depth/image_overlay', 10)
        self.pub_cones3d      = self.create_publisher(ConeArray3D, '/cone_detections_3d', 10)
        self.pub_fps          = self.create_publisher(Float32, '/depth_fps', 10)

    def detections_cb(self, msg: ConeArray2D):
        self.latest_detections = msg

    def disparity_cb(self, img_msg: Image):
        if self.latest_detections is None:
            return
        now = time.time()
        dt = now - self.last_disp_time
        self.last_disp_time = now
        if dt > 1e-6:
            self.fps = 1.0 / dt

        disp = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='passthrough')
        if disp.ndim == 3:
            disp = disp[:,:,0]
        h, w = disp.shape

        # Normiertes Bild für Overlay
        norm = (disp.astype(np.float32) / max(disp.max(),1) * 255).astype(np.uint8)
        overlay = cv2.applyColorMap(norm, cv2.COLORMAP_JET)

        measurements = []
        # 3D-Messungen um Zentrum jeder 2D-Detection
        for c in self.latest_detections.cones:
            px, py = int(round(c.x)), int(round(c.y))
            if not (0 <= px < w and 0 <= py < h):
                continue
            disp_vals = []
            for dx in range(-SAMPLE_RADIUS, SAMPLE_RADIUS+1):
                for dy in range(-SAMPLE_RADIUS, SAMPLE_RADIUS+1):
                    if dx*dx + dy*dy <= SAMPLE_RADIUS*SAMPLE_RADIUS:
                        x, y = px+dx, py+dy
                        if 0 <= x < w and 0 <= y < h:
                            dv = float(disp[y,x])
                            if dv >= MIN_DISP:
                                disp_vals.append(dv)
            if not disp_vals:
                continue
            depths = [(FOCAL_LENGTH_PX*BASELINE_M)/dv for dv in disp_vals]
            z = float(np.median(depths))
            if z <= 0:
                continue
            X = (px - CAM_CX) * z / FX
            Y = (py - CAM_CY) * z / FY
            measurements.append((X, Y, z, c.color, c, px, py))
            # Visualisiere Kreisbereich
            cv2.circle(overlay, (px,py), SAMPLE_RADIUS, (255,255,0),1)

        if not measurements and self.last_measurements:
            measurements = self.last_measurements
        self.last_measurements = measurements.copy()

        # Predict aller Tracker
        for trk in self.trackers:
            trk.predict(dt)

        # Datenassoziation & Update
        unmatched = measurements.copy()
        for trk in self.trackers:
            if trk.time_since_update > 5:
                continue
            pred = trk.kf.x[:3,0]
            S = trk.kf.H.dot(trk.kf.P).dot(trk.kf.H.T) + trk.kf.R
            best_idx, best_m = None, float('inf')
            for i,(X,Y,Z,_,orig,px,py) in enumerate(unmatched):
                innov = np.array([X,Y,Z]) - pred
                try:
                    maha = innov.T.dot(np.linalg.inv(S)).dot(innov)
                except np.linalg.LinAlgError:
                    maha = float('inf')
                if maha < best_m:
                    best_m, best_idx = maha, i
            if best_idx is not None and best_m < self.mahalanobis_threshold:
                X,Y,Z,_,orig,px,py = unmatched.pop(best_idx)
                trk.update(X,Y,Z)
                # Sofort zeichnen nach Update (nahezu Live)
                xw, yw, zw = trk.kf.x[0,0], trk.kf.x[1,0], trk.kf.x[2,0]
                if zw > 0:
                    xi = int(round((xw*FX)/zw + CAM_CX))
                    yi = int(round((yw*FY)/zw + CAM_CY))
                    cv2.putText(overlay, f"{trk.id} {zw:.2f}m", (xi+5, yi),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)
        # Neue Tracker
        for X,Y,Z,col,orig,px,py in unmatched:
            if len(self.trackers) < MAX_TRACKERS:
                self.trackers.append(TrackPoint(X,Y,Z,col))
            else:
                idx = max(range(len(self.trackers)), key=lambda i: self.trackers[i].time_since_update)
                del self.trackers[idx]
                self.trackers.append(TrackPoint(X,Y,Z,col))

        # Alte Tracker entfernen
        self.trackers = [t for t in self.trackers if t.time_since_update <= 30]

        # Publish 3D-Detections
        msg3d = ConeArray3D()
        msg3d.header = self.latest_detections.header
        for trk in self.trackers:
            xw,yw,zw = trk.kf.x[0,0], trk.kf.x[1,0], trk.kf.x[2,0]
            c3 = Cone3D()
            c3.id, c3.x, c3.y, c3.z = trk.id, xw, yw, zw
            # Label/Farbe übernehmen
            best_det, best_d = None, float('inf')
            for c in self.latest_detections.cones:
                if zw <= 0:
                    continue
                rx = (xw*FX)/zw + CAM_CX
                ry = (yw*FY)/zw + CAM_CY
                d = math.hypot(c.x-rx, c.y-ry)
                if d < best_d:
                    best_d, best_det = d, c
            if best_det and best_d < 50.0:
                c3.label, c3.conf, c3.color = best_det.label, best_det.conf, best_det.color
            else:
                c3.label, c3.conf, c3.color = '', 0.0, trk.color
            msg3d.cones.append(c3)
            # Overlay-Text
            if zw > 0:
                xi = int(round((xw*FX)/zw + CAM_CX))
                yi = int(round((yw*FY)/zw + CAM_CY))
                if 0 <= xi < w and 0 <= yi < h:
                    cv2.putText(overlay, f"{trk.id} {zw:.2f}m", (xi+5,yi), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255),1)

        self.pub_cones3d.publish(msg3d)
        # Publish Overlay
        out = self.bridge.cv2_to_imgmsg(overlay, 'bgr8')
        out.header = img_msg.header
        self.pub_depth_overlay.publish(out)
        # Publish FPS
        fps_msg = Float32(); fps_msg.data = self.fps
        self.pub_fps.publish(fps_msg)


def main():
    rclpy.init()
    node = DepthTrackingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
