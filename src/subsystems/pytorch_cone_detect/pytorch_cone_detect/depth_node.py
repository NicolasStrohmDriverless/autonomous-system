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
from scipy.optimize import linear_sum_assignment

# --- KONSTANTEN ---
MAX_TRACKERS = 50
MIN_HITS_TO_CONFIRM = 3       # Mindestanzahl Hits bis Track als bestätigt gilt
MAX_TIME_SINCE_UPDATE = 5     # Max. Frames ohne Update bis Track gelöscht wird
VAR_THRESHOLD = 0.05          # Varianzgrenze für Tiefen-ROI

# --- STEREOKAMERA-PARAMETER ---
BASELINE_M = 0.075
FOCAL_LENGTH_PX = 870.0
MIN_DISP = 0.1
IMG_WIDTH, IMG_HEIGHT = 640, 416
CAM_CX, CAM_CY = IMG_WIDTH // 2, IMG_HEIGHT // 2
FX, FY = FOCAL_LENGTH_PX, FOCAL_LENGTH_PX
SAMPLE_RADIUS = 10            # Radius (px) für Tiefen-ROI

COLOR_PREFIX = {'blue':'b','orange':'o','red':'r','yellow':'g'}

class TrackPoint:
    _id_counter = 0

    @classmethod
    def _next_id(cls):
        cls._id_counter += 1
        return cls._id_counter

    def __init__(self, x, y, z, color):
        # Zustandsvektor: [x, y, z, vx, vy, vz]
        self.kf = KalmanFilter(dim_x=6, dim_z=3)
        self.kf.F = np.eye(6)
        self.kf.H = np.eye(3,6)
        self.kf.P *= 500.
        # Baseline Messrauschen
        self.base_R = 5.0
        self.kf.R = np.eye(3) * self.base_R
        # Prozessrauschen
        q_pos, q_vel = 0.01, 0.1
        Q = np.zeros((6,6))
        Q[:3,:3] = np.eye(3)*q_pos
        Q[3:,3:] = np.eye(3)*q_vel
        self.kf.Q = Q
        # Initialzustand
        self.kf.x = np.zeros((6,1))
        self.kf.x[:3,0] = [x, y, z]
        self.color = color
        # Eindeutige ID
        self.id = f"{COLOR_PREFIX.get(color,'x')}{TrackPoint._next_id():04d}"
        self.hits = 1
        self.confirmed = False
        self.time_since_update = 0

    def predict(self, dt):
        # F-Matrix an dt anpassen
        self.kf.F[0,3] = dt
        self.kf.F[1,4] = dt
        self.kf.F[2,5] = dt
        self.kf.predict()
        self.time_since_update += 1
        # Negative Z korrigieren
        if self.kf.x[2,0] < 0:
            self.kf.x[2,0] = 0.01
        return self.kf.x[:3,0].copy()

    def update(self, x, y, z, conf):
        # Messrauschen abhängig von Confidence anpassen
        R_adj = self.base_R * (1.0 / (conf + 1e-3))
        self.kf.R = np.eye(3) * R_adj
        self.kf.update(np.array([x, y, z]))
        self.time_since_update = 0
        self.hits += 1
        if not self.confirmed and self.hits >= MIN_HITS_TO_CONFIRM:
            self.confirmed = True

class DepthTrackingNode(Node):
    def __init__(self):
        super().__init__('depth_tracking_node')
        self.bridge = CvBridge()
        self.latest_detections = None
        self.trackers = []
        self.last_meas = []
        self.last_time = time.time()
        self.fps = 0.0

        # Subscriptions
        self.create_subscription(Image, '/camera/depth/disparity_raw', self.disparity_cb, 5)
        self.create_subscription(ConeArray2D, '/cone_detections_2d', self.detections_cb, 5)
        # Publishers
        self.pub_overlay = self.create_publisher(Image, '/camera/depth/image_overlay', 10)
        self.pub_3d      = self.create_publisher(ConeArray3D, '/cone_detections_3d', 10)
        self.pub_fps     = self.create_publisher(Float32, '/depth_fps', 10)

    def detections_cb(self, msg: ConeArray2D):
        self.latest_detections = msg

    def disparity_cb(self, img_msg: Image):
        if self.latest_detections is None:
            return
        # Zeit und FPS
        now = time.time()
        dt = now - self.last_time
        self.last_time = now
        if dt > 1e-6:
            self.fps = 1.0 / dt
        # Disparity lesen
        disp = self.bridge.imgmsg_to_cv2(img_msg, 'passthrough')
        if disp.ndim == 3:
            disp = disp[:,:,0]
        h, w = disp.shape
        # Overlay vorbereiten
        norm = (disp.astype(np.float32)/max(disp.max(),1)*255).astype(np.uint8)
        overlay = cv2.applyColorMap(norm, cv2.COLORMAP_JET)

        # --- Messungen sammeln ---
        measurements = []
        for c in self.latest_detections.cones:
            px, py = int(round(c.x)), int(round(c.y))
            if not (0 <= px < w and 0 <= py < h):
                continue
            vals = []
            for dx in range(-SAMPLE_RADIUS, SAMPLE_RADIUS+1):
                for dy in range(-SAMPLE_RADIUS, SAMPLE_RADIUS+1):
                    if dx*dx+dy*dy <= SAMPLE_RADIUS*SAMPLE_RADIUS:
                        x, y = px+dx, py+dy
                        if 0 <= x < w and 0 <= y < h:
                            dv = float(disp[y, x])
                            if dv >= MIN_DISP:
                                vals.append(dv)
            if not vals:
                continue
            depths = [(FOCAL_LENGTH_PX*BASELINE_M)/dv for dv in vals]
            # Varianz-Filter
            if np.var(depths) > VAR_THRESHOLD:
                continue
            z = float(np.median(depths))
            X = (px - CAM_CX) * z / FX
            Y = (py - CAM_CY) * z / FY
            measurements.append((X, Y, z, c.color, c.conf, px, py))
            cv2.circle(overlay, (px, py), SAMPLE_RADIUS, (255,255,0), 1)
        # Fallback
        if not measurements and self.last_meas:
            measurements = self.last_meas
        self.last_meas = measurements.copy()

        # --- Predict aller Tracker ---
        preds = [trk.predict(dt) for trk in self.trackers]

        # --- Hungarian-Datenassoziation ---
        N, M = len(self.trackers), len(measurements)
        matched = []
        if N and M:
            costs = np.full((N, M), np.inf)
            for i, trk in enumerate(self.trackers):
                S = trk.kf.H.dot(trk.kf.P).dot(trk.kf.H.T) + trk.kf.R
                for j, (X, Y, Z, *_ ) in enumerate(measurements):
                    innov = np.array([X, Y, Z]) - preds[i]
                    try:
                        costs[i,j] = innov.T.dot(np.linalg.inv(S)).dot(innov)
                    except np.linalg.LinAlgError:
                        pass
            row, col = linear_sum_assignment(costs)
            for i, j in zip(row, col):
                if costs[i,j] < trk.kf.R[0,0] * 3:
                    matched.append((i,j))
        unmatched_meas = set(range(M))
        for i, j in matched:
            unmatched_meas.discard(j)
            X, Y, z, color, conf, px, py = measurements[j]
            self.trackers[i].update(X, Y, z, conf)
            # Live overlay
            xi = int(round((X*FX)/z + CAM_CX))
            yi = int(round((Y*FY)/z + CAM_CY))
            if 0 <= xi < w and 0 <= yi < h:
                txt = f"{self.trackers[i].id} {z:.2f}m"
                cv2.putText(overlay, txt, (xi+5, yi), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)

        # --- Neue Tracker erstellen ---
        for j in unmatched_meas:
            X, Y, z, color, conf, px, py = measurements[j]
            if len(self.trackers) < MAX_TRACKERS:
                self.trackers.append(TrackPoint(X, Y, z, color))

        # --- Alte Tracker entfernen ---
        self.trackers = [t for t in self.trackers if t.time_since_update <= MAX_TIME_SINCE_UPDATE]

        # --- 3D publizieren nur bestätigter Tracks ---
        msg3d = ConeArray3D()
        msg3d.header = self.latest_detections.header
        for trk in self.trackers:
            if not trk.confirmed:
                continue
            x, y, z = trk.kf.x[0,0], trk.kf.x[1,0], trk.kf.x[2,0]
            c3 = Cone3D()
            c3.id = trk.id
            c3.x = x
            c3.y = y
            c3.z = z
            # Farbe weitergeben
            c3.color = trk.color
            # Label und Confidence bleiben leer oder 0, falls nicht benötigt
            c3.label = ''
            c3.conf = 0.0
            msg3d.cones.append(c3)
        self.pub_3d.publish(msg3d)

        # --- Overlay & FPS publishen ---
        out = self.bridge.cv2_to_imgmsg(overlay, 'bgr8')
        out.header = img_msg.header
        self.pub_overlay.publish(out)
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
