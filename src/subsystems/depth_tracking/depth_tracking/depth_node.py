#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from oak_cone_detect_interfaces.msg import ConeArray2D, ConeArray3D, Cone3D
from cv_bridge import CvBridge
import numpy as np
import cv2
from std_msgs.msg import Float32
from filterpy.kalman import KalmanFilter

COLOR_PREFIX = {
    'blue': 'b',
    'orange': 'o',
    'red': 'r',
    'yellow': 'g'
}

# --- STEREOKAMERA-PARAMETER (für OAK-D, ggf. anpassen!) ---
BASELINE_M = 0.075         # Meter (75 mm)
FOCAL_LENGTH_PX = 870.0    # Pixel (siehe OAK-D Datenblatt)
MIN_DISP = 0.1             # Minimalwert Disparität, sonst ungültig

# --- BILDGRÖßE für KAMERA-Hauptpunkt ---
IMG_WIDTH = 640
IMG_HEIGHT = 416
CAM_CX = IMG_WIDTH // 2    # = 320
CAM_CY = IMG_HEIGHT // 2   # = 208
FX = FOCAL_LENGTH_PX
FY = FOCAL_LENGTH_PX

class TrackPoint:
    _color_counters = {c: 0 for c in COLOR_PREFIX}

    def __init__(self, x, y, z, color):
        self.kf = KalmanFilter(dim_x=6, dim_z=3)
        dt = 1.0
        self.kf.F = np.array([
            [1, 0, 0, dt, 0, 0],
            [0, 1, 0, 0, dt, 0],
            [0, 0, 1, 0, 0, dt],
            [0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1]
        ], dtype=float)
        self.kf.H = np.array([
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0]
        ], dtype=float)
        self.kf.P *= 500.0
        self.kf.R *= 5.0
        q = 0.01
        self.kf.Q = np.eye(6) * q
        self.kf.x[:3, 0] = [x, y, z]
        self.color = color
        TrackPoint._color_counters[color] += 1
        prefix = COLOR_PREFIX.get(color, 'x')
        count = TrackPoint._color_counters[color]
        self.id = f"{prefix}{count:04d}"
        self.time_since_update = 0

    def predict(self):
        self.kf.predict()
        self.time_since_update += 1
        return self.kf.x[:3, 0]

    def update(self, x, y, z):
        self.kf.update([x, y, z])
        self.time_since_update = 0

class DepthTrackingNode(Node):
    def __init__(self):
        super().__init__('depth_tracking_node')
        self.bridge = CvBridge()
        self.latest_detections = None

        self.create_subscription(Image, '/camera/depth/disparity_raw', self.disparity_cb, 5)
        self.create_subscription(ConeArray2D, '/cone_detections_2d', self.detections_cb, 5)

        self.pub_depth_overlay = self.create_publisher(Image, '/camera/depth/image_overlay', 10)
        self.pub_cones3d = self.create_publisher(ConeArray3D, '/cone_detections_3d', 10)
        self.pub_fps = self.create_publisher(Float32, '/depth_fps', 10)

        self.trackers = []
        self.max_age = 5
        self.match_threshold = 50.0
        self.depth_threshold_change = 0.3

        self.last_time = time.time()
        self.fps = 0.0

    def detections_cb(self, msg: ConeArray2D):
        self.latest_detections = msg

    def disparity_cb(self, img_msg: Image):
        if self.latest_detections is None:
            return

        # compute FPS
        now = time.time()
        dt = now - self.last_time
        if dt > 0:
            self.fps = 1.0 / dt
        self.last_time = now

        # convert disparity image
        disp_img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='passthrough')
        if len(disp_img.shape) == 3:
            disp_img = disp_img[:, :, 0]
        h, w = disp_img.shape

        # visualize disparity
        disp_norm = (disp_img.astype(np.float32) / (disp_img.max() or 1.0) * 255.0).astype(np.uint8)
        overlay = cv2.applyColorMap(disp_norm, cv2.COLORMAP_JET)

        # extract measurements from 2D detections
        measurements = []
        for c in self.latest_detections.cones:
            xi, yi = int(round(c.x)), int(round(c.y))
            if 0 <= xi < w and 0 <= yi < h:
                disp = float(disp_img[yi, xi])
                if disp < MIN_DISP:
                    continue
                z = (FOCAL_LENGTH_PX * BASELINE_M) / disp
                measurements.append((xi, yi, z, c.color, c))

        # predict existing tracks
        for trk in self.trackers:
            trk.predict()

        # data association & update
        unmatched = measurements.copy()
        for trk in list(self.trackers):
            pred = trk.kf.x[:3, 0]
            dists = [np.hypot(m[0]-pred[0], m[1]-pred[1]) for m in unmatched]
            if dists and min(dists) < self.match_threshold:
                idx = int(np.argmin(dists))
                mx, my, mz, mcol, mc = unmatched.pop(idx)
                prev_z = pred[2]
                if abs(mz - prev_z)/max(prev_z, 1e-6) < self.depth_threshold_change and mcol == trk.color:
                    trk.update(mx, my, mz)
                else:
                    trk.kf.update([mx, my, prev_z])
                    trk.time_since_update = 0

        # create new trackers for unmatched measurements
        for mx, my, mz, mcol, mc in unmatched:
            self.trackers.append(TrackPoint(mx, my, mz, mcol))
        # remove old trackers
        self.trackers = [t for t in self.trackers if t.time_since_update <= self.max_age]

        # build and publish ConeArray3D
        msg3d = ConeArray3D()
        msg3d.header = self.latest_detections.header
        for trk in self.trackers:
            x_px, y_px, z = float(trk.kf.x[0]), float(trk.kf.x[1]), float(trk.kf.x[2])
            X_m = (x_px - CAM_CX) * z / FX
            Y_m = (y_px - CAM_CY) * z / FY
            Z_m = z

            c3 = Cone3D()
            c3.id = trk.id
            c3.x, c3.y, c3.z = X_m, Y_m, Z_m

            # find the best matching 2D detection
            best, bd = None, float('inf')
            for c in self.latest_detections.cones:
                d = np.hypot(c.x - x_px, c.y - y_px)
                if d < bd:
                    bd, best = d, c

            if best and bd < self.match_threshold:
                c3.label = best.label
                c3.conf = best.conf
                c3.color = best.color
            else:
                # no fresh 2D match → retain tracker color
                c3.label = ''
                c3.conf  = 0.0
                c3.color = trk.color

            msg3d.cones.append(c3)

            # draw overlay markers
            cv2.circle(overlay, (int(x_px), int(y_px)), 5, (0, 0, 255), -1)
            cv2.putText(overlay, f"{trk.id} {Z_m:.2f}m", (int(x_px)+5, int(y_px)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        self.pub_cones3d.publish(msg3d)
        img_out = self.bridge.cv2_to_imgmsg(overlay, 'bgr8')
        img_out.header = img_msg.header
        self.pub_depth_overlay.publish(img_out)

        fps_msg = Float32()
        fps_msg.data = self.fps
        self.pub_fps.publish(fps_msg)


def main():
    rclpy.init()
    node = DepthTrackingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
