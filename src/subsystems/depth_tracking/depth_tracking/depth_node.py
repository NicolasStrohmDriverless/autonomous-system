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
ROI_RADIUS_PX = 10         # Radius um Mittelpunkt für Tiefenberechnung

# --- BILDGRÖßE für KAMERA-Hauptpunkt ---
IMG_WIDTH = 640
IMG_HEIGHT = 416
CAM_CX = IMG_WIDTH // 2    # = 320
CAM_CY = IMG_HEIGHT // 2   # = 208
FX = FOCAL_LENGTH_PX
FY = FOCAL_LENGTH_PX


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


        self.last_time = time.time()
        self.fps = 0.0

        # Confirm initialization
        self.get_logger().info('DepthTrackingNode started')

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
                x0 = max(0, xi - ROI_RADIUS_PX)
                x1 = min(w, xi + ROI_RADIUS_PX + 1)
                y0 = max(0, yi - ROI_RADIUS_PX)
                y1 = min(h, yi + ROI_RADIUS_PX + 1)
                patch = disp_img[y0:y1, x0:x1].astype(float)
                patch = patch[patch >= MIN_DISP]
                if patch.size == 0:
                    continue
                disp_med = float(np.median(patch))
                if disp_med < MIN_DISP:
                    continue
                z = (FOCAL_LENGTH_PX * BASELINE_M) / disp_med
                measurements.append((xi, yi, z, c.color, c))

        # build and publish ConeArray3D without temporal tracking
        msg3d = ConeArray3D()
        msg3d.header = self.latest_detections.header
        for xi, yi, z, col, det in measurements:
            X_m = (xi - CAM_CX) * z / FX
            Y_m = (yi - CAM_CY) * z / FY
            Z_m = z

            c3 = Cone3D()
            c3.id = det.id
            c3.x, c3.y, c3.z = X_m, Y_m, Z_m
            c3.label = det.label
            c3.conf = det.conf
            c3.color = det.color
            msg3d.cones.append(c3)

            cv2.circle(overlay, (int(xi), int(yi)), ROI_RADIUS_PX, (0, 0, 255), 1)
            cv2.putText(overlay, f"{Z_m:.2f}m", (int(xi)+5, int(yi)),
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
