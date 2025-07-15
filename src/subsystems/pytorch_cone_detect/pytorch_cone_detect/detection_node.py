#!/usr/bin/env python3
"""
ROS2-Node: DepthAIDriver mit Ultralytics YOLO-PyTorch (best.pt) und CUDA 12.9
----------------------------------------------
Topics:
- /camera/rgb/image_raw         (sensor_msgs/Image)   Farb-Bild (640x400, bgr8)
- /camera/rgb/image_overlay     (sensor_msgs/Image)   Overlay-Bild (Cone-Detections)
- /camera/depth/image_raw       (sensor_msgs/Image)   Disparitätsbild (falschfarben, bgr8)
- /camera/depth/disparity_raw   (sensor_msgs/Image)   ECHTES Disparitätsbild (mono16)
- /camera/imu_raw               (sensor_msgs/Imu)     IMU-Daten
- /cone_detections_2d           (ConeArray2D)         Detektionen als Mittelpunkte
"""
import os
# CUDA-Umgebung
os.environ['CUDA_HOME'] = '/usr/local/cuda'
os.environ['CUDNN_FORWARD_COMPAT_DISABLE'] = '0'

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu
from std_msgs.msg import Float32
from oak_cone_detect_interfaces.msg import Cone2D, ConeArray2D
from sort_tracking.sort import Sort, iou
from cv_bridge import CvBridge
import depthai as dai
import numpy as np
import cv2
import torch
from ultralytics import YOLO  # Ultralytics-PyTorch
import time  

# === Modell- und Hardware-Konfiguration ===
# Ort der Gewichtsdateien auf dem Fahrzeug
RESOURCE_DIR = "/home/strohmo/autonomous-system/resource"
MODEL_PATH = os.path.join(RESOURCE_DIR, "best.pt")
PREFERRED_DEVICE = 'cuda:0'  # GPU-ID für PyTorch
CONF_THRESHOLD = 0.6         # Konfidenz-Schwelle
IOU_THRESHOLD = 0.65         # NMS IoU-Schwelle

# Farbzuordnung
CLASS_TO_COLOR = {
    0: 'blue', 1: 'orange', 2: 'orange',
    3: 'red',    4: 'yellow',
}
COLOR_MAP = {
    'blue':   (255, 0,   0),
    'orange': (0,   165, 255),
    'red':    (0,   0,   255),
    'yellow': (0,   255, 255),
}

class DepthAIDriver(Node):
    def __init__(self):
        super().__init__('depthai_driver')
        self.bridge = CvBridge()

        # Publisher
        self.pub_rgb     = self.create_publisher(Image, '/camera/rgb/image_raw', 10)
        self.pub_overlay = self.create_publisher(Image, '/camera/rgb/image_overlay', 10)
        self.pub_depth   = self.create_publisher(Image, '/camera/depth/image_raw', 10)
        self.pub_disparity_raw = self.create_publisher(Image, '/camera/depth/disparity_raw', 10)  # ECHTE Disparität
        self.pub_imu     = self.create_publisher(Imu, '/camera/imu_raw', 50)
        self.pub_det2d   = self.create_publisher(ConeArray2D, '/cone_detections_2d', 10)
        self.pub_fps     = self.create_publisher(Float32, '/detection_fps', 10)

        # FPS-Variablen für YOLO-Inferencing
        self.last_inference_time = None
        self.current_fps = 0.0
        self.tracker = Sort(max_age=5, min_hits=1, iou_threshold=0.3)

        # DepthAI-Pipeline Setup
        pipeline = dai.Pipeline()
        cam_rgb = pipeline.create(dai.node.ColorCamera)
        cam_rgb.setPreviewSize(640, 416)
        cam_rgb.setInterleaved(False)
        cam_rgb.setBoardSocket(dai.CameraBoardSocket.RGB)
        cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1200_P)
        xout_rgb = pipeline.create(dai.node.XLinkOut); xout_rgb.setStreamName("rgb")
        cam_rgb.preview.link(xout_rgb.input)

        # Stereo Depth
        mono_l = pipeline.create(dai.node.MonoCamera)
        mono_r = pipeline.create(dai.node.MonoCamera)
        for mono, sock in [(mono_l, dai.CameraBoardSocket.LEFT),
                           (mono_r, dai.CameraBoardSocket.RIGHT)]:
            mono.setResolution(dai.MonoCameraProperties.SensorResolution.THE_1200_P)
            mono.setBoardSocket(sock)
        manip_l = pipeline.create(dai.node.ImageManip)
        manip_r = pipeline.create(dai.node.ImageManip)
        manip_l.initialConfig.setResize(640, 416)
        manip_r.initialConfig.setResize(640, 416)
        mono_l.out.link(manip_l.inputImage)
        mono_r.out.link(manip_r.inputImage)
        stereo = pipeline.create(dai.node.StereoDepth)
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        manip_l.out.link(stereo.left)
        manip_r.out.link(stereo.right)
        xout_depth = pipeline.create(dai.node.XLinkOut); xout_depth.setStreamName("disparity")
        stereo.disparity.link(xout_depth.input)

        # IMU
        imu = pipeline.create(dai.node.IMU)
        imu.enableIMUSensor([
            dai.IMUSensor.ACCELEROMETER_RAW,
            dai.IMUSensor.GYROSCOPE_RAW
        ], 100)
        imu.setBatchReportThreshold(1); imu.setMaxBatchReports(10)
        xout_imu = pipeline.create(dai.node.XLinkOut); xout_imu.setStreamName("imu")
        imu.out.link(xout_imu.input)

        # Device und Queues
        self.device = dai.Device(pipeline)
        self.q_rgb   = self.device.getOutputQueue("rgb", maxSize=4, blocking=False)
        self.q_depth = self.device.getOutputQueue("disparity", maxSize=4, blocking=False)
        self.q_imu   = self.device.getOutputQueue("imu", maxSize=50, blocking=False)

        # Timer
        self.create_timer(0.01, self.on_timer)

        # Ultralytics YOLO laden und Device-Fallback + CUDA-Test
        self.model = YOLO(MODEL_PATH, verbose=False)
        self.device_name = PREFERRED_DEVICE

        # ---- CUDA/GPU CHECK & LOGGING ----
        cuda_avail = torch.cuda.is_available()
        torch_version = torch.__version__
        cuda_version = torch.version.cuda if hasattr(torch.version, 'cuda') else 'n/a'
        if cuda_avail:
            try:
                self.model.to(self.device_name)
                torch_device = torch.device(self.device_name)
                device_name = torch.cuda.get_device_name(torch_device)
                # Test-Tensor, um sicherzustellen, dass Device funktioniert
                test_tensor = torch.rand(1).to(torch_device)
                self.get_logger().info(
                    f"CUDA verfügbar: {device_name} (torch {torch_version}, CUDA {cuda_version})"
                )
            except Exception as e:
                self.get_logger().warn(
                    f"Fehler bei Modell-Transfer auf CUDA: {e}. Nutze CPU."
                )
                self.device_name = 'cpu'
                self.model.to(self.device_name)
        else:
            self.get_logger().warn(
                f"Keine CUDA-GPU erkannt! Nutze stattdessen CPU. (torch {torch_version}, CUDA {cuda_version})"
            )
            self.device_name = 'cpu'
            self.model.to(self.device_name)

        # Abschließendes Device-Logging
        self.get_logger().info(
            f"DepthAIDriver: YOLO-Modell geladen von {MODEL_PATH} auf {self.device_name}"
        )

        # Status Flag f\u00fcr Logging, wenn noch kein Kamerabild vorhanden ist
        self.warned_no_frame = False

    def on_timer(self):
        # RGB
        in_rgb = self.q_rgb.tryGet()
        if in_rgb:
            frame = in_rgb.getCvFrame()
            t = self.get_clock().now().to_msg()

            # Raw RGB
            raw_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            raw_msg.header.stamp = t
            self.pub_rgb.publish(raw_msg)

            # Detection + Overlay
            dets, overlay, fps = self.run_yolo_and_draw(frame)
            boxes = np.array([d['bbox'] for d in dets if d.get('bbox') is not None])
            if boxes.size == 0:
                boxes = np.empty((0,4))
            tracks = self.tracker.update(boxes)

            # FPS-Overlay direkt ins Bild (optional, kann entfernt werden)
            cv2.putText(overlay, f"FPS: {fps:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,255,0), 2)

            # Publish Overlay
            ov_msg = self.bridge.cv2_to_imgmsg(overlay, "bgr8")
            ov_msg.header.stamp = t
            self.pub_overlay.publish(ov_msg)

            arr2d = ConeArray2D()
            arr2d.header.stamp = t
            for tr in tracks:
                x1,y1,x2,y2,tid = tr
                cx = (x1+x2)/2.0
                cy = (y1+y2)/2.0
                best=None
                best_i=0.0
                for d in dets:
                    if d.get('bbox') is None:
                        continue
                    iv = iou(d['bbox'], [x1,y1,x2,y2])
                    if iv>best_i:
                        best_i=iv
                        best=d
                c = Cone2D()
                c.id = str(int(tid))
                if best:
                    c.label=best['label']
                    c.conf=best['conf']
                    c.color=best['color']
                else:
                    c.label=''
                    c.conf=0.0
                    c.color='blue'
                c.x=float(cx)
                c.y=float(cy)
                arr2d.cones.append(c)
            self.pub_det2d.publish(arr2d)

            # Publish FPS als Topic
            fps_msg = Float32()
            fps_msg.data = float(fps)
            self.pub_fps.publish(fps_msg)

            if len(arr2d.cones) == 0:
                self.get_logger().error("Keine Kegel erkannt!")

            self.warned_no_frame = False
        else:
            # Kein Kameraframe verf\u00fcgbar, dennoch leeres Array publizieren
            t = self.get_clock().now().to_msg()
            arr2d = ConeArray2D()
            arr2d.header.stamp = t
            self.pub_det2d.publish(arr2d)
            if not self.warned_no_frame:
                self.get_logger().error("Noch kein Kamerabild - keine Kegel erkannt!")
                self.warned_no_frame = True

        # Disparity
        in_depth = self.q_depth.tryGet()
        if in_depth and (self.pub_depth.get_subscription_count() > 0 or self.pub_disparity_raw.get_subscription_count() > 0):
            disp = in_depth.getFrame()  # Typischerweise uint8 oder uint16

            # --- Falschfarbenbild (bgr8) für Visualisierung
            mx = disp.max() or 1.0
            norm = (disp * (255.0/mx)).astype(np.uint8)
            cm = cv2.applyColorMap(norm, cv2.COLORMAP_JET)
            dmsg = self.bridge.cv2_to_imgmsg(cm, "bgr8")
            dmsg.header.stamp = self.get_clock().now().to_msg()
            self.pub_depth.publish(dmsg)

            # --- ECHTES Disparitätsbild (mono16) für Algorithmen/Tracking
            if disp.dtype != np.uint16:
                disp_to_publish = disp.astype(np.uint16)
            else:
                disp_to_publish = disp
            disp_msg = self.bridge.cv2_to_imgmsg(disp_to_publish, encoding="mono16")
            disp_msg.header.stamp = dmsg.header.stamp  # gleiche Zeit wie Overlay
            self.pub_disparity_raw.publish(disp_msg)

        # IMU
        if self.pub_imu.get_subscription_count() > 0:
            imu_packets = self.q_imu.tryGet()
            if imu_packets:
                for p in imu_packets.packets:
                    im = Imu()
                    im.header.stamp = self.get_clock().now().to_msg()
                    im.linear_acceleration.x = p.acceleroMeter.x
                    im.linear_acceleration.y = p.acceleroMeter.y
                    im.linear_acceleration.z = p.acceleroMeter.z
                    im.angular_velocity.x    = p.gyroscope.x
                    im.angular_velocity.y    = p.gyroscope.y
                    im.angular_velocity.z    = p.gyroscope.z
                    self.pub_imu.publish(im)

    def run_yolo_and_draw(self, frame: np.ndarray):
        """
        Führt YOLO-Erkennung mit Ultralytics durch und erzeugt Overlay und misst FPS
        """
        # --- FPS-Messung ---
        t_now = time.time()
        if self.last_inference_time is not None:
            dt = t_now - self.last_inference_time
            if dt > 0.0:
                self.current_fps = 1.0 / dt
        self.last_inference_time = t_now

        h0, w0 = frame.shape[:2]
        try:
            results = self.model(frame, device=self.device_name, conf=CONF_THRESHOLD, iou=IOU_THRESHOLD)
        except RuntimeError as e:
            if 'no kernel image is available' in str(e).lower():
                self.get_logger().error(
                    f"CUDA inference failed: {e}. Falling back to CPU."
                )
                self.device_name = 'cpu'
                self.model.to('cpu')
                results = self.model(frame, device='cpu', conf=CONF_THRESHOLD, iou=IOU_THRESHOLD)
            else:
                raise
        dets = []
        overlay = frame.copy()
        for box in results[0].boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            conf = float(box.conf[0])
            cls_id = int(box.cls[0])
            label = CLASS_TO_COLOR.get(cls_id, 'blue')
            col = COLOR_MAP[label]
            cx, cy = (x1 + x2) / 2.0, (y1 + y2) / 2.0
            cv2.rectangle(overlay, (x1, y1), (x2, y2), col, 2)
            cv2.circle(overlay, (int(cx), int(cy)), 4, col, -1)
            cv2.putText(overlay, f"{label} {conf:.2f}", (x1, y1-5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1)
            dets.append({'label': label,
                         'conf': conf,
                         'cx': cx,
                         'cy': cy,
                         'color': label,
                         'bbox': [x1, y1, x2, y2]})
        # FPS aus dem Statusfeld des Objekts
        return dets, overlay, self.current_fps

def main(args=None):
    rclpy.init(args=args)
    node = DepthAIDriver()
    try:
        rclpy.spin(node)
    finally:
        node.device.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
