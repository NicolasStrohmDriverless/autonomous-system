#!/usr/bin/env python3
"""
ROS2-Node: DepthAIDriver mit ONNX Runtime v11n_416x416.onnx (CUDA)
----------------------------------------------
Topics:
- /camera/rgb/image_raw        (sensor_msgs/Image)   Farb-Bild (640×400, bgr8)
- /camera/rgb/image_overlay    (sensor_msgs/Image)   Overlay-Bild (YOLO-Detections + FPS)
- /camera/depth/image_raw      (sensor_msgs/Image)   Disparitätsbild (Falschfarben, bgr8)
- /camera/depth/disparity_raw  (sensor_msgs/Image)   ECHTES Disparitätsbild (mono16)
- /camera/imu_raw              (sensor_msgs/Imu)     IMU-Daten
- /cone_detections_2d          (ConeArray2D)         YOLO-Kegeldetektionen als Mittelpunkte
- /inference_fps               (std_msgs/Float32)    Inferenz-FPS
"""
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
import onnxruntime as ort
import os

# Farbzuordnung
CLASS_TO_COLOR = {
    0: 'yellow', 1: 'orange', 2: 'orange',
    3: 'red', 4: 'blue',
}
COLOR_MAP = {
    'blue':   (255,   0,   0),
    'orange': (  0, 165, 255),
    'red':    (  0,   0, 255),
    'yellow': (  0, 255, 255),
}

class DepthAIDriver(Node):
    def __init__(self):
        super().__init__('depthai_driver')
        self.bridge = CvBridge()

        # Publisher
        # 1) RGB-Frame (142 bytes/pixel)
        self.pub_rgb            = self.create_publisher(Image,       '/camera/rgb/image_raw',      10)
        # 2) Overlay-Frame (YOLO + FPS)
        self.pub_overlay        = self.create_publisher(Image,       '/camera/rgb/image_overlay',  10)
        # 3) Disparitätsbild (Falschfarben, bgr8)
        self.pub_depth          = self.create_publisher(Image,       '/camera/depth/image_raw',    10)
        # 4) ECHTES Disparitätsbild (mono16)
        self.pub_disparity_raw  = self.create_publisher(Image,       '/camera/depth/disparity_raw',10)
        # 5) IMU
        self.pub_imu            = self.create_publisher(Imu,         '/camera/imu_raw',            50)
        # 6) 2D-Detections
        self.pub_det2d          = self.create_publisher(ConeArray2D, '/cone_detections_2d',        10)
        # 7) Inferenz-FPS
        self.pub_fps            = self.create_publisher(Float32,     '/inference_fps',             10)

        # DepthAI-Pipeline (unverändert)
        pipeline = dai.Pipeline()
        cam_rgb = pipeline.create(dai.node.ColorCamera)
        cam_rgb.setPreviewSize(640, 400)
        cam_rgb.setInterleaved(False)
        cam_rgb.setBoardSocket(dai.CameraBoardSocket.RGB)
        cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1200_P)
        xout_rgb = pipeline.create(dai.node.XLinkOut)
        xout_rgb.setStreamName("rgb")
        cam_rgb.preview.link(xout_rgb.input)

        mono_l = pipeline.create(dai.node.MonoCamera)
        mono_r = pipeline.create(dai.node.MonoCamera)
        for mono, sock in [(mono_l, dai.CameraBoardSocket.LEFT),
                           (mono_r, dai.CameraBoardSocket.RIGHT)]:
            mono.setResolution(dai.MonoCameraProperties.SensorResolution.THE_1200_P)
            mono.setBoardSocket(sock)
        manip_l = pipeline.create(dai.node.ImageManip)
        manip_r = pipeline.create(dai.node.ImageManip)
        manip_l.initialConfig.setResize(640, 400)
        manip_r.initialConfig.setResize(640, 400)
        mono_l.out.link(manip_l.inputImage)
        mono_r.out.link(manip_r.inputImage)
        stereo = pipeline.create(dai.node.StereoDepth)
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        manip_l.out.link(stereo.left)
        manip_r.out.link(stereo.right)
        xout_depth = pipeline.create(dai.node.XLinkOut)
        xout_depth.setStreamName("disparity")
        stereo.disparity.link(xout_depth.input)

        imu = pipeline.create(dai.node.IMU)
        imu.enableIMUSensor([dai.IMUSensor.ACCELEROMETER_RAW, dai.IMUSensor.GYROSCOPE_RAW], 100)
        imu.setBatchReportThreshold(1)
        imu.setMaxBatchReports(10)
        xout_imu = pipeline.create(dai.node.XLinkOut)
        xout_imu.setStreamName("imu")
        imu.out.link(xout_imu.input)

        self.device = dai.Device(pipeline)
        self.q_rgb   = self.device.getOutputQueue("rgb",       maxSize=4,  blocking=False)
        self.q_depth = self.device.getOutputQueue("disparity", maxSize=4,  blocking=False)
        self.q_imu   = self.device.getOutputQueue("imu",       maxSize=50, blocking=False)

        # Timer für Polling (100 Hz)
        self.create_timer(0.01, self.on_timer)

        # ONNX Runtime laden (416×416 Modell)
        resource_dir = "/home/strohmo/autonomous-system/resource"
        onnx_path = os.path.join(resource_dir, "v11n_416x416.onnx")
        so = ort.SessionOptions()
        so.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        self.session = ort.InferenceSession(
            onnx_path,
            sess_options=so,
            providers=['CUDAExecutionProvider','CPUExecutionProvider']
        )
        self.input_shape    = (416, 416)
        self.conf_threshold = 0.6
        self.iou_threshold  = 0.65

        # Tracker for 2D detections
        self.tracker = Sort(max_age=5, min_hits=1, iou_threshold=0.3)

        self.get_logger().info("DepthAIDriver: v11n_416x416.onnx geladen, starte Inferenz auf GPU")

        # Flag f\u00fcr einmaliges Logging, wenn kein Kamerabild vorhanden ist
        self.warned_no_frame = False

    def on_timer(self):
        # === 1) RGB-Frame + Detection + Overlay + 2D-Publish + FPS ===
        in_rgb = self.q_rgb.tryGet()
        if in_rgb:
            frame = in_rgb.getCvFrame()
            t = self.get_clock().now().to_msg()

            # 1.1) Raw RGB publishen
            raw_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            raw_msg.header.stamp = t
            self.pub_rgb.publish(raw_msg)

            # 1.2) Detection + Overlay + Inferenz-FPS-Berechnung
            dets, overlay, fps = self.run_yolo_and_draw(frame)
            # Update tracker with bounding boxes
            boxes = []
            for d in dets:
                if d.get('bbox'):
                    boxes.append(d['bbox'])
            boxes = np.array(boxes) if len(boxes) > 0 else np.empty((0, 4))
            tracks = self.tracker.update(boxes)

            # 1.3) FPS in Overlay einzeichnen
            fps_text = f"{fps:.1f} FPS"
            cv2.putText(
                overlay,
                fps_text,
                (10, 25),  # Oben links
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,       # Textskalierung
                (255, 255, 255),  # Weiß
                2
            )

            # 1.4) Overlay publishen
            ov_msg = self.bridge.cv2_to_imgmsg(overlay, "bgr8")
            ov_msg.header.stamp = t
            self.pub_overlay.publish(ov_msg)

            # 1.5) ConeArray2D publishen
            arr2d = ConeArray2D()
            arr2d.header.stamp = t
            for tr in tracks:
                x1, y1, x2, y2, tid = tr
                cx = (x1 + x2) / 2.0
                cy = (y1 + y2) / 2.0
                best = None
                best_iou = 0.0
                for d in dets:
                    if d.get('bbox') is None:
                        continue
                    iou_val = iou(d['bbox'], [x1, y1, x2, y2])
                    if iou_val > best_iou:
                        best_iou = iou_val
                        best = d
                c = Cone2D()
                c.id = str(int(tid))
                if best:
                    c.label = best['label']
                    c.conf = best['conf']
                    c.color = best['color']
                else:
                    c.label = ''
                    c.conf = 0.0
                    c.color = 'blue'
                c.x = float(cx)
                c.y = float(cy)
                if c.y > 0:
                    arr2d.cones.append(c)
            self.pub_det2d.publish(arr2d)
            # 1.6) Inferenz-FPS publishen
            fps_msg = Float32()
            fps_msg.data = float(fps)
            self.pub_fps.publish(fps_msg)

            if len(arr2d.cones) == 0:
                self.get_logger().error("Keine Kegel erkannt!")

            self.warned_no_frame = False
        else:
            t = self.get_clock().now().to_msg()
            arr2d = ConeArray2D()
            arr2d.header.stamp = t
            self.pub_det2d.publish(arr2d)
            if not self.warned_no_frame:
                self.get_logger().error("Noch kein Kamerabild - keine Kegel erkannt!")
                self.warned_no_frame = True

        # === 2) Disparität-Frame publishen (Falschfarben + ECHTES raw) ===
        in_depth = self.q_depth.tryGet()
        # Prüfen: mindestens einer der Abonnenten für Falschfarben oder Raw vorhanden?
        if in_depth and (self.pub_depth.get_subscription_count() > 0 or self.pub_disparity_raw.get_subscription_count() > 0):
            disp = in_depth.getFrame()  # Typischerweise uint8 oder uint16

            # --- 2.1) Falschfarbenbild (bgr8) für Visualisierung
            mx   = disp.max() or 1.0
            norm = (disp * (255.0 / mx)).astype(np.uint8)
            cm   = cv2.applyColorMap(norm, cv2.COLORMAP_JET)

            # (Optional: Hier könnte man auch die Disparity-FPS einzeichnen, falls gewünscht)

            dmsg = self.bridge.cv2_to_imgmsg(cm, "bgr8")
            dmsg.header.stamp = self.get_clock().now().to_msg()
            # Topic: /camera/depth/image_raw (Falschfarben)
            self.pub_depth.publish(dmsg)

            # --- 2.2) ECHTES Disparitätsbild (mono16) für Algorithmen/Tracking
            if disp.dtype != np.uint16:
                disp_to_publish = disp.astype(np.uint16)
            else:
                disp_to_publish = disp
            disp_msg = self.bridge.cv2_to_imgmsg(disp_to_publish, encoding="mono16")
            disp_msg.header.stamp = dmsg.header.stamp  # gleiche Zeit wie Falschfarbenbild
            # Topic: /camera/depth/disparity_raw (mono16)
            self.pub_disparity_raw.publish(disp_msg)

        # === 3) IMU-Daten publishen ===
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
        1) Resize auf 416×416 + Blob
        2) Inferenz → pred [1,C,H,W]
        3) Decode+NMS auf 416×416 → dets_small, overlay_small
        4) Overlay_small auf 640×400 hochskalieren
        5) Detections auf 640×400 umrechnen
        → return dets (Liste der Diktate), overlay (640×400×3 BGR), fps
        """
        h0, w0 = frame.shape[:2]
        small  = cv2.resize(frame, self.input_shape)

        # 1) Blob CHW
        blob = small.astype(np.float32) / 255.0
        blob = blob.transpose(2, 0, 1)[None]

        # 2) Inferenz
        t0   = self.get_clock().now().nanoseconds
        outs = self.session.run(
            None, {self.session.get_inputs()[0].name: blob}
        )
        pred = outs[0]  # (1,C,H,W) oder (1,N,6)

        # 3) Decode auf small
        dets_small, overlay_small = self.yolo_postprocess_and_draw(small, pred)

        # 4) Upscale Overlay
        overlay = cv2.resize(overlay_small, (w0, h0), interpolation=cv2.INTER_LINEAR)

        # 5) Scale Detections
        sx   = w0 / self.input_shape[0]
        sy   = h0 / self.input_shape[1]
        dets = []
        for d in dets_small:
            bbox = d.get('bbox')
            if bbox is not None:
                scaled_bbox = [bbox[0]*sx, bbox[1]*sy, bbox[2]*sx, bbox[3]*sy]
            else:
                scaled_bbox = None
            dets.append({
                'label': d['label'],
                'conf':  d['conf'],
                'cx':    d['cx'] * sx,
                'cy':    d['cy'] * sy,
                'color': d['color'],
                'bbox':  scaled_bbox
            })

        fps = 1e9 / (self.get_clock().now().nanoseconds - t0 + 1)  # Nanosec → Hz
        return dets, overlay, fps

    def yolo_postprocess_and_draw(self, img: np.ndarray, pred: np.ndarray):
        """
        pred: (1,C,H,W) oder (1,N,6)
        img: 416×416 BGR → dets + overlay auf 416×416
        Rückgabe: dets (Liste), overlay_img (416×416×3)
        """
        # --- FALL 1: bereits decoded (1,N,6) ---
        arr = np.squeeze(pred, 0)
        if arr.ndim == 2 and arr.shape[1] >= 6:
            boxes_xyxy = arr[:, :4].astype(int)
            scores     = arr[:, 4]
            cls_ids    = arr[:, 5].astype(int)
            raw_boxes  = [[int(x), int(y), int(x2 - x), int(y2 - y)]
                          for (x, y, x2, y2) in boxes_xyxy]
            keep = cv2.dnn.NMSBoxes(raw_boxes, scores.tolist(),
                                     self.conf_threshold, self.iou_threshold)
            if len(keep) == 0:
                return [], img.copy()
            keep = keep.flatten()
            out  = img.copy()
            dets = []
            for i in keep:
                x1, y1, x2, y2 = boxes_xyxy[i]
                lbl = CLASS_TO_COLOR.get(cls_ids[i], 'blue')
                col = COLOR_MAP[lbl]
                conf = float(scores[i])
                cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
                cv2.rectangle(out, (x1, y1), (x2, y2), col, 2)
                cv2.circle(   out, (int(cx), int(cy)), 4, col, -1)
                cv2.putText(out, f"{lbl} {conf:.2f}", (x1, y1 - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1)
                dets.append({'label': lbl,
                             'conf': conf,
                             'cx': cx,
                             'cy': cy,
                             'color': lbl,
                             'bbox': [x1, y1, x2, y2]})
            return dets, out

        # --- FALL 2: klassisches Grid-Format (1,C,H,W) ---
        H, W = img.shape[:2]
        flat = pred.transpose(0, 2, 3, 1).reshape(-1, pred.shape[1])

        # 1) Objectness + Klassenscore
        obj     = 1 / (1 + np.exp(-flat[:, 4]))
        cls_p   = 1 / (1 + np.exp(-flat[:, 5:]))
        cls_ids = np.argmax(cls_p, axis=1)
        cls_cf  = cls_p[np.arange(len(cls_ids)), cls_ids]
        scores  = obj * cls_cf

        # 2) Threshold
        mask = scores >= self.conf_threshold
        if not mask.any():
            return [], img.copy()
        flat    = flat[mask]
        scores  = scores[mask]
        cls_ids = cls_ids[mask]

        # 3) Grid-Koordinaten
        gx, gy = np.meshgrid(np.arange(W), np.arange(H))
        gx = gx.reshape(-1)[mask]
        gy = gy.reshape(-1)[mask]

        # 4) Decode auf 416×416
        tx, ty, tw, th = flat[:, 0], flat[:, 1], flat[:, 2], flat[:, 3]
        xc  = (1 / (1 + np.exp(-tx)) + gx) / W * W
        yc  = (1 / (1 + np.exp(-ty)) + gy) / H * H
        bw  = np.exp(tw) * (W / W)
        bh  = np.exp(th) * (H / H)

        x1 = (xc - bw / 2).astype(int)
        y1 = (yc - bh / 2).astype(int)
        x2 = (xc + bw / 2).astype(int)
        y2 = (yc + bh / 2).astype(int)

        # 5) NMS
        boxes = [[x1[i], y1[i], x2[i] - x1[i], y2[i] - y1[i]] for i in range(x1.shape[0])]
        keep  = cv2.dnn.NMSBoxes(boxes, scores.tolist(),
                                 self.conf_threshold, self.iou_threshold)
        if len(keep) == 0:
            return [], img.copy()
        keep = keep.flatten()

        # 6) Overlay und Dets
        out  = img.copy()
        dets = []
        for i in keep:
            xx1, yy1, xx2, yy2 = x1[i], y1[i], x2[i], y2[i]
            cls = int(cls_ids[i])
            lbl = CLASS_TO_COLOR.get(cls, 'blue')
            col = COLOR_MAP[lbl]
            conf= float(scores[i])
            cx, cy = (xx1 + xx2) / 2.0, (yy1 + yy2) / 2.0
            cv2.rectangle(out, (xx1, yy1), (xx2, yy2), col, 2)
            cv2.circle(   out, (int(cx), int(cy)), 4, col, -1)
            cv2.putText(out, f"{lbl} {conf:.2f}", (xx1, yy1 - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1)
            dets.append({'label': lbl,
                         'conf': conf,
                         'cx': cx,
                         'cy': cy,
                         'color': lbl,
                         'bbox': [xx1, yy1, xx2, yy2]})

        return dets, out

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
