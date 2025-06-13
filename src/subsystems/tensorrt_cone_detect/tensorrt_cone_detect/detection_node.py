#!/usr/bin/env python3
"""
ROS2-Node: DepthAIDriver mit ONNX Runtime + TensorRT (v11n_416x416.onnx)
----------------------------------------------
Topics:
- /camera/rgb/image_raw     (sensor_msgs/Image)   Farb-Bild (640x400, bgr8)
- /camera/rgb/image_overlay (sensor_msgs/Image)   Overlay-Bild (YOLO-Detections)
- /camera/depth/image_raw   (sensor_msgs/Image)   Disparitätsbild (falschfarben, bgr8)
- /camera/imu_raw           (sensor_msgs/Imu)     IMU-Daten
- /cone_detections_2d       (ConeArray2D)         YOLO-Kegeldetektionen als Mittelpunkte
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu
from std_msgs.msg import Float32
from oak_cone_detect_interfaces.msg import Cone2D, ConeArray2D
from cv_bridge import CvBridge
import depthai as dai
import numpy as np
import cv2
import onnxruntime as ort
import tensorrt  # TensorRT library added
import os

# === GPU-Konfiguration (zum globalen Anpassen) ===
# GPU ID und Provider-Optionen können hier verändert werden
USE_TENSORRT = True
TRT_MAX_WORKSPACE_SIZE = 3 * (1 << 30)  # 3 GB
TRT_FP16_ENABLE = True        
TRT_DEVICE_ID = 0                      
CUDA_DEVICE_ID = 0                      

# Farbzuordnung
CLASS_TO_COLOR = {
    0: 'yellow', 1: 'orange', 2: 'orange',
    3: 'red', 4: 'blue',
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

        # Publisher Definitions
        self.pub_rgb     = self.create_publisher(Image,       '/camera/rgb/image_raw',     10)
        self.pub_overlay = self.create_publisher(Image,       '/camera/rgb/image_overlay', 10)
        self.pub_depth   = self.create_publisher(Image,       '/camera/depth/disparity_raw',   10)
        self.pub_imu     = self.create_publisher(Imu,         '/camera/imu_raw',           50)
        self.pub_det2d   = self.create_publisher(ConeArray2D, '/cone_detections_2d',       10)
        self.pub_fps     = self.create_publisher(Float32,     '/inference_fps',            10)

        # DepthAI-Pipeline Setup (unverändert)
        pipeline = dai.Pipeline()
        cam_rgb = pipeline.create(dai.node.ColorCamera)
        cam_rgb.setPreviewSize(640, 400)
        cam_rgb.setInterleaved(False)
        cam_rgb.setBoardSocket(dai.CameraBoardSocket.RGB)
        cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1200_P)
        xout_rgb = pipeline.create(dai.node.XLinkOut); xout_rgb.setStreamName("rgb")
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
        xout_depth = pipeline.create(dai.node.XLinkOut); xout_depth.setStreamName("disparity")
        stereo.disparity.link(xout_depth.input)

        imu = pipeline.create(dai.node.IMU)
        imu.enableIMUSensor([dai.IMUSensor.ACCELEROMETER_RAW, dai.IMUSensor.GYROSCOPE_RAW], 100)
        imu.setBatchReportThreshold(1); imu.setMaxBatchReports(10)
        xout_imu = pipeline.create(dai.node.XLinkOut); xout_imu.setStreamName("imu")
        imu.out.link(xout_imu.input)

        self.device = dai.Device(pipeline)
        self.q_rgb   = self.device.getOutputQueue("rgb",       maxSize=4,  blocking=False)
        self.q_depth = self.device.getOutputQueue("disparity", maxSize=4,  blocking=False)
        self.q_imu   = self.device.getOutputQueue("imu",       maxSize=50, blocking=False)

        # Timer für Polling
        self.create_timer(0.01, self.on_timer)

        # ONNX Runtime Laden mit TensorRT- und CUDA-Execution-Providern
        resource_dir = "/home/strohmo/autonomous-system/.devcontainer/resource"
        onnx_path = os.path.join(resource_dir, "v11n_416x416.onnx")
        so = ort.SessionOptions()
        so.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL

        providers = []
        if USE_TENSORRT:
            trt_opts = {
                'trt_max_workspace_size': TRT_MAX_WORKSPACE_SIZE,
                'trt_fp16_enable': TRT_FP16_ENABLE,
                'device_id': TRT_DEVICE_ID
            }
            providers.append(('TensorrtExecutionProvider', trt_opts))
        # CUDA als Fallback
        providers.append(('CUDAExecutionProvider', {'device_id': CUDA_DEVICE_ID}))
        # CPU zuletzt
        providers.append(('CPUExecutionProvider', {}))

        provider_names = [p[0] for p in providers]
        provider_opts  = [p[1] for p in providers]

        self.session = ort.InferenceSession(
            onnx_path,
            sess_options=so,
            providers=provider_names,
            provider_options=provider_opts
        )
        self.input_shape   = (416, 416)
        self.conf_threshold = 0.6
        self.iou_threshold  = 0.65

        self.get_logger().info(
            f"DepthAIDriver: ONNX Runtime geladen mit Providern {provider_names}, starte Inferenz"
        )

    def on_timer(self):
        # === RGB-Frame + Detection + Overlay + 2D-Publish ===
        in_rgb = self.q_rgb.tryGet()
        if in_rgb:
            frame = in_rgb.getCvFrame()
            t = self.get_clock().now().to_msg()

            # 1) Raw RGB publishen
            raw_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            raw_msg.header.stamp = t
            self.pub_rgb.publish(raw_msg)

            # 2) Detection + Overlay
            dets, overlay, fps = self.run_yolo_and_draw(frame)

            # 3) Overlay publishen
            ov_msg = self.bridge.cv2_to_imgmsg(overlay, "bgr8")
            ov_msg.header.stamp = t
            self.pub_overlay.publish(ov_msg)

            # 4) ConeArray2D publishen
            arr2d = ConeArray2D()
            arr2d.header.stamp = t
            for i, d in enumerate(dets):
                c = Cone2D()
                c.id    = str(i)
                c.label = d['label']
                c.conf  = d['conf']
                c.x     = d['cx']
                c.y     = d['cy']
                c.color = d['color']
                arr2d.cones.append(c)
            self.pub_det2d.publish(arr2d)

            # 5) FPS publishen
            fps_msg = Float32()
            fps_msg.data = float(fps)
            self.pub_fps.publish(fps_msg)

        # === Disparity-Frame publishen ===
        in_depth = self.q_depth.tryGet()
        if in_depth and self.pub_depth.get_subscription_count()>0:
            disp = in_depth.getFrame()
            mx = disp.max() or 1.0
            norm = (disp * (255.0/mx)).astype(np.uint8)
            cm   = cv2.applyColorMap(norm, cv2.COLORMAP_JET)
            dmsg = self.bridge.cv2_to_imgmsg(cm, "bgr8")
            dmsg.header.stamp = self.get_clock().now().to_msg()
            self.pub_depth.publish(dmsg)

        # === IMU-Daten publishen ===
        if self.pub_imu.get_subscription_count()>0:
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
        Inferenz und Overlay-Erstellung
        """
        h0, w0 = frame.shape[:2]
        small = cv2.resize(frame, self.input_shape)

        # Blob CHW und Normierung
        blob = small.astype(np.float32) / 255.0
        blob = blob.transpose(2,0,1)[None]

        # Inferenz
        t0 = self.get_clock().now().nanoseconds
        outs = self.session.run(
            None, {self.session.get_inputs()[0].name: blob}
        )
        pred = outs[0]

        # Postprocess + Draw
        dets_small, overlay_small = self.yolo_postprocess_and_draw(small, pred)
        overlay = cv2.resize(overlay_small, (w0, h0), interpolation=cv2.INTER_LINEAR)

        # Skaliere Detections zurück auf Originalgröße
        sx = w0 / self.input_shape[0]
        sy = h0 / self.input_shape[1]
        dets = [{
            'label': d['label'], 'conf': d['conf'],
            'cx': d['cx']*sx, 'cy': d['cy']*sy, 'color': d['color']
        } for d in dets_small]

        fps = 1e9 / (self.get_clock().now().nanoseconds - t0 + 1)
        return dets, overlay, fps

    def yolo_postprocess_and_draw(self, img: np.ndarray, pred: np.ndarray):
        """
        pred: (1,C,H,W) oder direkt (1,N,6)
        img: 416×416 BGR → dets + overlay on 416
        """
        # --- FALL 1: bereits decoded (1,N,6) ---
        arr = np.squeeze(pred, 0)
        if arr.ndim == 2 and arr.shape[1] >= 6:
            boxes_xyxy = arr[:, :4].astype(int)
            scores     = arr[:, 4]
            cls_ids    = arr[:, 5].astype(int)
            raw_boxes = [[int(x), int(y), int(x2-x), int(y2-y)]
                         for (x,y,x2,y2) in boxes_xyxy]
            keep = cv2.dnn.NMSBoxes(raw_boxes, scores.tolist(),
                                    self.conf_threshold, self.iou_threshold)
            if len(keep) == 0:
                return [], img.copy()
            keep = keep.flatten()
            out = img.copy()
            dets = []
            for i in keep:
                x1,y1,x2,y2 = boxes_xyxy[i]
                lbl = CLASS_TO_COLOR.get(cls_ids[i], 'blue')
                col = COLOR_MAP[lbl]
                conf = float(scores[i])
                cx, cy = (x1+x2)/2, (y1+y2)/2
                cv2.rectangle(out, (x1,y1), (x2,y2), col, 2)
                cv2.circle(   out, (int(cx),int(cy)), 4, col, -1)
                cv2.putText( out, f"{lbl} {conf:.2f}", (x1, y1-5),
                             cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1)
                dets.append({'label':lbl,'conf':conf,'cx':cx,'cy':cy,'color':lbl})
            return dets, out

        # --- FALL 2: klassisches Grid-Format (1,C,H,W) ---
        H, W = img.shape[:2]
        flat = pred.transpose(0,2,3,1).reshape(-1, pred.shape[1])

        # 1) Objectness + Klassenscore
        obj    = 1/(1+np.exp(-flat[:,4]))
        cls_p  = 1/(1+np.exp(-flat[:,5:]))
        cls_ids= np.argmax(cls_p, axis=1)
        cls_cf = cls_p[np.arange(len(cls_ids)), cls_ids]
        scores = obj * cls_cf

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
        tx, ty, tw, th = flat[:,0], flat[:,1], flat[:,2], flat[:,3]
        xc = (1/(1+np.exp(-tx)) + gx) / W * W
        yc = (1/(1+np.exp(-ty)) + gy) / H * H
        bw = np.exp(tw) * (W / W)
        bh = np.exp(th) * (H / H)

        x1 = (xc - bw/2).astype(int)
        y1 = (yc - bh/2).astype(int)
        x2 = (xc + bw/2).astype(int)
        y2 = (yc + bh/2).astype(int)

        # 5) NMS
        boxes = [[x1[i], y1[i], x2[i]-x1[i], y2[i]-y1[i]] for i in range(x1.shape[0])]
        keep  = cv2.dnn.NMSBoxes(boxes, scores.tolist(),
                                 self.conf_threshold, self.iou_threshold)
        if len(keep)==0:
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
            cx, cy = (xx1+xx2)/2.0, (yy1+yy2)/2.0
            cv2.rectangle(out, (xx1,yy1), (xx2,yy2), col, 2)
            cv2.circle(   out, (int(cx),int(cy)), 4, col, -1)
            cv2.putText(out, f"{lbl} {conf:.2f}", (xx1,yy1-5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1)
            dets.append({'label':lbl,'conf':conf,'cx':cx,'cy':cy,'color':lbl})

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
