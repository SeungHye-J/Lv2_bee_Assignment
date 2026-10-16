#!/usr/bin/env python3
"""🛠 perception (YOLO nano, 담당: 전승혜) — 설계: documents/ros2-design/02-sense.md

  입력: pole_cam·gripper_cam 컬러·정렬 depth·camera_info (best_effort)
  출력: /tracked_objects (perception/TrackedObjectArray, reliable) — vlm_task_planner · coordinator · grasp_scorer

  camera_info 내부 파라미터로 3D 역투영 → TF(header.stamp 시각)로 map 기준 변환 → 최근접 ID 추적.
  모델 파일이 없으면 빈 목록을 내보낸다 (진단 WARN) — test-bringup 기동 검사용.
"""
import math
import threading
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PointStamped
from perception.msg import TrackedObject, TrackedObjectArray
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformListener

CLASSES = ["puck", "goal_zone", "obstacle", "person", "traffic_light"]
# 크기 추정 실패 시 클래스 기본값 (sx, sy, sz) [m]
SIZE_FALLBACK = {
    "puck": (0.06, 0.06, 0.04),
    "goal_zone": (0.30, 0.30, 0.01),
    "obstacle": (0.20, 0.20, 0.30),
    "person": (0.40, 0.40, 1.70),
    "traffic_light": (0.14, 0.14, 0.45),
}


def sensor_qos():
    return QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                      durability=QoSDurabilityPolicy.VOLATILE,
                      history=QoSHistoryPolicy.KEEP_LAST, depth=5)


class PerceptionNode(Node):
    def __init__(self):
        super().__init__("perception")
        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("model_path", "")
        self.declare_parameter("confidence", 0.4)
        self.declare_parameter("imgsz", 416)
        self.declare_parameter("device", "cpu")
        self.rate_hz = float(self.get_parameter("rate_hz").value)
        conf = float(self.get_parameter("confidence").value)
        self.conf_th = conf
        self.imgsz = int(self.get_parameter("imgsz").value)
        self.device = str(self.get_parameter("device").value)

        model_path = str(self.get_parameter("model_path").value)
        if not model_path:
            try:
                import os

                from ament_index_python import get_package_share_directory
                cand = os.path.join(get_package_share_directory("perception"), "model", "best.pt")
                if os.path.exists(cand):
                    model_path = cand
            except Exception:  # noqa: BLE001
                pass
        self.model = None
        self.model_path = model_path
        if model_path:
            try:
                from ultralytics import YOLO
                self.model = YOLO(model_path)
                self.get_logger().info(f"YOLO 모델 로드: {model_path} (device={self.device})")
            except Exception as e:  # noqa: BLE001
                self.get_logger().warning(f"YOLO 로드 실패 ({e}) — 빈 목록 발행")
        else:
            self.get_logger().warning("model_path 없음 — 빈 목록 발행 (기동 검사용)")

        self.bridge = CvBridge()
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.lock = threading.Lock()
        self.frames = {}  # cam → dict(bgr, depth_mm, fx, fy, cx, cy, frame_id, stamp)
        self.tracks = {}  # id → dict(class_name, x, y, z, sx, sy, sz, last_seen)
        self.next_id = 1
        self.published = 0

        for cam in ("pole_cam", "gripper_cam"):
            self.create_subscription(Image, f"/{cam}/color/image_raw",
                                     lambda m, c=cam: self.on_color(c, m), sensor_qos())
            self.create_subscription(Image, f"/{cam}/aligned_depth_to_color/image_raw",
                                     lambda m, c=cam: self.on_depth(c, m), sensor_qos())
            self.create_subscription(CameraInfo, f"/{cam}/color/camera_info",
                                     lambda m, c=cam: self.on_info(c, m), sensor_qos())
        self.pub = self.create_publisher(TrackedObjectArray, "/tracked_objects", 5)
        self.diag_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 1)
        self.create_timer(1.0 / max(self.rate_hz, 0.1), self.on_publish)
        self.create_timer(0.25, self.on_infer)  # 카메라당 최대 4Hz 추론
        self.create_timer(1.0, self.on_diag)

    # ---------- 입력 캐시
    def _slot(self, cam):
        with self.lock:
            return self.frames.setdefault(cam, {})

    def on_color(self, cam, msg):
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:  # noqa: BLE001
            self.get_logger().warning(f"{cam} 변환 실패: {e}", throttle_duration_sec=5.0)
            return
        s = self._slot(cam)
        s["bgr"] = np.asarray(bgr)
        s["stamp"] = msg.header.stamp
        s["frame_id"] = msg.header.frame_id or f"{cam}_color_optical_frame"

    def on_depth(self, cam, msg):
        try:
            d = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as e:  # noqa: BLE001
            self.get_logger().warning(f"{cam} 깊이 실패: {e}", throttle_duration_sec=5.0)
            return
        s = self._slot(cam)
        s["depth_mm"] = np.asarray(d)
        s["stamp"] = msg.header.stamp

    def on_info(self, cam, msg):
        s = self._slot(cam)
        s["fx"], s["fy"] = float(msg.k[0]), float(msg.k[4])
        s["cx"], s["cy"] = float(msg.k[2]), float(msg.k[5])

    # ---------- 추론 (4Hz, 카메라당 최대 4Hz)
    def on_infer(self):
        if self.model is None:
            return
        with self.lock:
            cams = [(c, dict(s)) for c, s in self.frames.items()
                    if s.get("bgr") is not None and s.get("depth_mm") is not None
                    and s.get("fx") and time.time() - s.get("infer_t", 0.0) > 0.25]
        for cam, s in cams:
            try:
                res = self.model.predict(s["bgr"], imgsz=self.imgsz, conf=self.conf_th,
                                         device=self.device, verbose=False)[0]
            except Exception as e:  # noqa: BLE001
                self.get_logger().warning(f"추론 실패: {e}", throttle_duration_sec=5.0)
                continue
            with self.lock:
                self.frames[cam]["infer_t"] = time.time()
            self.update_tracks(cam, s, res)

    def update_tracks(self, cam, s, res):
        h, w = s["bgr"].shape[:2]
        depth_m = s["depth_mm"].astype(np.float32) / 1000.0
        now_stamp = s["stamp"]
        for b in res.boxes:
            cls_id = int(b.cls[0])
            if not 0 <= cls_id < len(CLASSES):
                continue
            x0, y0, x1, y1 = (int(v) for v in b.xyxy[0].tolist())
            x0, y0 = max(x0, 0), max(y0, 0)
            x1, y1 = min(x1, w - 1), min(y1, h - 1)
            if x1 <= x0 or y1 <= y0:
                continue
            region = depth_m[y0:y1, x0:x1]
            valid = region[(region > 0.05) & (region < 6.0)]
            if valid.size < 8:
                continue
            zm = float(np.median(valid))
            u, v = (x0 + x1) / 2.0, (y0 + y1) / 2.0
            px = (u - s["cx"]) * zm / s["fx"]
            py = (v - s["cy"]) * zm / s["fy"]
            mp = self.to_map(s["frame_id"], now_stamp, px, py, zm)
            if mp is None:
                continue
            cls = CLASSES[cls_id]
            fb = SIZE_FALLBACK[cls]
            sx = max((x1 - x0) * zm / s["fx"], 0.01)
            sy = max((y1 - y0) * zm / s["fy"], 0.01)
            tid = self.match(cls, mp[0], mp[1], mp[2])
            with self.lock:
                self.tracks[tid] = {"class_name": cls, "x": mp[0], "y": mp[1], "z": mp[2],
                                    "sx": sx, "sy": sy, "sz": fb[2],
                                    "conf": float(b.conf[0]), "last_seen": now_stamp,
                                    "camera": cam, "px": (int(u), int(v))}
        with self.lock:
            now_s = now_stamp.sec + now_stamp.nanosec * 1e-9
            for tid in [t for t, tr in self.tracks.items()
                        if now_s - (tr["last_seen"].sec + tr["last_seen"].nanosec * 1e-9) > 2.0]:
                del self.tracks[tid]

    def match(self, cls, x, y, z):
        best, best_d = None, 0.4
        with self.lock:
            items = list(self.tracks.items())
        for tid, tr in items:
            if tr["class_name"] != cls:
                continue
            d = math.dist((x, y, z), (tr["x"], tr["y"], tr["z"]))
            if d < best_d:
                best, best_d = tid, d
        if best is None:
            with self.lock:
                best = self.next_id
                self.next_id += 1
        return best

    def to_map(self, frame_id, stamp, x, y, z):
        from geometry_msgs.msg import PointStamped
        pt = PointStamped()
        pt.header.frame_id = frame_id
        pt.header.stamp = stamp
        pt.point.x, pt.point.y, pt.point.z = x, y, z
        try:
            t = self.tf.lookup_transform("map", frame_id, rclpy.time.Time(),
                                         timeout=rclpy.duration.Duration(seconds=0.2))
            out = do_transform_point(pt, t)
            return (out.point.x, out.point.y, out.point.z)
        except Exception as e:  # noqa: BLE001
            self.get_logger().warning(f"TF 실패: {e}", throttle_duration_sec=5.0)
            return None

    # ---------- 발행 (10Hz)
    def on_publish(self):
        out = TrackedObjectArray()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "map"
        with self.lock:
            items = sorted(self.tracks.items())
        for tid, tr in items:
            o = TrackedObject()
            o.id = tid
            o.class_name = tr["class_name"]
            o.confidence = float(tr.get("conf", 0.5))
            o.pose.pose.position.x = tr["x"]
            o.pose.pose.position.y = tr["y"]
            o.pose.pose.position.z = tr["z"]
            o.pose.pose.orientation.w = 1.0
            o.size.x, o.size.y, o.size.z = tr["sx"], tr["sy"], tr["sz"]
            o.last_seen = tr["last_seen"]
            o.camera = tr.get("camera", "")
            o.px = [int(v) for v in tr.get("px", (0, 0))]
            out.objects.append(o)
        self.pub.publish(out)
        with self.lock:
            self.published += 1

    def on_diag(self):
        s = DiagnosticStatus()
        s.name = "perception: 상태"
        s.hardware_id = "beebot"
        if self.model is None:
            s.level = DiagnosticStatus.WARN
            s.message = "모델 없음(기본 대기)"
        else:
            s.level = DiagnosticStatus.OK
            s.message = "인지 중"
        with self.lock:
            n = len(self.tracks)
            pub = self.published
        s.values = [KeyValue(key="추적 물체", value=str(n)),
                    KeyValue(key="발행한 /tracked_objects", value=str(pub))]
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status = [s]
        self.diag_pub.publish(msg)


def main():
    rclpy.init()
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
