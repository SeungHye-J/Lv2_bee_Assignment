"""perception 기본 테스트: 영상이 없어도 노드가 떠서 빈 /tracked_objects와 진단을 낸다."""
import os
import sys
from pathlib import Path

import rclpy

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "scripts"))

from perception_node import PerceptionNode  # noqa: E402
from perception.msg import TrackedObjectArray  # noqa: E402


def test_empty_without_model():
    os.environ.setdefault("ROS_DOMAIN_ID", "101")
    rclpy.init()
    try:
        node = PerceptionNode()
        got = {}

        def cb(msg):
            got["n"] = len(msg.objects)

        node.create_subscription(TrackedObjectArray, "/tracked_objects", cb, 5)
        from diagnostic_msgs.msg import DiagnosticArray

        diag = {}

        def dcb(msg):
            diag["names"] = [s.name for s in msg.status]

        node.create_subscription(DiagnosticArray, "/diagnostics", dcb, 1)
        # 파라미터 선언 확인 (test-bringup yaml 검사와 동일 항목)
        for p in ("rate_hz", "model_path", "confidence", "imgsz", "device"):
            assert node.has_parameter(p), p
        end = node.get_clock().now() + rclpy.duration.Duration(seconds=6)
        while node.get_clock().now() < end and ("n" not in got or "names" not in diag):
            rclpy.spin_once(node, timeout_sec=0.2)
        assert got.get("n") == 0
        assert diag.get("names") == ["perception: 상태"]
    finally:
        rclpy.shutdown()
