"""Gazebo 깊이(32FC1, m) → 팀 API 형식(16UC1, mm) — /<cam>/depth_m → /<cam>/aligned_depth_to_color/image_raw
   측정 안 됨(inf·nan·범위 밖)은 0 (RealSense와 같은 규칙)"""
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class DepthMM(Node):
    def __init__(self):
        super().__init__('depth_mm')
        self.subs, self.pubs = [], {}
        for cam in ('pole_cam', 'gripper_cam'):
            self.pubs[cam] = self.create_publisher(Image, f'/{cam}/aligned_depth_to_color/image_raw', qos_profile_sensor_data)
            self.subs.append(self.create_subscription(Image, f'/{cam}/depth_m', lambda m, c=cam: self.on(c, m), qos_profile_sensor_data))

    def on(self, cam, m):
        d = np.frombuffer(m.data, dtype=np.float32).reshape(m.height, m.width)
        mm = np.where(np.isfinite(d) & (d > 0) & (d < 65.0), d * 1000.0, 0).round().astype(np.uint16)
        out = Image(header=m.header, height=m.height, width=m.width, encoding='16UC1', is_bigendian=0, step=m.width * 2)
        out.data = mm.tobytes()
        self.pubs[cam].publish(out)


def main():
    rclpy.init()
    try:
        rclpy.spin(DepthMM())
    except (KeyboardInterrupt, ExternalShutdownException):  # Ctrl+C · launch 종료 — 조용히 끝냄
        pass


if __name__ == '__main__':
    main()
