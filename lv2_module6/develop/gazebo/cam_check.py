#!/usr/bin/env python3
"""Gazebo 카메라 확인 (ROS 없이) — Gazebo 토픽을 gz.transport로 직접 구독해 RGB·깊이 값을 보고, 로봇도 키보드로 몬다

  준비 (develop/ 스크립트 그대로):
    lv2_module6/develop/gazebo.sh -r          # 맵 (-r = 바로 재생. 안 붙였으면 Gazebo 화면 왼쪽 아래 ▶)
    lv2_module6/develop/gazebo_spawn.sh       # 로봇 (URDF의 RGB-D 카메라 2대 · DiffDrive 포함)
  실행:
    python3 lv2_module6/develop/gazebo/cam_check.py                  # pole_cam
    python3 lv2_module6/develop/gazebo/cam_check.py --cam gripper_cam

  키: w 전진 · s 후진 · a 왼쪽 회전 · d 오른쪽 회전 · space 정지 · c 사진 저장 · q 종료
  출력 (0.5초마다): 컬러·깊이 주기(Hz) · 화면 5곳 거리(mm) · 깊이 측정된 비율(%)
  - 카메라 토픽: URDF(urdf/default)의 <topic><cam>/rgbd</topic> → /<cam>/rgbd/image (RGB) · /<cam>/rgbd/depth_image (m, float32)
  - 주행 토픽: URDF DiffDrive의 /model/beebot/cmd_vel (gz.msgs.Twist)
  - 깊이는 RealSense·팀 API처럼 mm로 보여 준다. 못 잰 곳(inf·nan, 0.1m보다 가까움)은 0
  - 사진: 이 폴더 captures/ (gitignore) — <시각>_<cam>.png (컬러) · _depth.png (16비트 mm)
  필요: ROS 2 Lyrical의 gz 파이썬 라이브러리 (source /opt/ros/lyrical/setup.bash 하면 잡힘) · numpy · opencv
"""
import argparse
import os
import select
import sys
import termios
import threading
import time
import tty
from pathlib import Path

import cv2
import numpy as np
from gz.msgs.image_pb2 import Image
from gz.msgs.twist_pb2 import Twist
from gz.transport import Node

LIN, ANG = 0.15, 0.6  # 키 한 번에 m/s · rad/s (DiffDrive 한계 0.5m/s)
POINTS = {'가운데': (0.5, 0.5), '위': (0.25, 0.5), '아래': (0.75, 0.5), '왼쪽': (0.5, 0.25), '오른쪽': (0.5, 0.75)}


class CamCheck:
    def __init__(self, cam, model):
        self.cam = cam
        self.lock = threading.Lock()
        self.color = self.depth = None
        self.n = {'color': 0, 'depth': 0}
        self.node = Node()
        for kind, topic in (('color', f'/{cam}/rgbd/image'), ('depth', f'/{cam}/rgbd/depth_image')):
            if not self.node.subscribe(Image, topic, lambda m, k=kind: self.on_image(k, m)):
                sys.exit(f'구독 실패: {topic}')
        self.cmd = self.node.advertise(f'/model/{model}/cmd_vel', Twist)
        self.vx = self.wz = 0.0

    def on_image(self, kind, m):
        if kind == 'color':
            img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3)  # RGB_INT8
        else:
            d = np.frombuffer(m.data, np.float32).reshape(m.height, m.width)  # R_FLOAT32, m
            img = np.where(np.isfinite(d) & (d > 0) & (d < 65.0), d * 1000.0, 0).round().astype(np.uint16)
        with self.lock:
            setattr(self, kind, img)
            self.n[kind] += 1

    def drive(self, key):
        if key == 'w':
            self.vx = LIN
        elif key == 's':
            self.vx = -LIN
        elif key == 'a':
            self.wz = ANG
        elif key == 'd':
            self.wz = -ANG
        elif key == ' ':
            self.vx = self.wz = 0.0
        t = Twist()
        t.linear.x, t.angular.z = self.vx, self.wz
        self.cmd.publish(t)

    def save(self):
        with self.lock:
            color, depth = self.color, self.depth
        if color is None or depth is None:
            print('  아직 영상 없음 — Gazebo가 재생(▶) 중인지 확인', flush=True)
            return
        out = Path(__file__).resolve().parent / 'captures'
        out.mkdir(exist_ok=True)
        stem = time.strftime('%Y%m%d_%H%M%S') + f'_{self.cam}'
        cv2.imwrite(str(out / f'{stem}.png'), cv2.cvtColor(color, cv2.COLOR_RGB2BGR))
        cv2.imwrite(str(out / f'{stem}_depth.png'), depth)
        print(f'  저장: {out / stem}.png · _depth.png', flush=True)

    def report(self, dt):
        with self.lock:
            depth, n = self.depth, dict(self.n)
            self.n = {'color': 0, 'depth': 0}
        hz = f'컬러 {n["color"] / dt:4.1f}Hz · 깊이 {n["depth"] / dt:4.1f}Hz'
        if depth is None:
            print(f'[{self.cam}] {hz} · 깊이 기다리는 중… (Gazebo 재생 ▶ · 로봇 스폰 확인)', flush=True)
            return
        h, w = depth.shape
        pts = ' · '.join(f'{k} {depth[int(h * fy), int(w * fx)]:5d}mm' for k, (fy, fx) in POINTS.items())
        drive = f'주행 v={self.vx:+.2f} w={self.wz:+.1f}'
        print(f'[{self.cam}] {hz} · {pts} · 유효 {100 * (depth > 0).mean():3.0f}% · {drive}', flush=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--cam', default='pole_cam', choices=['pole_cam', 'gripper_cam'])
    ap.add_argument('--model', default='beebot', help='Gazebo 모델 이름 (gazebo_spawn.sh의 -name)')
    args = ap.parse_args()
    cc = CamCheck(args.cam, args.model)
    print('키: w 전진 · s 후진 · a 왼쪽 회전 · d 오른쪽 회전 · space 정지 · c 사진 저장 · q 종료', flush=True)
    tty_in = sys.stdin.isatty()
    old = termios.tcgetattr(sys.stdin) if tty_in else None
    try:
        if tty_in:
            tty.setcbreak(sys.stdin.fileno())  # 엔터 없이 키 한 글자씩
        last = time.time()
        while True:
            if tty_in and select.select([sys.stdin], [], [], 0.05)[0]:
                key = sys.stdin.read(1).lower()
                if key == 'q':
                    break
                if key == 'c':
                    cc.save()
                else:
                    cc.drive(key)
            elif not tty_in:
                time.sleep(0.05)
            now = time.time()
            if now - last >= 0.5:
                cc.report(now - last)
                last = now
    except KeyboardInterrupt:
        pass
    finally:
        cc.drive(' ')  # 끝날 때 로봇 정지
        time.sleep(0.2)  # 정지 명령이 나갈 시간
        if old:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        sys.stdout.flush()
        os._exit(0)  # gz.transport 파이썬 정리 단계에서 세그폴트가 나서 바로 종료


if __name__ == '__main__':
    main()
