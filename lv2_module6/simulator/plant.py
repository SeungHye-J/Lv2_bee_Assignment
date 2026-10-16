"""플랜트 — OpenCR + 로봇 몸 흉내 (SIL에서 실기 하드웨어 자리)

컨테이너 faker를 mock_hardware:=false로 띄우면 controller_manager(BeebotSystemHardware)가 가상 시리얼로
펌웨어 프로토콜(firmware/README.md)을 주고받는다. 가상 시리얼은 파일로 이어져 있다 (docker/entrypoint.sh):
  호스트 → OpenCR  C 명령  → debug/output/hardware/serial (이 플랜트가 읽음)
  OpenCR → 호스트  S 상태  ← debug/input/hardware/serial  (이 플랜트가 씀)
그래서 미끄럼·마찰·충돌·통신 끊김이 실기와 같은 경로로 ROS에 들어간다 — 바퀴는 도는데 덜 가면 /odom이 실제 위치와 어긋난다.

물리 (한 걸음 dt):
  바퀴: 명령 속도로 가속 (가속 한계 = ACCEL_MAX × 마찰) → 엔코더는 바퀴 회전 그대로,
        바닥으로 전해지는 속도는 (1 − 미끄럼) 만큼만 → 차동 구동으로 실제 자세 적분
  충돌: 로봇 원(반경 ROBOT_R)이 벽·바닥 장애물·사람과 겹치면 그 걸음의 이동을 막음 (bumped)
  팔·그리퍼: 관절 속도 한계(4.8 rad/s)로 목표를 따라감. 전류 = 기본 + '팔 붙잡힘'이면 과전류
  IMU: 실제 yaw · yaw rate (+ 잡음), 가속도 z = 9.81
  watchdog: 마지막 C부터 0.5초 지나면 바퀴 0 (펌웨어와 같음)
"""
import math
import os
import random
import threading
import time

ROBOT_R = 0.18           # 로봇 외곽 반경 [m] (Waffle 폭 0.306 + 상판·팔)
WHEEL_R = 0.033
WHEEL_SEP = 0.287
ACCEL_MAX = 8.0          # 바퀴 각가속도 한계 [rad/s²] (마찰 1.0일 때)
JOINT_VEL_MAX = 4.8      # XM430 [rad/s]
GRIPPER_VEL_MAX = 0.05   # [m/s]
COMMAND_TIMEOUT = 0.5
STATE_PERIOD = 0.02      # S 송신 주기 (50Hz)
ARM = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
GRIPPER = 'gripper_left_joint'


class Plant:
    def __init__(self, serial_in, serial_out, start_xy, collide):
        """collide(x, y, r) → 그 원이 무언가와 겹치면 True"""
        self.serial_in, self.serial_out = serial_in, serial_out
        self.collide = collide
        self.lock = threading.Lock()
        # 환경 (시뮬레이터 창에서 바꿈)
        self.slip = 0.0          # 0 = 미끄럼 없음, 0.5 = 바퀴가 돈 만큼의 절반만 나감
        self.friction = 1.0      # 가속 한계 배율 (낮으면 미끄러운 바닥처럼 천천히 붙음)
        self.noise = 0.01        # IMU yaw rate 잡음 [rad/s]
        self.comm_down = False   # OpenCR 통신 끊김 (C를 안 읽고 S를 안 보냄)
        self.arm_grabbed = False  # 팔 붙잡힘 (관절이 안 움직이고 과전류)
        self.reset(start_xy)
        self.offset = None       # output/hardware/serial 읽은 위치 (None = 지금 끝부터)
        self.rx = ''
        self.last_cmd = 0.0
        self.last_state = 0.0
        self.cmds = 0
        self.bumped = False
        self.running = True

    def reset(self, xy, yaw=0.0):
        with self.lock:
            self.x, self.y, self.yaw = xy[0], xy[1], yaw
            self.wheel_cmd = [0.0, 0.0]
            self.wheel_vel = [0.0, 0.0]
            self.wheel_pos = [0.0, 0.0]
            self.joint_cmd = {j: 0.0 for j in ARM + [GRIPPER]}
            self.joint_pos = {j: 0.0 for j in ARM + [GRIPPER]}
            self.joint_vel = {j: 0.0 for j in ARM + [GRIPPER]}
            self.yaw_rate = 0.0

    @property
    def alive(self):
        """최근 1초 안에 C 명령을 받았나 — controller_manager가 실기 모드로 붙어 있나"""
        return time.time() - self.last_cmd < 1.0

    # ---------- 시리얼 (파일)
    def read_commands(self):
        try:
            size = os.path.getsize(self.serial_out)
        except OSError:
            return
        if self.offset is None or size < self.offset:  # 처음 · 새 실행으로 파일이 비워짐
            self.offset = size if self.offset is None else 0
            self.rx = ''
        if size == self.offset:
            return
        with open(self.serial_out, 'rb') as f:
            f.seek(self.offset)
            chunk = f.read(min(size - self.offset, 1 << 20))
        self.offset += len(chunk)
        self.rx += chunk.decode('ascii', 'ignore')
        *lines, self.rx = self.rx.split('\n')
        for line in reversed(lines):  # 가장 최근의 완전한 C 줄만
            if line.startswith('C,'):
                try:
                    v = [float(t) for t in line[2:].split(',')]
                except ValueError:
                    continue
                if len(v) != 8:
                    continue
                with self.lock:
                    self.wheel_cmd = v[:2]
                    for j, target in zip(ARM + [GRIPPER], v[2:]):
                        self.joint_cmd[j] = target
                self.last_cmd = time.time()
                self.cmds += 1
                break

    def write_state(self):
        with self.lock:
            q = (0.0, 0.0, math.sin(self.yaw / 2), math.cos(self.yaw / 2))
            vals = [self.wheel_pos[0], self.wheel_vel[0], self.wheel_pos[1], self.wheel_vel[1]]
            for j in ARM + [GRIPPER]:
                current = (1.6 if self.arm_grabbed and j != GRIPPER else 0.15 + 0.05 * abs(self.joint_vel[j]))
                vals += [self.joint_pos[j], self.joint_vel[j], current]
            vals += [*q, 0.0, 0.0, self.yaw_rate + random.gauss(0, self.noise), 0.0, 0.0, 9.81]
        line = 'S,' + ','.join(f'{v:.5f}' for v in vals) + '\n'
        with open(self.serial_in, 'a') as f:
            f.write(line)

    # ---------- 물리
    def step(self, dt):
        with self.lock:
            stale = time.time() - self.last_cmd > COMMAND_TIMEOUT
            accel = ACCEL_MAX * self.friction * dt
            for i in (0, 1):
                target = 0.0 if stale else self.wheel_cmd[i]
                self.wheel_vel[i] += max(-accel, min(accel, target - self.wheel_vel[i]))
                self.wheel_pos[i] += self.wheel_vel[i] * dt  # 엔코더: 바퀴가 돈 만큼
            ground = [w * WHEEL_R * (1.0 - self.slip) for w in self.wheel_vel]  # 바닥으로 전해진 만큼
            v = (ground[0] + ground[1]) / 2
            w = (ground[1] - ground[0]) / WHEEL_SEP
            nx = self.x + v * math.cos(self.yaw) * dt
            ny = self.y + v * math.sin(self.yaw) * dt
            self.bumped = self.collide(nx, ny, ROBOT_R) and not self.collide(self.x, self.y, ROBOT_R)
            if not self.bumped:
                self.x, self.y = nx, ny
            self.yaw = (self.yaw + w * dt + math.pi) % (2 * math.pi) - math.pi
            self.yaw_rate = w
            for j in ARM + [GRIPPER]:
                if self.arm_grabbed and j != GRIPPER:
                    self.joint_vel[j] = 0.0
                    continue
                vmax = (GRIPPER_VEL_MAX if j == GRIPPER else JOINT_VEL_MAX) * dt
                delta = max(-vmax, min(vmax, self.joint_cmd[j] - self.joint_pos[j]))
                self.joint_pos[j] += delta
                self.joint_vel[j] = delta / dt if dt > 0 else 0.0

    def pose(self):
        with self.lock:
            return self.x, self.y, self.yaw, dict(self.joint_pos)

    def loop(self):
        t = time.time()
        while self.running:
            now = time.time()
            dt = min(now - t, 0.05)
            t = now
            if not self.comm_down:
                self.read_commands()
            self.step(dt)
            if not self.comm_down and self.alive and now - self.last_state >= STATE_PERIOD:
                self.write_state()
                self.last_state = now
            time.sleep(0.005)
