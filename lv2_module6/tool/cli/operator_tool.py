#!/usr/bin/env python3
"""운영자 수동 조작 CLI.

bringup이 떠 있는 상태에서 상태 확인, 자동/수동 전환, 비상정지, 바퀴·팔·그리퍼
수동 명령, 지도 모드/지도 전환을 한 곳에서 실행한다.

  ./tool/run-cli              # 대화형 메뉴
  ./tool/run-cli status       # 한 번만 출력 (자동화용)
  ./tool/run-cli mode manual
  ./tool/run-cli teleop
  ./tool/run-cli arm set joint1=0.0 joint2=0.4
  ./tool/run-cli map save

모든 수동 명령은 먼저 `mode manual`이어야 하고, 비상정지 중에는 거부한다.
실제 명령은 기존 control/manual_control 노드가 받아 안전 체인으로 보낸다.
"""

from __future__ import annotations

import argparse
import math
import os
import re
import select
import shutil
import subprocess
import sys
import termios
import time
import tty
import unicodedata
from pathlib import Path

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import TwistStamped
from rcl_interfaces.msg import ParameterType
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String

try:
    from nav2_msgs.srv import SaveMap as SaveMapSrv
except ImportError:  # nav2_msgs가 없는 환경에서도 나머지 명령은 동작
    SaveMapSrv = None


ARM_JOINTS = ('joint1', 'joint2', 'joint3', 'joint4', 'joint5')
WHEEL_JOINTS = ('wheel_left_joint', 'wheel_right_joint')
GRIPPER_JOINT = 'gripper_left_joint'
GRIPPER_OPEN = 0.0265
GRIPPER_CLOSE = 0.0

LATCHED = QoSProfile(
    depth=10,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)
LIVE = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)

STATE_TOPICS = {
    'operation_mode': '/state/operation_mode',
    'map_mode': '/state/map_mode',
    'map_name': '/state/map_name',
    'map_list': '/state/map_list',
    'devices': '/state/devices',
    'topics': '/state/topics',
    'restartable': '/state/restartable',
    'restart': '/state/restart',
}

DIAG_LEVELS = {0: 'OK', 1: 'WARN', 2: 'ERROR', 3: 'STALE'}

_PARAM_TYPE_NAMES = {
    ParameterType.PARAMETER_NOT_SET: 'not_set',
    ParameterType.PARAMETER_BOOL: 'bool',
    ParameterType.PARAMETER_INTEGER: 'int',
    ParameterType.PARAMETER_DOUBLE: 'double',
    ParameterType.PARAMETER_STRING: 'string',
    ParameterType.PARAMETER_BYTE_ARRAY: 'byte[]',
    ParameterType.PARAMETER_BOOL_ARRAY: 'bool[]',
    ParameterType.PARAMETER_INTEGER_ARRAY: 'int[]',
    ParameterType.PARAMETER_DOUBLE_ARRAY: 'double[]',
    ParameterType.PARAMETER_STRING_ARRAY: 'string[]',
}

_BOOL_TRUE = ('true', '1', 'yes', 'y', 'on')
_BOOL_FALSE = ('false', '0', 'no', 'n', 'off')


def _parse_bool_text(text: str) -> bool:
    low = text.strip().lower()
    if low in _BOOL_TRUE:
        return True
    if low in _BOOL_FALSE:
        return False
    raise ValueError(f'불리언이 아님 (true/false/1/0): {text}')


def _param_value_to_py(ptype: int, value) -> object:
    """ParameterValue 메시지 → 파이썬 값 (표시용)."""
    if ptype == ParameterType.PARAMETER_BOOL:
        return bool(value.bool_value)
    if ptype == ParameterType.PARAMETER_INTEGER:
        return int(value.integer_value)
    if ptype == ParameterType.PARAMETER_DOUBLE:
        return float(value.double_value)
    if ptype == ParameterType.PARAMETER_STRING:
        return str(value.string_value)
    if ptype == ParameterType.PARAMETER_BYTE_ARRAY:
        return list(value.byte_array_value)
    if ptype == ParameterType.PARAMETER_BOOL_ARRAY:
        return [bool(v) for v in value.bool_array_value]
    if ptype == ParameterType.PARAMETER_INTEGER_ARRAY:
        return [int(v) for v in value.integer_array_value]
    if ptype == ParameterType.PARAMETER_DOUBLE_ARRAY:
        return [float(v) for v in value.double_array_value]
    if ptype == ParameterType.PARAMETER_STRING_ARRAY:
        return [str(v) for v in value.string_array_value]
    return None


def _parse_param_text(text: str, ptype: int):
    """입력 문자열 → 파라미터 타입에 맞는 파이썬 값 (set용)."""
    t = text.strip()
    if ptype == ParameterType.PARAMETER_BOOL:
        return _parse_bool_text(t)
    if ptype == ParameterType.PARAMETER_INTEGER:
        try:
            return int(t)
        except ValueError:
            return int(t, 0)  # 0x·0o·0b 접두사 허용
    if ptype == ParameterType.PARAMETER_DOUBLE:
        return float(t)
    if ptype == ParameterType.PARAMETER_STRING:
        return t
    if ptype in (ParameterType.PARAMETER_BOOL_ARRAY,
                 ParameterType.PARAMETER_INTEGER_ARRAY,
                 ParameterType.PARAMETER_DOUBLE_ARRAY,
                 ParameterType.PARAMETER_STRING_ARRAY,
                 ParameterType.PARAMETER_BYTE_ARRAY):
        if t == '':
            return []
        items = [x.strip() for x in t.split(',')]
        if ptype == ParameterType.PARAMETER_BOOL_ARRAY:
            return [_parse_bool_text(x) for x in items]
        if ptype == ParameterType.PARAMETER_INTEGER_ARRAY:
            return [int(x) for x in items]
        if ptype in (ParameterType.PARAMETER_DOUBLE_ARRAY, ParameterType.PARAMETER_BYTE_ARRAY):
            return [float(x) for x in items]
        return items
    # NOT_SET: 있는 그대로 추론
    low = t.lower()
    if low in _BOOL_TRUE:
        return True
    if low in _BOOL_FALSE:
        return False
    try:
        return int(t)
    except ValueError:
        pass
    try:
        return int(t, 0)
    except ValueError:
        pass
    try:
        return float(t)
    except ValueError:
        pass
    return t

# 대화형 메뉴 반환값
BACK = 'back'
MAIN = 'main'
QUIT = 'quit'
STAY = 'stay'
REFRESH = 'refresh'
TIMEOUT = '__timeout__'  # 입력 대기 시간 초과 → 화면만 다시 그림 (에러 아님)

# 메뉴 입력 대기 [s] — 이 시간 동안 입력이 없으면 헤더 상태바를 다시 그린다.
MENU_REFRESH_S = 2.0

RESET = '\033[0m'
BOLD = '\033[1m'
DIM = '\033[2m'
RED = '\033[31m'
GREEN = '\033[32m'
YELLOW = '\033[33m'
CYAN = '\033[36m'


class OperatorTool(Node):
    def __init__(self):
        super().__init__('operator_tool')

        self._values = {}
        self._emergency = False
        self._devices = []
        self._device_details = {}
        self._device_subs = {}
        self._diagnostics = {}
        self._joints = {}
        self._last_joint_time = 0.0
        self._last_update = 0.0  # 마지막 토픽 수신 시각 (monotonic) — 헤더 갱신시각 표시용
        self._param_clients = {}  # (노드명, 네임스페이스) → AsyncParameterClient 재사용
        # 상시 상태바용 (없으면 0/빈 값으로 표시, 보드 미동작 아님)
        self._tracked_count = 0
        self._task_status = ''
        self._say_last = ''

        # 대화형 메뉴 설정값
        self._ui_linear = 0.15
        self._ui_angular = 0.8
        self._ui_duration = 1.0
        self._ui_linebuf = ''  # 새로고침을 넘겨 보존되는 입력 중 줄
        self._ui_frame_size = None  # 마지막 전체 지우기 때 터미널 크기
        self._ui_color = sys.stdout.isatty() and os.environ.get('NO_COLOR') is None

        self._cmd_operation = self.create_publisher(String, '/cmd/operation_mode', 10)
        self._cmd_estop = self.create_publisher(Bool, '/cmd/e_stop', 10)
        self._cmd_vel = self.create_publisher(TwistStamped, '/cmd/manual/cmd_vel', 10)
        self._cmd_arm = self.create_publisher(JointState, '/cmd/manual/arm', 10)
        self._cmd_map_mode = self.create_publisher(String, '/cmd/map_mode', 10)
        self._cmd_map_name = self.create_publisher(String, '/cmd/map_name', 10)
        self._cmd_restart = self.create_publisher(String, '/cmd/restart', 10)

        for key, topic in STATE_TOPICS.items():
            self.create_subscription(
                String, topic,
                lambda msg, key=key: self._values.__setitem__(key, msg.data),
                LATCHED)
        self.create_subscription(Bool, '/state/emergency', self._on_emergency, LATCHED)
        self.create_subscription(String, '/state/devices', self._on_devices, LATCHED)
        self.create_subscription(JointState, '/joint_states', self._on_joints, 10)
        self.create_subscription(DiagnosticArray, '/diagnostics_agg', self._on_diagnostics, LIVE)
        self.create_subscription(String, '/robot_say', self._on_say, 10)
        # 커스텀 msg가 빌드 전이면 해당 줄만 비활성 (cli 동작에는 지장 없음)
        try:
            from perception.msg import TrackedObjectArray  # type: ignore
            self.create_subscription(
                TrackedObjectArray, '/tracked_objects', self._on_objects, LIVE)
        except Exception:
            pass
        try:
            from plan.msg import TaskStatus  # type: ignore
            self.create_subscription(
                TaskStatus, '/task_status', self._on_task, LATCHED)
        except Exception:
            pass

    # ------------------------------------------------------------------ ROS
    def _touch(self):
        """토픽 수신 시각 기록 — 헤더 갱신시각 표시용."""
        self._last_update = time.monotonic()

    def _on_emergency(self, msg: Bool):
        self._touch()
        self._emergency = bool(msg.data)

    def _on_devices(self, msg: String):
        self._touch()
        names = [n.strip() for n in msg.data.split(',') if n.strip()]
        self._devices = names
        # 장치 목록은 state_manager가 latched로 계속 보낸다. 새 장치가 나타나면
        # /state/devices/<이름>을 그때 구독한다 (장치가 나중에 꽂혀도 상세를 본다).
        for name in names:
            if name in self._device_subs:
                continue
            self._device_subs[name] = self.create_subscription(
                String, f'/state/devices/{name}',
                lambda detail, n=name: self._device_details.__setitem__(n, detail.data),
                LATCHED)

    def _on_joints(self, msg: JointState):
        self._touch()
        for name, position in zip(msg.name, msg.position):
            self._joints[name] = position
        self._last_joint_time = time.monotonic()

    def _on_diagnostics(self, msg: DiagnosticArray):
        self._touch()
        for status in msg.status:
            level = status.level
            # DiagnosticStatus.level은 byte 타입이라 rclpy에서 bytes로 온다.
            if isinstance(level, (bytes, bytearray)):
                level = level[0] if len(level) else 0
            else:
                level = int(level)
            self._diagnostics[status.name] = (level, status.message)

    def _on_say(self, msg: String):
        self._touch()
        self._say_last = msg.data

    def _on_objects(self, msg):
        self._touch()
        try:
            self._tracked_count = len(msg.objects)
        except Exception:
            self._tracked_count = 0

    def _on_task(self, msg):
        self._touch()
        names = {0: 'RUNNING', 1: 'SUCCEEDED', 2: 'FAILED'}
        try:
            state = names.get(int(msg.state), str(msg.state))
            self._task_status = f'plan={msg.plan_id} step={msg.step} {state} {msg.reason}'
        except Exception:
            self._task_status = ''

    def _poll(self, seconds: float = 0.2):
        """콜백을 잠깐 돌려 상태를 갱신한다."""
        end = time.monotonic() + max(0.0, seconds)
        while time.monotonic() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=min(0.05, max(0.01, end - time.monotonic())))

    def _publish(self, publisher, msg):
        self._poll(0.2)  # DDS discovery
        count = publisher.get_subscription_count()
        if count == 0:
            print(f'WARN: {publisher.topic_name} 구독자 없음 — bringup이 떠 있는지 확인', file=sys.stderr)
        publisher.publish(msg)
        self._poll(0.15)

    def _wait_value(self, key: str, expected: str, timeout: float = 5.0) -> bool:
        end = time.monotonic() + timeout
        while time.monotonic() < end and rclpy.ok():
            self._poll(0.1)
            if self._values.get(key, '').upper() == expected.upper():
                return True
        return False

    def _wait_emergency(self, expected: bool, timeout: float = 2.0) -> bool:
        end = time.monotonic() + timeout
        while time.monotonic() < end and rclpy.ok():
            self._poll(0.1)
            if self._emergency == expected:
                return True
        return False

    # -------------------------------------------------------------- 상태/안전
    @property
    def operation(self) -> str:
        return (self._values.get('operation_mode') or '').upper()

    @property
    def map_mode(self) -> str:
        return (self._values.get('map_mode') or '').upper()

    def _ensure_manual(self, force: bool = False) -> bool:
        # discovery가 느린 Jetson에서도 state 토픽을 놓치지 않게 최대 8초 대기
        # (status는 5초. 모드 gate는 더 길게 — 못 보면 거부라 재시도만 하면 됨)
        end = time.monotonic() + 8.0
        while time.monotonic() < end and rclpy.ok():
            self._poll(0.2)
            if self.operation:
                break
        if self._emergency:
            print('거부: 비상정지 중입니다 — 먼저 `estop off`', file=sys.stderr)
            return False
        op = self.operation
        if op != 'MANUAL':
            if not force:
                print(f'거부: 현재 조작 모드 {op or "알 수 없음"} — 먼저 `mode manual`', file=sys.stderr)
                return False
            print(f'WARN: 현재 {op or "알 수 없음"} 모드 — --force로 명령을 보냅니다', file=sys.stderr)
        return True

    def _ensure_arm(self, force: bool = False) -> bool:
        if not self._ensure_manual(force):
            return False
        if self.map_mode == 'MAPPING' and not force:
            print('거부: MAPPING 중에는 팔 명령을 manual_control이 거절합니다 '
                  '(스캔 자세 고정) — gripper만 조작 가능', file=sys.stderr)
            return False
        return True

    def _ensure_manual_or_switch(self, force: bool = False) -> bool:
        """수동 명령 전 모드 확보. TTY면 MANUAL 전환을 물어보고 자동 전환한다.

        시뮬레이터("마지막 행동이 모드다")와 같은 사용감. 자동화(비TTY)·--force는
        기존 동작 유지 (거부 또는 확인 생략 — manual_control이 최종 판단).
        """
        if force:
            return self._ensure_manual(True)
        if self._ensure_manual(False):
            return True
        if not sys.stdin.isatty():
            return False
        if not self._ui_yes_no('MANUAL 모드로 전환할까요?', default=True):
            return False
        self.cmd_mode(argparse.Namespace(mode='manual'))
        return self._ensure_manual(False)

    def _ensure_arm_or_switch(self, force: bool = False) -> bool:
        if force:
            return self._ensure_arm(True)
        if self._ensure_arm(False):
            return True
        if not sys.stdin.isatty():
            return False
        if self.map_mode == 'MAPPING':
            return False  # MAPPING 중 팔 거부는 전환으로 해결 안 됨 (위에서 안내됨)
        if not self._ui_yes_no('MANUAL 모드로 전환할까요?', default=True):
            return False
        self.cmd_mode(argparse.Namespace(mode='manual'))
        return self._ensure_arm(False)

    # -------------------------------------------------------------- 명령 전송
    def _send_velocity(self, linear: float, angular: float):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_footprint'
        msg.twist.linear.x = float(linear)
        msg.twist.angular.z = float(angular)
        self._cmd_vel.publish(msg)

    def _send_arm(self, targets: dict[str, float]):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(targets)
        msg.position = [float(v) for v in targets.values()]
        self._publish(self._cmd_arm, msg)

    def _send_gripper(self, position: float):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [GRIPPER_JOINT]
        msg.position = [float(position)]
        self._publish(self._cmd_arm, msg)

    def _send_map_mode(self, mode: str):
        self._publish(self._cmd_map_mode, String(data=mode))

    def _send_map_name(self, name: str):
        self._publish(self._cmd_map_name, String(data=name))

    # ---------------------------------------------------------------- status
    @staticmethod
    def _rad_deg(value: float) -> str:
        return f'{value:+.3f} rad ({math.degrees(value):+.1f}°)'

    def _wait_joints(self, timeout: float = 6.0) -> bool:
        """/joint_states가 올 때까지 대기 (Jetson discovery 느림 대응)."""
        end = time.monotonic() + timeout
        while time.monotonic() < end and rclpy.ok():
            self._poll(0.2)
            if self._joints:
                return True
        return bool(self._joints)

    def cmd_status(self, args) -> int:
        # discovery가 느린 Jetson에서도 잡히게: /state/* 가 오거나 5초까지 대기.
        # joint_states(100Hz)가 먼저 와도 state 토픽 매칭까지 기다린다.
        end = time.monotonic() + 5.0
        while time.monotonic() < end and rclpy.ok():
            self._poll(0.2)
            if self._values or self._devices:
                break
        if self._devices:
            self._poll(0.5)  # /state/devices/<이름> 상세

        if not self._values and not self._devices and not self._joints:
            print('상태 없음 — bringup이 떠 있는지 확인하세요.', file=sys.stderr)
            return 1

        print(f'조작 모드   : {self.operation or "-"}')
        print(f'지도 모드   : {self.map_mode or "-"}')
        print(f'지도 이름   : {self._values.get("map_name") or "-"}')
        print(f'비상정지    : {"ON" if self._emergency else "OFF"}')
        print(f'장치        : {", ".join(self._devices) if self._devices else "-"}')
        if self._devices:
            for name in self._devices:
                print(f'  {name:16s} {self._device_details.get(name, "(상세 없음)")}')

        maps = [n for n in (self._values.get('map_list') or '').splitlines() if n]
        if maps:
            print('지도 목록   :')
            for name in maps:
                print(f'  - {name}')

        restartable = [n for n in (self._values.get('restartable') or '').splitlines() if n]
        if restartable:
            print('재시작 가능 : ' + ', '.join(line.split()[0] for line in restartable))

        if self._joints:
            wheels = [f'{name}={self._joints[name]:.3f}' for name in WHEEL_JOINTS
                      if name in self._joints]
            arms = [f'{name}={self._joints[name]:+.3f}/{math.degrees(self._joints[name]):+.1f}°'
                    for name in (*ARM_JOINTS, GRIPPER_JOINT) if name in self._joints]
            if wheels:
                print('바퀴        : ' + '  '.join(wheels))
            if arms:
                print('관절        : ' + '  '.join(arms))
            if not wheels and not arms:
                print('관절        : (알 수 없는 관절명: ' +
                      ', '.join(sorted(self._joints)) + ')')

        if args.diagnostics:
            if not self._diagnostics:
                print('진단        : /diagnostics_agg 없음')
            else:
                print('/diagnostics_agg:')
                for name, (level, message) in sorted(self._diagnostics.items()):
                    print(f'  {name}: {DIAG_LEVELS.get(level, level)} {message}')

        if args.topics:
            print('/state/topics:')
            print(self._values.get('topics', '(없음)'))
        return 0

    # ------------------------------------------------------------------ mode
    def cmd_mode(self, args) -> int:
        target = args.mode.upper()
        if target == 'EXPLORE' and self.map_mode not in ('MAPPING', ''):
            print('WARN: EXPLORE는 지도 모드 MAPPING에서만 허용됩니다', file=sys.stderr)
        self._publish(self._cmd_operation, String(data=target))
        if self._wait_value('operation_mode', target, 2.5):
            print(f'조작 모드 → {target}')
            return 0
        print(f'요청 실패: 현재 조작 모드 {self.operation or "알 수 없음"} (state_manager 로그 확인)',
              file=sys.stderr)
        return 1

    # ----------------------------------------------------------------- estop
    def cmd_estop(self, args) -> int:
        if args.state == 'status':
            self._poll(0.5)
            print(f'비상정지: {"ON" if self._emergency else "OFF"}')
            return 0

        engaged = args.state == 'on'
        self._publish(self._cmd_estop, Bool(data=engaged))
        if engaged:
            # 정지 명령을 한 번 더 보낸다 (twist_mux 잠금과 별개로 manual_control watchdog도 즉시)
            self._send_velocity(0.0, 0.0)
        if self._wait_emergency(engaged, 2.0):
            print(f'비상정지 → {"ON" if engaged else "OFF"}')
            return 0
        print(f'요청 실패: 현재 비상정지 {"ON" if self._emergency else "OFF"}', file=sys.stderr)
        return 1

    # ------------------------------------------------------------------- drive
    def cmd_drive(self, args) -> int:
        if args.direction == 'stop':
            self._send_velocity(0.0, 0.0)
            print('바퀴 정지 명령')
            return 0
        if args.duration <= 0.0:
            print('거부: --duration은 0보다 커야 합니다', file=sys.stderr)
            return 2
        if not self._ensure_manual_or_switch(args.force):
            return 1
        if self._cmd_vel.get_subscription_count() == 0:
            print('WARN: /cmd/manual/cmd_vel 구독자 없음 — manual_control 확인', file=sys.stderr)

        linear = args.speed if args.direction == 'forward' else -args.speed if args.direction == 'back' else 0.0
        angular = args.angular if args.direction == 'left' else -args.angular if args.direction == 'right' else 0.0
        print(f'{args.direction}: linear={linear:.2f} m/s angular={angular:.2f} rad/s '
              f'for {args.duration:.2f}s')
        end = time.monotonic() + args.duration
        try:
            while time.monotonic() < end and rclpy.ok():
                self._send_velocity(linear, angular)
                self._poll(0.05)
        finally:
            self._send_velocity(0.0, 0.0)
            self._poll(0.15)
        return 0

    # ------------------------------------------------------------------ teleop
    def cmd_teleop(self, args) -> int:
        if not self._ensure_manual_or_switch(args.force):
            return 1
        if not sys.stdin.isatty():
            print('거부: teleop은 터미널에서 실행해야 합니다', file=sys.stderr)
            return 2

        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        linear = 0.0
        angular = 0.0
        lin_speed = args.linear
        ang_speed = args.angular
        last_send = 0.0
        print('수동 주행: w 전진 · x 후진 · s/space 정지 · a/d 회전 · '
              'e/c 속도▲▼ · q 종료 · h 도움말')
        try:
            tty.setcbreak(fd)
            while rclpy.ok():
                now = time.monotonic()
                if now - last_send >= 0.1:
                    self._send_velocity(linear, angular)
                    last_send = now
                ready, _, _ = select.select([sys.stdin], [], [], 0.05)
                if ready:
                    try:
                        raw = os.read(fd, 1)
                    except OSError:
                        continue
                    if raw == b'':
                        break
                    # buffered read는 select와 어긋나 키가 씹히므로 os.read 직접 사용
                    key = raw.decode('ascii', errors='ignore')
                    if not key:
                        continue
                    if key in ('q', '\x03'):
                        break
                    if key == 'w':
                        linear, angular = lin_speed, 0.0
                    elif key == 'x':
                        linear, angular = -lin_speed, 0.0
                    elif key == 'a':
                        linear, angular = 0.0, ang_speed
                    elif key == 'd':
                        linear, angular = 0.0, -ang_speed
                    elif key in ('s', ' '):
                        linear = angular = 0.0
                    elif key == 'e':
                        lin_speed = min(lin_speed + 0.05, 0.50)
                    elif key == 'c':
                        lin_speed = max(lin_speed - 0.05, 0.02)
                    elif key == 'h':
                        print('\nw 전진 · x 후진 · s/space 정지 · a/d 회전 · '
                            'e/c 속도▲▼ · q 종료\n')
                    else:
                        continue
                    print(f'linear={linear:.2f} angular={angular:.2f} '
                          f'(설정 {lin_speed:.2f} m/s)')
                self._poll(0.01)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
            self._send_velocity(0.0, 0.0)
            self._poll(0.2)
            print('\n바퀴 정지')
        return 0

    # --------------------------------------------------------------------- arm
    @staticmethod
    def _parse_targets(tokens: list[str]) -> dict[str, float]:
        targets = {}
        for token in tokens:
            if '=' not in token:
                raise ValueError(f'관절 목표는 이름=값 형식: {token}')
            name, raw = token.split('=', 1)
            name = name.strip()
            try:
                value = float(raw)
            except ValueError as exc:
                raise ValueError(f'숫자가 아님: {token}') from exc
            if not math.isfinite(value):
                raise ValueError(f'유한한 숫자가 아님: {token}')
            if name in ('all', 'arm'):
                for joint in ARM_JOINTS:
                    targets[joint] = value
            elif name in ARM_JOINTS:
                targets[name] = value
            else:
                raise ValueError(f'모르는 관절: {name} (사용: {", ".join(ARM_JOINTS)} 또는 all)')
        return targets

    def cmd_arm(self, args) -> int:
        if args.action == 'show':
            self._wait_joints()
            values = [f'{name}={self._rad_deg(self._joints[name])}' for name in ARM_JOINTS if name in self._joints]
            if not values:
                print('/joint_states 없음 — bringup/OpenCR 확인', file=sys.stderr)
                return 1
            print('  '.join(values))
            return 0

        if not self._ensure_arm_or_switch(args.force):
            return 1

        if args.action == 'home':
            targets = {name: 0.0 for name in ARM_JOINTS}
        else:
            try:
                targets = self._parse_targets(args.targets)
            except ValueError as exc:
                print(f'거부: {exc}', file=sys.stderr)
                return 2
            if not targets:
                print('거부: 관절 목표가 없습니다 (예: joint1=0.0 joint2=0.4)', file=sys.stderr)
                return 2
            if args.action == 'delta':
                self._wait_joints()
                for name, delta in list(targets.items()):
                    if name not in self._joints:
                        print(f'거부: {name} 현재 각을 모름 (/joint_states 대기)', file=sys.stderr)
                        return 1
                    targets[name] = self._joints[name] + delta

        # manual_control은 목표에 없는 관절을 /joint_states의 현재 각으로 채운다.
        # 그 관절의 현재 각이 없으면 명령 전체를 보류하므로 여기서 먼저 막는다.
        self._wait_joints()
        missing = [name for name in ARM_JOINTS if name not in targets and name not in self._joints]
        if missing:
            print('/joint_states에 없는 팔 관절: ' + ', '.join(missing) +
                  ' — bringup/OpenCR/controller_manager 확인', file=sys.stderr)
            return 1

        self._send_arm(targets)
        print('팔 목표: ' + '  '.join(f'{name}={value:.3f}' for name, value in targets.items()))
        return 0

    # ----------------------------------------------------------------- gripper
    def cmd_gripper(self, args) -> int:
        if args.action == 'show':
            self._wait_joints()
            if GRIPPER_JOINT not in self._joints:
                print('/joint_states에 gripper_left_joint 없음', file=sys.stderr)
                return 1
            print(f'{GRIPPER_JOINT}={self._joints[GRIPPER_JOINT]:.4f}')
            return 0

        if args.action == 'open':
            position = GRIPPER_OPEN
        elif args.action == 'close':
            position = GRIPPER_CLOSE
        else:
            if args.value is None:
                print('거부: gripper position에는 값이 필요합니다 (예: 0.01)', file=sys.stderr)
                return 2
            try:
                position = float(args.value)
            except ValueError:
                print(f'거부: 숫자가 아님: {args.value}', file=sys.stderr)
                return 2
            if not math.isfinite(position):
                print('거부: 유한한 숫자가 아님', file=sys.stderr)
                return 2
            if not 0.0 <= position <= GRIPPER_OPEN:
                print(f'WARN: gripper 위치 {position:.4f}은 일반 범위(0~{GRIPPER_OPEN}) 밖입니다',
                      file=sys.stderr)

        if not self._ensure_manual_or_switch(args.force):
            return 1
        self._send_gripper(position)
        print(f'그리퍼 목표: {position:.4f} m')
        return 0

    # --------------------------------------------------------------------- map
    @staticmethod
    def _default_map_dir(explicit: str | None = None) -> Path:
        if explicit:
            return Path(explicit).expanduser()
        env = os.environ.get('BEEBOT_MAP_DIR')
        if env:
            return Path(env).expanduser()
        # tool/cli/operator_tool.py → lv2_module6/ros2_ws/map
        return Path(__file__).resolve().parents[2] / 'ros2_ws' / 'map'

    @staticmethod
    def _local_maps(map_dir: Path) -> list[str]:
        if not map_dir.is_dir():
            return []
        return sorted(p.name for p in map_dir.iterdir()
                      if p.is_dir() and (p / 'rtabmap.db').is_file())

    def cmd_map(self, args) -> int:
        map_dir = self._default_map_dir(args.map_dir)

        if args.action == 'status':
            self._poll(0.6)
            print(f'지도 모드: {self.map_mode or "-"}')
            print(f'지도 이름: {self._values.get("map_name") or "-"}')
            maps = [n for n in (self._values.get('map_list') or '').splitlines() if n]
            if not maps:
                maps = self._local_maps(map_dir)
            print('지도 목록: ' + (', '.join(maps) if maps else '-'))
            return 0

        if args.action == 'list':
            self._poll(0.5)
            maps = [n for n in (self._values.get('map_list') or '').splitlines() if n]
            if not maps:
                maps = self._local_maps(map_dir)
            if maps:
                for name in maps:
                    print(name)
            else:
                print(f'지도 없음 ({map_dir})', file=sys.stderr)
            return 0

        if args.action == 'mode':
            target = args.name.upper() if args.name else ''
            if target not in ('MAPPING', 'LOCALIZATION'):
                print('거부: map mode는 mapping 또는 localization', file=sys.stderr)
                return 2
            self._send_map_mode(target)
            if self._wait_value('map_mode', target, 4.0):
                print(f'지도 모드 → {target}')
                return 0
            print(f'요청 실패: 현재 지도 모드 {self.map_mode or "알 수 없음"} '
                  '(RTAB-Map이 꺼져 있거나 전환 중)', file=sys.stderr)
            return 1

        if args.action == 'switch':
            if not args.name:
                print('거부: map switch에는 지도 이름이 필요합니다', file=sys.stderr)
                return 2
            self._send_map_name(args.name)
            if self._wait_value('map_name', args.name, 5.0):
                print(f'지도 → {args.name}')
                return 0
            print(f'요청 실패: 현재 지도 {self._values.get("map_name") or "-"} '
                  '(없는 지도이거나 RTAB-Map/state_manager 확인)', file=sys.stderr)
            return 1

        if args.action == 'save':
            return self._map_save(args, map_dir)

        print(f'모르는 map action: {args.action}', file=sys.stderr)
        return 2

    def _map_save(self, args, map_dir: Path) -> int:
        if SaveMapSrv is None:
            print('거부: nav2_msgs가 없어 map save를 사용할 수 없습니다', file=sys.stderr)
            return 2
        name = self._values.get('map_name') or ''
        if args.file:
            path = Path(args.file).expanduser()
        else:
            if not name:
                print('거부: 현재 지도 이름이 없습니다 — --file로 경로를 지정하세요', file=sys.stderr)
                return 2
            path = map_dir / name / 'map'
        if path.suffix.lower() in ('.yaml', '.pgm'):
            path = path.with_suffix('')
        path = path.resolve()
        path.parent.mkdir(parents=True, exist_ok=True)

        client = self.create_client(SaveMapSrv, '/map_saver/save_map')
        if not client.wait_for_service(timeout_sec=2.0):
            print('거부: /map_saver/save_map 서비스 없음 (RTAB-Map/지도 저장 서버 확인)', file=sys.stderr)
            self.destroy_client(client)
            return 1

        req = SaveMapSrv.Request()
        req.map_topic = args.topic
        req.map_url = path.as_uri()
        req.image_format = args.format
        req.map_mode = 'trinary'
        req.free_thresh = 0.25
        req.occupied_thresh = 0.65
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if not future.done():
            print('요청 실패: /map_saver/save_map 응답 없음', file=sys.stderr)
            self.destroy_client(client)
            return 1
        response = future.result()
        self.destroy_client(client)
        if not response or not response.result:
            print('요청 실패: map_saver가 저장하지 못했습니다', file=sys.stderr)
            return 1
        print(f'지도 저장 → {path}.pgm · {path}.yaml')
        return 0

    # ----------------------------------------------------------------- restart
    def cmd_restart(self, args) -> int:
        if not args.yes:
            print('거부: 재시작은 전체 스택에 영향을 줍니다 — 확인 후 --yes를 붙이세요',
                  file=sys.stderr)
            return 2
        data = 'all' if args.target == 'all' else f'node {args.target}'
        self._publish(self._cmd_restart, String(data=data))
        print(f'재시작 요청: {data}')
        return 0

    # ------------------------------------------------------------------- menu
    def cmd_menu(self, args) -> int:
        if not sys.stdin.isatty():
            print('거부: 대화형 메뉴는 터미널(TTY)에서 실행해야 합니다. '
                  '`./run-cli status`처럼 명령을 직접 쓰세요.', file=sys.stderr)
            return 2
        return self.run_menu()

    def _ui_paint(self, text: str, color: str) -> str:
        return f'{color}{text}{RESET}' if self._ui_color else text

    @staticmethod
    def _ui_clear():
        if sys.stdout.isatty():
            print('\033[2J\033[H', end='', flush=True)

    _ANSI_RE = re.compile(r'\033\[[0-9;]*m')

    @classmethod
    def _char_cols(cls, ch: str) -> int:
        # 터미널 표시 너비: 한글·한자(W/F)는 2칸, 나머지는 1칸
        return 2 if unicodedata.east_asian_width(ch) in ('W', 'F') else 1

    @classmethod
    def _disp_len(cls, s: str) -> int:
        return sum(cls._char_cols(ch) for ch in cls._ANSI_RE.sub('', s))

    @classmethod
    def _fit(cls, s: str, w: int) -> str:
        """표시 너비 기준 잘라내기/채우기 (ANSI 색 코드는 길이에서 뺌)."""
        w = max(1, w)
        if cls._disp_len(s) <= w:
            return s + ' ' * (w - cls._disp_len(s)) + RESET
        out: list[str] = []
        vis = 0
        pos = 0
        done = False
        for m in cls._ANSI_RE.finditer(s):
            if done:
                break
            for ch in s[pos:m.start()]:
                if vis + cls._char_cols(ch) > w - 1:
                    done = True
                    break
                out.append(ch)
                vis += cls._char_cols(ch)
            if done:
                break
            out.append(m.group())
            pos = m.end()
        if not done:
            for ch in s[pos:]:
                if vis + cls._char_cols(ch) > w - 1:
                    break
                out.append(ch)
                vis += cls._char_cols(ch)
        out.append('…')
        vis += 1
        out.append(RESET)
        return ''.join(out) + ' ' * max(0, w - vis)

    def _term_size(self) -> tuple[int, int]:
        try:
            size = shutil.get_terminal_size()
            return max(60, size.columns), max(16, size.lines)
        except Exception:
            return 80, 24

    def _ui_top_lines(self, title: str, width: int = 64) -> list[str]:
        """상단 고정 영역 6줄: 타이틀 3 + 상태바 2 + 구분선 1."""
        bold = BOLD if self._ui_color else ''
        reset = RESET if self._ui_color else ''
        lines = [
            f'{bold}{"=" * width}{reset}',
            f'{bold}  BeeBot Operator Console  ·  {title}{reset}',
            f'{bold}{"=" * width}{reset}',
        ]
        op = self.operation or '?'
        mode = self.map_mode or '?'
        op_c = {'AUTO': GREEN, 'MANUAL': YELLOW, 'EXPLORE': CYAN}.get(op, RED)
        mode_c = {'MAPPING': YELLOW, 'LOCALIZATION': GREEN}.get(mode, '')
        op_txt = self._ui_paint(op, op_c + BOLD)
        mode_txt = self._ui_paint(mode, mode_c) if mode_c else self._ui_paint(mode, DIM)
        if self._emergency:
            estop = self._ui_paint('ON', RED + BOLD)
        else:
            estop = self._ui_paint('OFF', GREEN)
        active = 0
        for name in self._devices:
            _, state, _ = self._device_info(name)
            if state == 'active':
                active += 1
        if self._devices:
            dev_txt = self._ui_paint(f'{active}/{len(self._devices)}',
                                     GREEN if active == len(self._devices) else YELLOW)
        else:
            dev_txt = self._ui_paint('0/0', DIM)
        # 갱신시각 (그리는 시점 벽시계 + 마지막 수신 뒤 경과)
        now_wall = time.strftime('%m-%d %H:%M:%S', time.localtime())
        if self._last_update <= 0:
            upd_txt = self._ui_paint('갱신 없음', YELLOW)
        else:
            age = time.monotonic() - self._last_update
            if age <= 5.0:
                upd_txt = self._ui_paint(f'갱신 {now_wall} ({age:.1f}초 전)', DIM)
            else:
                upd_txt = self._ui_paint(f'갱신 {now_wall} ({age:.0f}초 전 — 끊김?)', YELLOW)
        lines.append(f'  조작 {op_txt} | 지도 {mode_txt} | E-STOP {estop} | '
                     f'장치 {dev_txt} | {upd_txt}')
        # 2단 상시 상태바: 인지·임무·발화 (GUI 보드와 같은 내용, 1줄 요약)
        task = (self._task_status or '-')
        if len(task) > 38:
            task = task[:37] + '…'
        say = (self._say_last or '')
        if len(say) > 22:
            say = say[:21] + '…'
        diag_levels = [lv for _, (lv, _) in self._diagnostics.items() if lv >= 1]
        if not self._diagnostics:
            diag_txt = self._ui_paint('진단 -', DIM)
        elif not diag_levels:
            diag_txt = self._ui_paint(f'진단 OK({len(self._diagnostics)})', GREEN)
        else:
            worst = max(diag_levels)
            diag_txt = self._ui_paint(f'진단 주의 {len(diag_levels)}',
                                      RED + BOLD if worst >= 2 else YELLOW)
        lines.append(f'  인지 {self._tracked_count:3d}개 | 임무 {task} | {diag_txt}'
                     + (f' | 발화 {say}' if say else ''))
        if not self._values and not self._devices and not self._joints:
            lines.append(f'  {self._ui_paint("상태 없음 — bringup이 떠 있는지 확인", YELLOW)}')
        lines.append('-' * width)
        return lines

    def _ui_header(self, title: str):
        self._poll(0.35)
        self._ui_clear()
        cols, _ = self._term_size()
        width = min(64, cols)
        for line in self._ui_top_lines(title, width):
            print(self._fit(line, width))

    def _ui_hints_line(self, back: bool = True, main: bool = True) -> str:
        hints = []
        if back:
            hints.append('b) 뒤로')
        hints.append('r) 새로고침')
        if main:
            hints.append('m) 메인 메뉴')
        hints.append('q) 종료')
        return '  ' + '   '.join(hints)

    def _ui_center_summary(self) -> list[str]:
        """중앙 공용 요약: 장치·임무·관절·진단 (왼쪽이 메뉴일 때 오른쪽에 보여줌)."""
        lines = ['[상태 요약]']
        if not self._devices:
            lines.append(' 장치: - (대기 중)')
        else:
            for name in self._devices[:8]:
                kind, state, topics = self._device_info(name)
                if state == 'active':
                    mark = self._ui_paint('●', GREEN)
                elif state:
                    mark = self._ui_paint('○', RED)
                else:
                    mark = self._ui_paint('?', YELLOW)
                lines.append(f' {mark} {name} ({kind})')
            if len(self._devices) > 8:
                lines.append(f' … 외 {len(self._devices) - 8}개')
        lines.append('')
        lines.append(f' 임무: {self._task_status or "-"}')
        wheels = [f'{n}={self._joints[n]:.2f}' for n in WHEEL_JOINTS
                  if n in self._joints]
        arms = [f'{n}={self._joints[n]:+.2f}/{math.degrees(self._joints[n]):+.0f}°'
                for n in (*ARM_JOINTS, GRIPPER_JOINT) if n in self._joints]
        if wheels:
            lines.append(' 바퀴: ' + ' '.join(wheels))
        if arms:
            lines.append(' 관절: ' + ' '.join(arms[:3]))
            if len(arms) > 3:
                lines.append(' ' + ' '.join(arms[3:]))
        if not wheels and not arms:
            lines.append(' 관절: -')
        bad = [(n, lv, m) for n, (lv, m) in sorted(self._diagnostics.items()) if lv >= 1][:5]
        if not self._diagnostics:
            lines.append(' 진단: -')
        elif not bad:
            lines.append(self._ui_paint(f' 진단 OK ({len(self._diagnostics)}개)', GREEN))
        else:
            lines.append(f' 진단 주의 {len(bad)}:')
            for n, lv, m in bad:
                lines.append(f'  - {n}: {DIAG_LEVELS.get(lv, lv)} {m}')
        return lines

    def _ui_draw(self, title: str, left: list[str], center: list[str],
                 back: bool = True, main: bool = True):
        """4분할 화면: 상단 타이틀+상태바 고정 · 왼쪽 메뉴 · 중앙 내용 · 하단 힌트+입력.

        터미널 크기에 맞춰 다시 계산하므로 창을 늘리면 다음 그리기부터 넓어진다.
        폭이 좁으면(<70) 세로로 쌓는다.
        전체 지우기 없이 커서를 맨 위로 옮겨 제자리에서 다시 그려 깜빡이지 않는다
        (처음·크기 변경 때만 전체 지우기).
        """
        self._poll(0.3)
        cols, rows = self._term_size()
        size = (cols, rows)
        if cols < 70 or rows < 18:
            self._ui_clear()
            for line in self._ui_top_lines(title, cols):
                print(self._fit(line, cols))
            for line in left:
                print(self._fit(line, cols))
            for line in center:
                print(self._fit(line, cols))
            self._ui_footer(back=back, main=main)
            self._ui_frame_size = size
            return
        if self._ui_frame_size != size:
            self._ui_clear()
            self._ui_frame_size = size
        else:
            sys.stdout.write('\033[H')
            sys.stdout.flush()
        top = [self._fit(ln, cols) for ln in self._ui_top_lines(title, cols)]
        lw = min(32, max(24, cols // 3))
        cw = cols - lw - 4
        body_h = max(1, rows - 1 - len(top) - 2)
        left_lines = list(left)
        center_lines = list(center)
        if len(left_lines) > body_h:
            left_lines = left_lines[:body_h - 1] + [f'  … 외 {len(left_lines) - body_h + 1}줄']
        if len(center_lines) > body_h:
            center_lines = center_lines[:body_h - 1] + [f' … 외 {len(center_lines) - body_h + 1}줄']
        frame = list(top)
        for i in range(body_h):
            l = self._fit(left_lines[i] if i < len(left_lines) else '', lw)
            c = self._fit(center_lines[i] if i < len(center_lines) else '', cw)
            frame.append(f' {l}| {c}')
        frame.append('-' * cols)
        frame.append(self._fit(self._ui_hints_line(back=back, main=main), cols))
        # 항상 rows-1줄로 정규화 → 짧아진 프레임의 남은 줄 흔적이 남지 않는다.
        if len(frame) > rows - 1:
            frame = frame[:rows - 1]
        else:
            frame += [''] * (rows - 1 - len(frame))
        for ln in frame:
            print('\033[K' + ln if ln else '\033[K')

    def _ui_footer(self, back: bool = True, main: bool = True):
        cols, _ = self._term_size()
        print('-' * min(64, cols))
        hints = []
        if back:
            hints.append('b) 뒤로')
        hints.append('r) 새로고침')
        if main:
            hints.append('m) 메인 메뉴')
        hints.append('q) 종료')
        print('  ' + '   '.join(hints))

    def _ui_prompt(self, timeout: float | None = None) -> str:
        """메뉴 입력. timeout[s] 안에 Enter가 없으면 TIMEOUT을 돌려 화면만 다시 그림.

        터미널을 cbreak로 바꿔 한 글자씩 직접 읽고 에코한다. 그래서 새로고침이
        일어나도 입력 중이던 글이 `self._ui_linebuf`에 남아 다음 프롬프트에
        그대로 복원된다. 대기 중 0.1s씩 쪼개 _poll로 ROS 콜백을 돌리므로,
        가만히 둬도 상단 상태바가 MENU_REFRESH_S마다 다시 그려진다.
        """
        if timeout is None:
            timeout = MENU_REFRESH_S
        if not sys.stdin.isatty():
            try:
                return input('선택> ').strip().lower()
            except EOFError:
                return 'q'
        fd = sys.stdin.fileno()
        try:
            old = termios.tcgetattr(fd)
        except Exception:
            try:
                return input('선택> ').strip().lower()
            except EOFError:
                return 'q'
        buf = getattr(self, '_ui_linebuf', '')
        try:
            tty.setcbreak(fd)
            sys.stdout.write('선택> ' + buf)
            sys.stdout.flush()
            # 주의: select와 buffered sys.stdin.read를 섞으면, read가 커널에서
            # 여러 바이트를 가져가 버퍼에 숨겨 select가 못 깨운다. os.read로
            # 1바이트씩 직접 읽어야 남은 바이트가 커널에 있어 다음 select가 깬다.
            end = time.monotonic() + max(0.0, timeout)
            pbuf = bytearray()
            while True:
                remain = end - time.monotonic()
                if remain <= 0:
                    if pbuf:
                        buf += pbuf.decode('utf-8', errors='replace')
                        pbuf.clear()
                    self._ui_linebuf = buf  # 입력 중이던 글 보존 → 다음 그리기 때 복원
                    sys.stdout.write('\r\033[K')  # 줄바꿈 없이 프롬프트줄만 지움 (스크롤·깜빡임 방지)
                    sys.stdout.flush()
                    return TIMEOUT
                ready, _, _ = select.select([fd], [], [], min(0.1, remain))
                self._poll(0.05)
                if not ready:
                    continue
                try:
                    raw = os.read(fd, 1)
                except OSError:
                    continue
                if raw == b'':
                    self._ui_linebuf = ''
                    return 'q'
                if raw in (b'\n', b'\r'):
                    if pbuf:
                        buf += pbuf.decode('utf-8', errors='replace')
                        pbuf.clear()
                    sys.stdout.write('\n')
                    sys.stdout.flush()
                    self._ui_linebuf = ''
                    return buf.strip().lower()
                b0 = raw[0]
                if b0 == 0x04:  # Ctrl+D
                    if not buf and not pbuf:
                        self._ui_linebuf = ''
                        return 'q'
                    continue
                if raw in (b'\x7f', b'\x08'):  # Backspace
                    if pbuf:
                        pbuf.pop()
                    elif buf:
                        buf = buf[:-1]
                        sys.stdout.write('\b \b')
                        sys.stdout.flush()
                    continue
                if b0 == 0x15:  # Ctrl+U: 줄 지우기
                    pbuf.clear()
                    while buf:
                        buf = buf[:-1]
                        sys.stdout.write('\b \b')
                    sys.stdout.flush()
                    continue
                if b0 == 0x1b or b0 < 0x20:  # 방향키 등 이스케이프·제어문자 무시
                    continue
                pbuf += raw
                try:
                    ch = pbuf.decode('utf-8')
                except UnicodeDecodeError:
                    continue  # 멀티바이트 미완성 → 다음 바이트 대기
                pbuf.clear()
                buf += ch
                sys.stdout.write(ch)
                sys.stdout.flush()
        finally:
            try:
                termios.tcsetattr(fd, termios.TCSADRAIN, old)
            except Exception:
                pass

    def _ui_pause(self, back_label: str | None = None, refresh: bool = False) -> str:
        """결과 화면에서 Enter/r/b/m/q를 받는다."""
        hints = []
        if refresh:
            hints.append('r) 새로고침')
        if back_label:
            hints.append(f'b) {back_label}')
        hints += ['m) 메인 메뉴', 'q) 종료']
        try:
            raw = input('Enter 계속 · ' + ' · '.join(hints) + ' > ').strip().lower()
        except EOFError:
            return QUIT
        if raw in ('q', 'quit', 'exit'):
            return QUIT
        if raw in ('m', 'main', 'menu'):
            return MAIN
        if raw in ('b', 'back') and back_label:
            return BACK
        if raw in ('r', 'refresh') and refresh:
            return REFRESH
        return STAY

    def _ui_yes_no(self, prompt: str, default: bool = False) -> bool:
        suffix = 'Y/n' if default else 'y/N'
        while True:
            try:
                raw = input(f'{prompt} [{suffix}] ').strip().lower()
            except EOFError:
                return False
            if not raw:
                return default
            if raw in ('y', 'yes', '예', 'ㅇ'):
                return True
            if raw in ('n', 'no', '아니오', 'ㄴ'):
                return False
            if raw in ('q', 'quit', 'exit'):
                return False
            print('y 또는 n으로 답하세요.')

    def _ui_read_float(self, prompt: str, current: float,
                       positive: bool = False) -> tuple[float, bool]:
        try:
            raw = input(f'{prompt} [{current:g}] (Enter=유지): ').strip()
        except EOFError:
            return current, False
        if not raw:
            return current, True
        try:
            value = float(raw)
        except ValueError:
            print(f'거부: 숫자가 아님: {raw}')
            return current, False
        if not math.isfinite(value):
            print('거부: 유한한 숫자가 아님')
            return current, False
        if positive and value <= 0.0:
            print('거부: 0보다 커야 합니다')
            return current, False
        if not positive and value < 0.0:
            print('거부: 0 이상이어야 합니다')
            return current, False
        return value, True

    def _ui_invalid(self, choice: str):
        print(self._ui_paint(f'알 수 없는 선택: {choice}', RED))
        time.sleep(0.45)

    def _device_info(self, name: str) -> tuple[str, str, list[str]]:
        """state_manager의 `종류|active/disconnected|토픽,토픽`을 푼다."""
        raw = (self._device_details.get(name) or '').strip()
        if not raw:
            return '-', '', []
        parts = raw.split('|', 2)
        kind = parts[0].strip() if parts else '-'
        state = parts[1].strip().lower() if len(parts) > 1 else ''
        topics = [t for t in parts[2].split(',') if t] if len(parts) > 2 else []
        return kind or '-', state, topics

    @staticmethod
    def _status_args(diagnostics: bool = False, topics: bool = False):
        return argparse.Namespace(diagnostics=diagnostics, topics=topics)

    def run_menu(self) -> int:
        """번호 선택 + b/m/q 내비게이션을 쓰는 대화형 운영 콘솔."""
        self._poll(0.6)
        aliases = {
            'status': '1', 'state': '1',
            'devices': '2', 'device': '2',
            'mode': '3',
            'manual': '4', 'teleop': '4', 'drive': '4',
            'arm': '5', 'gripper': '6', 'map': '7',
            'estop': '8', 'e-stop': '8', 'restart': '9', 'rviz': '10',
        }
        while rclpy.ok():
            left = [
                '  1) 상태',
                '  2) 장치',
                '  3) 조작 모드',
                '  4) 수동 주행 / 텔레오퍼',
                '  5) 팔',
                '  6) 그리퍼',
                '  7) 지도',
                '  8) 비상정지',
                '  9) 재시작',
                ' 10) RViz 열기 (Best Effort QoS)',
                '',
                '  번호를 고르거나 명령 이름',
                '  (status, devices, teleop …)',
            ]
            self._ui_draw('메인 메뉴', left, self._ui_center_summary(),
                          back=False, main=False)
            choice = self._ui_prompt()
            if choice == TIMEOUT:
                continue
            if choice in ('r', 'refresh'):
                continue
            choice = aliases.get(choice, choice)
            if choice in ('q', 'quit', 'exit'):
                break
            if choice == '1':
                result = self._menu_status()
            elif choice == '2':
                result = self._menu_devices()
            elif choice == '3':
                result = self._menu_mode()
            elif choice == '4':
                result = self._menu_manual()
            elif choice == '5':
                result = self._menu_arm()
            elif choice == '6':
                result = self._menu_gripper()
            elif choice == '7':
                result = self._menu_map()
            elif choice == '8':
                result = self._menu_estop()
            elif choice == '9':
                result = self._menu_restart()
            elif choice in ('10', 'rviz'):
                result = self._menu_rviz()
            else:
                self._ui_invalid(choice)
                continue
            if result == QUIT:
                break
            # BACK은 하위 메뉴에서 메인으로 돌아온 것이다. MAIN과 같이 메인을 다시 그린다.
        self._ui_clear()
        print('operator_tool 종료')
        return 0

    def _menu_rviz(self) -> str:
        self._ui_header('RViz')
        if not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
            print('거부: DISPLAY/WAYLAND_DISPLAY가 없습니다 — GUI가 있는 터미널에서 실행하세요.')
            result = self._ui_pause(back_label='메인 메뉴')
            return result if result in (MAIN, QUIT) else STAY
        script = Path(__file__).resolve().parent.parent / 'run-rviz'
        print('  카메라/점구름은 Best Effort, queue 1로 구독합니다.')
        print(f'  실행: {script}')
        if not script.is_file():
            print(f'거부: 실행 파일 없음: {script}')
            result = self._ui_pause(back_label='메인 메뉴')
            return result if result in (MAIN, QUIT) else STAY
        if not self._ui_yes_no('RViz를 새 창으로 열까요?', default=True):
            return STAY
        log = Path('/tmp/operator_tool_rviz.log')
        try:
            with log.open('ab') as out:
                proc = subprocess.Popen(
                    [str(script)], cwd=str(script.parent),
                    stdout=out, stderr=out, start_new_session=True)
        except OSError as exc:
            print(f'거부: RViz 실행 실패: {exc}')
            result = self._ui_pause(back_label='메인 메뉴')
            return result if result in (MAIN, QUIT) else STAY
        print(f'RViz 시작: pid {proc.pid} · 로그 {log}')
        result = self._ui_pause(back_label='메인 메뉴')
        return result if result in (MAIN, QUIT) else STAY

    def _menu_status(self) -> str:
        while rclpy.ok():
            left = [
                '  1) 전체 요약',
                '  2) 조작·지도 상태',
                '  3) 장치 상태 / 상세',
                '  4) 관절 상태',
                '  5) 진단 (/diagnostics_agg)',
                '  6) 토픽 (/state/topics)',
            ]
            center = [
                '[조작·지도]',
                f' 조작: {(self.operation or "-")}',
                f' 지도: {(self.map_mode or "-")}',
                f' 이름: {self._values.get("map_name") or "-"}',
                f' E-STOP: {"ON" if self._emergency else "OFF"}',
                '',
            ] + self._ui_center_summary()
            self._ui_draw('상태', left, center)
            choice = self._ui_prompt()
            if choice == TIMEOUT:
                continue
            if choice in ('r', 'refresh'):
                continue
            if choice in ('b', 'back'):
                return BACK
            if choice in ('m', 'main', 'menu'):
                return MAIN
            if choice in ('q', 'quit', 'exit'):
                return QUIT

            if choice == '1':
                self.cmd_status(self._status_args())
                result = self._ui_pause(back_label='상태 메뉴')
            elif choice == '2':
                self._show_operation_state()
                result = self._ui_pause(back_label='상태 메뉴')
            elif choice == '3':
                result = self._menu_devices()
            elif choice == '4':
                self._show_joints()
                result = self._ui_pause(back_label='상태 메뉴')
            elif choice == '5':
                self.cmd_status(self._status_args(diagnostics=True))
                result = self._ui_pause(back_label='상태 메뉴')
            elif choice == '6':
                self.cmd_status(self._status_args(topics=True))
                result = self._ui_pause(back_label='상태 메뉴')
            else:
                self._ui_invalid(choice)
                continue
            if result in (MAIN, QUIT):
                return result

    def _show_operation_state(self):
        self._poll(0.5)
        print(f'조작 모드 : {self.operation or "-"}')
        print(f'지도 모드 : {self.map_mode or "-"}')
        print(f'지도 이름 : {self._values.get("map_name") or "-"}')
        print(f'비상정지  : {"ON" if self._emergency else "OFF"}')
        maps = [n for n in (self._values.get('map_list') or '').splitlines() if n]
        if maps:
            print('지도 목록 : ' + ', '.join(maps))
        else:
            print('지도 목록 : -')

    def _show_joints(self):
        self._poll(0.5)
        found = False
        for name in (*WHEEL_JOINTS, *ARM_JOINTS, GRIPPER_JOINT):
            if name in self._joints:
                deg = '' if name in WHEEL_JOINTS else f'  ({math.degrees(self._joints[name]):+.1f} deg)'
                print(f'  {name:18s} {self._joints[name]: .4f} rad{deg}')
                found = True
        if not found:
            print('/joint_states에 팔 관절이 없습니다 — bringup/OpenCR 확인', file=sys.stderr)

    def _menu_devices(self) -> str:
        while rclpy.ok():
            self._poll(0.6)
            left = []
            if not self._devices:
                left.append('  장치 목록 없음')
                left.append('  (/state/devices 대기 중)')
            else:
                for index, name in enumerate(self._devices, 1):
                    kind, state, topics = self._device_info(name)
                    if state == 'active':
                        mark = self._ui_paint('●', GREEN)
                    elif state:
                        mark = self._ui_paint('○', RED)
                    else:
                        mark = self._ui_paint('?', YELLOW)
                    left.append(f'  {index:2d}) {mark} {name} ({kind})')
            left += ['', '  번호 입력 → 상세 보기']
            center = ['[장치 상세는 번호 입력]'] + self._ui_center_summary()
            self._ui_draw('장치', left, center)
            choice = self._ui_prompt()
            if choice == TIMEOUT:
                continue
            if choice in ('r', 'refresh'):
                continue
            if choice in ('b', 'back'):
                return BACK
            if choice in ('m', 'main', 'menu'):
                return MAIN
            if choice in ('q', 'quit', 'exit'):
                return QUIT
            if not choice.isdigit():
                self._ui_invalid(choice)
                continue
            index = int(choice) - 1
            if not 0 <= index < len(self._devices):
                self._ui_invalid(choice)
                continue
            result = self._device_detail_menu(self._devices[index])
            if result in (MAIN, QUIT):
                return result

    # -------------------------------------------------------- 동적 파라미터
    def _param_client(self, node_name: str, node_ns: str):
        """대상 노드의 파라미터 클라이언트 (없으면 None). 재사용 캐시."""
        try:
            from rclpy.parameter_client import AsyncParameterClient
        except ImportError:
            return None
        key = (node_name, node_ns)
        cli = self._param_clients.get(key)
        if cli is None:
            try:
                # Lyrical AsyncParameterClient는 namespace 인자가 없고
                # 원격 노드 FQN('/ns/이름')으로 서비스명을 만든다.
                fqn = f'/{node_name}' if node_ns in ('/', '') else f'{node_ns}/{node_name}'
                cli = AsyncParameterClient(self, fqn)
            except Exception:
                return None
            self._param_clients[key] = cli
        return cli

    def _param_target_nodes(self, entries: list[str]) -> list[tuple[str, str]]:
        """장치 세부 항목(토픽/관절)에서 파라미터 대상 노드 찾기.

        토픽 발행자를 역추적한다. 관절 이름만 있으면 /joint_states 발행자로.
        """
        topics = [e for e in entries if isinstance(e, str) and e.startswith('/')]
        if not topics:
            topics = ['/joint_states']
        found: list[tuple[str, str]] = []
        seen = set()
        me = self.get_name()
        for topic in topics:
            try:
                infos = self.get_publishers_info_by_topic(topic)
            except Exception:
                continue
            for info in infos or []:
                key = (info.node_name, info.node_namespace or '/')
                if key in seen or info.node_name == me:
                    continue
                seen.add(key)
                found.append(key)
        return found

    def _param_wait(self, future, timeout: float = 5.0):
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done():
            return None
        try:
            return future.result()
        except Exception:
            return None

    def _param_list_names(self, cli) -> list[str] | None:
        try:
            res = self._param_wait(cli.list_parameters())
        except Exception:
            return None
        if res is None or res.result is None:
            return None
        try:
            return sorted(res.result.names)
        except Exception:
            return None

    def _param_describe(self, cli, names: list[str]):
        try:
            res = self._param_wait(cli.describe_parameters(names))
        except Exception:
            return None
        if res is None or getattr(res, 'descriptors', None) is None:
            return None
        return list(res.descriptors)

    def _param_get_vals(self, cli, names: list[str]):
        try:
            res = self._param_wait(cli.get_parameters(names))
        except Exception:
            return None
        if res is None or getattr(res, 'values', None) is None:
            return None
        return list(res.values)

    def _choose_param_node(self, entries: list[str]) -> tuple[str, str] | None:
        cands = self._param_target_nodes(entries)
        if not cands:
            print('거부: 이 장치의 발행 노드를 찾지 못했습니다 — bringup/토픽 확인',
                  file=sys.stderr)
            return None
        if len(cands) == 1:
            return cands[0]
        print('  노드 선택:')
        for i, (n, ns) in enumerate(cands, 1):
            print(f'  {i:2d}) {ns if ns.endswith("/") else ns + "/"}{n}')
        try:
            raw = input('노드 번호: ').strip()
        except EOFError:
            return None
        if raw.isdigit() and 1 <= int(raw) <= len(cands):
            return cands[int(raw) - 1]
        print(f'거부: 모르는 번호: {raw}')
        return None

    def _param_act_list(self, node_name: str, node_ns: str) -> int:
        cli = self._param_client(node_name, node_ns)
        if cli is None:
            print('거부: parameter_client 사용 불가', file=sys.stderr)
            return 1
        print(f'  {node_ns}{node_name} 파라미터 조회 중…')
        if not cli.wait_for_services(timeout_sec=5.0):
            print('거부: 파라미터 서비스 무응답 (노드 종료/비활성?)', file=sys.stderr)
            return 1
        names = self._param_list_names(cli)
        if names is None:
            print('거부: 목록 조회 실패', file=sys.stderr)
            return 1
        if not names:
            print('  (파라미터 없음)')
            return 0
        descs = self._param_describe(cli, names) or []
        vals = self._param_get_vals(cli, names) or []
        for i, pname in enumerate(names):
            ptype = descs[i].type if i < len(descs) else ParameterType.PARAMETER_NOT_SET
            ro = descs[i].read_only if i < len(descs) else False
            py = _param_value_to_py(ptype, vals[i]) if i < len(vals) else None
            ro_txt = ' [읽기전용]' if ro else ''
            print(f'  {pname} = {py!r} [{_PARAM_TYPE_NAMES.get(ptype, ptype)}]{ro_txt}')
        print(f'  ({len(names)}개)')
        return 0

    def _param_act_get(self, node_name: str, node_ns: str) -> int:
        try:
            pname = input('파라미터 이름: ').strip()
        except EOFError:
            return 1
        if not pname:
            print('거부: 이름이 비었습니다', file=sys.stderr)
            return 2
        cli = self._param_client(node_name, node_ns)
        if cli is None or not cli.wait_for_services(timeout_sec=5.0):
            print('거부: 파라미터 서비스 무응답', file=sys.stderr)
            return 1
        descs = self._param_describe(cli, [pname])
        vals = self._param_get_vals(cli, [pname])
        if not descs or not vals:
            print('거부: 조회 실패 (없는 이름?)', file=sys.stderr)
            return 1
        print(f'  {pname} = {_param_value_to_py(descs[0].type, vals[0])!r} '
              f'[{_PARAM_TYPE_NAMES.get(descs[0].type, "?")}]'
              + (' [읽기전용]' if descs[0].read_only else ''))
        return 0

    def _param_act_set(self, node_name: str, node_ns: str) -> int:
        try:
            pname = input('파라미터 이름: ').strip()
        except EOFError:
            return 1
        if not pname:
            print('거부: 이름이 비었습니다', file=sys.stderr)
            return 2
        cli = self._param_client(node_name, node_ns)
        if cli is None or not cli.wait_for_services(timeout_sec=5.0):
            print('거부: 파라미터 서비스 무응답', file=sys.stderr)
            return 1
        descs = self._param_describe(cli, [pname])
        vals = self._param_get_vals(cli, [pname])
        if not descs or not vals:
            print('거부: 조회 실패 (없는 이름?)', file=sys.stderr)
            return 1
        ptype = descs[0].type
        if descs[0].read_only:
            print(f'거부: 읽기전용 파라미터입니다: {pname}', file=sys.stderr)
            return 1
        current = _param_value_to_py(ptype, vals[0])
        print(f'  현재: {pname} = {current!r} [{_PARAM_TYPE_NAMES.get(ptype, "?")}]')
        try:
            raw = input('새 값 (bool/int/double/문자열, 배열은 쉼표 구분): ').strip()
        except EOFError:
            return 1
        try:
            new_val = _parse_param_text(raw, ptype)
        except (ValueError, TypeError) as exc:
            print(f'거부: {exc}', file=sys.stderr)
            return 2
        print(f'  변경: {current!r} → {new_val!r}')
        if not self._ui_yes_no(f'{pname} 값을 바꾸겠습니까?', default=False):
            return 0
        try:
            res = self._param_wait(cli.set_parameters([Parameter(pname, value=new_val)]))
        except Exception as exc:
            print(f'거부: 설정 호출 실패: {exc}', file=sys.stderr)
            return 1
        if res is None or not getattr(res, 'results', None):
            print('거부: 설정 응답 없음', file=sys.stderr)
            return 1
        ok = bool(res.results[0].successful)
        print(f'  {"성공" if ok else "실패: " + (res.results[0].reason or "?")}')
        return 0 if ok else 1

    def _device_detail_menu(self, name: str) -> str:
        while rclpy.ok():
            self._poll(0.45)
            kind, state, topics = self._device_info(name)
            if state == 'active':
                state_txt = self._ui_paint('● active', GREEN)
            elif state:
                state_txt = self._ui_paint(f'○ {state}', RED)
            else:
                state_txt = self._ui_paint('? 상세 정보 없음', YELLOW)
            left = [
                '  1) 파라미터 목록',
                '  2) 값 보기 (get)',
                '  3) 설정 (set)',
                '  4) 전체 상세 보기',
            ]
            center = [f'[장치 · {name}]', f' 종류: {kind}', f' 상태: {state_txt}',
                      f' 토픽/관절: {len(topics)}개']
            center += [f'  - {t}' for t in topics[:6]]
            if len(topics) > 6:
                center.append(f'  … 외 {len(topics) - 6}개')
            related = []
            for diag_name, (level, message) in self._diagnostics.items():
                if name in diag_name or any(topic in diag_name for topic in topics):
                    related.append((diag_name, level, message))
            if related:
                center.append(' 관련 진단:')
                for diag_name, level, message in sorted(related)[:3]:
                    center.append(f'  - {diag_name}: {DIAG_LEVELS.get(level, level)}')
            else:
                center.append(' 관련 진단: -')
            self._ui_draw(f'장치 · {name}', left, center)
            choice = self._ui_prompt()
            if choice == TIMEOUT:
                continue
            if choice in ('r', 'refresh'):
                continue
            if choice in ('b', 'back'):
                return BACK
            if choice in ('m', 'main', 'menu'):
                return MAIN
            if choice in ('q', 'quit', 'exit'):
                return QUIT
            if choice in ('1', '2', '3'):
                node = self._choose_param_node(topics)
                if node is None:
                    result = self._ui_pause(back_label='장치 상세')
                elif choice == '1':
                    self._param_act_list(*node)
                    result = self._ui_pause(back_label='장치 상세')
                elif choice == '2':
                    self._param_act_get(*node)
                    result = self._ui_pause(back_label='장치 상세')
                else:
                    self._param_act_set(*node)
                    result = self._ui_pause(back_label='장치 상세')
            elif choice == '4':
                self._show_device_full(name, kind, state_txt, topics, related)
                result = self._ui_pause(back_label='장치 상세')
            else:
                self._ui_invalid(choice)
                continue
            if result in (MAIN, QUIT):
                return result

    def _show_device_full(self, name: str, kind: str, state_txt: str,
                          topics: list[str], related: list) -> None:
        """장치 전체 상세 (기존 상세 화면 — 잘림 없이 전부)."""
        self._poll(0.45)
        self._ui_header(f'장치 · {name}')
        print(f'  종류   : {kind}')
        print(f'  상태   : {state_txt}')
        print('  세부 토픽/관절:')
        if topics:
            for topic in topics:
                print(f'    - {topic}')
        else:
            print('    -')
        if related:
            print('  관련 진단:')
            for diag_name, level, message in sorted(related):
                print(f'    - {diag_name}: {DIAG_LEVELS.get(level, level)} {message}')
        else:
            print('  관련 진단: -')

    def _menu_mode(self) -> str:
        targets = {'1': 'AUTO', '2': 'MANUAL', '3': 'EXPLORE'}
        while rclpy.ok():
            left = [
                '  1) AUTO      — 자동 주행',
                '  2) MANUAL    — 수동 주행/팔/그리퍼',
                '  3) EXPLORE   — MAPPING 중 탐사',
            ]
            center = [
                '[현재 모드]',
                f' 조작: {self.operation or "-"}',
                f' E-STOP: {"ON" if self._emergency else "OFF"}',
                f' 지도: {self.map_mode or "-"}',
                '',
                ' AUTO 전환 시 자동 주행 시작 주의',
            ]
            self._ui_draw('조작 모드', left, center)
            choice = self._ui_prompt()
            if choice == TIMEOUT:
                continue
            if choice in ('r', 'refresh'):
                continue
            if choice in ('b', 'back'):
                return BACK
            if choice in ('m', 'main', 'menu'):
                return MAIN
            if choice in ('q', 'quit', 'exit'):
                return QUIT
            target = targets.get(choice)
            if target is None:
                self._ui_invalid(choice)
                continue
            if target == 'AUTO' and not self._ui_yes_no(
                    'AUTO로 바꾸면 자동 주행이 시작될 수 있습니다. 전환할까요?'):
                continue
            if target == 'EXPLORE' and self.map_mode != 'MAPPING':
                print('WARN: EXPLORE는 지도 모드 MAPPING에서만 허용됩니다.')
                if not self._ui_yes_no('그래도 EXPLORE로 전환할까요?'):
                    continue
            self.cmd_mode(argparse.Namespace(mode=target.lower()))
            result = self._ui_pause(back_label='조작 모드')
            if result in (MAIN, QUIT):
                return result

    def _menu_estop(self) -> str:
        while rclpy.ok():
            est = self._ui_paint('ON', RED + BOLD) if self._emergency else self._ui_paint('OFF', GREEN)
            left = [
                '  1) 상태 새로고침',
                '  2) 비상정지 ON',
                '  3) 비상정지 OFF',
            ]
            center = ['[비상정지]', f' 현재: {est}', '', ' ON은 바퀴 명령 차단']
            self._ui_draw('비상정지', left, center)
            choice = self._ui_prompt()
            if choice == TIMEOUT:
                continue
            if choice in ('r', 'refresh'):
                continue
            if choice in ('b', 'back'):
                return BACK
            if choice in ('m', 'main', 'menu'):
                return MAIN
            if choice in ('q', 'quit', 'exit'):
                return QUIT
            if choice == '1':
                self.cmd_estop(argparse.Namespace(state='status'))
                result = self._ui_pause(back_label='비상정지')
            elif choice == '2':
                if self._emergency:
                    print('이미 비상정지 ON입니다.')
                    result = self._ui_pause(back_label='비상정지')
                elif self._ui_yes_no('비상정지를 ON 할까요?', default=False):
                    self.cmd_estop(argparse.Namespace(state='on'))
                    result = self._ui_pause(back_label='비상정지')
                else:
                    continue
            elif choice == '3':
                if not self._emergency:
                    print('이미 비상정지 OFF입니다.')
                    result = self._ui_pause(back_label='비상정지')
                elif self._ui_yes_no('주변 안전을 확인했습니다. 비상정지를 해제할까요?', default=False):
                    self.cmd_estop(argparse.Namespace(state='off'))
                    result = self._ui_pause(back_label='비상정지')
                else:
                    continue
            else:
                self._ui_invalid(choice)
                continue
            if result in (MAIN, QUIT):
                return result

    def _prepare_manual(self) -> bool:
        self._poll(0.5)
        if self._emergency:
            print('거부: 비상정지 중입니다 — 먼저 비상정지를 해제하세요.')
            return False
        if self.operation == 'MANUAL':
            return True
        print(f'현재 조작 모드: {self.operation or "알 수 없음"}')
        if not self._ui_yes_no('MANUAL 모드로 전환할까요?', default=True):
            return False
        self.cmd_mode(argparse.Namespace(mode='manual'))
        return self.operation == 'MANUAL'

    def _menu_manual(self) -> str:
        while rclpy.ok():
            left = [
                '  1) 텔레오퍼 (w/x/a/d · e/c 속도)',
                '  2) 전진',
                '  3) 후진',
                '  4) 좌회전',
                '  5) 우회전',
                '  6) 정지',
                '  7) 속도/시간 설정',
            ]
            center = [
                '[수동 주행]',
                f' 모드: {self.operation or "-"}',
                f' E-STOP: {"ON" if self._emergency else "OFF"}',
                f' 선속도 {self._ui_linear:.2f} m/s',
                f' 각속도 {self._ui_angular:.2f} rad/s',
                f' 시간 {self._ui_duration:.2f} s',
            ]
            self._ui_draw('수동 주행 / 텔레오퍼', left, center)
            choice = self._ui_prompt()
            if choice == TIMEOUT:
                continue
            if choice in ('r', 'refresh'):
                continue
            if choice in ('b', 'back'):
                return BACK
            if choice in ('m', 'main', 'menu'):
                return MAIN
            if choice in ('q', 'quit', 'exit'):
                return QUIT
            if choice == '7':
                result = self._menu_drive_settings()
                if result in (MAIN, QUIT):
                    return result
                continue
            if choice == '1':
                if not self._prepare_manual():
                    self._ui_pause(back_label='수동 메뉴')
                    continue
                self.cmd_teleop(argparse.Namespace(
                    linear=self._ui_linear, angular=self._ui_angular, force=False))
                result = self._ui_pause(back_label='수동 메뉴')
            elif choice in ('2', '3', '4', '5', '6'):
                if choice != '6' and not self._prepare_manual():
                    self._ui_pause(back_label='수동 메뉴')
                    continue
                direction = {'2': 'forward', '3': 'back', '4': 'left',
                             '5': 'right', '6': 'stop'}[choice]
                self.cmd_drive(argparse.Namespace(
                    direction=direction, speed=self._ui_linear,
                    angular=self._ui_angular, duration=self._ui_duration, force=False))
                result = self._ui_pause(back_label='수동 메뉴')
            else:
                self._ui_invalid(choice)
                continue
            if result in (MAIN, QUIT):
                return result

    def _menu_drive_settings(self) -> str:
        while rclpy.ok():
            left = [
                f'  1) 선속도   : {self._ui_linear:.2f} m/s',
                f'  2) 각속도   : {self._ui_angular:.2f} rad/s',
                f'  3) 주행시간 : {self._ui_duration:.2f} s',
            ]
            center = ['[현재 설정]', f' 선속도 {self._ui_linear:.2f} m/s',
                      f' 각속도 {self._ui_angular:.2f} rad/s',
                      f' 주행시간 {self._ui_duration:.2f} s']
            self._ui_draw('수동 주행 설정', left, center)
            choice = self._ui_prompt()
            if choice == TIMEOUT:
                continue
            if choice in ('r', 'refresh'):
                continue
            if choice in ('b', 'back'):
                return BACK
            if choice in ('m', 'main', 'menu'):
                return MAIN
            if choice in ('q', 'quit', 'exit'):
                return QUIT
            if choice == '1':
                value, ok = self._ui_read_float('선속도 [m/s]', self._ui_linear)
                if ok:
                    self._ui_linear = value
                else:
                    self._ui_pause(back_label='설정')
            elif choice == '2':
                value, ok = self._ui_read_float('각속도 [rad/s]', self._ui_angular)
                if ok:
                    self._ui_angular = value
                else:
                    self._ui_pause(back_label='설정')
            elif choice == '3':
                value, ok = self._ui_read_float('주행시간 [s]', self._ui_duration, positive=True)
                if ok:
                    self._ui_duration = value
                else:
                    self._ui_pause(back_label='설정')
            else:
                self._ui_invalid(choice)

    def _menu_arm(self) -> str:
        while rclpy.ok():
            values = [f'{name}={self._joints[name]:+.3f} rad ({math.degrees(self._joints[name]):+.1f}°)'
                      for name in ARM_JOINTS if name in self._joints]
            left = [
                '  1) 관절 상태',
                '  2) 관절 목표 설정',
                '  3) 관절 델타 이동',
                '  4) 홈 (모든 팔 관절 0)',
            ]
            center = ['[팔 관절]']
            center += [f' {v}' for v in values] if values else [' (joint_states 없음)']
            if self.operation != 'MANUAL':
                center.append(f' {self._ui_paint("MANUAL 모드가 아닙니다", YELLOW)}')
            if self.map_mode == 'MAPPING':
                center.append(f' {self._ui_paint("MAPPING 중 팔 명령 거절", YELLOW)}')
            self._ui_draw('팔', left, center)
            choice = self._ui_prompt()
            if choice == TIMEOUT:
                continue
            if choice in ('r', 'refresh'):
                continue
            if choice in ('b', 'back'):
                return BACK
            if choice in ('m', 'main', 'menu'):
                return MAIN
            if choice in ('q', 'quit', 'exit'):
                return QUIT
            if choice == '1':
                self.cmd_arm(argparse.Namespace(action='show', targets=[], force=False))
                result = self._ui_pause(back_label='팔 메뉴')
            elif choice in ('2', '3'):
                if not any(name in self._joints for name in ARM_JOINTS):
                    print('/joint_states에 팔 관절이 없습니다 — bringup/OpenCR 확인', file=sys.stderr)
                    self._ui_pause(back_label='팔 메뉴')
                    continue
                if not self._prepare_manual():
                    self._ui_pause(back_label='팔 메뉴')
                    continue
                if self._arm_change(delta=(choice == '3')):
                    result = self._ui_pause(back_label='팔 메뉴')
                else:
                    continue
            elif choice == '4':
                print('WARN: 현재 자세와 주변을 확인하세요. joint2가 URDF 한계 밖이면 홈 이동이 위험합니다.')
                if not self._prepare_manual():
                    self._ui_pause(back_label='팔 메뉴')
                    continue
                if not self._ui_yes_no('모든 팔 관절을 0으로 이동할까요?', default=False):
                    continue
                self.cmd_arm(argparse.Namespace(action='home', targets=[], force=False))
                result = self._ui_pause(back_label='팔 메뉴')
            else:
                self._ui_invalid(choice)
                continue
            if result in (MAIN, QUIT):
                return result

    def _arm_change(self, delta: bool) -> bool:
        self._poll(0.5)
        if not any(name in self._joints for name in ARM_JOINTS):
            print('/joint_states에 팔 관절이 없습니다 — bringup/OpenCR 확인', file=sys.stderr)
            return False
        print('  ' + '  '.join(f'{i}) {name}' for i, name in enumerate(ARM_JOINTS, 1)))
        try:
            raw = input('관절 [1-5 또는 이름] (b=뒤로): ').strip().lower()
        except EOFError:
            return False
        if raw in ('b', 'back', ''):
            return False
        if raw.isdigit() and 1 <= int(raw) <= len(ARM_JOINTS):
            name = ARM_JOINTS[int(raw) - 1]
        elif raw in ARM_JOINTS:
            name = raw
        else:
            print(f'거부: 모르는 관절: {raw}')
            return False
        label = '변화량 [rad]' if delta else '목표 [rad]'
        try:
            raw_value = input(f'{name} {label}: ').strip()
        except EOFError:
            return False
        try:
            value = float(raw_value)
        except ValueError:
            print(f'거부: 숫자가 아님: {raw_value!r}')
            return False
        if not math.isfinite(value):
            print('거부: 유한한 숫자가 아님')
            return False
        action = 'delta' if delta else 'set'
        self.cmd_arm(argparse.Namespace(
            action=action, targets=[f'{name}={value:g}'], force=False))
        return True

    def _menu_gripper(self) -> str:
        while rclpy.ok():
            if GRIPPER_JOINT in self._joints:
                grip_txt = f'{GRIPPER_JOINT}={self._joints[GRIPPER_JOINT]:.4f} m'
            else:
                grip_txt = '(joint_states 없음)'
            left = [
                '  1) 상태',
                '  2) 열기',
                '  3) 닫기',
                '  4) 위치 지정',
            ]
            center = ['[그리퍼]', f' 현재: {grip_txt}']
            self._ui_draw('그리퍼', left, center)
            choice = self._ui_prompt()
            if choice == TIMEOUT:
                continue
            if choice in ('r', 'refresh'):
                continue
            if choice in ('b', 'back'):
                return BACK
            if choice in ('m', 'main', 'menu'):
                return MAIN
            if choice in ('q', 'quit', 'exit'):
                return QUIT
            if choice == '1':
                self.cmd_gripper(argparse.Namespace(action='show', value=None, force=False))
                result = self._ui_pause(back_label='그리퍼 메뉴')
            elif choice in ('2', '3'):
                if not self._prepare_manual():
                    self._ui_pause(back_label='그리퍼 메뉴')
                    continue
                action = 'open' if choice == '2' else 'close'
                self.cmd_gripper(argparse.Namespace(action=action, value=None, force=False))
                result = self._ui_pause(back_label='그리퍼 메뉴')
            elif choice == '4':
                try:
                    raw = input(f'위치 [m] (0=닫힘, {GRIPPER_OPEN}=열림, b=뒤로): ').strip()
                except EOFError:
                    return QUIT
                if raw.lower() in ('b', 'back', ''):
                    continue
                if not self._prepare_manual():
                    self._ui_pause(back_label='그리퍼 메뉴')
                    continue
                self.cmd_gripper(argparse.Namespace(action='position', value=raw, force=False))
                result = self._ui_pause(back_label='그리퍼 메뉴')
            else:
                self._ui_invalid(choice)
                continue
            if result in (MAIN, QUIT):
                return result

    def _map_names(self) -> list[str]:
        self._poll(0.5)
        maps = [n for n in (self._values.get('map_list') or '').splitlines() if n]
        if not maps:
            maps = self._local_maps(self._default_map_dir())
        return maps

    @staticmethod
    def _map_args(action: str, name: str | None = None, file: str | None = None):
        return argparse.Namespace(
            action=action, name=name, file=file, map_dir=None,
            format='pgm', topic='/map')

    def _menu_map(self) -> str:
        while rclpy.ok():
            maps = self._map_names()
            left = [
                '  1) 지도 상태',
                '  2) 지도 목록',
                '  3) MAPPING 모드',
                '  4) LOCALIZATION 모드',
                '  5) 지도 전환',
                '  6) 지도 저장',
            ]
            center = ['[지도]',
                      f' 모드: {self.map_mode or "-"}',
                      f' 이름: {self._values.get("map_name") or "-"}',
                      f' 목록: {len(maps)}개'] + [f'  - {n}' for n in maps[:8]]
            self._ui_draw('지도', left, center)
            choice = self._ui_prompt()
            if choice == TIMEOUT:
                continue
            if choice in ('r', 'refresh'):
                continue
            if choice in ('b', 'back'):
                return BACK
            if choice in ('m', 'main', 'menu'):
                return MAIN
            if choice in ('q', 'quit', 'exit'):
                return QUIT
            if choice == '1':
                self.cmd_map(self._map_args('status'))
                result = self._ui_pause(back_label='지도 메뉴')
            elif choice == '2':
                self.cmd_map(self._map_args('list'))
                result = self._ui_pause(back_label='지도 메뉴')
            elif choice == '3':
                self.cmd_map(self._map_args('mode', name='mapping'))
                result = self._ui_pause(back_label='지도 메뉴')
            elif choice == '4':
                self.cmd_map(self._map_args('mode', name='localization'))
                result = self._ui_pause(back_label='지도 메뉴')
            elif choice == '5':
                result = self._menu_map_switch()
            elif choice == '6':
                result = self._menu_map_save()
            else:
                self._ui_invalid(choice)
                continue
            if result in (MAIN, QUIT):
                return result

    def _menu_map_switch(self) -> str:
        while rclpy.ok():
            maps = self._map_names()
            left = []
            if maps:
                for index, name in enumerate(maps, 1):
                    left.append(f'  {index:2d}) {name}')
            else:
                left.append('  지도 목록 없음')
                left.append('  (이름 직접 입력)')
            left += ['', '   n) 이름 직접 입력']
            center = ['[지도 전환]', f' 현재: {self._values.get("map_name") or "-"}',
                      f' 모드: {self.map_mode or "-"}']
            self._ui_draw('지도 전환', left, center)
            choice = self._ui_prompt()
            if choice == TIMEOUT:
                continue
            if choice in ('r', 'refresh'):
                continue
            if choice in ('b', 'back'):
                return BACK
            if choice in ('m', 'main', 'menu'):
                return MAIN
            if choice in ('q', 'quit', 'exit'):
                return QUIT
            name = None
            if choice.isdigit():
                index = int(choice) - 1
                if 0 <= index < len(maps):
                    name = maps[index]
            elif choice in ('n', 'name'):
                try:
                    name = input('지도 이름: ').strip()
                except EOFError:
                    return QUIT
            if not name:
                self._ui_invalid(choice)
                continue
            self.cmd_map(self._map_args('switch', name=name))
            result = self._ui_pause(back_label='지도 전환')
            if result in (MAIN, QUIT):
                return result

    def _menu_map_save(self) -> str:
        self._ui_header('지도 저장')
        name = self._values.get('map_name') or ''
        default = self._default_map_dir() / (name or '<지도이름>') / 'map'
        print(f'  현재 지도: {name or "-"}')
        print(f'  기본 경로: {default}')
        if not self._ui_yes_no('현재 /map을 저장할까요?', default=False):
            return STAY
        try:
            raw = input('저장 경로 (Enter=기본, b=뒤로): ').strip()
        except EOFError:
            return QUIT
        if raw.lower() in ('b', 'back'):
            return BACK
        if not name and not raw:
            print('거부: 현재 지도 이름이 없습니다 — 저장 경로를 입력하세요.')
            self._ui_pause(back_label='지도 메뉴')
            return STAY
        self.cmd_map(self._map_args('save', file=(raw or None)))
        result = self._ui_pause(back_label='지도 메뉴')
        return result

    def _menu_restart(self) -> str:
        while rclpy.ok():
            self._poll(0.6)
            note = self._values.get('restart') or '-'
            restartable = [line.split()[0] for line in
                           (self._values.get('restartable') or '').splitlines() if line.split()]
            left = []
            if restartable:
                for index, name in enumerate(restartable, 1):
                    left.append(f'  {index:2d}) {name}')
            else:
                left.append('  재시작 가능한 노드 없음')
            left += ['', '   a) 전체 스택 재시작', '   n) 노드 이름 직접 입력']
            center = ['[재시작]', f' 최근 요청: {note}', '',
                      ' 전체 재시작은 RESTART 입력 필요']
            self._ui_draw('재시작', left, center)
            choice = self._ui_prompt()
            if choice == TIMEOUT:
                continue
            if choice in ('r', 'refresh'):
                continue
            if choice in ('b', 'back'):
                return BACK
            if choice in ('m', 'main', 'menu'):
                return MAIN
            if choice in ('q', 'quit', 'exit'):
                return QUIT
            target = None
            if choice.isdigit():
                index = int(choice) - 1
                if 0 <= index < len(restartable):
                    target = restartable[index]
            elif choice == 'a':
                target = 'all'
            elif choice in ('n', 'name'):
                try:
                    target = input('노드 이름 (all=전체): ').strip()
                except EOFError:
                    return QUIT
            if not target:
                self._ui_invalid(choice)
                continue
            if target == 'all':
                print('WARN: 전체 스택이 재시작됩니다. 운영 중인 로봇에서는 주의하세요.')
                if not self._ui_yes_no('정말 전체 스택을 재시작할까요?', default=False):
                    continue
                try:
                    confirm = input('확인을 위해 RESTART를 입력하세요: ').strip()
                except EOFError:
                    return QUIT
                if confirm != 'RESTART':
                    print('취소: 입력이 일치하지 않습니다.')
                    self._ui_pause(back_label='재시작 메뉴')
                    continue
            elif not self._ui_yes_no(f'{target} 노드를 재시작할까요?', default=False):
                continue
            self.cmd_restart(argparse.Namespace(target=target, yes=True))
            result = self._ui_pause(back_label='재시작 메뉴')
            if result in (MAIN, QUIT):
                return result


def _positive_float(value: str) -> float:
    number = float(value)
    if not math.isfinite(number) or number <= 0.0:
        raise argparse.ArgumentTypeError('0보다 큰 유한한 숫자여야 합니다')
    return number


def _nonnegative_float(value: str) -> float:
    number = float(value)
    if not math.isfinite(number) or number < 0.0:
        raise argparse.ArgumentTypeError('0 이상의 유한한 숫자여야 합니다')
    return number


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog='operator_tool',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''예시:
  ./tool/run-cli status
  ./tool/run-cli menu
  ./tool/run-cli
  ./tool/run-cli mode manual
  ./tool/run-cli drive forward --speed 0.15 --duration 1
  ./tool/run-cli teleop
  ./tool/run-cli arm set joint1=0.0 joint2=0.4
  ./tool/run-cli gripper open
  ./tool/run-cli map status
  ./tool/run-cli map save
''')
    sub = parser.add_subparsers(dest='command', required=True)

    sub.add_parser('menu', help='대화형 메뉴 (인자 없이 실행했을 때 기본값)')

    status = sub.add_parser('status', help='조작·지도·장치 상태')
    status.add_argument('--diagnostics', action='store_true', help='/diagnostics_agg 요약')
    status.add_argument('--topics', action='store_true', help='/state/topics 전체 출력')

    mode = sub.add_parser('mode', help='조작 모드 AUTO/MANUAL/EXPLORE 전환')
    mode.add_argument('mode', type=str.lower, choices=('auto', 'manual', 'explore'))

    estop = sub.add_parser('estop', help='비상정지 on/off/status')
    estop.add_argument('state', type=str.lower, choices=('on', 'off', 'status'))

    drive = sub.add_parser('drive', help='바퀴 수동 주행 (MANUAL 모드)')
    drive.add_argument('direction', type=str.lower, choices=('forward', 'back', 'left', 'right', 'stop'))
    drive.add_argument('--speed', type=_nonnegative_float, default=0.15, help='선속도 [m/s] (기본 0.15)')
    drive.add_argument('--angular', type=_nonnegative_float, default=0.8, help='각속도 [rad/s] (기본 0.8)')
    drive.add_argument('--duration', type=_positive_float, default=1.0, help='주행 시간 [s] (기본 1.0)')
    drive.add_argument('--force', action='store_true', help='MANUAL 상태 확인을 건너뜀 (위험)')

    teleop = sub.add_parser('teleop', help='터미널 키보드 수동 주행 (MANUAL 모드)')
    teleop.add_argument('--linear', type=_nonnegative_float, default=0.15, help='전후진 속도 [m/s]')
    teleop.add_argument('--angular', type=_nonnegative_float, default=0.8, help='회전 속도 [rad/s]')
    teleop.add_argument('--force', action='store_true', help='MANUAL 상태 확인을 건너뜀 (위험)')

    arm = sub.add_parser('arm', help='팔 관절 수동 명령 (MANUAL 모드)')
    arm.add_argument('action', type=str.lower, choices=('show', 'set', 'delta', 'home'))
    arm.add_argument('targets', nargs='*', help='관절 목표: joint1=0.0 joint2=0.4 (all=0.0 가능)')
    arm.add_argument('--force', action='store_true', help='MANUAL/MAPPING 확인을 건너뜀 (위험)')

    gripper = sub.add_parser('gripper', help='그리퍼 수동 명령 (MANUAL 모드)')
    gripper.add_argument('action', type=str.lower, choices=('open', 'close', 'position', 'show'))
    gripper.add_argument('value', nargs='?', help='position일 때 목표 [m] (0=닫힘, 0.0265=열림)')
    gripper.add_argument('--force', action='store_true', help='MANUAL 상태 확인을 건너뜀 (위험)')

    map_parser = sub.add_parser('map', help='지도 모드·지도 전환·저장')
    map_parser.add_argument('action', type=str.lower, choices=('status', 'list', 'mode', 'switch', 'save'))
    map_parser.add_argument('name', nargs='?', help='mode: mapping/localization · switch: 지도 이름')
    map_parser.add_argument('--file', help='save 출력 경로 (기본: <map_dir>/<현재 지도>/map)')
    map_parser.add_argument('--map-dir', help='save 기본 지도 폴더 (기본: ros2_ws/map)')
    map_parser.add_argument('--format', default='pgm', choices=('pgm', 'png', 'bmp'),
                            help='save 이미지 형식 (기본 pgm)')
    map_parser.add_argument('--topic', default='/map', help='save할 지도 토픽 (기본 /map)')

    restart = sub.add_parser('restart', help='state_manager에 재시작 요청 (위험)')
    restart.add_argument('target', help='all 또는 노드 이름')
    restart.add_argument('--yes', action='store_true', help='실제로 재시작')

    return parser


def main(argv=None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    if not argv:  # 인자 없이 터미널에서 실행하면 대화형 메뉴
        argv = ['menu'] if sys.stdin.isatty() else ['status']
    parser = build_parser()
    args = parser.parse_args(argv)
    rclpy.init(args=[])
    node = OperatorTool()
    try:
        method = getattr(node, f'cmd_{args.command}')
        return int(method(args) or 0)
    except KeyboardInterrupt:
        return 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
