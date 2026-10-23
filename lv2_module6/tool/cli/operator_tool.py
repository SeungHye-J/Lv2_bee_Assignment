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
import select
import subprocess
import sys
import termios
import time
import tty
from pathlib import Path

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
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

# 대화형 메뉴 반환값
BACK = 'back'
MAIN = 'main'
QUIT = 'quit'
STAY = 'stay'
REFRESH = 'refresh'

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

        # 대화형 메뉴 설정값
        self._ui_linear = 0.15
        self._ui_angular = 0.8
        self._ui_duration = 1.0
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

    # ------------------------------------------------------------------ ROS
    def _on_emergency(self, msg: Bool):
        self._emergency = bool(msg.data)

    def _on_devices(self, msg: String):
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
        for name, position in zip(msg.name, msg.position):
            self._joints[name] = position
        self._last_joint_time = time.monotonic()

    def _on_diagnostics(self, msg: DiagnosticArray):
        for status in msg.status:
            self._diagnostics[status.name] = (status.level, status.message)

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

    def _wait_value(self, key: str, expected: str, timeout: float = 2.5) -> bool:
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
        self._poll(0.5)
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
    def cmd_status(self, args) -> int:
        self._poll(1.2)
        if self._devices:
            self._poll(0.35)  # /state/devices/<이름> 상세

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
            values = [f'{name}={self._joints[name]:.3f}' for name in
                      (*ARM_JOINTS, GRIPPER_JOINT) if name in self._joints]
            if values:
                print('관절        : ' + '  '.join(values))

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
        if not self._ensure_manual(args.force):
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
        if not self._ensure_manual(args.force):
            return 1
        if not sys.stdin.isatty():
            print('거부: teleop은 터미널에서 실행해야 합니다', file=sys.stderr)
            return 2

        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        linear = 0.0
        angular = 0.0
        last_send = 0.0
        print('수동 주행: w/s 전후진 · a/d 좌우회전 · space 정지 · q 종료 · h 도움말')
        try:
            tty.setcbreak(fd)
            while rclpy.ok():
                now = time.monotonic()
                if now - last_send >= 0.1:
                    self._send_velocity(linear, angular)
                    last_send = now
                ready, _, _ = select.select([sys.stdin], [], [], 0.05)
                if ready:
                    key = sys.stdin.read(1)
                    if key in ('q', '\x03'):
                        break
                    if key == 'w':
                        linear, angular = args.linear, 0.0
                    elif key == 's':
                        linear, angular = -args.linear, 0.0
                    elif key == 'a':
                        linear, angular = 0.0, args.angular
                    elif key == 'd':
                        linear, angular = 0.0, -args.angular
                    elif key == ' ':
                        linear = angular = 0.0
                    elif key == 'h':
                        print('\nw/s 전후진 · a/d 좌우회전 · space 정지 · q 종료\n')
                    else:
                        continue
                    print(f'linear={linear:.2f} angular={angular:.2f}')
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
            self._poll(0.5)
            values = [f'{name}={self._joints[name]:.3f}' for name in ARM_JOINTS if name in self._joints]
            if not values:
                print('/joint_states 없음 — bringup/OpenCR 확인', file=sys.stderr)
                return 1
            print('  '.join(values))
            return 0

        if not self._ensure_arm(args.force):
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
                self._poll(0.5)
                for name, delta in list(targets.items()):
                    if name not in self._joints:
                        print(f'거부: {name} 현재 각을 모름 (/joint_states 대기)', file=sys.stderr)
                        return 1
                    targets[name] = self._joints[name] + delta

        # manual_control은 목표에 없는 관절을 /joint_states의 현재 각으로 채운다.
        # 그 관절의 현재 각이 없으면 명령 전체를 보류하므로 여기서 먼저 막는다.
        self._poll(0.5)
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
            self._poll(0.5)
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

        if not self._ensure_manual(args.force):
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

    def _ui_header(self, title: str):
        self._poll(0.35)
        self._ui_clear()
        bold = BOLD if self._ui_color else ''
        reset = RESET if self._ui_color else ''
        print(f'{bold}{"=" * 64}{reset}')
        print(f'{bold}  BeeBot Operator Console  ·  {title}{reset}')
        print(f'{bold}{"=" * 64}{reset}')

        op = self.operation or '?'
        mode = self.map_mode or '?'
        estop = self._ui_paint('ON', RED + BOLD) if self._emergency else 'OFF'
        active = 0
        for name in self._devices:
            _, state, _ = self._device_info(name)
            if state == 'active':
                active += 1
        print(f'  조작 {op:9s} | 지도 {mode:12s} | E-STOP {estop:3s} | '
              f'장치 {active}/{len(self._devices)}')
        if not self._values and not self._devices and not self._joints:
            print(f'  {self._ui_paint("상태 없음 — bringup이 떠 있는지 확인", YELLOW)}')
        print('-' * 64)

    def _ui_footer(self, back: bool = True, main: bool = True):
        print('-' * 64)
        hints = []
        if back:
            hints.append('b) 뒤로')
        if main:
            hints.append('m) 메인 메뉴')
        hints.append('q) 종료')
        print('  ' + '   '.join(hints))

    @staticmethod
    def _ui_prompt() -> str:
        try:
            return input('선택> ').strip().lower()
        except EOFError:
            return 'q'

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
            self._ui_header('메인 메뉴')
            print('  1) 상태')
            print('  2) 장치')
            print('  3) 조작 모드')
            print('  4) 수동 주행 / 텔레오퍼')
            print('  5) 팔')
            print('  6) 그리퍼')
            print('  7) 지도')
            print('  8) 비상정지')
            print('  9) 재시작')
            print(' 10) RViz 열기 (Best Effort QoS)')
            print()
            print('  번호를 고르거나 명령 이름(status, devices, teleop …)을 입력하세요.')
            self._ui_footer(back=False, main=False)
            choice = self._ui_prompt()
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
            self._ui_header('상태')
            print('  1) 전체 요약')
            print('  2) 조작·지도 상태')
            print('  3) 장치 상태 / 상세')
            print('  4) 관절 상태')
            print('  5) 진단 (/diagnostics_agg)')
            print('  6) 토픽 (/state/topics)')
            self._ui_footer()
            choice = self._ui_prompt()
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
        for name in (*ARM_JOINTS, GRIPPER_JOINT):
            if name in self._joints:
                print(f'  {name:18s} {self._joints[name]: .4f} rad')
                found = True
        if not found:
            print('/joint_states에 팔 관절이 없습니다 — bringup/OpenCR 확인', file=sys.stderr)

    def _menu_devices(self) -> str:
        while rclpy.ok():
            self._poll(0.6)
            self._ui_header('장치')
            if not self._devices:
                print('  장치 목록 없음 — /state/devices 대기 중')
            else:
                for index, name in enumerate(self._devices, 1):
                    kind, state, topics = self._device_info(name)
                    if state == 'active':
                        mark = self._ui_paint('● active', GREEN)
                    elif state:
                        mark = self._ui_paint(f'○ {state}', RED)
                    else:
                        mark = self._ui_paint('? 상세 없음', YELLOW)
                    print(f'  {index:2d}) {name:20s} {kind:8s} {mark}  ({len(topics)} topics)')
            self._ui_footer()
            choice = self._ui_prompt()
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

    def _device_detail_menu(self, name: str) -> str:
        while rclpy.ok():
            self._poll(0.45)
            kind, state, topics = self._device_info(name)
            self._ui_header(f'장치 · {name}')
            print(f'  종류   : {kind}')
            if state == 'active':
                print(f'  상태   : {self._ui_paint("● active", GREEN)}')
            elif state:
                print(f'  상태   : {self._ui_paint(f"○ {state}", RED)}')
            else:
                print(f'  상태   : {self._ui_paint("? 상세 정보 없음", YELLOW)}')
            print('  세부 토픽/관절:')
            if topics:
                for topic in topics:
                    print(f'    - {topic}')
            else:
                print('    -')

            related = []
            for diag_name, (level, message) in self._diagnostics.items():
                if name in diag_name or any(topic in diag_name for topic in topics):
                    related.append((diag_name, level, message))
            if related:
                print('  관련 진단:')
                for diag_name, level, message in sorted(related):
                    print(f'    - {diag_name}: {DIAG_LEVELS.get(level, level)} {message}')
            else:
                print('  관련 진단: -')

            action = self._ui_pause(back_label='장치 목록', refresh=True)
            if action == BACK:
                return BACK
            if action == MAIN:
                return MAIN
            if action == QUIT:
                return QUIT

    def _menu_mode(self) -> str:
        targets = {'1': 'AUTO', '2': 'MANUAL', '3': 'EXPLORE'}
        while rclpy.ok():
            self._ui_header('조작 모드')
            print(f'  현재: {self.operation or "-"}   비상정지: {"ON" if self._emergency else "OFF"}')
            print('  1) AUTO      — 자동 주행')
            print('  2) MANUAL    — 수동 주행/팔/그리퍼')
            print('  3) EXPLORE   — MAPPING 중 탐사')
            self._ui_footer()
            choice = self._ui_prompt()
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
            self._ui_header('비상정지')
            state = self._ui_paint('ON', RED + BOLD) if self._emergency else 'OFF'
            print(f'  현재 비상정지: {state}')
            print('  1) 상태 새로고침')
            print('  2) 비상정지 ON')
            print('  3) 비상정지 OFF')
            self._ui_footer()
            choice = self._ui_prompt()
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
            self._ui_header('수동 주행 / 텔레오퍼')
            print(f'  현재 모드: {self.operation or "-"}   '
                  f'비상정지: {"ON" if self._emergency else "OFF"}')
            print(f'  설정: 선속도 {self._ui_linear:.2f} m/s · '
                  f'각속도 {self._ui_angular:.2f} rad/s · 시간 {self._ui_duration:.2f}s')
            print('  1) 텔레오퍼 시작 (w/s/a/d · space 정지 · q 종료)')
            print('  2) 전진')
            print('  3) 후진')
            print('  4) 좌회전')
            print('  5) 우회전')
            print('  6) 정지')
            print('  7) 속도/시간 설정')
            self._ui_footer()
            choice = self._ui_prompt()
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
            self._ui_header('수동 주행 설정')
            print(f'  1) 선속도   : {self._ui_linear:.2f} m/s')
            print(f'  2) 각속도   : {self._ui_angular:.2f} rad/s')
            print(f'  3) 주행시간 : {self._ui_duration:.2f} s')
            self._ui_footer()
            choice = self._ui_prompt()
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
            self._ui_header('팔')
            values = [f'{name}={self._joints[name]:.3f}' for name in ARM_JOINTS
                      if name in self._joints]
            print('  현재: ' + ('  '.join(values) if values else '(joint_states 없음)'))
            if self.operation != 'MANUAL':
                print(f'  {self._ui_paint("MANUAL 모드가 아닙니다", YELLOW)}')
            if self.map_mode == 'MAPPING':
                print(f'  {self._ui_paint("MAPPING 중에는 팔 명령이 거절됩니다", YELLOW)}')
            print('  1) 관절 상태')
            print('  2) 관절 목표 설정')
            print('  3) 관절 델타 이동')
            print('  4) 홈 (모든 팔 관절 0)')
            self._ui_footer()
            choice = self._ui_prompt()
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
            self._ui_header('그리퍼')
            if GRIPPER_JOINT in self._joints:
                print(f'  현재: {GRIPPER_JOINT}={self._joints[GRIPPER_JOINT]:.4f} m')
            else:
                print('  현재: (joint_states 없음)')
            print('  1) 상태')
            print('  2) 열기')
            print('  3) 닫기')
            print('  4) 위치 지정')
            self._ui_footer()
            choice = self._ui_prompt()
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
            self._ui_header('지도')
            maps = self._map_names()
            print(f'  모드: {self.map_mode or "-"}   이름: {self._values.get("map_name") or "-"}')
            print(f'  지도 목록: {len(maps)}개')
            print('  1) 지도 상태')
            print('  2) 지도 목록')
            print('  3) MAPPING 모드')
            print('  4) LOCALIZATION 모드')
            print('  5) 지도 전환')
            print('  6) 지도 저장')
            self._ui_footer()
            choice = self._ui_prompt()
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
            self._ui_header('지도 전환')
            if maps:
                for index, name in enumerate(maps, 1):
                    print(f'  {index:2d}) {name}')
            else:
                print('  지도 목록 없음 — 이름을 직접 입력하세요.')
            print('   n) 이름 직접 입력')
            self._ui_footer()
            choice = self._ui_prompt()
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
            self._ui_header('재시작')
            note = self._values.get('restart') or '-'
            print(f'  최근 요청: {note}')
            restartable = [line.split()[0] for line in
                           (self._values.get('restartable') or '').splitlines() if line.split()]
            if restartable:
                for index, name in enumerate(restartable, 1):
                    print(f'  {index:2d}) {name}')
            else:
                print('  재시작 가능한 노드 목록 없음')
            print('   a) 전체 스택 재시작')
            print('   n) 노드 이름 직접 입력')
            self._ui_footer()
            choice = self._ui_prompt()
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
