#!/usr/bin/env python3
"""운영자 수동 조작 CLI.

bringup이 떠 있는 상태에서 상태 확인, 자동/수동 전환, 비상정지, 바퀴·팔·그리퍼
수동 명령, 지도 모드/지도 전환을 한 곳에서 실행한다.

  ./operator_tool/run.sh status
  ./operator_tool/run.sh mode manual
  ./operator_tool/run.sh teleop
  ./operator_tool/run.sh arm set joint1=0.0 joint2=0.4
  ./operator_tool/run.sh map save

모든 수동 명령은 먼저 `mode manual`이어야 하고, 비상정지 중에는 거부한다.
실제 명령은 기존 control/manual_control 노드가 받아 안전 체인으로 보낸다.
"""

from __future__ import annotations

import argparse
import math
import os
import select
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
}

DIAG_LEVELS = {0: 'OK', 1: 'WARN', 2: 'ERROR', 3: 'STALE'}


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
        # operator_tool/operator_tool.py → lv2_module6/ros2_ws/map
        return Path(__file__).resolve().parents[1] / 'ros2_ws' / 'map'

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
  ./operator_tool/run.sh status
  ./operator_tool/run.sh mode manual
  ./operator_tool/run.sh drive forward --speed 0.15 --duration 1
  ./operator_tool/run.sh teleop
  ./operator_tool/run.sh arm set joint1=0.0 joint2=0.4
  ./operator_tool/run.sh gripper open
  ./operator_tool/run.sh map status
  ./operator_tool/run.sh map save
''')
    sub = parser.add_subparsers(dest='command', required=True)

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
    if not argv:  # 인자 없이 실행하면 상태부터 보여 준다
        argv = ['status']
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
