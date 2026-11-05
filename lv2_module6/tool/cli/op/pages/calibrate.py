"""칼리브레이션 메뉴 — 지금은 바퀴. 팔(calibrate arm)은 나중에 이 메뉴에 항목으로 붙인다."""
from __future__ import annotations

import argparse
from pathlib import Path

from ..calibrate import load_plan
from ..defs import STAY
from .base import GoMenu, Menu, MenuItem

DEFAULT_PLAN = Path(__file__).resolve().parents[3] / 'calibration' / 'wheel_square.json'   # tool/calibration/ — sim과 같은 네모 54 s


class WheelState:
    """바퀴 칼리브레이션 화면이 공유하는 선택값 — 지금 계획 파일."""
    plan = str(DEFAULT_PLAN)


def _plan_text() -> list[str]:
    try:
        plan = load_plan(WheelState.plan)
    except ValueError as exc:
        return [f' 계획: {WheelState.plan}', f' ⚠ {exc}']
    n = len(plan['steps'])
    return [f' 계획: {Path(WheelState.plan).name} — {n}스텝 · {n * plan["dt"]:.1f} s']


def _args(check_only: bool) -> argparse.Namespace:
    return argparse.Namespace(part='wheel', plan=WheelState.plan, out=None, check_only=check_only, force=False)


class WheelCheckItem(MenuItem):
    key = '1'

    def label(self):
        return '준비 확인 (안 움직임)'

    def run(self):
        self.bus.cmd_calibrate(_args(True))
        return STAY


class WheelRunItem(MenuItem):
    key = '2'

    def label(self):
        return '재생 + 기록 (로봇이 움직임)'

    def run(self):
        try:
            plan = load_plan(WheelState.plan)
        except ValueError as exc:
            print(f'거부: {exc}')
            return STAY
        body = [f'{len(plan["steps"])}스텝 · {len(plan["steps"]) * plan["dt"]:.1f} s · {Path(WheelState.plan).name}',
                '로봇이 움직입니다 — 사방 약 1.5 m가 비어 있어야 합니다 (팔은 시작 자세)',
                '보정(rl_nav_correction)은 건너뛰는 수동 경로입니다 (순수 응답)',
                '멈추려면: Ctrl+C · 다른 터미널 ./run-cli estop on']
        if not self.ui.confirm('재생할까요?', default=False, body=body):
            return STAY
        self.bus.cmd_calibrate(_args(False))   # MANUAL이 아니면 전환을 먼저 묻는다
        return STAY


class WheelPlanItem(MenuItem):
    key = '3'

    def label(self):
        return '계획 파일 바꾸기'

    def run(self):
        def check(raw):
            try:
                load_plan(str(Path(raw).expanduser()))
            except ValueError as exc:
                return str(exc)
            return None

        raw = self.ui.modal_input('계획 파일', ['{"dt": 0.05, "steps": [[v, w], ...]} JSON — 기본은 sim과 같은 네모'],
                                  '경로', default=WheelState.plan, validate=check)
        if raw is not None:
            WheelState.plan = str(Path(raw).expanduser())
        return STAY


class WheelCalibrateMenu(Menu):
    title = '칼리브레이션 — 바퀴'

    def items(self):
        return [WheelCheckItem(self.app), WheelRunItem(self.app), WheelPlanItem(self.app)]

    def center(self):
        st = self.state
        return [
            '[바퀴 칼리브레이션]',
            f' 모드: {st.operation or "-"} (재생 때 MANUAL이 아니면 전환을 묻습니다)',
            f' E-STOP: {"ON" if st.emergency else "OFF"}',
            *_plan_text(),
            '',
            ' 하는 일: 계획의 (v, w)를 /cmd/manual/cmd_vel로 20 Hz 재생하며',
            '   /odom · /imu · /joint_states · /cmd_vel을 raw.csv로 기록',
            ' 저장: run-cli를 실행한 폴더의 calibration_<시각>/raw.csv',
            ' 순서: 1) 준비 확인 → 공간 비움 · 팔 시작 자세 → 2) 재생',
            ' PC로: scp bee@bee.local:<폴더>/raw.csv … 후 isaac report.py (sim과 비교)',
        ]


class CalibrateMenu(Menu):
    title = '칼리브레이션'

    def items(self):
        return [GoMenu(self.app, '1', '바퀴', WheelCalibrateMenu(self.app))]

    def center(self):
        return ['[칼리브레이션]', ' 1) 바퀴 — 모터 응답(gain · 지연)을 재서 sim에 맞춤', ' (팔은 나중에 여기에 추가)']
