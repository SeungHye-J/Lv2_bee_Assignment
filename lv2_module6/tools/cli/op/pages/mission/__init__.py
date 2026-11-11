"""미션 메뉴 — mission_manager에 시작·중지(/cmd/mission)를 보내고, /mission/state를 중앙에 보여준다."""
from __future__ import annotations

import argparse

from ...defs import STAY
from ...menus import GoMenu, Menu, MenuItem


class MissionCmdItem(MenuItem):
    def __init__(self, app, key: str, cmd: str, label: str, ask: str):
        super().__init__(app)
        self.key, self._cmd, self._label, self._ask = key, cmd, label, ask

    def label(self):
        return self._label

    def run(self):
        if not self.ui.confirm(self._ask, default=False, body=[f'현재 단계: {self._phase()}']):
            return STAY
        self.bus.cmd_mission(self._cmd)
        return STAY

    def _phase(self):
        m = self.state.mission
        return m.phase if m else '-'


# 세부 명령 — skill마다 (키, 이름, target 설명, 예). 정의: documents/project/ros/05-arm.md 3절
SKILLS = [
    ('1', 'navigate', '이동', ['물체 id 또는 "x,y[,yaw]" (map 좌표)'], '1.5,0.8,0'),
    ('2', 'approach', '박스 앞 직선 접근', ['"x,y,yaw" (map) — 박스 앞 마지막 몇 cm를 저속 직선'], '1.5,0.8,0'),
    ('3', 'pose', '팔 자세', ['stow · carry_left · carry_right · scan · look'], 'stow'),
    ('4', 'inspect', '퍽 보기 (태그 읽기)', ['퍽 id 또는 "x,y,z"'], '5'),
    ('5', 'pick', '집기', ['퍽 id (/tracked_objects)'], '5'),
    ('6', 'place', '놓기', ['장소 상자 id 또는 "x,y,z"'], '14'),
    ('7', 'load', '적재함에 싣기', ['칸 0~11 또는 next'], 'next'),
    ('8', 'unload', '적재함에서 꺼내기', ['칸 0~11 또는 last'], 'last'),
    ('9', 'stand', '누운 퍽 세우기', ['누운 퍽 id'], '5'),
    ('0', 'stand_pick', '세운 퍽 다시 집기', ['직전 stand의 퍽 id (기록용)'], '5'),
]


class SkillItem(MenuItem):
    def __init__(self, app, key: str, skill: str, desc: str, hint: list[str], example: str):
        super().__init__(app)
        self.key, self._skill, self._desc, self._hint, self._example = key, skill, desc, hint, example

    def label(self):
        return f'{self._skill} — {self._desc}'

    def run(self):
        m = self.state.mission
        busy = m is not None and m.phase not in ('IDLE', 'DONE', 'ERROR')
        target = self.ui.modal_input(
            f'{self._skill} — {self._desc}',
            self._hint + [f'예: {self._example}'] +
            ([f'주의: 미션이 {m.phase} 단계 — 같은 coordinator를 쓰니 먼저 미션 중지'] if busy else []),
            'target>', validate=lambda t: None if t.strip() else 'target이 비었습니다')
        if target is None:
            return STAY
        target = target.strip()
        if not self.ui.confirm(f'{self._skill} "{target}" 을(를) 실행할까요?', default=False,
                               body=['로봇이 움직입니다.', f'조작 모드: {self.state.operation or "-"}']):
            return STAY
        if self.state.operation != 'AUTO':
            if not self.ui.confirm('AUTO로 전환할까요?', default=True,
                                   body=['coordinator는 AUTO에서만 계획을 받습니다.']):
                return STAY
            self.bus.cmd_mode(argparse.Namespace(mode='auto'))
        self.bus.cmd_skill(self._skill, target)
        return STAY


class SkillStopItem(MenuItem):
    key = 's'

    def label(self):
        return '진행 중 전부 취소 (정지)'

    def run(self):
        self.bus.cmd_skill('stop', '')
        return STAY


class SkillMenu(Menu):
    title = '세부 명령 (skill 직접)'

    def items(self):
        return [SkillItem(self.app, *s) for s in SKILLS] + [SkillStopItem(self.app)]

    def center(self):
        return ['[세부 명령]', f' 조작: {self.state.operation or "-"}',
                ' mission_manager가 쓰는 길(/task/sub_goals → coordinator → /skills/<이름>)로 한 단계만 보냄',
                ' 미션이 돌고 있으면 먼저 중지 · AUTO에서만 실행',
                f' task: {self.state.task_status or "-"}']


class MissionMenu(Menu):
    title = '미션'

    def items(self):
        return [MissionCmdItem(self.app, '1', 'start', '미션 시작', '미션을 시작할까요?'),
                MissionCmdItem(self.app, '2', 'abort', '미션 중지 (abort)', '미션을 중지할까요? (coordinator도 정지)'),
                GoMenu(self.app, '3', '세부 명령 (navigate · pick · place … skill 직접)', SkillMenu(self.app))]

    def center(self):
        m = self.state.mission
        if m is None:
            return ['[미션]', ' /mission/state 대기 중 — mission_manager(plan) 확인']
        lines = ['[미션 상태]',
                 f' 단계: {m.phase}',
                 f' 하는 일: {m.note or "-"}',
                 f' 놓은 퍽: {m.placed}/{m.total}',
                 f' 바코드: {m.barcode or "-"}',
                 f' 신호: R{m.count_r} Y{m.count_y} G{m.count_g} · 순서 {m.order or "-"}',
                 f' 적재함: {" ".join(m.tray) or "-"}']
        if m.error:
            lines.append(f' ERROR: {m.error}')
        return lines + [' 결과:'] + [f'  - {r}' for r in list(m.results)[-8:]]
