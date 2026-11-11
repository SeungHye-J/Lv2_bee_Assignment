"""미션 메뉴 — mission_manager에 시작·중지(/cmd/mission)를 보내고, /mission/state를 중앙에 보여준다."""
from __future__ import annotations

from ...defs import STAY
from ...menus import Menu, MenuItem


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


class MissionMenu(Menu):
    title = '미션'

    def items(self):
        return [MissionCmdItem(self.app, '1', 'start', '미션 시작', '미션을 시작할까요?'),
                MissionCmdItem(self.app, '2', 'abort', '미션 중지 (abort)', '미션을 중지할까요? (coordinator도 정지)')]

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
