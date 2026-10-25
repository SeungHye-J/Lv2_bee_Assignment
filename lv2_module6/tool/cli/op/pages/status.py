"""상태·진단·RViz 화면."""
from __future__ import annotations

import os
import subprocess
from pathlib import Path

from ..defs import MAIN, QUIT
from ..terminal import DIM, GREEN, RED, YELLOW
from .base import GoMenu, Menu, MenuItem, Page


class ShowAllItem(MenuItem):
    key = '1'

    def label(self):
        return '전체 요약'

    def run(self):
        self.bus.cmd_status(self.bus.status_args())
        return 'stay'


class ShowOpStateItem(MenuItem):
    key = '2'

    def label(self):
        return '조작·지도 상태'

    def run(self):
        self.bus.show_operation_state()
        return 'stay'


class ShowJointsItem(MenuItem):
    key = '4'

    def label(self):
        return '관절 상태'

    def run(self):
        self.bus.show_joints()
        return 'stay'


class ShowDiagItem(MenuItem):
    key = '5'

    def label(self):
        return '진단 (/diagnostics_agg)'

    def run(self):
        self.bus.cmd_status(self.bus.status_args(diagnostics=True))
        return 'stay'


class ShowTopicsItem(MenuItem):
    key = '6'

    def label(self):
        return '토픽 (/state/topics)'

    def run(self):
        self.bus.cmd_status(self.bus.status_args(topics=True))
        return 'stay'


class StatusMenu(Menu):
    title = '상태'

    def items(self):
        return [ShowAllItem(self.app), ShowOpStateItem(self.app),
                GoMenu(self.app, '3', '장치 상태 / 상세', self.app.screen('2')),
                ShowJointsItem(self.app), ShowDiagItem(self.app), ShowTopicsItem(self.app)]

    def center(self):
        st = self.state
        return [
            '[조작·지도]',
            f' 조작: {(st.operation or "-")}',
            f' 지도: {(st.map_mode or "-")}',
            f' 이름: {st.values.get("map_name") or "-"}',
            f' E-STOP: {"ON" if st.emergency else "OFF"}',
            '',
        ] + self.ui.center_summary()


class DiagItem(MenuItem):
    """진단 항목 하나 — Enter가 상세 화면."""

    def __init__(self, app, key: str, name: str, level: int, message: str):
        super().__init__(app)
        self.key = key
        self._name = name
        self._level = level
        self._message = message

    def label(self):
        lv = self._level
        mark = self.term.paint('●', GREEN if lv == 0 else YELLOW if lv == 1 else RED if lv == 2 else DIM)
        return f'{mark} {self.bus.diag_short(self._name)} — {self._message}'

    def run(self):
        return DiagDetailPage(self.app, self._name).show()


class _EmptyItem(MenuItem):
    def __init__(self, app, key: str, text: str):
        super().__init__(app)
        self.key = key
        self._text = text

    def label(self):
        return self._text

    def run(self):
        return 'stay'


class DiagnosticsMenu(Menu):
    """진단 목록 (문제 있는 것 먼저) → 번호를 고르면 그 항목의 key-value 상세."""

    title = '진단'

    def items(self):
        entries = self.bus.diag_entries()
        if not entries:
            return [_EmptyItem(self.app, '1', '진단 없음 (/diagnostics_agg 대기 중 — bringup 확인)')]
        return [DiagItem(self.app, str(i), name, lv, msg)
                for i, (name, lv, msg) in enumerate(entries, 1)]

    def center(self):
        entries = self.bus.diag_entries()
        bad = sum(1 for _, lv, _ in entries if lv >= 1)
        return ([f'[진단 {len(entries)}개 · 주의 {bad}]', ' Enter = 그 항목 상세 (값 전부)',
                 ' 레벨: ● OK · ● WARN · ● ERROR · ● STALE(보고 끊김)']
                + self.ui.center_summary())


class DiagDetailPage(Page):
    """진단 항목 하나 — 2초마다 다시 그려 값이 바뀌는 게 보임."""

    def __init__(self, app, name: str):
        super().__init__(app)
        self._name = name
        self.title = f'진단 · {app.bus.diag_short(name)}'

    def show(self) -> str:
        from ..defs import BACK, MAIN, QUIT, TIMEOUT
        while True:
            choice = self.ui.select(self.title, [('r', '새로고침 (자동 갱신)')],
                                    self.bus.diag_detail_lines(self._name))
            if choice == TIMEOUT:
                continue
            if choice == BACK:
                return BACK
            if choice == MAIN:
                return MAIN
            if choice in ('q', 'quit', 'exit'):
                return QUIT
        return QUIT


class RvizPage(Page):
    title = 'RViz'

    def show(self) -> str:
        self.ui.header('RViz')
        if not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
            print('거부: DISPLAY/WAYLAND_DISPLAY가 없습니다 — GUI가 있는 터미널에서 실행하세요.')
            result = self.ui.pause(back_label='메인 메뉴')
            return result if result in (MAIN, QUIT) else 'stay'
        script = Path(__file__).resolve().parent.parent.parent / 'run-rviz'
        print('  카메라/점구름은 Best Effort, queue 1로 구독합니다.')
        print(f'  실행: {script}')
        if not script.is_file():
            print(f'거부: 실행 파일 없음: {script}')
            result = self.ui.pause(back_label='메인 메뉴')
            return result if result in (MAIN, QUIT) else 'stay'
        if not self.ui.confirm('RViz를 새 창으로 열까요?', default=True):
            return 'stay'
        log = Path('/tmp/operator_tool_rviz.log')
        try:
            with log.open('ab') as out:
                proc = subprocess.Popen(
                    [str(script)], cwd=str(script.parent),
                    stdout=out, stderr=out, start_new_session=True)
        except OSError as exc:
            print(f'RViz 실행 실패: {exc}')
            result = self.ui.pause(back_label='메인 메뉴')
            return result if result in (MAIN, QUIT) else 'stay'
        print(f'RViz 시작: pid {proc.pid} · 로그 {log}')
        result = self.ui.pause(back_label='메인 메뉴')
        return result if result in (MAIN, QUIT) else 'stay'
