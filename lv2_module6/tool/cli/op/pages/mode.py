"""조작 모드·비상정지·재시작 메뉴."""
from __future__ import annotations

import argparse

from ..defs import MAIN, QUIT
from ..terminal import BOLD, GREEN, RED
from .base import Menu, MenuItem


class ModeItem(MenuItem):
    """조작 모드 전환 한 가지."""

    def __init__(self, app, key: str, target: str, desc: str):
        super().__init__(app)
        self.key = key
        self._target = target
        self._desc = desc

    def label(self):
        return f'{self._target:<10} — {self._desc}'

    def run(self):
        if self._target == 'AUTO' and not self.ui.confirm(
                'AUTO로 바꾸면 자동 주행이 시작될 수 있습니다. 전환할까요?'):
            return 'stay'
        if self._target == 'EXPLORE' and self.state.map_mode != 'MAPPING':
            print('WARN: EXPLORE는 지도 모드 MAPPING에서만 허용됩니다.')
            if not self.ui.confirm('그래도 EXPLORE로 전환할까요?'):
                return 'stay'
        self.bus.cmd_mode(argparse.Namespace(mode=self._target.lower()))
        return 'stay'


class ModeMenu(Menu):
    title = '조작 모드'

    def items(self):
        return [ModeItem(self.app, '1', 'AUTO', '자동 주행'),
                ModeItem(self.app, '2', 'MANUAL', '수동 주행/팔/그리퍼'),
                ModeItem(self.app, '3', 'EXPLORE', 'MAPPING 중 탐사')]

    def center(self):
        st = self.state
        return [
            '[현재 모드]',
            f' 조작: {st.operation or "-"}',
            f' E-STOP: {"ON" if st.emergency else "OFF"}',
            f' 지도: {st.map_mode or "-"}',
            '',
            ' AUTO 전환 시 자동 주행 시작 주의',
        ]


class EstopRefreshItem(MenuItem):
    key = '1'

    def label(self):
        return '상태 새로고침'

    def run(self):
        self.bus.cmd_estop(argparse.Namespace(state='status'))
        return 'stay'


class EstopOnItem(MenuItem):
    key = '2'

    def label(self):
        return '비상정지 ON'

    def run(self):
        if self.state.emergency:
            print('이미 비상정지 ON입니다.')
            return 'stay'
        if not self.ui.confirm('비상정지를 ON 할까요?', default=False):
            return 'stay'
        self.bus.cmd_estop(argparse.Namespace(state='on'))
        return 'stay'


class EstopOffItem(MenuItem):
    key = '3'

    def label(self):
        return '비상정지 OFF'

    def run(self):
        self.bus.cmd_estop(argparse.Namespace(state='off'))
        return 'stay'


class EstopMenu(Menu):
    title = '비상정지'

    def items(self):
        return [EstopRefreshItem(self.app), EstopOnItem(self.app), EstopOffItem(self.app)]

    def center(self):
        est = self.term.paint('ON', RED + BOLD) if self.state.emergency else self.term.paint('OFF', GREEN)
        return ['[비상정지]', f' 현재: {est}', '', ' ON은 바퀴 명령 차단']


class RestartItem(MenuItem):
    """재시작 대상 한 개 (노드 이름)."""

    def __init__(self, app, key: str, target: str):
        super().__init__(app)
        self.key = key
        self._target = target

    def label(self):
        return self._target

    def run(self):
        if not self.ui.confirm(f'{self._target} 노드를 재시작할까요?', default=False):
            return 'stay'
        self.bus.cmd_restart(argparse.Namespace(target=self._target, yes=True))
        return 'stay'


class RestartAllItem(MenuItem):
    key = 'a'

    def label(self):
        return '전체 스택 재시작'

    def run(self):
        print('WARN: 전체 스택이 재시작됩니다. 운영 중인 로봇에서는 주의하세요.')
        if not self.ui.confirm('정말 전체 스택을 재시작할까요?', default=False):
            return 'stay'
        confirm = self.ui.modal_input(
            '재시작 확인', ['확인을 위해 RESTART를 입력 (Esc=취소)'], '확인>')
        if confirm is None:
            return 'stay'
        if confirm.strip() != 'RESTART':
            print('취소: 입력이 일치하지 않습니다.')
            return 'stay'
        self.bus.cmd_restart(argparse.Namespace(target='all', yes=True))
        return 'stay'


class RestartNameItem(MenuItem):
    key = 'n'

    def label(self):
        return '노드 이름 직접 입력'

    def run(self):
        target = self.ui.modal_input('노드 이름', ['재시작할 노드 (all=전체 · Esc=취소)'], '이름>')
        if target is not None:
            target = target.strip()
        if not target:
            return 'stay'
        if target == 'all':
            return RestartAllItem(self.app).run()
        if not self.ui.confirm(f'{target} 노드를 재시작할까요?', default=False):
            return 'stay'
        self.bus.cmd_restart(argparse.Namespace(target=target, yes=True))
        return 'stay'


class _NoRestartItem(MenuItem):
    key = '1'

    def label(self):
        return '재시작 가능한 노드 없음'

    def run(self):
        return 'stay'


class RestartMenu(Menu):
    title = '재시작'

    def items(self):
        restartable = [line.split()[0] for line in
                       (self.state.values.get('restartable') or '').splitlines() if line.split()]
        items: list[MenuItem] = [RestartItem(self.app, str(i + 1), n)
                                 for i, n in enumerate(restartable)]
        if not items:
            items = [_NoRestartItem(self.app)]
        return items + [RestartAllItem(self.app), RestartNameItem(self.app)]

    def center(self):
        note = self.state.values.get('restart') or '-'
        return ['[재시작]', f' 최근 요청: {note}', '',
                ' 전체 재시작은 RESTART 입력 필요']
