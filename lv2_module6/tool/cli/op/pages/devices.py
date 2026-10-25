"""장치 메뉴 + 장치 상세 + 파라미터 도구."""
from __future__ import annotations

import sys

from ..consts import DIAG_LEVELS
from ..terminal import GREEN, RED, YELLOW
from .base import Menu, MenuItem


class DeviceItem(MenuItem):
    """장치 하나 — Enter가 상세 화면."""

    def __init__(self, app, key: str, name: str):
        super().__init__(app)
        self.key = key
        self._name = name

    def label(self):
        kind, state, _ = self.state.device_info(self._name)
        if state == 'active':
            mark = self.term.paint('●', GREEN)
        elif state:
            mark = self.term.paint('○', RED)
        else:
            mark = self.term.paint('?', YELLOW)
        return f'{mark} {self._name} ({kind})'

    def run(self):
        return DeviceDetailMenu(self.app, self._name).show()


class DevicesMenu(Menu):
    title = '장치'

    def items(self):
        if not self.state.devices:
            return [_EmptyItem(self.app)]
        return [DeviceItem(self.app, str(i + 1), n)
                for i, n in enumerate(self.state.devices)]

    def center(self):
        return ['[장치 상세는 Enter]'] + self.ui.center_summary()


class _EmptyItem(MenuItem):
    key = '1'

    def label(self):
        return '장치 목록 없음 (/state/devices 대기 중)'

    def run(self):
        return 'stay'


class ParamTool:
    """파라미터 조회/설정 — 장치 상세에서 쓴다."""

    def __init__(self, app):
        self.app = app
        self.bus = app.bus
        self.ui = app.ui

    def choose_node(self, topics) -> tuple[str, str] | None:
        cands = self.bus.param_target_nodes(topics)
        if not cands:
            print('거부: 이 장치의 발행 노드를 찾지 못했습니다 — bringup/토픽 확인',
                  file=sys.stderr)
            return None
        if len(cands) == 1:
            return cands[0]
        items = [(str(i + 1), f'{ns if ns.endswith("/") else ns + "/"}{n}')
                 for i, (n, ns) in enumerate(cands)]
        choice = self.ui.select('노드 선택', items, ['[파라미터 대상 노드]'])
        from ..defs import BACK, MAIN, TIMEOUT
        if choice in (BACK, MAIN, TIMEOUT, 'q', 'quit', 'exit'):
            return None
        for (n, ns), (key, _) in zip(cands, items):
            if key == choice:
                return n, ns
        print(f'거부: 모르는 번호: {choice}')
        return None

    def list(self, node_name: str, node_ns: str) -> int:
        return self.bus.param_act_list(node_name, node_ns)

    def get(self, node_name: str, node_ns: str) -> int:
        return self.bus.param_act_get(node_name, node_ns)

    def set(self, node_name: str, node_ns: str) -> int:
        return self.bus.param_act_set(node_name, node_ns)


class ParamListItem(MenuItem):
    key = '1'

    def __init__(self, app, params: ParamTool, topics):
        super().__init__(app)
        self._params = params
        self._topics = topics

    def label(self):
        return '파라미터 목록'

    def run(self):
        node = self._params.choose_node(self._topics)
        if node is None:
            return 'stay'
        self._params.list(*node)
        return 'stay'


class ParamGetItem(MenuItem):
    key = '2'

    def __init__(self, app, params: ParamTool, topics):
        super().__init__(app)
        self._params = params
        self._topics = topics

    def label(self):
        return '값 보기 (get)'

    def run(self):
        node = self._params.choose_node(self._topics)
        if node is None:
            return 'stay'
        if self._params.get(*node) is None:
            return 'stay'  # Esc=그냥 닫기
        return 'stay'


class ParamSetItem(MenuItem):
    key = '3'

    def __init__(self, app, params: ParamTool, topics):
        super().__init__(app)
        self._params = params
        self._topics = topics

    def label(self):
        return '설정 (set)'

    def run(self):
        node = self._params.choose_node(self._topics)
        if node is None:
            return 'stay'
        if self._params.set(*node) is None:
            return 'stay'  # Esc=그냥 닫기
        return 'stay'


class ShowFullItem(MenuItem):
    key = '4'

    def __init__(self, app, detail: 'DeviceDetailMenu'):
        super().__init__(app)
        self._detail = detail

    def label(self):
        return '전체 상세 보기'

    def run(self):
        self._detail.show_full()
        return 'stay'


class DeviceDetailMenu(Menu):
    def __init__(self, app, name: str):
        super().__init__(app)
        self._name = name
        self.title = f'장치 · {name}'
        self._params = ParamTool(app)

    def items(self):
        topics = self._topics()
        return [ParamListItem(self.app, self._params, topics),
                ParamGetItem(self.app, self._params, topics),
                ParamSetItem(self.app, self._params, topics),
                ShowFullItem(self.app, self)]

    def _topics(self):
        _, _, topics = self.state.device_info(self._name)
        return topics

    def center(self):
        kind, state, topics = self.state.device_info(self._name)
        if state == 'active':
            state_txt = self.term.paint('● active', GREEN)
        elif state:
            state_txt = self.term.paint(f'○ {state}', RED)
        else:
            state_txt = self.term.paint('? 상세 정보 없음', YELLOW)
        center = [f'[장치 · {self._name}]', f' 종류: {kind}', f' 상태: {state_txt}',
                  f' 토픽/관절: {len(topics)}개']
        center += [f'  - {t}' for t in topics[:6]]
        if len(topics) > 6:
            center.append(f'  … 외 {len(topics) - 6}개')
        related = []
        for diag_name, (level, message) in self.state.diagnostics.items():
            if self._name in diag_name or any(t in diag_name for t in topics):
                related.append((diag_name, level, message))
        self._related = sorted(related)[:3]
        if related:
            center.append(' 관련 진단:')
            for diag_name, level, message in self._related:
                center.append(f'  - {diag_name}: {DIAG_LEVELS.get(level, level)}')
        else:
            center.append(' 관련 진단: -')
        self._state_txt = state_txt
        self._kind = kind
        return center

    def show_full(self) -> None:
        """장치 전체 상세 (잘림 없이 전부)."""
        self.app.poll(0.45)
        print(f'  종류   : {self._kind}')
        print(f'  상태   : {self._state_txt}')
        print('  세부 토픽/관절:')
        topics = self._topics()
        if topics:
            for topic in topics:
                print(f'    - {topic}')
        else:
            print('    -')
        if self._related:
            print('  관련 진단:')
            for diag_name, level, message in self._related:
                print(f'    - {diag_name}: {level} {message}')
        else:
            print('  관련 진단: -')
