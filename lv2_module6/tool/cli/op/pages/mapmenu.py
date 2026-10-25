"""지도 메뉴 + 전환 + 저장."""
from __future__ import annotations

from .base import Menu, MenuItem, Page


class MapStatusItem(MenuItem):
    key = '1'

    def label(self):
        return '지도 상태'

    def run(self):
        self.bus.cmd_map(self.bus.map_args('status'))
        return 'stay'


class MapListItem(MenuItem):
    key = '2'

    def label(self):
        return '지도 목록'

    def run(self):
        self.bus.cmd_map(self.bus.map_args('list'))
        return 'stay'


class MapModeItem(MenuItem):
    def __init__(self, app, key: str, mode: str, desc: str):
        super().__init__(app)
        self.key = key
        self._mode = mode
        self._desc = desc

    def label(self):
        return f'{self._desc} 모드'

    def run(self):
        if not self.ui.confirm(f'{self._desc} 모드로 전환할까요?', default=False,
                              body=[f'현재: {self.state.map_mode or "-"}']):
            return 'stay'
        self.bus.cmd_map(self.bus.map_args('mode', name=self._mode))
        return 'stay'


class MapMenu(Menu):
    title = '지도'

    def items(self):
        return [MapStatusItem(self.app), MapListItem(self.app),
                MapModeItem(self.app, '3', 'mapping', 'MAPPING'),
                MapModeItem(self.app, '4', 'localization', 'LOCALIZATION'),
                MapSwitchItem(self.app), MapSaveItem(self.app)]

    def center(self):
        st = self.state
        maps = self.bus.map_names()
        return (['[지도]',
                 f' 모드: {st.map_mode or "-"}',
                 f' 이름: {st.values.get("map_name") or "-"}',
                 f' 목록: {len(maps)}개'] + [f'  - {n}' for n in maps[:8]])


class MapSwitchItem(MenuItem):
    key = '5'

    def label(self):
        return '지도 전환'

    def run(self):
        return MapSwitchMenu(self.app).show()


class MapSaveItem(MenuItem):
    key = '6'

    def label(self):
        return '지도 저장'

    def run(self):
        return MapSavePage(self.app).show()


class SwitchTargetItem(MenuItem):
    """전환할 지도 한 개."""

    def __init__(self, app, key: str, name: str):
        super().__init__(app)
        self.key = key
        self._name = name

    def label(self):
        return self._name

    def run(self):
        if not self.ui.confirm(f'지도 "{self._name}"(으)로 전환할까요?', default=False,
                              body=[f'현재: {self.state.values.get("map_name") or "-"}']):
            return 'stay'
        self.bus.cmd_map(self.bus.map_args('switch', name=self._name))
        return 'stay'


class SwitchNameItem(MenuItem):
    key = 'n'

    def label(self):
        return '이름 직접 입력'

    def run(self):
        name = self.ui.modal_input('지도 이름', ['전환할 지도 폴더 이름 (Esc=취소)'], '이름>')
        if name is not None:
            name = name.strip()
        if not name:
            return 'stay'
        self.bus.cmd_map(self.bus.map_args('switch', name=name))
        return 'stay'


class MapSwitchMenu(Menu):
    title = '지도 전환'

    def items(self):
        maps = self.bus.map_names()
        items: list[MenuItem] = [SwitchTargetItem(self.app, str(i + 1), n)
                                 for i, n in enumerate(maps)]
        if not items:
            items = [_NoMapItem(self.app)]
        return items + [SwitchNameItem(self.app)]

    def center(self):
        st = self.state
        return ['[지도 전환]', f' 현재: {st.values.get("map_name") or "-"}',
                f' 모드: {st.map_mode or "-"}']


class _NoMapItem(MenuItem):
    key = '1'

    def label(self):
        return '지도 목록 없음'

    def run(self):
        return 'stay'


class MapSavePage(Page):
    title = '지도 저장'

    def show(self) -> str:
        from ..defs import BACK, MAIN, QUIT
        st = self.state
        name = st.values.get('map_name') or ''
        default = self.bus.default_map_dir() / (name or '<지도이름>') / 'map'
        self.ui.header('지도 저장')
        print(f'  현재 지도: {name or "-"}')
        print(f'  기본 경로: {default}')
        if not self.ui.confirm('현재 /map을 저장할까요?', default=False):
            return 'stay'
        raw = self.ui.modal_input('저장 경로', ['(Enter=기본 · Esc=뒤로)'], '경로>')
        if raw is None:
            return BACK
        raw = raw.strip()
        if raw.lower() in ('b', 'back'):
            return BACK
        if not name and not raw:
            print('거부: 현재 지도 이름이 없습니다 — 저장 경로를 입력하세요.')
            return 'stay'
        self.bus.cmd_map(self.bus.map_args('save', file=raw or None))
        return 'stay'
