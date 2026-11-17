"""PerceptionState — 지금 보이는 물체 수 칸 (/tracked_objects — 2 s 안 보이면 빠짐). Enter → 보이는 물체 화면.
주행 중 인지를 쉬는 동안(perception 진단 '주행 중 인지' = 쉼)은 ' · 주행 중 쉼'을 붙임 — 물체 수가 안 바뀌는 이유."""
from __future__ import annotations

from ...fielddebug import perception_diag
from .state import State


class PerceptionState(State):
    focusable = True

    def activate(self):
        return self.ui.window.navigate('objects')

    def text(self) -> str:
        rest = perception_diag(getattr(self.state, 'diag_values', {})).get('주행 중 인지') == '쉼'
        return f'보이는 물체 {self.state.tracked_count}' + (' · 주행 중 쉼' if rest else '')
