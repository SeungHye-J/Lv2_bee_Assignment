"""Page(기반) → Menu(선택 루프) → 각 메뉴 화면.

메뉴 항목은 MenuItem 객체다: 라벨·노출 조건·실행을 캡슐화해서
키 문자열 if-chain 없이 다형성으로 동작한다.
"""
from __future__ import annotations

from abc import ABC, abstractmethod

import rclpy

from ..defs import BACK, MAIN, QUIT, TIMEOUT


class Page(ABC):
    """한 화면. show()는 BACK/MAIN/QUIT 중 하나를 돌려준다."""

    title: str = ''

    def __init__(self, app):
        self.app = app
        self.state = app.state
        self.bus = app.bus
        self.ui = app.ui
        self.term = app.term

    @abstractmethod
    def show(self) -> str:
        ...


class MenuItem(ABC):
    """메뉴 항목 한 개 — 라벨·노출·실행을 한 몸에."""

    #: True면 실행 중 print를 가로채지 않는다 (자체 화면을 그리는 항목용)
    raw_ui = False
    key: str = ''

    def __init__(self, app):
        self.app = app
        self.state = app.state
        self.bus = app.bus
        self.ui = app.ui
        self.term = app.term

    def label(self) -> str:
        return ''

    def visible(self) -> bool:
        return True

    @abstractmethod
    def run(self) -> str:
        """BACK/MAIN/QUIT/STAY — 프레임은 Menu가, 이동은 반환값으로."""
        ...


class GoMenu(MenuItem):
    """서브메뉴 진입 항목."""

    def __init__(self, app, key: str, label: str, page):
        super().__init__(app)
        self.key = key
        self._label = label
        self._page = page

    def label(self) -> str:
        return self._label

    def run(self) -> str:
        if self._page is None:
            return 'stay'
        return run_page(self.ui, self._page)


def run_page(ui, page) -> str:
    """서브 페이지 실행 — print 출력은 가로채서 결과 모달에 넣는다."""
    with ui.capture() as (buf_out, buf_err):
        result = page.show()
    out = (buf_out.getvalue() + buf_err.getvalue()).strip('\n')
    if out:
        nav = ui.pager('실행 결과', out)
        if nav in (MAIN, QUIT):
            return nav
        if nav == BACK:
            return BACK
    return result if result in (MAIN, QUIT, BACK) else 'stay'


class Menu(Page):
    """방향키 메뉴 — items()의 MenuItem들을 보여주고 고른 항목의 run()을 실행."""

    def items(self) -> list[MenuItem]:
        return []

    def center(self) -> list[str]:
        return []

    def show(self) -> str:
        self.ui.reset_selection()  # 메뉴 진입은 항상 첫 항목에 포커스
        while rclpy.ok():
            self.app.poll(0.2)
            visible = [i for i in self.items() if i.visible()]
            if not visible:
                return 'stay'
            keys = [(it.key, it.label()) for it in visible]
            choice = self.ui.select(self.title, keys, self.center())
            if choice == TIMEOUT:
                continue
            if choice == BACK:
                return BACK
            if choice == MAIN:
                return MAIN
            if choice in ('q', 'quit', 'exit'):
                return QUIT
            item = next((it for it in visible if it.key == choice), None)
            if item is None:
                continue
            if item.raw_ui:
                result = item.run()
            else:
                # 실행 중 print는 화면을 스크롤시키므로 가로채서 결과 모달에 넣는다.
                # 모달이 b/m/q 내비까지 맡으므로 뒤따르는 일시정지는 붙이지 않는다.
                with self.ui.capture() as (buf_out, buf_err):
                    result = item.run()
                out = (buf_out.getvalue() + buf_err.getvalue()).strip('\n')
                if out:
                    nav = self.ui.pager('실행 결과', out)
                    if nav in (MAIN, QUIT):
                        return nav
                    if nav == BACK:
                        return BACK
            if result in (MAIN, QUIT):
                return result
            if result == BACK:
                return BACK
