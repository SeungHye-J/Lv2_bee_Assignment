"""메인 메뉴."""
from __future__ import annotations

import time

import rclpy

from ..defs import BACK, MAIN, QUIT, TIMEOUT
from .base import Menu


class MainMenu(Menu):
    title = '메인 메뉴'

    def items(self):
        return [
            ('1', '상태'),
            ('2', '장치'),
            ('3', '조작 모드'),
            ('4', '수동 주행 / 텔레오퍼'),
            ('5', '팔'),
            ('6', '그리퍼'),
            ('7', '지도'),
            ('8', '비상정지'),
            ('9', '재시작'),
            ('10', 'RViz 열기 (Best Effort QoS)'),
            ('11', '파라미터 (노드별)'),
        ]

    def center(self):
        return self.ui.center_summary()

    def on_select(self, key: str) -> str:
        from .base import run_page
        page = self.app.screen(key)
        if page is None:
            return 'stay'
        return run_page(self.ui, page)

    def show(self) -> str:
        self.app.poll(0.6)
        self.ui.reset_selection()
        while rclpy.ok():
            choice = self.ui.select(self.title, self.items(), self.center(),
                                    back=False, main=False)
            if choice == TIMEOUT:
                continue
            if choice in (BACK, MAIN):
                continue  # 최상위라 뒤·메인 없음
            if choice in ('q', 'quit', 'exit'):
                break
            result = self.on_select(choice)
            if result == QUIT:
                break
        self.term.clear()
        print('operator_tool 종료')
        return 0
