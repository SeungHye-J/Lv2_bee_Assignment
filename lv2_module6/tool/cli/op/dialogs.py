"""단일 키 입력·다이얼로그 코어.

Ui.select/modal_input/modal_key/pager가 모두 이 구현을 공유한다.
규약: Enter=확인/실행 · Esc=취소/닫기 · ◀=뒤로 · ↑↓=이동/스크롤 · 숫자=포커스.
"""
from __future__ import annotations

import os
import select
import sys
import termios
import time
import tty

from .defs import BACK, MAIN, MENU_REFRESH_S, TIMEOUT


class KeyReader:
    """cbreak 터미널에서 키 하나를 읽는다. 방향키·UTF-8도 지원."""

    def __init__(self, term):
        self.term = term
        self.fd = None
        self._old = None

    def __enter__(self):
        self.fd = sys.stdin.fileno()
        self._old = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        # 이전 화면에서 누른 키가 새 다이얼로그에 새지 않게
        try:
            termios.tcflush(self.fd, termios.TCIFLUSH)
        except Exception:
            pass
        return self

    def __exit__(self, exc_type, exc, tb):
        if self._old is not None:
            try:
                termios.tcsetattr(self.fd, termios.TCSADRAIN, self._old)
            except Exception:
                pass

    def read(self, timeout=0.1):
        """키 하나 → ('enter'|'esc'|'up'|'down'|'left'|'right'|'pgup'|'pgdn'|
        'home'|'end'|'backspace'|'ctrl_u'|<문자>) · 없으면 None."""
        if self.fd is None:
            return None
        ready, _, _ = select.select([self.fd], [], [], timeout)
        if not ready:
            return None
        try:
            raw = os.read(self.fd, 1)
        except OSError:
            return None
        if not raw:
            return 'eof'
        if raw in (b'\n', b'\r'):
            return 'enter'
        if raw in (b'\x7f', b'\x08'):
            return 'backspace'
        if raw == b'\x15':
            return 'ctrl_u'
        if raw == b'\x1b':
            seq = b''
            for _ in range(3):
                r, _, _ = select.select([self.fd], [], [], 0.04)
                if not r:
                    break
                try:
                    seq += os.read(self.fd, 1)
                except OSError:
                    break
                if seq in (b'[A', b'[B', b'[C', b'[D', b'[H', b'[F') or seq.endswith(b'~'):
                    break
            mapping = {
                b'[A': 'up', b'[B': 'down', b'[C': 'right', b'[D': 'left',
                b'[H': 'home', b'[F': 'end',
                b'[5~': 'pgup', b'[6~': 'pgdn',
            }
            if seq:
                return mapping.get(seq, 'unknown')
            return 'esc'
        try:
            n = (raw[0] & 0xF0)
            need = 1 if raw[0] < 0x80 else 2 if n == 0xC0 else 3 if n == 0xE0 else 4 if n == 0xF0 else 1
            buf = raw
            while len(buf) < need:
                buf += os.read(self.fd, 1)
            return buf.decode('utf-8', errors='replace')
        except (OSError, UnicodeDecodeError):
            return None


class Dialog:
    """박스 하나 + KeyReader로 처리하는 모달. 텍스트 흐름은 Ui.capture가 가로채도 출력은 console로."""

    def __init__(self, ui, title: str, body: list[str]):
        self.ui = ui
        self.title = title
        self.body = list(body)
        self.lines = [title, ''] + self.body
        self._geom = None

    def _width(self, extra: int = 0):
        try:
            import shutil
            cols = shutil.get_terminal_size().columns
        except Exception:
            cols, _ = self.ui.term.size()
        content = max([self.ui.term.disp_len(s) for s in self.lines] + [20]) + extra + 4
        return min(max(cols - 4, 20), max(20, content))

    def draw(self, mode: str, buf: str = '', prompt: str = '', off: int = 0,
             h: int = 0) -> None:
        cols, rows = self.ui.term.size()
        extra = self.ui.term.disp_len(prompt) + 2 if mode == 'input' else 0
        w = self._width(extra)
        lines = self.lines
        if mode == 'pager':
            vis = self.body[off:off + h]
            content = lines + vis + ['↑↓ 스크롤 · Enter/Esc 닫기']
        elif mode == 'input':
            content = lines + [prompt + buf, 'Enter 확인 · Esc 취소']
        else:
            content = lines
        left = max(1, (cols - w) // 2 + 1)
        hbox = len(content) + 1
        top = max(1, (rows - hbox - 1) // 2 + 1)
        if self._geom is not None:
            old_l, old_t, old_w, old_h = self._geom
            if (left, top, w, hbox) != (old_l, old_t, old_w, old_h):
                self.ui._erase_rect(old_l, old_t, old_w, old_h)
        self._geom = (left, top, w, hbox)
        self.ui.console.write(f'\033[{top};{left}H┌' + '─' * (w - 2) + '┐')
        for i, s in enumerate(content):
            self.ui.console.write(f'\033[{top + 1 + i};{left}H│' + self.ui.term.fit(s, w - 2) + '│')
        self.ui.console.write(f'\033[{top + hbox};{left}H└' + '─' * (w - 2) + '┘')
        self.ui.console.flush()

    def input(self, prompt: str, default: str = '') -> str | None:
        """한 줄 입력 — Enter 확인(빈칸=default) · Esc/◀ 취소(None)."""
        if not sys.stdin.isatty():
            try:
                raw = input(f'{prompt} ').strip()
                return raw if raw else default
            except EOFError:
                return None
        buf = ''
        with KeyReader(self.ui.term) as keys:
            while True:
                self.draw('input', buf, prompt)
                k = keys.read(0.1)
                self.ui._poll(0.01)
                if k is None:
                    continue
                if k == 'enter':
                    return buf if buf else default
                if k in ('esc', 'left'):
                    return None
                if k == 'backspace':
                    buf = buf[:-1]
                elif k == 'ctrl_u':
                    buf = ''
                elif k == 'eof':
                    return None
                elif isinstance(k, str) and len(k) == 1 and k.isprintable():
                    buf += k

    def key(self) -> str | None:
        """단일 키 확인 — Enter='enter' · Esc/◀=None · 문자는 그대로."""
        if not sys.stdin.isatty():
            try:
                raw = input(f'{self.title} (Enter/b/m/q): ').strip().lower()
                return {'': 'enter', 'b': 'b', 'm': 'm', 'q': 'q'}.get(raw, raw or 'enter')
            except EOFError:
                return None
        with KeyReader(self.ui.term) as keys:
            while True:
                self.draw('key')
                k = keys.read(0.1)
                self.ui._poll(0.01)
                if k is None:
                    continue
                if k == 'enter':
                    return 'enter'
                if k in ('esc', 'left'):
                    return None
                if k == 'eof':
                    return None
                if isinstance(k, str) and len(k) == 1 and k.isprintable():
                    return k.lower()

    def pager(self) -> str:
        """결과 페이저 — ↑↓ 스크롤 · Enter/Esc/◀ 닫기(메뉴·종료 이동 없음)."""
        import shutil
        lines = self.body or ['(출력 없음)']
        if not sys.stdin.isatty():
            self.ui.console.write('\n'.join(lines) + '\n')
            self.ui.console.flush()
            return 'stay'
        off = 0
        try:
            _, rows = shutil.get_terminal_size()
        except Exception:
            rows = 24
        h = max(5, min(20, rows - 10))
        with KeyReader(self.ui.term) as keys:
            while True:
                off = max(0, min(off, max(0, len(lines) - h)))
                self.draw('pager', off=off, h=h)
                k = keys.read(0.1)
                self.ui._poll(0.01)
                if k is None:
                    continue
                if k in ('enter', 'esc', 'left', 'eof'):
                    return 'stay'
                if k == 'up':
                    off = max(0, off - 1)
                elif k == 'down':
                    off = min(max(0, len(lines) - h), off + 1)
                elif k == 'pgup':
                    off = max(0, off - h)
                elif k == 'pgdn':
                    off = min(max(0, len(lines) - h), off + h)
                elif k == 'home':
                    off = 0
                elif k == 'end':
                    off = max(0, len(lines) - h)


def run_select(ui, title: str, items: list[tuple[str, str]], center: list[str],
               back: bool = True, main: bool = True,
               timeout: float | None = None) -> str:
    """메뉴 선택 — ↑↓ 이동 · Enter 실행 · 숫자 포커스 · Esc/◀ 뒤로 · m 메인 · q 종료."""
    if not items:
        return BACK
    if timeout is None:
        timeout = MENU_REFRESH_S
    idx = getattr(ui, '_sel_idx', 0) % len(items)
    if not sys.stdin.isatty():
        return items[0][0]
    dbuf = ''
    ddeadline = 0.0
    with KeyReader(ui.term) as keys:
        end = time.monotonic() + timeout
        while True:
            left = []
            for i, (key, label) in enumerate(items):
                row = f"  {'❯' if i == idx else ' '} {key}) {label}"
                left.append(ui.term.paint(row, '\033[7m\033[36m\033[1m') if i == idx else row)
            left += ['', '  ↑↓ 이동 · Enter 실행 · 숫자 포커스 · ◀ 뒤로 · m 메인 · q 종료']
            ui.draw(title, left, center, back=back, main=main, poll=False)
            ui._sel_idx = idx
            while True:
                remain = end - time.monotonic()
                if remain <= 0:
                    return TIMEOUT
                k = keys.read(min(0.1, remain))
                ui._poll(0.02)
                if k is None:
                    continue
                if k == 'enter':
                    return items[idx][0]
                if k in ('q', 'Q'):
                    return 'q'
                if k in ('b', 'B'):
                    return BACK
                if k in ('m', 'M'):
                    return MAIN
                if k in ('r', 'R'):
                    return TIMEOUT
                if k in ('esc', 'left'):
                    return BACK
                if k in ('up', 'down'):
                    idx = (idx - 1 if k == 'up' else idx + 1) % len(items)
                    break
                if k in ('eof',):
                    return 'q'
                if isinstance(k, str) and len(k) == 1 and k.isdigit():
                    now = time.monotonic()
                    if now - ddeadline > 1.0:
                        dbuf = ''
                    dbuf += k
                    ddeadline = now
                    exact = [i for i, (key, _) in enumerate(items) if key == dbuf]
                    longer = any(key.startswith(dbuf) and key != dbuf for key, _ in items)
                    if exact and not longer:
                        idx = exact[0]
                        dbuf = ''
                        break
                    if exact:
                        idx = exact[0]
                        break
                    dbuf = ''
                else:
                    dbuf = ''
