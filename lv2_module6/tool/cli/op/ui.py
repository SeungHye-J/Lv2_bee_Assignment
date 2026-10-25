"""화면 전부 — 4분할 프레임 + 방향키 선택 + 모달 입력.

term=Terminal, state=StateStore(상태바용), poll=ROS 콜백 돌리기.
"""
from __future__ import annotations

import os
import select
import shutil
import sys
import termios
import time
import tty

from .consts import ARM_JOINTS, GRIPPER_JOINT, WHEEL_JOINTS
from .defs import BACK, MAIN, MENU_REFRESH_S, QUIT, REFRESH, STAY, TIMEOUT
from .terminal import BOLD, CYAN, DIM, GREEN, RED, RESET, REV, YELLOW, Terminal
import math


class Ui:
    def __init__(self, term: Terminal, state, poll):
        self.term = term
        self.term.out = self.console = sys.stdout  # 화면 출력은 여기로만 — capture(sys.stdout 교체)에 안 잡힘
        self.state = state
        self._poll = poll
        self._linebuf = ""
        self._sel_idx = 0
        self._frame_size = None
        self.console = sys.stdout  # 화면 그리기 전용 — capture()가 sys.stdout을 가로채도 UI는 여기로 씀

    @property
    def operation(self) -> str:
        return self.state.operation

    @property
    def map_mode(self) -> str:
        return self.state.map_mode

    def capture(self):
        """with 블록 안의 print()를 가로챈다. 메뉴 실행 결과를 결과 모달에 넣기용.
        (Ui 자체 그리기는 self.console이라 영향 없음)"""
        import contextlib
        import io

        @contextlib.contextmanager
        def _cap():
            buf_out, buf_err = io.StringIO(), io.StringIO()
            with contextlib.redirect_stdout(buf_out), contextlib.redirect_stderr(buf_err):
                yield buf_out, buf_err

        return _cap()

    def modal_input(self, title: str, body: list[str], prompt: str,
                        default: str = '') -> str | None:
        """가운데 팝업 박스에서 한 줄 입력 — 레이아웃을 깨지 않는다.
        Enter=확인(빈칸이면 default), Esc=취소(None)."""
        if not sys.stdin.isatty():
            try:
                raw = input(f'{prompt} ').strip()
                return raw if raw else default
            except EOFError:
                return None
        cols, rows = self.term.size()
        lines = [title, ''] + list(body) + ['']
        size = (cols, rows)
        w = h = left = top = in_r = 0

        def draw_box(buf=''):
            nonlocal cols, rows, size, w, h, left, top, in_r
            cols, rows = self.term.size()
            size = (cols, rows)
            try:
                aw = shutil.get_terminal_size().columns
            except Exception:
                aw = cols
            w = min(max(aw - 4, 20), max([self.term.disp_len(s) for s in lines] + [len(prompt) + 20]) + 4)
            h = len(lines) + 2  # 입력줄 1 + 아래 테두리 1
            left = max(1, (cols - w) // 2 + 1)
            top = max(1, (rows - h) // 2 + 1)
            in_r = top + 1 + len(lines)
            at(top, left, '┌' + '─' * (w - 2) + '┐')
            for i, s in enumerate(lines):
                row(top + 1 + i, s)
            at(in_r, left, '│' + self.term.fit(base + buf, w - 2) + '│')
            at(top + h, left, '└' + '─' * (w - 2) + '┘')

        def at(r, c, s=''):
            self.console.write(f'\033[{r};{c}H{s}')

        def row(r, s):
            at(r, left, '│' + self.term.fit(s, w - 2) + '│')
        base = f'{prompt} '
        draw_box()
        self.console.flush()
        buf = ''
        pbuf = bytearray()
        try:
            fd = sys.stdin.fileno()
            old = termios.tcgetattr(fd)
        except Exception:
            return None
        try:
            tty.setcbreak(fd)
            self._drain(fd)
            while True:
                if self.term.size() != size:  # 리사이즈 → 옛 박스 지우고 다시 그림
                    self._erase_rect(left, top, w, h + 1)
                    draw_box(buf)
                at(in_r, left, '│' + self.term.fit(base + buf, w - 2) + '│')
                at(top + h, left, '└' + '─' * (w - 2) + '┘')
                at(in_r, left + 1 + self.term.disp_len(base + buf))
                self.console.flush()
                r, _, _ = select.select([fd], [], [], 0.1)
                self._poll(0.02)
                if not r:
                    continue
                try:
                    raw = os.read(fd, 1)
                except OSError:
                    continue
                if raw in (b'\n', b'\r'):
                    at(top + h + 1, 1)
                    self.console.flush()
                    return buf if buf else default
                if raw == b'\x1b':
                    seq = b''
                    for _ in range(2):
                        rr, _, _ = select.select([fd], [], [], 0.05)
                        if not rr:
                            break
                        try:
                            seq += os.read(fd, 1)
                        except OSError:
                            break
                    at(top + h + 1, 1)
                    self.console.flush()
                    return None  # Esc(방향키 포함)=취소
                if raw in (b'\x7f', b'\x08'):
                    buf = buf[:-1]
                    continue
                if raw in (b'\x15',):
                    buf = ''
                    continue
                pbuf += raw
                try:
                    ch = pbuf.decode('utf-8')
                except UnicodeDecodeError:
                    continue
                pbuf.clear()
                if ch.isprintable():
                    buf += ch
        finally:
            try:
                termios.tcsetattr(fd, termios.TCSADRAIN, old)
            except Exception:
                pass

    def modal_key(self, title: str, body: list[str]) -> str | None:
        """팝업에서 단일 키 입력 — Enter는 'enter', Esc는 None, 그 외 소문자 키."""
        if not sys.stdin.isatty():
            try:
                raw = input(f'{title} (Enter/b/m/q): ').strip().lower()
                return {'': 'enter', 'b': 'b', 'm': 'm', 'q': 'q'}.get(raw, raw or 'enter')
            except EOFError:
                return None
        cols, rows = self.term.size()
        lines = [title, ''] + list(body)
        size = (cols, rows)
        w = left = top = 0

        def draw_box():
            nonlocal cols, rows, size, w, left, top
            cols, rows = self.term.size()
            size = (cols, rows)
            try:
                aw = shutil.get_terminal_size().columns
            except Exception:
                aw = cols
            w = min(max(aw - 4, 20), max([self.term.disp_len(s) for s in lines]) + 4)
            left = max(1, (cols - w) // 2 + 1)
            top = max(1, (rows - len(lines) - 2) // 2 + 1)
            self.console.write(f'\033[{top};{left}H┌' + '─' * (w - 2) + '┐')
            for i, s in enumerate(lines):
                self.console.write(f'\033[{top + 1 + i};{left}H│' + self.term.fit(s, w - 2) + '│')
            self.console.write(f'\033[{top + 1 + len(lines)};{left}H└' + '─' * (w - 2) + '┘')

        draw_box()
        self.console.flush()
        try:
            fd = sys.stdin.fileno()
            old = termios.tcgetattr(fd)
        except Exception:
            return None
        try:
            tty.setcbreak(fd)
            self._drain(fd)
            while True:
                if self.term.size() != size:  # 리사이즈 → 옛 박스 지우고 다시 그림
                    self._erase_rect(left, top, w, len(lines) + 1)
                    draw_box()
                    self.console.flush()
                r, _, _ = select.select([fd], [], [], 0.1)
                self._poll(0.02)
                if not r:
                    continue
                try:
                    raw = os.read(fd, 1)
                except OSError:
                    continue
                if raw in (b'\n', b'\r'):
                    return 'enter'
                if raw == b'\x1b':
                    return None
                if raw == b'\x7f':
                    continue
                try:
                    ch = raw.decode('utf-8', errors='replace')
                except Exception:
                    continue
                if ch.isprintable() and ch.strip():
                    return ch.lower()
        finally:
            try:
                termios.tcsetattr(fd, termios.TCSADRAIN, old)
            except Exception:
                pass

    def pause(self, back_label: str | None = None, refresh: bool = False) -> str:
        """결과 화면에서 Enter/r/b/m/q를 받는다 (팝업). 안내는 맨 아랫줄에 있다."""
        hints = ['Enter 계속']
        key = self.modal_key('일시정지', hints)
        if key == 'q':
            return QUIT
        if key == 'm':
            return MAIN
        if key == 'b' and back_label:
            return BACK
        if key == 'r' and refresh:
            return REFRESH
        return STAY

    def confirm(self, prompt: str, default: bool = False, body: list[str] | None = None) -> bool:
        suffix = 'Y/n' if default else 'y/N'
        lines = list(body or []) + ([prompt] if body else [prompt])
        lines.append(f'[{suffix}] (y/n · Enter=기본 · Esc=아니오)')
        key = self.modal_key('확인', lines)
        if key in ('y', 'Y'):
            return True
        if key in ('n', 'N'):
            return False
        if key == 'enter':
            return default
        return False

    def pager(self, title: str, text: str) -> str:
        """실행 결과 모달 — ↑↓/PgUp/PgDn/Home/End 스크롤 · Enter/Esc 닫기."""
        import shutil
        lines = text.splitlines() or ['(출력 없음)']
        try:
            _, rows = shutil.get_terminal_size()
        except Exception:
            rows = 24
        h = max(5, min(20, rows - 10))
        off = 0
        if not sys.stdin.isatty():
            self.console.write(text + ('' if text.endswith('\n') else '\n'))
            self.console.flush()
            return
        try:
            fd = sys.stdin.fileno()
            old = termios.tcgetattr(fd)
        except Exception:
            self.console.write(text + '\n')
            self.console.flush()
            return
        cols, _ = self.term.size()
        w = min(cols - 4, max([self.term.disp_len(s) for s in lines] + [len(title)]) + 4)
        left = max(1, (cols - w) // 2 + 1)
        top = max(1, (rows - h - 2) // 2 + 1)

        def draw():
            n = len(lines)
            vis = lines[off:off + h]
            self.console.write(f'\033[{top};{left}H┌' + '─' * (w - 2) + '┐')
            self.console.write(f'\033[{top + 1};{left}H│' + self.term.fit(title, w - 2) + '│')
            for i in range(h):
                s = vis[i] if i < len(vis) else ''
                self.console.write(f'\033[{top + 2 + i};{left}H│' + self.term.fit(s, w - 2) + '│')
            bar = f'{min(off + h, n)}/{n} · ↑↓ 스크롤 · Enter/Esc 닫기'
            self.console.write(f'\033[{top + 2 + h};{left}H└' + '─' * (w - 2) + '┘')
            self.console.write(f'\033[{top + 3 + h};{left}H' + self.term.fit(bar, w))
            self.console.flush()

        try:
            tty.setcbreak(fd)
            self._drain(fd)
            draw()
            while True:
                r, _, _ = select.select([fd], [], [], 0.1)
                self._poll(0.02)
                if not r:
                    if self.term.size() != (cols, rows):
                        self._erase_rect(left, top, w, h + 3)  # 옛 박스 + 상태줄 지움
                        try:
                            _, rows = shutil.get_terminal_size()
                        except Exception:
                            pass
                        h = max(5, min(20, rows - 10))
                        off = max(0, min(off, max(0, len(lines) - h)))
                        cols, _ = self.term.size()
                        w = min(cols - 4, max([self.term.disp_len(s) for s in lines] + [len(title)]) + 4)
                        left = max(1, (cols - w) // 2 + 1)
                        top = max(1, (rows - h - 2) // 2 + 1)
                        draw()
                    continue
                try:
                    raw = os.read(fd, 1)
                except OSError:
                    continue
                if raw in (b'\n', b'\r'):
                    return 'stay'  # Enter도 닫기
                if raw in (b'q', b'Q', b'b', b'B', b'm', b'M'):
                    return 'stay'  # 결과창에선 내비 없음 — 그냥 닫기
                if raw == b'\x1b':
                    seq = b''
                    for _ in range(3):
                        rr, _, _ = select.select([fd], [], [], 0.05)
                        if not rr:
                            break
                        try:
                            seq += os.read(fd, 1)
                        except OSError:
                            break
                    if seq == b'':
                        return 'stay'
                    n = len(lines)
                    if seq == b'[D':
                        return BACK  # ◀ 닫기 = 뒤로
                    if seq == b'[A':
                        off = max(0, off - 1)
                    elif seq == b'[B':
                        off = min(max(0, n - h), off + 1)
                    elif seq == b'[5~':
                        off = max(0, off - h)
                    elif seq == b'[6~':
                        off = min(max(0, n - h), off + h)
                    elif seq == b'[H':
                        off = 0
                    elif seq == b'[F':
                        off = max(0, n - h)
                    else:
                        continue
                    draw()
        finally:
            try:
                termios.tcsetattr(fd, termios.TCSADRAIN, old)
            except Exception:
                pass

    def read_float(self, prompt: str, current: float,
                       positive: bool = False) -> tuple[float, bool]:
        raw = self.modal_input('값 입력', [prompt, f'현재: {current:g} (Enter=유지 · Esc=취소)'],
                                   '값>', default='')
        if raw is None:
            return current, False
        raw = raw.strip()
        if not raw:
            return current, True
        try:
            value = float(raw)
        except ValueError:
            print(f'거부: 숫자가 아님: {raw!r}')
            return current, False
        if not math.isfinite(value):
            print('거부: 유한한 숫자가 아님')
            return current, False
        if positive and value <= 0.0:
            print('거부: 0보다 커야 합니다')
            return current, False
        if not positive and value < 0:
            print('거부: 0 이상이어야 합니다')
            return current, False
        return value, True

    def invalid(self, choice: str):
        print(self.term.paint(f'알 수 없는 선택: {choice}', RED))
        time.sleep(0.45)

    def reset_selection(self) -> None:
        """메뉴 진입 시 포커스 초기화 (항상 첫 항목)."""
        self._sel_idx = 0

    @staticmethod
    def _drain(fd: int) -> None:
        """입력 버퍼 비우기 — 이전 화면에서 친 키가 다음 다이얼로그로 새는 것(타입어헤드) 방지."""
        import fcntl
        import errno
        try:
            flags = fcntl.fcntl(fd, fcntl.F_GETFL)
            fcntl.fcntl(fd, fcntl.F_SETFL, flags | os.O_NONBLOCK)
            try:
                while os.read(fd, 1024):
                    pass
            except OSError as e:
                if e.errno not in (errno.EAGAIN, errno.EWOULDBLOCK):
                    pass
            finally:
                fcntl.fcntl(fd, fcntl.F_SETFL, flags)
        except Exception:
            pass

    def _erase_rect(self, left: int, top: int, w: int, h: int) -> None:
        """박스 잔상 지우기 (리사이즈 전 위치)."""
        blank = ' ' * w
        for r in range(top, top + h + 1):
            self.console.write(f'\033[{r};{left}H{blank}')
        self.console.flush()

    def select(self, title: str, items: list[tuple[str, str]], center: list[str],
                   back: bool = True, main: bool = True,
                   timeout: float | None = None) -> str:
        """방향키 메뉴 — ↑↓ 이동 · Enter 선택 · 숫자 점프 · q 종료.
        items: [(선택값, 라벨)]. 선택값 반환, timeout이면 화면만 다시 그리라고 TIMEOUT."""
        if timeout is None:
            timeout = MENU_REFRESH_S
        idx = getattr(self, '_sel_idx', 0) % max(1, len(items))
        if not sys.stdin.isatty():
            return items[0][0]
        try:
            fd = sys.stdin.fileno()
            old = termios.tcgetattr(fd)
        except Exception:
            return items[0][0]
        try:
            tty.setcbreak(fd)
            self._drain(fd)
            end = time.monotonic() + max(0.0, timeout)
            dbuf = ''       # 숫자 연속 입력 버퍼 ('1'→'10' 같은 두 자리 키용)
            ddeadline = 0.0
            while True:
                left = []
                for i, (key, label) in enumerate(items):
                    row = f"  {'❯' if i == idx else ' '} {key}) {label}"
                    left.append(self.term.paint(row, REV + CYAN + BOLD) if i == idx else row)
                left += ['', '  ↑↓ 이동 · Enter 실행 · 숫자 포커스 · ◀ 뒤로 · m 메인 · q 종료']
                self.draw(title, left, center, back=back, main=main, poll=False)
                self._sel_idx = idx
                while True:
                    remain = end - time.monotonic()
                    if remain <= 0:
                        self.console.write('\r\033[K')
                        self.console.flush()
                        return TIMEOUT
                    ready, _, _ = select.select([fd], [], [], min(0.05, remain))
                    self._poll(0.02)
                    if not ready:
                        continue
                    try:
                        raw = os.read(fd, 1)
                    except OSError:
                        continue
                    if raw in (b'\n', b'\r'):
                        dbuf = ''
                        return items[idx][0]
                    if raw in (b'q', b'Q'):
                        return 'q'
                    if raw in (b'b', b'B'):
                        return BACK
                    if raw in (b'm', b'M'):
                        return MAIN
                    if raw in (b'r', b'R'):
                        dbuf = ''
                        return TIMEOUT  # 새로고침 = 화면만 다시 그림
                    if raw == b'\x1b':  # 방향키 ESC [ A/B/D (단독 ESC는 종료)
                        seq = b''
                        for _ in range(2):
                            r, _, _ = select.select([fd], [], [], 0.05)
                            if not r:
                                break
                            try:
                                seq += os.read(fd, 1)
                            except OSError:
                                break
                        if seq == b'':
                            return BACK  # 단독 Esc=뒤로 (종료 아님)
                        if seq == b'[A':
                            idx = (idx - 1) % len(items)
                            break  # 다시 그림
                        if seq == b'[B':
                            idx = (idx + 1) % len(items)
                            break
                        if seq == b'[D':
                            return BACK  # ◀ 뒤로
                        continue
                    if raw in (b'0', b'1', b'2', b'3', b'4', b'5', b'6', b'7', b'8', b'9'):
                        # 숫자는 포커스만 옮긴다 (바로 실행 안 함 — Enter로 실행).
                        # 두 자리 키('10')용으로 1초 안 연속 입력은 이어 붙인다.
                        now = time.monotonic()
                        if now - ddeadline > 1.0:
                            dbuf = ''
                        dbuf += raw.decode()
                        ddeadline = now
                        exact = [i for i, (key, _) in enumerate(items) if key == dbuf]
                        longer = any(key.startswith(dbuf) and key != dbuf
                                     for key, _ in items)
                        if exact and not longer:
                            idx = exact[0]
                            dbuf = ''
                            break  # 다시 그림 (실행은 Enter)
                        if exact:
                            idx = exact[0]
                            break  # 더 올 수 있으니 일단 포커스만
                        dbuf = ''
                        continue  # 해당 번호 없음 — 무시
                    dbuf = ''
                    # 그 외 키 무시
        finally:
            try:
                termios.tcsetattr(fd, termios.TCSADRAIN, old)
            except Exception:
                pass

    def draw(self, title: str, left: list[str], center: list[str],
                 back: bool = True, main: bool = True, poll: bool = True):
        """4분할 화면: 상단 타이틀+상태바 고정 · 왼쪽 메뉴 · 중앙 내용 · 하단 힌트+입력.

        터미널 크기에 맞춰 다시 계산하므로 창을 늘리면 다음 그리기부터 넓어진다.
        폭이 좁으면(<70) 세로로 쌓는다.
        전체 지우기 없이 커서를 맨 위로 옮겨 제자리에서 다시 그려 깜빡이지 않는다
        (처음·크기 변경 때만 전체 지우기).
        poll=False면 ROS 폴링을 건너뛴다 (방향키 연속 입력 때 0.3초 지연 방지).
        """
        if poll:
            self._poll(0.3)
        cols, rows = self.term.size()
        size = (cols, rows)
        if cols < 70 or rows < 18:
            self.term.clear()
            for line in self.top_lines(title, cols):
                self.console.write(self.term.fit(line, cols) + '\n')
            for line in left:
                self.console.write(self.term.fit(line, cols) + '\n')
            for line in center:
                self.console.write(self.term.fit(line, cols) + '\n')
            self.footer(back=back, main=main)
            self.console.flush()
            self._frame_size = size
            return
        if self._frame_size != size:
            self.term.clear()
            self._frame_size = size
        else:
            self.console.write('\033[H')
            self.console.flush()
        top = [self.term.fit(ln, cols) for ln in self.top_lines(title, cols)]
        lw = min(32, max(24, cols // 3))
        cw = cols - lw - 4
        body_h = max(1, rows - 1 - len(top) - 2)
        left_lines = list(left)
        center_lines = list(center)
        if len(left_lines) > body_h:
            left_lines = left_lines[:body_h - 1] + [f'  … 외 {len(left_lines) - body_h + 1}줄']
        if len(center_lines) > body_h:
            center_lines = center_lines[:body_h - 1] + [f' … 외 {len(center_lines) - body_h + 1}줄']
        frame = list(top)
        for i in range(body_h):
            l = self.term.fit(left_lines[i] if i < len(left_lines) else '', lw)
            c = self.term.fit(center_lines[i] if i < len(center_lines) else '', cw)
            frame.append(f' {l}| {c}')
        frame.append('-' * cols)
        frame.append(self.term.fit(self.hints_line(back=back, main=main), cols))
        # 항상 rows-1줄로 정규화 → 짧아진 프레임의 남은 줄 흔적이 남지 않는다.
        if len(frame) > rows - 1:
            frame = frame[:rows - 1]
        else:
            frame += [''] * (rows - 1 - len(frame))
        for ln in frame:
            self.console.write('\033[K' + ln if ln else '\033[K')
            self.console.write('\n')
        self.console.flush()

    def header(self, title: str):
        self._poll(0.35)
        self.term.clear()
        cols, _ = self.term.size()
        width = min(64, cols)
        for line in self.top_lines(title, width):
            self.console.write(self.term.fit(line, width) + '\n')
        self.console.flush()

    def footer(self, back: bool = True, main: bool = True):
        cols, _ = self.term.size()
        self.console.write('-' * min(64, cols) + '\n')
        hints = []
        if back:
            hints.append('◀/b) 뒤로')
        hints.append('r) 새로고침')
        if main:
            hints.append('m) 메인 메뉴')
        hints.append('q) 종료')
        self.console.write('  ' + '   '.join(hints) + '\n')
        self.console.flush()

    def hints_line(self, back: bool = True, main: bool = True) -> str:
        hints = []
        if back:
            hints.append('◀/b) 뒤로')
        hints.append('r) 새로고침')
        if main:
            hints.append('m) 메인 메뉴')
        hints.append('q) 종료')
        return '  ' + '   '.join(hints)

    def top_lines(self, title: str, width: int = 64) -> list[str]:
        """상단 고정 영역 6줄: 타이틀 3 + 상태바 2 + 구분선 1."""
        bold = BOLD if self.term.color else ''
        reset = RESET if self.term.color else ''
        lines = [
            f'{bold}{"=" * width}{reset}',
            f'{bold}  BeeBot Operator Console  ·  {title}{reset}',
            f'{bold}{"=" * width}{reset}',
        ]
        op = self.operation or '?'
        mode = self.map_mode or '?'
        op_c = {'AUTO': GREEN, 'MANUAL': YELLOW, 'EXPLORE': CYAN}.get(op, RED)
        mode_c = {'MAPPING': YELLOW, 'LOCALIZATION': GREEN}.get(mode, '')
        op_txt = self.term.paint(op, op_c + BOLD)
        mode_txt = self.term.paint(mode, mode_c) if mode_c else self.term.paint(mode, DIM)
        if self.state.emergency:
            estop = self.term.paint('ON', RED + BOLD)
        else:
            estop = self.term.paint('OFF', GREEN)
        active = 0
        for name in self.state.devices:
            _, state, _ = self.state.device_info(name)
            if state == 'active':
                active += 1
        if self.state.devices:
            dev_txt = self.term.paint(f'{active}/{len(self.state.devices)}',
                                     GREEN if active == len(self.state.devices) else YELLOW)
        else:
            dev_txt = self.term.paint('0/0', DIM)
        # 갱신시각 (그리는 시점 벽시계 + 마지막 수신 뒤 경과)
        now_wall = time.strftime('%m-%d %H:%M:%S', time.localtime())
        if self.state.last_update <= 0:
            upd_txt = self.term.paint('갱신 없음', YELLOW)
        else:
            age = time.monotonic() - self.state.last_update
            if age <= 5.0:
                upd_txt = self.term.paint(f'갱신 {now_wall} ({age:.1f}초 전)', DIM)
            else:
                upd_txt = self.term.paint(f'갱신 {now_wall} ({age:.0f}초 전 — 끊김?)', YELLOW)
        lines.append(f'  조작 {op_txt} | 지도 {mode_txt} | E-STOP {estop} | '
                     f'장치 {dev_txt} | {upd_txt}')
        # 2단 상시 상태바: 인지·임무·발화 (GUI 보드와 같은 내용, 1줄 요약)
        task = (self.state.task_status or '-')
        if len(task) > 38:
            task = task[:37] + '…'
        say = (self.state.say_last or '')
        if len(say) > 22:
            say = say[:21] + '…'
        diag_levels = [lv for _, (lv, _) in self.state.diagnostics.items() if lv >= 1]
        if not self.state.diagnostics:
            diag_txt = self.term.paint('진단 -', DIM)
        elif not diag_levels:
            diag_txt = self.term.paint(f'진단 OK({len(self.state.diagnostics)})', GREEN)
        else:
            worst = max(diag_levels)
            diag_txt = self.term.paint(f'진단 주의 {len(diag_levels)}',
                                      RED + BOLD if worst >= 2 else YELLOW)
        lines.append(f'  인지 {self.state.tracked_count:3d}개 | 임무 {task} | {diag_txt}'
                     + (f' | 발화 {say}' if say else ''))
        if not self.state.values and not self.state.devices and not self.state.joints:
            lines.append(f'  {self.term.paint("상태 없음 — bringup이 떠 있는지 확인", YELLOW)}')
        lines.append('-' * width)
        return lines

    def center_summary(self) -> list[str]:
        """중앙 공용 요약: 장치·임무·관절·진단 (왼쪽이 메뉴일 때 오른쪽에 보여줌)."""
        lines = ['[상태 요약]']
        if not self.state.devices:
            lines.append(' 장치: - (대기 중)')
        else:
            for name in self.state.devices[:8]:
                kind, state, topics = self.state.device_info(name)
                if state == 'active':
                    mark = self.term.paint('●', GREEN)
                elif state:
                    mark = self.term.paint('○', RED)
                else:
                    mark = self.term.paint('?', YELLOW)
                lines.append(f' {mark} {name} ({kind})')
            if len(self.state.devices) > 8:
                lines.append(f' … 외 {len(self.state.devices) - 8}개')
        lines.append('')
        lines.append(f' 임무: {self.state.task_status or "-"}')
        wheels = [f'{n}={self.state.joints[n]:.2f}' for n in WHEEL_JOINTS
                  if n in self.state.joints]
        arms = [f'{n}={self.state.joints[n]:+.2f}/{math.degrees(self.state.joints[n]):+.0f}°'
                for n in (*ARM_JOINTS, GRIPPER_JOINT) if n in self.state.joints]
        if wheels:
            lines.append(' 바퀴: ' + ' '.join(wheels))
        if arms:
            lines.append(' 관절: ' + ' '.join(arms[:3]))
            if len(arms) > 3:
                lines.append(' ' + ' '.join(arms[3:]))
        if not wheels and not arms:
            lines.append(' 관절: -')
        bad = [(n, lv, m) for n, (lv, m) in sorted(self.state.diagnostics.items()) if lv >= 1][:5]
        if not self.state.diagnostics:
            lines.append(' 진단: -')
        elif not bad:
            lines.append(self.term.paint(f' 진단 OK ({len(self.state.diagnostics)}개)', GREEN))
        else:
            lines.append(f' 진단 주의 {len(bad)}:')
            for n, lv, m in bad:
                lines.append(f'  - {n}: {DIAG_LEVELS.get(lv, lv)} {m}')
        return lines
