#!/usr/bin/env python3
"""cli-test 시나리오 — 실제 pty 키 입력으로 UI 흐름을 검증한다.

실행: python3 tool/cli-test/scenarios.py
각 시나리오는 가짜 상태를 초기화한 뒤 시작한다.
"""
from __future__ import annotations

import os
import re
import select
import signal
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
CLI = HERE.parent / 'cli'
STATE = HERE / '.fake_state.json'

UP, DOWN, LEFT, RIGHT = b'\x1b[A', b'\x1b[B', b'\x1b[D', b'\x1b[C'
ENTER, ESC, BACKSPACE = b'\r', b'\x1b', b'\x7f'


def plain(data: bytes) -> str:
    text = data.decode('utf-8', errors='replace')
    return re.sub(r'\x1b\[[0-9;]*[A-Za-z]', '', text)


class Session:
    def __init__(self, name: str):
        self.name = name
        self.out = bytearray()
        STATE.unlink(missing_ok=True)
        self.pid, self.fd = os.forkpty() if hasattr(os, 'forkpty') else (None, None)
        if self.pid == 0:
            os.chdir(HERE.parent.parent)
            os.execv(sys.executable, [sys.executable, str(HERE / 'cli_test.py'), 'menu'])
        if self.pid is None:
            import pty
            self.pid, self.fd = pty.fork()
            if self.pid == 0:
                os.chdir(HERE.parent.parent)
                os.execv(sys.executable, [sys.executable, str(HERE / 'cli_test.py'), 'menu'])
        self.closed = False

    def _read(self, timeout=0.2) -> bool:
        r, _, _ = select.select([self.fd], [], [], timeout)
        if not r:
            return False
        try:
            data = os.read(self.fd, 65536)
        except OSError:
            self.closed = True
            return False
        if not data:
            self.closed = True
            return False
        self.out.extend(data)
        return True

    def send(self, data: bytes) -> None:
        os.write(self.fd, data)

    def expect(self, marker: str, timeout=8.0) -> None:
        target = marker.encode('utf-8')
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if target in self.out:
                return
            self._read(min(0.2, end - time.monotonic()))
        raise AssertionError(f'[{self.name}] 기대 문자열 없음: {marker!r}')

    def step(self, data: bytes, marker: str | None = None, wait=0.7) -> None:
        self.send(data)
        if marker is not None:
            self.expect(marker)
        else:
            time.sleep(wait)
            while self._read(0.05):
                pass

    def finish(self, marker='operator_tool 종료', timeout=8.0) -> None:
        self.step(b'q', marker, wait=0) if not self.closed else None
        if not self.closed:
            self.expect(marker, timeout)

    def close(self) -> None:
        if self.closed:
            return
        try:
            os.kill(self.pid, signal.SIGKILL)
        except OSError:
            pass
        self.closed = True
        try:
            os.close(self.fd)
        except OSError:
            pass
        try:
            os.waitpid(self.pid, 0)
        except ChildProcessError:
            pass

    def assert_clean(self) -> None:
        text = plain(bytes(self.out))
        if 'Traceback' in text:
            raise AssertionError(f'[{self.name}] Traceback:\n{text[-2000:]}')


def scenario(name: str):
    def deco(fn):
        def wrapped():
            s = Session(name)
            try:
                fn(s)
                s.assert_clean()
                print(f'PASS {name}')
            except Exception:
                (Path('/tmp') / f'cli-scenario-{name}.bin').write_bytes(bytes(s.out))
                raise
            finally:
                s.close()
        return wrapped
    return deco


@scenario('menu-arrows-esc')
def _(s):
    s.step(DOWN, wait=0.5)
    s.step(DOWN, wait=0.5)
    s.step(ENTER, '조작 모드')
    s.step(ESC, '메인 메뉴')
    s.finish()


@scenario('manual-cancel-no-extra-dialog')
def _(s):
    s.step(b'4', wait=0.4)
    s.step(ENTER, '수동 주행 / 텔레오퍼')
    s.step(DOWN, wait=0.4)
    s.step(ENTER, 'MANUAL 모드로 전환할까요?')
    s.step(ESC, '수동 주행 / 텔레오퍼')
    if '일시정지'.encode() in s.out:
        raise AssertionError('취소 후 일시정지 모달이 뜸')
    s.finish()


@scenario('drive-confirm-execute')
def _(s):
    s.step(b'4', wait=0.4)
    s.step(ENTER, '수동 주행 / 텔레오퍼')
    s.step(DOWN, wait=0.4)
    s.step(ENTER, 'MANUAL 모드로 전환할까요?')
    s.step(b'y', wait=0.5)
    s.expect('/cmd/operation_mode')
    s.expect('실행 결과')
    s.step(ENTER, '수동 주행 / 텔레오퍼')
    s.finish()


@scenario('status-pager-scroll-close')
def _(s):
    s.step(ENTER, '전체 요약')
    s.step(ENTER, '실행 결과')
    s.step(DOWN, wait=0.4)
    s.step(ESC, '메인 메뉴')
    s.finish()


@scenario('map-mode-course-restart')
def _(s):
    s.step(b'7', wait=0.4)
    s.step(ENTER, '지도')
    s.step(DOWN, wait=0.4)
    s.step(DOWN, wait=0.4)
    s.step(ENTER, 'MAPPING 모드로 전환할까요?')
    s.step(b'y', wait=0.5)
    s.expect('재시작할까요?')
    s.step(b'y', wait=0.5)
    s.expect('/cmd/restart')
    s.expect('실행 결과')
    s.step(ESC, '지도')
    s.finish()


@scenario('arm-value-modal-cancel')
def _(s):
    s.step(b'5', wait=0.4)
    s.step(ENTER, '팔')
    s.step(DOWN, wait=0.4)
    s.step(ENTER, 'MANUAL 모드로 전환할까요?')
    s.step(b'y', wait=0.5)
    s.expect('관절 선택')
    s.step(ENTER, '값 입력')
    s.step(ESC, '관절 선택')
    s.step(b'b', '팔')
    s.finish()


@scenario('teleop-hud')
def _(s):
    s.step(b'4', wait=0.4)
    s.step(ENTER, '수동 주행 / 텔레오퍼')
    s.step(ENTER, 'MANUAL 모드로 전환할까요?')
    s.step(b'y', wait=0.5)
    s.expect('텔레오퍼 (w/x/a/d')
    s.step(b'w', wait=0.5)
    s.expect('키 w')
    s.step(b'q', wait=0.5)
    s.finish()


def main() -> int:
    failed = []
    for fn in [v for k, v in globals().items() if k.startswith('_') and callable(v) and k != '_']:
        try:
            fn()
        except Exception as e:
            failed.append((fn.__name__, str(e)))
            print(f'FAIL {fn.__name__}: {e}')
    print(f'\n{7 - len(failed)}/7 통과')
    return 1 if failed else 0


if __name__ == '__main__':
    raise SystemExit(main())
