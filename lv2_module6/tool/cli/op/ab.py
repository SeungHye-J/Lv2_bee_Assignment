"""rl_nav_correction ONNX A/B — 상태·ON/OFF·rosbag 기록·리포트.

사용자 진입점은 `run-cli ab <노드> <동작>` / 메뉴 14 하나다. 이 파일은 UI 없이 동작만 담당한다.
  - ON/OFF: /rl_nav_correction/toggle (std_srvs/SetBool), 안 되면 파라미터 enabled로 대체
  - 기록: ros2 bag record를 백그라운드로 띄우고 ~/.cache/beebot/ab.json에 pid·bag 경로를 남긴다
  - 리포트: tool/cli/node/rl_nav_correction/report.py (rosbag2 → report.png)

bee는 ROS_DOMAIN_ID=88 + fastdds_local_only.xml(SHM 전용)이라 PC에서 원격 기록할 수 없다.
그래서 bag은 기본적으로 bee의 ~/bags/ab에 쌓는다.
"""
from __future__ import annotations

import json
import os
import re
import signal
import subprocess
import sys
import time
from pathlib import Path

import rclpy

from .consts import _param_value_to_py

# A/B 메뉴에 노출할 노드 — 새 rl_nav_*가 생기면 여기와 pages/ab.py의 화면 매핑에 추가한다.
AB_NODES = (
    ('rl_nav_correction', 'ONNX 주행 보정 (Δv·Δω)'),
)

NODE = 'rl_nav_correction'
TOGGLE = '/rl_nav_correction/toggle'
BAG_DIR = Path.home() / 'bags' / 'ab'
STATE = Path.home() / '.cache' / 'beebot' / 'ab.json'
RECORD_LOG = Path.home() / '.cache' / 'beebot' / 'ab-record.log'
REPORT = Path(__file__).resolve().parents[1] / 'node' / 'rl_nav_correction' / 'report.py'

# /cmd_vel_smoothed·/cmd_vel까지 넣어 실제 바퀴 명령과 보정 결과를 같이 본다.
TOPICS = (
    '/cmd_vel_nav', '/cmd_vel_corrected', '/cmd_vel_smoothed', '/cmd_vel',
    '/odometry/filtered', '/odom', '/imu', '/plan', '/tf', '/tf_static',
    '/rl_nav_correction/debug',
)


class AbTest:
    """run-cli ab의 실제 동작. Bus가 node·param_client·diag를 제공한다."""

    def __init__(self, bus):
        self.bus = bus
        self.node = bus.node
        self.prompt = bus.prompt
        self.term = bus.term
        self._toggle_cli = None
        self._param_cache = (0.0, None, None)  # (읽은 시각, enabled, model_path)

    # --------------------------------------------------------------- 상태 파일
    @staticmethod
    def _load() -> dict:
        try:
            data = json.loads(STATE.read_text())
            if isinstance(data, dict):
                data.setdefault('recording', None)
                data.setdefault('history', [])
                return data
        except Exception:
            pass
        return {'recording': None, 'history': []}

    @staticmethod
    def _save(data: dict) -> None:
        STATE.parent.mkdir(parents=True, exist_ok=True)
        tmp = STATE.with_suffix('.tmp')
        tmp.write_text(json.dumps(data, ensure_ascii=False, indent=1))
        tmp.replace(STATE)

    @staticmethod
    def _alive(pid) -> bool:
        try:
            pid = int(pid)
            os.kill(pid, 0)
        except (TypeError, ValueError, ProcessLookupError, PermissionError):
            return False
        try:
            cmd = Path(f'/proc/{pid}/cmdline').read_bytes().replace(b'\0', b' ').decode(errors='replace')
        except OSError:
            return True
        return 'bag' in cmd and 'record' in cmd

    def _finalize(self, data: dict, rec: dict, stopped: float | None = None) -> dict:
        rec = dict(rec)
        rec.setdefault('stopped', stopped or time.time())
        hist = [h for h in data.get('history', []) if h.get('bag') != rec.get('bag')]
        hist.append(rec)
        data['history'] = hist[-50:]
        if data.get('recording') and data['recording'].get('bag') == rec.get('bag'):
            data['recording'] = None
        self._save(data)
        return rec

    def current(self) -> dict | None:
        """지금 녹화 중인 항목. 프로세스가 죽었으면 history로 옮긴다."""
        data = self._load()
        rec = data.get('recording')
        if not rec:
            return None
        if self._alive(rec.get('pid')):
            return rec
        self._finalize(data, rec, stopped=rec.get('stopped') or time.time())
        return None

    def history(self) -> list[dict]:
        data = self._load()
        raw = data.get('history', [])
        hist = [h for h in raw if h.get('bag') and Path(h['bag']).exists()]
        if len(hist) != len(raw):  # 지워진 bag은 상태 파일에서도 정리
            data['history'] = hist
            self._save(data)
        return hist

    # ------------------------------------------------------------------ 상태
    def params(self, force: bool = False) -> tuple[bool | None, str | None]:
        now = time.monotonic()
        at, enabled, model = self._param_cache
        if not force and now - at < 1.0:
            return enabled, model
        enabled = model = None
        cli = self.bus.param_client(NODE, '/')
        if cli is not None and cli.wait_for_services(timeout_sec=1.5):
            descs = self.bus.param_describe(cli, ['enabled', 'model_path']) or []
            vals = self.bus.param_get_vals(cli, ['enabled', 'model_path']) or []
            for i, name in enumerate(('enabled', 'model_path')):
                if i >= len(descs) or i >= len(vals):
                    continue
                try:
                    value = _param_value_to_py(descs[i].type, vals[i])
                except Exception:
                    value = None
                if name == 'enabled' and isinstance(value, bool):
                    enabled = value
                elif name == 'model_path' and isinstance(value, str):
                    model = value
        self._param_cache = (time.monotonic(), enabled, model)
        return enabled, model

    def status(self) -> dict:
        enabled, model = self.params()
        return {
            'alive': enabled is not None,
            'enabled': enabled,
            'model': model,
            'recording': self.current(),
            'history': self.history(),
        }

    def status_lines(self) -> list[str]:
        st = self.status()
        on = 'ON' if st['enabled'] else 'OFF' if st['enabled'] is False else '? (노드 응답 없음)'
        lines = [f'[rl_nav_correction] 보정 {on}']
        if st['model']:
            lines.append(f' 모델: {st["model"]}')
        diags = [(n, lv, msg) for n, lv, msg in self.bus.diag_entries() if NODE in n]
        for n, lv, msg in diags[:4]:
            mark = 'OK' if lv == 0 else 'WARN' if lv == 1 else 'ERR' if lv == 2 else '?'
            lines.append(f' 진단 {self.bus.diag_short(n)}: {mark} {msg}')
        rec = st['recording']
        if rec:
            elapsed = max(0.0, time.time() - float(rec.get('started', time.time())))
            lines.append(f' 녹화 중: {rec.get("label", "-")} · {elapsed:.0f}s · {rec.get("bag")}')
        else:
            lines.append(' 녹화 중: 없음')
        hist = st['history'][-2:]
        if hist:
            lines.append(' 최근: ' + ' / '.join(
                f'{h.get("label", "?")} {Path(h["bag"]).name}' for h in hist))
        return lines

    # ------------------------------------------------------------------ 토글
    def toggle(self, on: bool) -> tuple[bool, str]:
        """서비스 우선, 실패하면 파라미터 enabled로 바꾼다."""
        try:
            from std_srvs.srv import SetBool
        except Exception:
            SetBool = None
        if SetBool is not None:
            if self._toggle_cli is None:
                self._toggle_cli = self.node.create_client(SetBool, TOGGLE)
            cli = self._toggle_cli
            if cli.wait_for_service(timeout_sec=2.0):
                req = SetBool.Request()
                req.data = bool(on)
                fut = cli.call_async(req)
                try:
                    rclpy.spin_until_future_complete(self.node, fut, timeout_sec=5.0)
                except Exception:
                    pass
                if fut.done():
                    try:
                        res = fut.result()
                    except Exception:
                        res = None
                    if res is not None and getattr(res, 'success', False):
                        self._param_cache = (0.0, None, None)
                        return True, str(getattr(res, 'message', '') or f'enabled={bool(on)}')

        from rclpy.parameter import Parameter
        cli = self.bus.param_client(NODE, '/')
        if cli is not None and cli.wait_for_services(timeout_sec=2.0):
            try:
                res = self.bus.param_wait(
                    cli.set_parameters([Parameter('enabled', value=bool(on))]), timeout=3.0)
            except Exception as exc:
                return False, f'파라미터 설정 실패: {exc}'
            if res is not None and getattr(res, 'results', None) and res.results[0].successful:
                self._param_cache = (0.0, None, None)
                return True, f'enabled={bool(on)}'
        return False, '토글 서비스·파라미터 모두 응답 없음'

    # ------------------------------------------------------------------ 기록
    @staticmethod
    def _log_tail(n: int = 8) -> str:
        try:
            lines = RECORD_LOG.read_text(errors='replace').splitlines()
            return '\n'.join(lines[-n:])
        except Exception:
            return ''

    def start(self, label: str | None = None, out: str | None = None) -> dict:
        cur = self.current()
        if cur:
            raise RuntimeError(f'이미 녹화 중: {cur.get("bag")}')
        label = re.sub(r'[^\w.-]+', '_', (label or 'run').strip()) or 'run'
        if out:
            bag = Path(out).expanduser()
        else:
            stamp = time.strftime('%Y%m%d_%H%M%S')
            bag = BAG_DIR / f'{stamp}_{label}'
            n = 1
            while bag.exists():
                n += 1
                bag = BAG_DIR / f'{stamp}_{label}_{n}'
        bag = bag.resolve()
        bag.parent.mkdir(parents=True, exist_ok=True)
        RECORD_LOG.parent.mkdir(parents=True, exist_ok=True)
        cmd = ['ros2', 'bag', 'record', '-s', 'mcap', '--disable-keyboard-controls',
               '-o', str(bag), '--topics', *TOPICS]
        started = time.time()
        with open(RECORD_LOG, 'wb') as log:
            log.write((' '.join(cmd) + '\n').encode())
            log.flush()
            proc = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT,
                                    stdin=subprocess.DEVNULL, start_new_session=True)
        # bag 폴더가 실제로 생길 때까지 기다린다 — discovery가 느리면 1~2초 걸리고,
        # 그 전에 stop하면 rosbag2가 폴더를 만들지 못한 채 끝날 수 있다.
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline:
            if proc.poll() is not None:
                raise RuntimeError(
                    f'ros2 bag record 시작 실패 (exit {proc.returncode})\n{self._log_tail()}')
            if bag.is_dir():
                break
            time.sleep(0.1)
        else:
            try:
                os.killpg(proc.pid, signal.SIGKILL)
            except Exception:
                pass
            raise RuntimeError(f'ros2 bag record가 8초 안에 bag 폴더를 만들지 못함\n{self._log_tail()}')
        time.sleep(0.2)
        if proc.poll() is not None:
            raise RuntimeError(f'ros2 bag record 시작 실패 (exit {proc.returncode})\n{self._log_tail()}')
        rec = {'pid': proc.pid, 'bag': str(bag), 'label': label,
               'started': started, 'log': str(RECORD_LOG)}
        data = self._load()
        data['recording'] = rec
        self._save(data)
        return rec

    def stop(self) -> dict | None:
        data = self._load()
        rec = data.get('recording')
        if not rec:
            return None
        pid = int(rec.get('pid') or 0)
        if pid and self._alive(pid):
            try:
                os.killpg(pid, signal.SIGINT)
            except ProcessLookupError:
                pass
            except Exception:
                try:
                    os.kill(pid, signal.SIGINT)
                except Exception:
                    pass
            end = time.monotonic() + 10.0
            while time.monotonic() < end and self._alive(pid):
                time.sleep(0.2)
            if self._alive(pid):
                try:
                    os.killpg(pid, signal.SIGKILL)
                except Exception:
                    pass
                time.sleep(0.5)
        return self._finalize(data, rec)

    # ------------------------------------------------------------------ 리포트
    def report(self, bags: list[str], labels: list[str] | None = None,
               out: str | None = None) -> int:
        if not REPORT.is_file():
            raise RuntimeError(f'리포트 스크립트 없음: {REPORT}')
        bags = [str(Path(b).expanduser()) for b in bags]
        for bag in bags:
            if not Path(bag).exists():
                raise RuntimeError(f'bag 없음: {bag}')
        if out is None:
            out = str(Path(bags[0]).parent / f'report_{time.strftime("%Y%m%d_%H%M%S")}.png')
        out = str(Path(out).expanduser())
        Path(out).parent.mkdir(parents=True, exist_ok=True)
        cmd = [sys.executable, str(REPORT), *bags]
        if labels:
            cmd += ['--labels', *labels]
        cmd += ['--out', out]
        print(f'리포트 생성: {" ".join(cmd)}')
        # subprocess 출력은 OS fd로 나가 TUI의 캡처를 피한다 — 직접 받아 print로 넘긴다.
        res = subprocess.run(cmd, capture_output=True, text=True)
        text = ((res.stdout or '') + (res.stderr or '')).rstrip()
        if text:
            print(text)
        if res.returncode == 0:
            print(f'저장: {out}')
        return int(res.returncode)

    def latest(self, n: int = 2) -> list[dict]:
        return self.history()[-n:]

    # ------------------------------------------------------------------ 명령
    LEGACY_ACTIONS = ('status', 'on', 'off', 'record', 'stop', 'report', 'guide')

    def cmd(self, args) -> int:
        target = getattr(args, 'node', None) or 'nodes'
        action = getattr(args, 'action', None)
        # 예전 `ab status`도 한동안 받아 준다 — 새 문법은 `ab rl_nav_correction status`.
        if target in self.LEGACY_ACTIONS and action is None:
            print(f'WARN: `ab {target}`는 예전 문법 — `ab {NODE} {target}`로 바뀌었습니다',
                  file=sys.stderr)
            action = target
            target = NODE
        try:
            if target == 'nodes':
                for name, desc in AB_NODES:
                    print(f'  {name:<24} {desc}')
                print('\n사용: ab <노드> <동작> — 예: ab rl_nav_correction status')
                return 0
            if target != NODE:
                print(f'FAIL: A/B 미구현 노드: {target} (가능: {", ".join(n for n, _ in AB_NODES)})',
                      file=sys.stderr)
                return 2
            action = action or 'status'
            if action == 'status':
                self.bus._poll(0.8)   # /diagnostics_agg가 올 시간을 준다
                print('\n'.join(self.status_lines()))
                return 0
            if action in ('on', 'off'):
                ok, msg = self.toggle(action == 'on')
                print(('ok: ' if ok else 'FAIL: ') + msg)
                return 0 if ok else 1
            if action == 'record':
                rec = self.start(getattr(args, 'label', None), getattr(args, 'out', None))
                print(f'녹화 시작: {rec["bag"]} (pid {rec["pid"]})')
                print(f'  로그: {rec["log"]}')
                print('  주행 후: ./run-cli ab rl_nav_correction stop')
                return 0
            if action == 'stop':
                rec = self.stop()
                if rec is None:
                    print('녹화 중 아님')
                    return 0
                dur = max(0.0, float(rec.get('stopped', time.time())) - float(rec.get('started', time.time())))
                print(f'녹화 종료: {rec["bag"]} ({dur:.1f}s)')
                return 0
            if action == 'report':
                bags = list(getattr(args, 'bags', []) or [])
                if getattr(args, 'last', False) or not bags:
                    pair = self.latest(2)
                    if len(pair) < 2:
                        print('FAIL: 최근 녹화가 2개 미만 — 먼저 A/B를 기록하세요', file=sys.stderr)
                        return 2
                    bags = [h['bag'] for h in pair]
                labels = getattr(args, 'labels', None)
                if not labels and len(bags) == 2:
                    by_bag = {h['bag']: h.get('label') for h in self.history()}
                    labels = [by_bag.get(bags[0], 'A'), by_bag.get(bags[1], 'B')]
                return self.report(bags, labels, getattr(args, 'out', None))
            if action == 'guide':
                return self.guide(getattr(args, 'label', None))
            print(f'FAIL: 모르는 ab 동작: {action}', file=sys.stderr)
            return 2
        except (RuntimeError, OSError) as exc:
            print(f'FAIL: {exc}', file=sys.stderr)
            return 1

    def guide(self, label: str | None = None) -> int:
        """터미널용 안내형 A/B — A(ON) → B(OFF) → 리포트. 주행은 사람이 한다."""
        if not sys.stdin.isatty():
            print('FAIL: guide는 터미널에서만 실행하세요', file=sys.stderr)
            return 2
        label = (label or time.strftime('%Y%m%d_%H%M%S')).strip()
        st = self.status()
        if not st['alive']:
            print('FAIL: rl_nav_correction 응답 없음 — bringup 확인', file=sys.stderr)
            return 1
        bags: list[str] = []
        changed = False
        try:
            if not st['enabled']:
                ok, msg = self.toggle(True)
                if not ok:
                    print(f'FAIL: {msg}', file=sys.stderr)
                    return 1
                changed = True
                print('보정 ON')
            rec = self.start(f'{label}_A')
            bags.append(rec['bag'])
            print(f'A 녹화: {rec["bag"]}')
            input('A 주행을 마친 뒤 Enter... ')
            self.stop()
            print('A 종료')

            ok, msg = self.toggle(False)
            if not ok:
                print(f'FAIL: {msg}', file=sys.stderr)
                return 1
            changed = True
            print('보정 OFF')
            input('로봇을 A와 같은 출발 자세·방향으로 두고 Enter... ')
            rec = self.start(f'{label}_B')
            bags.append(rec['bag'])
            print(f'B 녹화: {rec["bag"]}')
            input('B 주행을 마친 뒤 Enter... ')
            self.stop()
            print('B 종료')
        except (EOFError, KeyboardInterrupt):
            print('\n중단', file=sys.stderr)
            return 130
        finally:
            if self.current():
                self.stop()
            if changed:
                st2 = self.status()
                if st2['enabled'] is not True:
                    self.toggle(True)
        if len(bags) < 2:
            return 1
        print('리포트...')
        return self.report(bags, ['A', 'B'])
