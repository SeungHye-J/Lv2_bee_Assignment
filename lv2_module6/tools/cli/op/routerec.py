"""가르친 경로 녹화 (teach & repeat, 11-15) — 수동 조작으로 모는 동안 로봇 자리(map→base_footprint)를 5 cm마다 남겨
arena 좌표로 ~/.ros/routes/<이름>.txt에 저장. AUTO에서 skill_navigate가 시작이 로봇 근처 · 끝이 목표 근처인 경로를
경유점으로 씀 (거꾸로도 — 복귀). 끝 = 다음 navigate 목표 자리(상자 앞 다가설 자리)에서 상자를 보고 멈춘 뒤 Enter."""
from __future__ import annotations

import math
import select
import sys
import time
from pathlib import Path

ROUTES = Path.home() / '.ros' / 'routes'
START = (0.734, 0.950, math.pi)   # plan.yaml · skills.yaml start_pose_arena (map 원점 = 켠 자리)
STEP_M = 0.05


def to_arena(mx: float, my: float, start=START) -> tuple[float, float]:
    """map → arena (skill_navigate의 arena → map 역변환: map = R(−θ0)·(arena − p0))."""
    c, s = math.cos(start[2]), math.sin(start[2])
    return start[0] + c * mx - s * my, start[1] + s * mx + c * my


def _save(out: Path, name: str, pts) -> None:
    length = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:]))
    out.write_text(f'# {name} · {time.strftime("%Y-%m-%d %H:%M")} · {len(pts)}점 · {length:.2f} m · arena x y\n'
                   + ''.join(f'{x:.3f} {y:.3f}\n' for x, y in pts))
    print(f'\n저장: {out.name} ({len(pts)}점 · {length:.2f} m) — 시작 ({pts[0][0]:.2f}, {pts[0][1]:.2f}) → 끝 ({pts[-1][0]:.2f}, {pts[-1][1]:.2f})')


def record(node, tf_buf, name: str, start=START) -> int:
    """한 번에 쭉 몰면서 구간마다 Enter — <이름>_01.txt, _02.txt …로 나눠 저장 (끝 점 = 다음 구간 시작).
    q + Enter = 지금 구간까지 저장하고 끝 · Ctrl+C = 지금 구간은 버리고 끝."""
    import rclpy
    from rclpy.time import Time
    ROUTES.mkdir(parents=True, exist_ok=True)
    seg, pts, saved = 1, [], 0
    print(f'경로 녹화: {name} — 수동으로 몰다가 구간 끝(상자 앞)에서 멈추고 Enter = 그 구간 저장 · 이어서 다음 구간\n'
          f'  q + Enter = 끝 · 저장 폴더 {ROUTES}')
    try:
        while True:
            rclpy.spin_once(node, timeout_sec=0.1)
            try:
                tr = tf_buf.lookup_transform('map', 'base_footprint', Time()).transform.translation
                p = to_arena(tr.x, tr.y, start)
                if not pts or math.hypot(p[0] - pts[-1][0], p[1] - pts[-1][1]) >= STEP_M:
                    pts.append(p)
                    print(f'\r  구간 {seg} · 점 {len(pts)}개 · 지금 arena ({p[0]:.2f}, {p[1]:.2f})   ', end='', flush=True)
            except Exception:
                pass
            if not select.select([sys.stdin], [], [], 0)[0]:
                continue
            quit_ = sys.stdin.readline().strip().lower() == 'q'
            if len(pts) >= 2 and math.hypot(pts[-1][0] - pts[0][0], pts[-1][1] - pts[0][1]) >= 0.3:
                _save(ROUTES / f'{name}_{seg:02d}.txt', f'{name} 구간 {seg}', pts)
                saved += 1
                seg += 1
                pts = pts[-1:]   # 다음 구간은 이 자리에서 시작
            elif not quit_:
                print('\n  구간이 0.3 m보다 짧음 — 저장 안 함 (계속 몰기)')
            if quit_:
                break
    except KeyboardInterrupt:
        print('\n  Ctrl+C — 지금 구간은 버림')
    print(f'끝 — 구간 {saved}개 저장')
    return 0 if saved else 1


if __name__ == '__main__':   # 자체 확인: 출발 자리(map 원점) = arena 출발 · map +x(로봇 앞) = arena −x
    assert to_arena(0, 0) == START[:2]
    ax, ay = to_arena(1.0, 0.0)
    assert abs(ax - (START[0] - 1.0)) < 1e-9 and abs(ay - START[1]) < 1e-9
    ax, ay = to_arena(-2.202, -3.854)   # 11-15 실기 S3 목표(map) = 상자 3 (2.936, 5.354) −y 앞 0.55 m
    assert abs(ax - 2.936) < 1e-3 and abs(ay - 4.804) < 1e-3
    print('ok routerec')
