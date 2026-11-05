#!/usr/bin/env python3
"""rl_nav_correction — rosbag2 A/B 리포트 (report.png).

  ./run-cli ab report ~/bags/on ~/bags/off --labels ON OFF --out ~/report.png
  ./run-cli ab record --label on   # (다른 터미널에서) 주행

읽는 토픽: /cmd_vel_nav · /cmd_vel_corrected · /odometry/filtered(/odom) · /odom · /imu · /plan · /rl_nav_correction/debug
지표: 횡오차(RMSE·최대) · 헤딩오차(MAE) · 슬립(|ωimu−ωodom|) · 적용 Δ(평균·최대) · 시간·거리·보정 ON 비율
Δ는 /rl_nav_correction/debug의 Δv·Δω를 직접 읽어 토픽 개수 차이로 생기는 정렬 오차를 피한다.
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
from common.bagio import plan_points, project_to_polyline, read_bag, series, wrap, yaw_of  # noqa: E402

KOREAN_FONTS = [
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc",
]


def stats(label: str, bag: str, data: dict) -> dict:
    res = {"bag": bag}
    # Δ는 debug 한 메시지 안의 입력·출력·Δ를 같이 읽는다 — 토픽별 메시지 수가
    # bag 시작/끝에서 한두 개 어긋나도 Δ가 과대평가되지 않는다.
    dbg = data.get("/rl_nav_correction/debug", [])
    rows = [m.data for _, m in dbg if len(m.data) >= 11]
    if rows:
        dv = np.array([r[6] for r in rows], float)
        dw = np.array([r[7] for r in rows], float)
        res["dv_abs"] = float(np.mean(np.abs(dv)))
        res["dv_max"] = float(np.max(np.abs(dv)))
        res["dw_abs"] = float(np.mean(np.abs(dw)))
        res["dw_max"] = float(np.max(np.abs(dw)))
        res["on"] = float(np.mean([r[10] for r in rows]))
    else:  # 예전 bag 호환 — debug가 없으면 토픽 차이로 계산
        _, v_nav = series(data.get("/cmd_vel_nav", []), lambda m: m.twist.linear.x)
        _, v_out = series(data.get("/cmd_vel_corrected", []), lambda m: m.twist.linear.x)
        _, w_nav = series(data.get("/cmd_vel_nav", []), lambda m: m.twist.angular.z)
        _, w_out = series(data.get("/cmd_vel_corrected", []), lambda m: m.twist.angular.z)
        if len(v_nav) and len(v_out):
            n = min(len(v_nav), len(v_out))
            res["dv_abs"] = float(np.mean(np.abs(v_out[:n] - v_nav[:n])))
            res["dv_max"] = float(np.max(np.abs(v_out[:n] - v_nav[:n])))
            res["dw_abs"] = float(np.mean(np.abs(w_out[:n] - w_nav[:n])))
            res["dw_max"] = float(np.max(np.abs(w_out[:n] - w_nav[:n])))
    odom = data.get("/odometry/filtered") or data.get("/odom") or []
    if odom:
        t, _ = series(odom, lambda m: 0.0)
        xy = np.array([[m.pose.pose.position.x, m.pose.pose.position.y] for _, m in odom], float)
        yaw = np.array([yaw_of(m.pose.pose.orientation) for _, m in odom], float)
        res["duration"] = float(t[-1] - t[0]) if len(t) > 1 else 0.0
        res["length"] = float(np.sum(np.linalg.norm(np.diff(xy, axis=0), axis=1)))
        poly = plan_points(data.get("/plan", []))
        if len(poly) >= 2:
            d, s, head = project_to_polyline(xy, poly)
            res["lat_rmse"] = float(np.sqrt(np.mean(d ** 2)))
            res["lat_max"] = float(np.max(d))
            res["head_mae"] = float(np.mean(np.abs(wrap(yaw - head))))
            res["progress"] = float(s[-1]) if len(s) else 0.0
            res["goal_err"] = float(np.linalg.norm(xy[-1] - poly[-1]))
    od = data.get("/odom", [])
    imu = data.get("/imu", [])
    if od and imu:
        _, w_od = series(od, lambda m: m.twist.twist.angular.z)
        _, w_im = series(imu, lambda m: m.angular_velocity.z)
        n = min(len(w_od), len(w_im))
        res["slip_mean"] = float(np.mean(np.abs(w_im[:n] - w_od[:n])))
    return res


def plot(runs, out: Path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    from matplotlib import font_manager as fm

    for f in KOREAN_FONTS:
        if Path(f).is_file():
            try:
                fm.fontManager.addfont(f)
            except Exception:
                pass
    import matplotlib.pyplot as plt
    matplotlib.rcParams["font.family"] = ["Noto Sans CJK JP", "Noto Sans CJK KR", "DejaVu Sans"]
    matplotlib.rcParams["axes.unicode_minus"] = False

    colors = ["#1e88e5", "#e53935", "#43a047", "#8e24aa"]
    fig, axes = plt.subplots(2, 2, figsize=(13, 9.0))
    fig.subplots_adjust(left=0.06, right=0.98, top=0.92, bottom=0.17, hspace=0.32, wspace=0.22)
    fig.suptitle("rl_nav_correction — A/B report", fontsize=13, fontweight="bold")

    # 1) 선속도
    ax = axes[0, 0]
    for i, (label, d, _st) in enumerate(runs):
        c = colors[i % len(colors)]
        for topic, ls, lw, name in (("/cmd_vel_nav", "-", 1.0, "cmd_nav"),
                                    ("/cmd_vel_corrected", "-", 1.5, "corrected")):
            if d.get(topic):
                t, v = series(d[topic], lambda m: m.twist.linear.x)
                if t.size:
                    ax.plot(t - t[0], v, c, lw=lw, ls=ls, alpha=0.85, label=f"{label} {name}")
        od = d.get("/odometry/filtered") or d.get("/odom") or []
        if od:
            t, v = series(od, lambda m: m.twist.twist.linear.x)
            if t.size:
                ax.plot(t - t[0], v, c, lw=0.8, ls="--", alpha=0.6, label=f"{label} odom")
    ax.set_title("선속도 [m/s]")
    ax.set_xlabel("time [s]")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=7, ncol=2)

    # 2) XY 궤적
    ax = axes[0, 1]
    poly = None
    for i, (label, d, _st) in enumerate(runs):
        c = colors[i % len(colors)]
        od = d.get("/odometry/filtered") or d.get("/odom") or []
        if od:
            xy = np.array([[m.pose.pose.position.x, m.pose.pose.position.y] for _, m in od])
            if len(xy):
                ax.plot(xy[:, 0], xy[:, 1], c, lw=1.5, label=label)
        if poly is None and d.get("/plan"):
            poly = plan_points(d["/plan"])
    if poly is not None and len(poly) >= 2:
        ax.plot(poly[:, 0], poly[:, 1], "k--", lw=1.0, alpha=0.6, label="plan")
    ax.set_title("XY 궤적 [m]")
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)

    # 3) 횡오차
    ax = axes[1, 0]
    for i, (label, d, _st) in enumerate(runs):
        c = colors[i % len(colors)]
        od = d.get("/odometry/filtered") or d.get("/odom") or []
        if od and poly is not None and len(poly) >= 2:
            xy = np.array([[m.pose.pose.position.x, m.pose.pose.position.y] for _, m in od])
            dist, s, _ = project_to_polyline(xy, poly)
            ax.plot(s, dist * 100.0, c, lw=1.3, label=label)
    ax.set_title("경로 횡오차 [cm]")
    ax.set_xlabel("진행 거리 [m]")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)

    # 4) 적용 Δ
    ax = axes[1, 1]
    for i, (label, d, _st) in enumerate(runs):
        c = colors[i % len(colors)]
        dbg = d.get("/rl_nav_correction/debug", [])
        if dbg and len(dbg[0][1].data) >= 8:
            t = np.array([s for s, _ in dbg])
            dv = np.array([m.data[6] for _, m in dbg])
            dw = np.array([m.data[7] for _, m in dbg])
            ax.plot(t - t[0], dv, c, lw=1.2, label=f"{label} Δv [m/s]")
            ax.plot(t - t[0], dw, c, lw=1.0, ls=":", alpha=0.85, label=f"{label} Δω [rad/s]")
    ax.set_title("적용 Δ (정책 출력)")
    ax.set_xlabel("time [s]")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=7, ncol=2)

    lines = []
    for label, _d, st in runs:
        parts = []
        if "lat_rmse" in st:
            parts.append(f"횡오차 RMSE {st['lat_rmse']:.3f} m / 최대 {st['lat_max']:.3f} m")
        if "head_mae" in st:
            parts.append(f"헤딩 MAE {math.degrees(st['head_mae']):.1f}°")
        if "slip_mean" in st:
            parts.append(f"슬립 {st['slip_mean']:.3f} rad/s")
        if "dv_abs" in st:
            parts.append(f"Δv {st['dv_abs']:.4f} m/s (최대 {st['dv_max']:.4f})")
        if "goal_err" in st:
            parts.append(f"최종 goal 오차 {st['goal_err']:.3f} m")
        if "duration" in st:
            parts.append(f"{st['duration']:.1f}s · {st['length']:.2f} m")
        if "on" in st:
            parts.append(f"보정 ON {st['on']:.0%}")
        lines.append(f"{label}: " + " · ".join(parts))
    fig.text(0.01, 0.02, "\n".join(lines), fontsize=8.5, va="bottom")
    fig.savefig(out, dpi=140)
    print(f"저장: {out}")
    for ln in lines:
        print(ln)


def main() -> int:
    ap = argparse.ArgumentParser(description="rl_nav_correction rosbag2 A/B report")
    ap.add_argument("bags", nargs="+", help="bag 폴더 (1개=요약, 2개=ON/OFF 비교)")
    ap.add_argument("--labels", nargs="*", help="범례 이름 (기본 ON/OFF)")
    ap.add_argument("--out", default="rl_nav_correction_report.png")
    a = ap.parse_args()
    labels = a.labels or (["ON", "OFF"] if len(a.bags) == 2 else [Path(b).name for b in a.bags])
    runs = []
    for i, bag in enumerate(a.bags):
        if not Path(bag).exists():
            print(f"FAIL: bag 없음: {bag}", file=sys.stderr)
            return 2
        data = read_bag(bag)
        label = labels[i] if i < len(labels) else Path(bag).name
        st = stats(label, bag, data)
        runs.append((label, data, st))
        print(f"[{label}] " + " · ".join(f"{k}={v:.4g}" for k, v in st.items() if isinstance(v, float)))
    plot(runs, Path(a.out))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
