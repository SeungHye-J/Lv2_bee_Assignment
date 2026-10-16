#!/usr/bin/env python3
"""YOLO 데이터셋 자동 생성 (호스트) — test-faker 기동 상태에서 실행.

시뮬 렌더 + 정답 위치를 pole_cam에 투영해 YOLO 라벨 생성. 원본 마킹 없이
순수 장면만 저장한다 (마킹은 ai-service가 추론용으로 따로 붙임).
클래스: 0 puck · 1 goal_zone · 2 obstacle · 3 person(이동물체 정지분) · 4 traffic_light(신호등, 상태 랜덤)

  cd lv2_module6/simulator && .venv/bin/python export_dataset.py --out dataset --n 800 --views 3
  → dataset/{train,val}/{images,labels}/ + dataset/data.yaml (ultralytics용)

URDF는 실행 중인 테스트베드의 /robot_description을 그대로 쓴다 (bridge.read_msg).
"""
import argparse
import math
import random
import shutil
from pathlib import Path

import cv2
import numpy as np

import maze
import scene
from bridge import read_msg

HERE = Path(__file__).resolve().parent
CLASSES = ["puck", "goal_zone", "obstacle", "person", "traffic_light"]
IMG_W, IMG_H = 640, 480  # fake_camera 출력과 동일


def free_pose(cells, boxes, rng):
    """벽에서 0.25m 이상 떨어진 칸 안 무작위 (x, y, yaw)."""
    for _ in range(100):
        cx, cy = rng.choice(cells)
        x, y = cx + rng.uniform(-0.2, 0.2), cy + rng.uniform(-0.2, 0.2)
        if all(abs(x - bx) > (sx / 2 + 0.25) or abs(y - by) > (sy / 2 + 0.25)
               for bx, by, _, sx, sy, _ in boxes):
            return x, y, rng.uniform(-math.pi, math.pi)
    return None


def corners_3d(p):
    """물체 점유 8점 (world). z0=바닥 띄움."""
    zo = float(p.get("z", 0.0))
    x, y, yaw = p["x"], p["y"], p.get("yaw", 0.0)
    cy, sy = math.cos(yaw), math.sin(yaw)
    if p["shape"] == "sphere":
        r = p["size"][0]
        pts = [(dx, dy, dz) for dx in (-r, r) for dy in (-r, r) for dz in (0, 2 * r)]
    elif p["shape"] == "cylinder":
        r, h = p["size"][0], p["size"][1]
        pts = [(r * math.cos(a), r * math.sin(a), dz)
               for a in (0, math.pi / 2, math.pi, 3 * math.pi / 2) for dz in (0, h)]
    else:
        sx = p.get("sx", p["size"][0])
        syy = p.get("sy", p["size"][0])
        h = p["size"][1]
        pts = [(dx, dy, dz) for dx in (-sx / 2, sx / 2) for dy in (-syy / 2, syy / 2)
               for dz in (0, h)]
    R = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    out = []
    for dx, dy, dz in pts:
        w = R @ np.array([dx, dy, dz])
        out.append((x + w[0], y + w[1], zo + w[2]))
    return out


def project_box(p, T_cw, fx, cx, cy, W, H, depth):
    """→ (xc, yc, w, h) 정규화 bbox 또는 None (뒤·프레임밖·가려짐·너무작음 제외)."""
    pts = []
    for X, Y, Z in corners_3d(p):
        q = T_cw @ np.array([X, Y, Z, 1.0])
        if q[2] < 0.05:
            return None
        pts.append((q[0], q[1], q[2]))
    us = [fx * q[0] / q[2] + cx for q in pts]
    vs = [fx * q[1] / q[2] + cy for q in pts]
    x0, x1, y0, y1 = min(us), max(us), min(vs), max(vs)
    if x1 < 0 or x0 >= W or y1 < 0 or y0 >= H:
        return None
    # 가림 검사: 중심 픽셀 깊이가 물체보다 앞에 있으면 벽·다른 물체 뒤
    u, v = int((x0 + x1) / 2), int((y0 + y1) / 2)
    u, v = min(max(u, 0), W - 1), min(max(v, 0), H - 1)
    zc = sum(q[2] for q in pts) / len(pts)
    dm = depth[v, u] / 1000.0
    if dm > 0 and dm < zc - 0.08:
        return None
    x0c, x1c, y0c, y1c = max(x0, 0), min(x1, W - 1), max(y0, 0), min(y1, H - 1)
    if (x1c - x0c) * (y1c - y0c) < 64:
        return None
    return ((x0c + x1c) / 2 / W, (y0c + y1c) / 2 / H, (x1c - x0c) / W, (y1c - y0c) / H)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="dataset")
    ap.add_argument("--n", type=int, default=800, help="장면 수 (=이미지 n×views)")
    ap.add_argument("--views", type=int, default=3, help="장면당 시점 수")
    ap.add_argument("--seed", type=int, default=7)
    args = ap.parse_args()

    desc, _ = read_msg("robot_description")
    if desc:
        urdf_xml = desc["data"]
    else:
        # 테스트베드가 아직 robot_description을 안 뿌림 — xacro 직접 전개
        import xacro as xacro_lib
        urdf_xml = xacro_lib.process_file(
            str(HERE.parent / "urdf" / "beebot.urdf.xacro")).toxml()
    kin = scene.Kinematics(urdf_xml)
    course = maze.load()
    renderer = scene.Renderer(maze.wall_boxes(course))
    cells = scene.maze_cells(course)
    boxes = maze.wall_boxes(course)
    goal = scene.goal_zone(course, cells)
    fx = (scene.RENDER_W / 2) / math.tan(math.radians(scene.HFOV_DEG) / 2)
    cx, cy = scene.RENDER_W / 2, scene.RENDER_H / 2
    rng = random.Random(args.seed)

    out = Path(args.out)
    imgdir, labdir = out / "images", out / "labels"
    for d in (imgdir, labdir):
        d.mkdir(parents=True, exist_ok=True)

    kinds = {"puck": 0, "goal_zone": 1, "obstacle": 2, "person": 3, "traffic_light": 4}
    n_img = 0
    for i in range(args.n):
        pucks = scene.random_pucks(rng.randint(4, 8), cells, course["spawn"], rng)
        props = []
        for _ in range(rng.randint(0, 2)):
            ob = scene.random_obstacle(cells, [(p["x"], p["y"]) for p in pucks])
            if ob:
                props.append(ob)
        persons = []
        for _ in range(rng.randint(0, 2)):
            for _ in range(50):
                cx0, cy0 = rng.choice(cells)
                x, y = cx0 + rng.uniform(-0.2, 0.2), cy0 + rng.uniform(-0.2, 0.2)
                if all(math.dist((x, y), (p["x"], p["y"])) > 0.5 for p in pucks + props) \
                        and all(abs(x - bx) > (sx / 2 + 0.3) or abs(y - by) > (sy / 2 + 0.3)
                                for bx, by, _, sx, sy, _ in boxes):
                    persons.append({"kind": "mover", "shape": "cylinder",
                                    "color": (200, 120, 50), "x": round(x, 3), "y": round(y, 3),
                                    "yaw": 0.0, "size": [0.2, 1.7]})
                    break
        tls = []
        for _ in range(rng.randint(0, 1)):
            for _ in range(50):
                cx0, cy0 = rng.choice(cells)
                x, y = cx0 + rng.uniform(-0.2, 0.2), cy0 + rng.uniform(-0.2, 0.2)
                if all(math.dist((x, y), (p["x"], p["y"])) > 0.5 for p in pucks + props + persons) \
                        and all(abs(x - bx) > (sx / 2 + 0.3) or abs(y - by) > (sy / 2 + 0.3)
                                for bx, by, _, sx, sy, _ in boxes):
                    tls.append({"kind": "traffic_light", "shape": "box",
                                "color": scene.TL_COLORS["off"], "x": round(x, 3), "y": round(y, 3),
                                "yaw": 0.0, "sx": 0.14, "sy": 0.14, "size": [0.14, 0.45],
                                "state": rng.choice(["red", "yellow", "green", "off"])})
                    break
        labeled = ([(p, 0) for p in pucks] + [(goal, 1)] +
                   [(p, 2) for p in props] + [(p, 3) for p in persons] +
                   [(p, 4) for p in tls])
        for v in range(args.views):
            pose = free_pose(cells, boxes, rng)
            if pose is None:
                continue
            x, y, _ = pose
            # 시점을 물체 쪽으로 — 빈 벽만 보는 장면 방지
            tx, ty = rng.choice([(p["x"], p["y"]) for p, _ in labeled])
            if math.dist((x, y), (tx, ty)) < 0.3:
                continue
            yaw = math.atan2(ty - y, tx - x) + rng.uniform(-0.4, 0.4)
            T_wb = scene.transform(scene.rpy_matrix(0, 0, yaw), (x, y, 0))
            T_wc = T_wb @ kin.pose("pole_cam_color_optical_frame", {})
            T_cw = np.linalg.inv(T_wc)
            things = [dict(p) for p in pucks + props + persons + tls] + [dict(goal)]
            bgr, depth = renderer.render(T_wc, things)[:2]
            lines = []
            for p, cls in labeled:
                b = project_box(p, T_cw, fx, cx, cy,
                                scene.RENDER_W, scene.RENDER_H, depth)
                if b:
                    lines.append(f"{cls} {b[0]:.6f} {b[1]:.6f} {b[2]:.6f} {b[3]:.6f}")
            if not lines:  # 빈 장면은 버림 (배경 편향 방지)
                continue
            name = f"{i:05d}_{v}"
            cv2.imwrite(str(imgdir / f"{name}.png"),
                         cv2.resize(bgr, (IMG_W, IMG_H), interpolation=cv2.INTER_LINEAR))
            (labdir / f"{name}.txt").write_text("\n".join(lines) + "\n")
            n_img += 1
    # ultralytics용 분할 + data.yaml
    names = sorted(p.stem for p in imgdir.glob("*.png"))
    if not names:
        print("경고: 라벨 있는 장면 0장 — --n/--views를 늘리시오")
        (out / "data.yaml").write_text(
            f"path: {out.resolve()}\ntrain: train/images\nval: val/images\n"
            f"nc: {len(CLASSES)}\nnames: {CLASSES}\n")
        return
    rng.shuffle(names)
    n_val = max(1, len(names) // 10)
    val = set(names[:n_val])
    for split, sel in (("train", [n for n in names if n not in val]),
                       ("val", [n for n in names if n in val])):
        (out / split / "images").mkdir(parents=True, exist_ok=True)
        (out / split / "labels").mkdir(parents=True, exist_ok=True)
        for n in sel:
            shutil.move(str(imgdir / f"{n}.png"), str(out / split / "images" / f"{n}.png"))
            shutil.move(str(labdir / f"{n}.txt"), str(out / split / "labels" / f"{n}.txt"))
    imgdir.rmdir()
    labdir.rmdir()
    (out / "data.yaml").write_text(
        f"path: {out.resolve()}\ntrain: train/images\nval: val/images\n"
        f"nc: {len(CLASSES)}\nnames: {CLASSES}\n")
    print(f"OK: {n_img}장 → {out}/train {len(names) - n_val}장 · val {n_val}장")


if __name__ == "__main__":
    main()
