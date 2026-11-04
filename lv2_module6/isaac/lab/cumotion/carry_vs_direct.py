"""Arm time of the demo cycle's carry detours vs one cuMotion move, offline (no Isaac app).

    ~/pa-course/IsaacLab-6.1/.venv/bin/python cumotion/carry_vs_direct.py [--vel_scale 0.3]

Moves, as demo_cycle.py plans them (same slots, cells, over-slot poses, carry poses):
  load    pick lift pose (start box, both sides) -> carry -> over slot          (12 slots)
  unload  over slot after the lift -> carry, 0.5 s hold -> over the dice cell   (12 slots x 6 cells)
Per move: IK route (via carry, synchronized trapezoids, stop at carry), straight joint line (no planner), and cuMotion
to the same joint target (c-space target, so the place policy starts from the same q_above it was trained on).
Times are at the same joint limits (vel_scale x 3 rad/s, 5 rad/s^2): cuMotion's trajectory is stretched to them,
so its peak joint speed / acceleration (base reaction) is no higher than the current route.
The held puck is not in cuMotion's model: every path is checked for it against the basket meshes here (30 x 30 x 60 mm
box under the fingers, as GRASP_Z).
"""

import argparse
import glob
import math
import sys
import tempfile
from pathlib import Path

import cumotion as cm
import numpy as np
import pinocchio as pin
import trimesh

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import bee_common as bc  # noqa: E402

HERE = Path(__file__).resolve().parent
T = cm.TrajectoryOptimizer
# demo_cycle.py constants (not imported: it starts the Isaac app)
SLOTS = [(x, y, 0.09125) for x in (-0.16984, -0.10484, -0.03984) for y in (-0.0975, -0.0325, 0.0325, 0.0975)]
JOINT1_XY = (0.047985, -0.00023)
OVER = {0: (0.05, -20.0), 1: (0.045, 0.0), 2: (0.06, 0.0)}
WRIST = math.pi / 2
CARRY = {1: (math.pi / 2, -0.465027, 0.102271, 1.933412, math.pi / 2),
         -1: (-math.pi / 2, -0.465027, 0.102271, 1.933412, -math.pi / 2)}
BOX_CX, START_GAP = 0.08 + bc.BOX_HALF, 0.080
CELLS = [(BOX_CX + bc.CELL_PITCH * (r - 1), bc.CELL_PITCH * (c - 1)) for r in (0, 1) for c in range(3)]
D_NOM = np.array([0.0, 0.0, bc.GRASP_Z])  # puck bottom in the end effector frame
CARRY_HOLD = 0.5  # unload waits at carry (demo_cycle "8 carry pose")
PUCK_W, PUCK_H = 0.030, bc.PUCK_H


def rot_tool(heading, pitch_deg, yaw):
    return pin.utils.rotate("z", heading) @ pin.utils.rotate("y", math.radians(pitch_deg)) @ bc.DOWN @ pin.utils.rotate("z", -(yaw - heading))


def wall_yaw(yaw):
    return yaw + math.remainder(-yaw, math.pi / 2)


ap = argparse.ArgumentParser()
ap.add_argument("--vel_scale", type=float, default=0.3)
ap.add_argument("--match_acc", action="store_true",
                help="plan with acceleration / jerk limits raised so that, stretched to vel_scale, cuMotion saturates both "
                     "the speed and the acceleration limit (by default it plans at 3 rad/s, 5 rad/s^2: stretched to "
                     "vel_scale its acceleration stays under the limit)")
ap.add_argument("--param", action="append", default=[], help="trajectory optimizer parameter name=value (repeatable)")
args = ap.parse_args()
VMAX, AMAX = bc.VMAX * args.vel_scale, bc.AMAX * args.vel_scale
ik = bc.ArmIK()
base = ik.base_in_root()
xrdf = (HERE / "beebot.xrdf").read_text()
if args.match_acc:  # stretch k = 1 / vel_scale reaches VMAX; plan acc = AMAX k^2, jerk 100 k^3 (100 rad/s^3 after the stretch)
    k = 1.0 / args.vel_scale
    xrdf = xrdf.replace("  - 5.0\n" * 5, f"  - {AMAX * k * k:.4f}\n" * 5).replace("  - 100.0\n" * 5, f"  - {100.0 * k ** 3:.1f}\n" * 5)
tmp = Path(tempfile.mkdtemp()) / "beebot.xrdf"
tmp.write_text(xrdf)
rd = cm.load_robot_from_file(str(tmp), str(bc.URDF_PATH))
world = cm.create_world()
box = cm.create_obstacle(cm.Obstacle.Type.CUBOID)  # start box / dice box in front (same size and dock distance)
box.set_attribute(cm.Obstacle.Attribute.SIDE_LENGTHS, cm.Obstacle.AttributeValue(np.array([2 * bc.BOX_HALF, 2 * bc.BOX_HALF, bc.BOX_H])))
world.add_obstacle(box, cm.Pose3.from_translation((np.array([BOX_CX, 0.0, bc.BOX_H / 2]) - base).reshape(3, 1)))
view = world.add_world_view()
cfg = cm.create_default_trajectory_optimizer_config(rd, "end_effector_link", view)
for kv in args.param:
    name, val = kv.split("=")
    val = int(val) if val.lstrip("-").isdigit() else (val == "true" if val in ("true", "false") else float(val))
    assert cfg.set_param(name, cm.TrajectoryOptimizerConfig.ParamValue(val)), f"bad parameter {kv}"
opt = cm.create_trajectory_optimizer(cfg)
insp = cm.create_robot_world_inspector(rd, view)

# basket (URDF storage collision meshes, base_link frame) for the held puck
parts = []
for f in sorted(glob.glob(str(bc.URDF_PATH.parent / "meshes" / "storage_collision" / "*.stl"))):
    m = trimesh.load(f)
    m.apply_scale(0.001)
    tf = trimesh.transformations.euler_matrix(1.570796, 0, 0)
    tf[:3, 3] = [-0.064, 0.0, -0.0005]
    m.apply_transform(tf)
    parts.append(m.convex_hull)
basket = trimesh.util.concatenate(parts)
g = np.linspace(-PUCK_W / 2, PUCK_W / 2, 4)
PUCK_PTS = np.array([[x, y, z] for x in g for y in g for z in np.linspace(0, -PUCK_H, 7)
                     if abs(x) == PUCK_W / 2 or abs(y) == PUCK_W / 2 or z in (0, -PUCK_H)])  # surface, from the bottom up


def puck_clearance(qs):
    """Min distance [m] of the held puck to the basket along the joint samples (negative = inside a wall)."""
    worst = np.inf
    for q in qs:
        p, r = ik.fk(q)
        pts = (p + r @ D_NOM) + PUCK_PTS @ r.T  # bottom + offsets (tool z points down: -z goes up the puck)
        d = trimesh.proximity.closest_point(basket, pts)[1].min()
        if any(pt.contains(pts).any() for pt in parts):
            d = -d
        worst = min(worst, d)
    return worst


def t_move(qa, qb):
    return bc.trapezoid_duration(float(np.abs(np.asarray(qb) - np.asarray(qa)).max()), args.vel_scale)


def line(qa, qb, n=40):
    return [np.asarray(qa) + s * (np.asarray(qb) - np.asarray(qa)) for s in np.linspace(0, 1, n)]


def arm_hits(qs):
    return any(insp.in_collision_with_obstacle(np.asarray(q, dtype=np.float64).reshape(5, 1))
               or insp.in_self_collision(np.asarray(q, dtype=np.float64).reshape(5, 1)) for q in qs)


def cumo(qa, qb):
    """-> (duration at the team limits x vel_scale, joint samples) or (None, None)."""
    res = opt.plan_to_cspace_target(np.asarray(qa, dtype=np.float64).reshape(5, 1), T.CSpaceTarget(np.asarray(qb, dtype=np.float64).reshape(5, 1)))
    if res.status() != T.Results.Status.SUCCESS:
        return None, None
    tr = res.trajectory()
    d = tr.domain()
    ts = np.linspace(d.lower, d.upper, 200)
    v = max(np.abs(tr.eval(t, 1)).max() for t in ts)
    a = max(np.abs(tr.eval(t, 2)).max() for t in ts)
    k = max(v / VMAX, math.sqrt(a / AMAX))  # time stretch so |qdot| <= VMAX and |qddot| <= AMAX
    return (d.upper - d.lower) * k, [tr.eval(t).ravel() for t in ts[::5]]


def q_over_slot(slot):
    sx, sy, sz = SLOTS[slot]
    hd = math.atan2(sy - JOINT1_XY[1], sx - JOINT1_XY[0])
    wr = -WRIST if sy > 0 else WRIST
    yaw = wall_yaw(hd + wr)  # box puck (cylinder slots 0, 3, ... use hd + wr; same reach)
    h, pitch = OVER[slot // 4]
    rot = rot_tool(hd, pitch, yaw)
    seeds = [np.array([hd, j2, j3, j4, hd - yaw]) for j2 in (-0.3, 0.3, 0.8) for j3 in (0.7, -0.5) for j4 in (1.0, 1.9)]
    q, e_p, e_a = ik.solve(np.array([sx, sy, sz + h]) - rot @ D_NOM, rot, seeds)
    assert e_p < 2e-3 and e_a < math.radians(2.0), f"IK over slot {slot}"
    return q


def q_lift_pick(side):
    pb = np.array([BOX_CX, side * START_GAP / 2, bc.BOX_H]) - base
    hd = math.atan2(pb[1], pb[0])
    rot = pin.utils.rotate("z", hd + math.remainder(math.pi / 2 - hd, math.pi)) @ bc.DOWN
    q, *_ = ik.solve(pb + [0, 0, bc.GRASP_Z + 0.05], rot, [np.array([hd, -0.2, 0.7, 1.0, s]) for s in (0.0, 0.5, -0.5)])
    return q


def q_above_cell(cx, cy):
    cell = np.array([cx, cy, bc.BOX_H]) - base
    rot, hd, yaw = bc.place_rot(cell)
    q, *_ = ik.solve(cell + [0, 0, 0.05] - rot @ D_NOM, rot, bc.place_seeds(hd, yaw))
    return q


def compare(name, qa, qb, carry, hold):
    t_ik = t_move(qa, carry) + hold + t_move(carry, qb)
    c_ik = min(puck_clearance(line(qa, carry)), puck_clearance(line(carry, qb)))
    t_ln = t_move(qa, qb)
    ln_ok = not arm_hits(line(qa, qb))
    c_ln = puck_clearance(line(qa, qb))
    t_cm, qs = cumo(qa, qb)
    c_cm = puck_clearance(qs) if qs is not None else np.nan
    return dict(name=name, t_ik=t_ik, c_ik=c_ik, t_ln=t_ln, ln_ok=ln_ok, c_ln=c_ln, t_cm=t_cm, c_cm=c_cm)


rows = []
q_over = [q_over_slot(s) for s in range(12)]
for s in range(12):
    side = 1 if SLOTS[s][1] >= 0 else -1
    for ps in (-1, 1):
        rows.append(("load", compare(f"pick side {ps:+d} -> slot {s}", q_lift_pick(ps), q_over[s], CARRY[side], 0.0)))
for s in range(12):
    side = 1 if SLOTS[s][1] >= 0 else -1
    for c, (cx, cy) in enumerate(CELLS):
        rows.append(("unload", compare(f"slot {s} -> cell {c}", q_over[s], q_above_cell(cx, cy), CARRY[side], CARRY_HOLD)))

CLEAR = 0.005  # puck to basket margin counted as safe
for kind in ("load", "unload"):
    rs = [r for k, r in rows if k == kind]
    ik_t = np.array([r["t_ik"] for r in rs])
    cm_ok = [r for r in rs if r["t_cm"] is not None]
    cm_safe = [r for r in cm_ok if r["c_cm"] >= CLEAR]
    ln_safe = [r for r in rs if r["ln_ok"] and r["c_ln"] >= CLEAR]
    print(f"== {kind}: {len(rs)} moves, vel_scale {args.vel_scale}")
    print(f"  IK via carry   : {ik_t.mean():.2f} s mean ({ik_t.min():.2f}..{ik_t.max():.2f}), puck-basket min {1000 * min(r['c_ik'] for r in rs):.0f} mm")
    if cm_ok:
        t = np.array([r["t_cm"] for r in cm_ok])
        sv = np.array([r["t_ik"] - r["t_cm"] for r in cm_ok])
        print(f"  cuMotion direct: {len(cm_ok)}/{len(rs)} planned, {t.mean():.2f} s mean ({t.min():.2f}..{t.max():.2f}), "
              f"saves {sv.mean():.2f} s mean ({sv.min():.2f}..{sv.max():.2f}); puck clears the basket by >= 5 mm in {len(cm_safe)}/{len(cm_ok)}")
    t = np.array([r["t_ln"] for r in rs])
    print(f"  joint line     : {t.mean():.2f} s mean (lower bound); arm free and puck >= 5 mm in {len(ln_safe)}/{len(rs)}")
    bad = [r for r in cm_ok if r["c_cm"] < CLEAR]
    for r in bad[:6]:
        print(f"    cuMotion puck {1000 * r['c_cm']:+.0f} mm: {r['name']}")
