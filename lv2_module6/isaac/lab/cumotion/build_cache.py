"""B0: cuMotion base trajectories for the residual envs, planned offline once and stored as tables.

    ~/pa-course/IsaacLab-6.1/.venv/bin/python cumotion/build_cache.py --task pick    (~7 min, 364 plans)
    ~/pa-course/IsaacLab-6.1/.venv/bin/python cumotion/build_cache.py --task place   (6 plans)

Same targets as the pinocchio IK plans in pick_env / place_env (same yaw bins, cells, heights); the start box /
dice box is a cuboid obstacle. Each move is sampled at N_S points uniform in cuMotion's own time, so the env can
replay its velocity profile, stretched to the env's vel_scale x team limits (3 rad/s, 5 rad/s^2).
Output: cumotion/cache_<task>.npz.
"""

import argparse
import math
import sys
import time
from pathlib import Path

import cumotion as cm
import numpy as np
import pinocchio as pin

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import bee_common as bc  # noqa: E402

HERE = Path(__file__).resolve().parent
N_S = 41
BOX_CX = 0.08 + bc.BOX_HALF
# same targets as pick_env / place_env (not imported: they need the Isaac app)
START_GAP = 0.080
YAW_BINS = np.radians(np.arange(-45, 46, 1))
CELLS = [(BOX_CX + bc.CELL_PITCH * (r - 1), bc.CELL_PITCH * (c - 1)) for r in (0, 1) for c in range(3)]
STOW = np.array([0.0, -1.9024, 1.344, 0.9023, 0.0])
T = cm.TrajectoryOptimizer


class Planner:
    def __init__(self):
        self.rd = cm.load_robot_from_file(str(HERE / "beebot.xrdf"), str(bc.URDF_PATH))
        self.ik = bc.ArmIK()
        self.base = self.ik.base_in_root()
        world = cm.create_world()
        box = cm.create_obstacle(cm.Obstacle.Type.CUBOID)
        box.set_attribute(cm.Obstacle.Attribute.SIDE_LENGTHS, cm.Obstacle.AttributeValue(np.array([2 * bc.BOX_HALF, 2 * bc.BOX_HALF, bc.BOX_H])))
        world.add_obstacle(box, cm.Pose3.from_translation((np.array([BOX_CX, 0.0, bc.BOX_H / 2]) - self.base).reshape(3, 1)))
        self.opt = cm.create_trajectory_optimizer(cm.create_default_trajectory_optimizer_config(self.rd, "end_effector_link", world.add_world_view()))

    def plan(self, q0, pos_b, rot, linear=False):
        """-> (samples (N_S, 5), duration [s] at the URDF limits, max |qdot|, max |qddot|)."""
        tc = T.TranslationConstraint.linear_path_constraint(pos_b.reshape(3, 1)) if linear else T.TranslationConstraint.target(pos_b.reshape(3, 1))
        oc = T.OrientationConstraint.terminal_and_path_target(cm.Rotation3.from_matrix(rot)) if linear else \
            T.OrientationConstraint.terminal_target(cm.Rotation3.from_matrix(rot))
        res = self.opt.plan_to_task_space_target(np.asarray(q0, dtype=np.float64).reshape(5, 1), T.TaskSpaceTarget(tc, oc))
        assert res.status() == T.Results.Status.SUCCESS, f"cuMotion failed: {res.status()} to {pos_b}"
        tr = res.trajectory()
        d = tr.domain()
        ts = np.linspace(d.lower, d.upper, N_S)
        q = np.array([tr.eval(t).ravel() for t in ts])
        fine = np.linspace(d.lower, d.upper, 200)
        v = max(np.abs(tr.eval(t, 1)).max() for t in fine)
        a = max(np.abs(tr.eval(t, 2)).max() for t in fine)
        return q, d.upper - d.lower, v, a


def build_pick(pl):
    out = {k: [] for k in ("tr_q", "tr_dur", "tr_v", "tr_a", "dn_q", "dn_dur", "dn_v", "dn_a")}
    for side in (-1.0, 1.0):
        pb = np.array([BOX_CX, side * START_GAP / 2, bc.BOX_H]) - pl.base
        hd = math.atan2(pb[1], pb[0])
        want = hd + math.remainder(math.pi / 2 - hd, math.pi)
        for k, dy in enumerate(YAW_BINS):
            rot = pin.utils.rotate("z", want + dy) @ bc.DOWN
            q, dur, v, a = pl.plan(STOW, pb + [0, 0, bc.GRASP_Z + 0.05], rot)
            out["tr_q"].append(q), out["tr_dur"].append(dur), out["tr_v"].append(v), out["tr_a"].append(a)
            q2, dur, v, a = pl.plan(q[-1], pb + [0, 0, bc.GRASP_Z], rot, linear=True)
            out["dn_q"].append(q2), out["dn_dur"].append(dur), out["dn_v"].append(v), out["dn_a"].append(a)
            if k % 10 == 0:
                print(f"[pick] side {side:+.0f} yaw {math.degrees(dy):+.0f} deg done", flush=True)
    shape = (2, len(YAW_BINS))
    return {k: np.array(v).reshape(*shape, *np.array(v).shape[1:]) for k, v in out.items()}


def build_place(pl):
    out = {k: [] for k in ("dn_q", "dn_dur", "dn_v", "dn_a", "q_above")}
    d_nom = np.array([0.0, 0.0, bc.GRASP_Z])
    for cx, cy in CELLS:
        cell_b = np.array([cx, cy, bc.BOX_H]) - pl.base
        rot, hd, yaw = bc.place_rot(cell_b)
        q_above, *_ = pl.ik.solve(cell_b + [0, 0, 0.05] - rot @ d_nom, rot, bc.place_seeds(hd, yaw))
        q, dur, v, a = pl.plan(q_above, cell_b - rot @ d_nom, rot, linear=True)
        out["dn_q"].append(q), out["dn_dur"].append(dur), out["dn_v"].append(v), out["dn_a"].append(a), out["q_above"].append(q_above)
    return {k: np.array(v) for k, v in out.items()}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--task", choices=["pick", "place"], required=True)
    args = ap.parse_args()
    t0 = time.time()
    pl = Planner()
    data = build_pick(pl) if args.task == "pick" else build_place(pl)
    out = HERE / f"cache_{args.task}.npz"
    np.savez_compressed(out, **data)
    print(f"wrote {out} in {time.time() - t0:.0f} s: " + ", ".join(f"{k} {v.shape}" for k, v in data.items()))


if __name__ == "__main__":
    main()
