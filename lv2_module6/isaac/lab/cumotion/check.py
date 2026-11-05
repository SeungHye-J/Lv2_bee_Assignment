"""B0 check: cuMotion FK = pinocchio FK, and one planned move stow -> above the start-box puck around the box."""
import math, sys, time
from pathlib import Path
import cumotion as cm
import numpy as np
import pinocchio as pin
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import bee_common as bc

HERE = Path(__file__).resolve().parent
rd = cm.load_robot_from_file(str(HERE / "beebot.xrdf"), str(bc.URDF_PATH))
kin = rd.kinematics()
ik = bc.ArmIK()
rng = np.random.default_rng(0)
err = []
for _ in range(200):
    q = rng.uniform(ik.lo, ik.hi)
    p_pin, r_pin = ik.fk(q)
    pose = kin.pose(q.reshape(5, 1), "end_effector_link")
    err.append((np.linalg.norm(np.asarray(pose.translation).ravel() - p_pin), cm.Rotation3.distance(pose.rotation, cm.Rotation3.from_matrix(r_pin))))
err = np.array(err)
print(f"[fk] base frame {kin.base_frame_name()}: max position diff {1000 * err[:, 0].max():.3f} mm, max angle diff {math.degrees(err[:, 1].max()):.3f} deg")
print("[lim] vel", [round(kin.cspace_coord_velocity_limit(i), 2) for i in range(5)], "acc", [kin.cspace_coord_acceleration_limit(i) for i in range(5)])

world = cm.create_world()
box = cm.create_obstacle(cm.Obstacle.Type.CUBOID)
box.set_attribute(cm.Obstacle.Attribute.SIDE_LENGTHS, cm.Obstacle.AttributeValue(np.array([0.205, 0.205, 0.07])) if hasattr(cm.Obstacle, "AttributeValue") else None)
base = ik.base_in_root()
world.add_obstacle(box, cm.Pose3.from_translation((np.array([0.1825, 0.0, 0.035]) - base).reshape(3, 1)))
view = world.add_world_view()
cfg = cm.create_default_trajectory_optimizer_config(rd, "end_effector_link", view)
opt = cm.create_trajectory_optimizer(cfg)
stow = np.array([0.0, -1.9024, 1.344, 0.9023, 0.0])
pb = np.array([0.1825, -0.04, bc.BOX_H]) - base
hd = math.atan2(pb[1], pb[0]); want = hd + math.remainder(math.pi / 2 - hd, math.pi)
rot = pin.utils.rotate("z", want) @ bc.DOWN
target = pb + [0, 0, bc.GRASP_Z + 0.05]
T = cm.TrajectoryOptimizer
tgt = T.TaskSpaceTarget(T.TranslationConstraint.target(target.reshape(3, 1)), T.OrientationConstraint.terminal_target(cm.Rotation3.from_matrix(rot)))
t0 = time.time()
res = opt.plan_to_task_space_target(stow.reshape(5, 1), tgt)
dt = time.time() - t0
print(f"[plan] status {res.status()} in {1000 * dt:.0f} ms")
tr = res.trajectory()
d = tr.domain()
print(f"[plan] duration {d.upper - d.lower:.2f} s (pinocchio trapezoid at 1.0 x limits: {bc.trapezoid_duration(np.abs(ik.solve(target, rot, [stow])[0] - stow).max(), 1.0):.2f} s)")
qe = tr.eval(d.upper).ravel()
p_end, _ = ik.fk(qe)
print(f"[plan] end position error {1000 * np.linalg.norm(p_end - target):.2f} mm, q_end deg {np.degrees(qe).round(1).tolist()}")
