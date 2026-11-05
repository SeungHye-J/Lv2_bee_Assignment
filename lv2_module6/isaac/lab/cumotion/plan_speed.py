import math, sys, time
from pathlib import Path
import cumotion as cm
import numpy as np
import pinocchio as pin
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import bee_common as bc
HERE = Path(__file__).resolve().parent
rd = cm.load_robot_from_file(str(HERE / "beebot.xrdf"), str(bc.URDF_PATH))
ik = bc.ArmIK(); base = ik.base_in_root()
world = cm.create_world()
box = cm.create_obstacle(cm.Obstacle.Type.CUBOID)
box.set_attribute(cm.Obstacle.Attribute.SIDE_LENGTHS, cm.Obstacle.AttributeValue(np.array([0.205, 0.205, 0.07])))
world.add_obstacle(box, cm.Pose3.from_translation((np.array([0.1825, 0.0, 0.035]) - base).reshape(3, 1)))
opt = cm.create_trajectory_optimizer(cm.create_default_trajectory_optimizer_config(rd, "end_effector_link", world.add_world_view()))
T = cm.TrajectoryOptimizer
stow = np.array([0.0, -1.9024, 1.344, 0.9023, 0.0])
for k, dy in enumerate(np.radians([0, 10, 20, 30, 40])):
    pb = np.array([0.1825, -0.04, bc.BOX_H]) - base
    hd = math.atan2(pb[1], pb[0]); want = hd + math.remainder(math.pi / 2 - hd, math.pi) + dy
    rot = cm.Rotation3.from_matrix(pin.utils.rotate("z", want) @ bc.DOWN)
    above, down = pb + [0, 0, bc.GRASP_Z + 0.05], pb + [0, 0, bc.GRASP_Z]
    t0 = time.time()
    r1 = opt.plan_to_task_space_target(stow.reshape(5, 1), T.TaskSpaceTarget(T.TranslationConstraint.target(above.reshape(3, 1)), T.OrientationConstraint.terminal_target(rot)))
    t1 = time.time()
    qa = r1.trajectory().eval(r1.trajectory().domain().upper)
    r2 = opt.plan_to_task_space_target(qa, T.TaskSpaceTarget(T.TranslationConstraint.linear_path_constraint(down.reshape(3, 1), float(sys.argv[1]) if len(sys.argv) > 1 else None), T.OrientationConstraint.terminal_and_path_target(rot)))
    t2 = time.time()
    tr = r2.trajectory(); d = tr.domain()
    dev = max(np.linalg.norm(np.cross(ik.fk(tr.eval(s).ravel())[0] - above, [0, 0, 1])) for s in np.linspace(d.lower, d.upper, 20))
    qd, *_ = ik.solve(down, pin.utils.rotate("z", want) @ bc.DOWN, [np.asarray(qa).ravel()])
    qa_ = np.asarray(qa).ravel()
    dev_ik = max(np.linalg.norm(np.cross(ik.fk(qa_ + bc.move_s(x, 1.0) * (qd - qa_))[0] - above, [0, 0, 1])) for x in np.linspace(0, 1, 20))
    print(f"[ik joint interp off the line {1000*dev_ik:.2f} mm] ", end="")
    print(f"yaw +{math.degrees(dy):.0f}: transit {r1.status()} {1000*(t1-t0):.0f} ms {r1.trajectory().domain().upper:.2f} s | descent {r2.status()} {1000*(t2-t1):.0f} ms {d.upper:.2f} s, off the vertical line max {1000*dev:.2f} mm", flush=True)
