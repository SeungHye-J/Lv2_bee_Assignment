"""Clear one box into the basket through the coordinator (/task/execute_plan) — the robot is assumed already docked at the box
(approach done). Unlike pick_loop.py (calls /skills/* directly), every step goes through the coordinator, so its fallbacks
apply: pick -> pick_far when the arm cannot reach straight down, approach when a puck is out of reach, lying puck ->
stand -> stand_pick, load "next" picks the slot and keeps the basket ledger. Done once as many pucks are loaded as the
first inspect saw on the box (or the box is empty / --max tries).

Per puck: plan [stow, inspect box] (only for the first puck, after a failure, or when no puck is known — otherwise straight
from the load to the next puck perception tracks or the last inspect listed) -> standing pucks first, then nearest the box centre
-> plan [pick id, load next]
(lying: [stand id] -> 1 s -> still lying = stand again, else [pick <stood puck>, load next] — stand_pick only if the stood puck is not seen). A remembered id the coordinator no longer knows (*_no_target) -> inspect and try again. Stops when the
box is empty, the basket is full, or --max pucks are done. A puck that failed is skipped on the next rounds (by position).
Logs each puck to logs/pick_coord_<ts>.csv (which skills actually ran, e.g. pick -> pick_far) and records one bag for the
whole run (manipulator-record.sh, --bag cam|nocam|off) -> logs/pick_coord_<ts>_bag/.

Needs the coordinator in AUTO mode (isaac_sim.sh default; bringup operation:=AUTO on the robot).

    python3 pick_coord.py use_sim=true                        # box centre 0.22 m ahead of base_footprint
    python3 pick_coord.py --box-x 0.25 --box-y 0.03 --max 3
Policy on/off is not set here: models/skills/<skill>/meta.yaml (use:) + colcon build.
"""

import argparse
import csv
import math
import os
import re
import signal
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from perception.msg import TrackedObjectArray
from plan.action import ExecutePlan
from plan.msg import SubGoal
from action_msgs.msg import GoalStatus, GoalStatusArray
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from rosbag2_interfaces.srv import Pause
from tf2_ros import Buffer, TransformListener

# launch-style key=value arguments (use_sim=true) → --key value
sys.argv[1:] = [t for a in sys.argv[1:] for t in (["--" + a.split("=", 1)[0], a.split("=", 1)[1]]
                                                  if re.fullmatch(r"[a-z_]+=.*", a) else [a])]
parser = argparse.ArgumentParser()
parser.add_argument("--box-x", type=float, default=0.22, help="box centre ahead of base_footprint [m]")
parser.add_argument("--box-y", type=float, default=0.0, help="box centre left of base_footprint [m]")
parser.add_argument("--support", type=float, default=0.07, help="box top height [m]")
parser.add_argument("--puck-h", type=float, default=0.06, help="standing puck height [m]")
parser.add_argument("--box-r", type=float, default=0.15, help="pucks within this of the box centre count as on the box [m]")
parser.add_argument("--stand-tries", type=int, default=3, help="stand attempts per lying puck (mission retries 2 + 1)")
parser.add_argument("--stand-check-s", type=float, default=1.0, help="wait after stand before checking it stood [s] (plan.yaml stand_check_s)")
parser.add_argument("--max", type=int, default=12, help="stop after this many pucks")
parser.add_argument("--put-back-clear", type=float, default=0.045,
                    help="failed-load puck: centre at least this far from other pucks' centres [m] (45 mm zone + puck half)")
parser.add_argument("--put-back-dx", type=float, default=0.04,
                    help="failed-load puck: box centre first, then this much nearer the robot, then sideways [m]")
parser.add_argument("--load-retries", type=int, default=1,
                    help="load failed with the puck in hand -> place it back on the box, look, pick again (this many times per puck)")
parser.add_argument("--bag", choices=["cam", "nocam", "off"], default="cam",
                    help="one manipulator-record.sh bag for the whole run -> logs/pick_coord_<ts>_bag/; nocam = no images")
parser.add_argument("--use_sim", type=lambda v: v.strip().lower() in ("1", "true", "yes"), default=False,
                    help="true: Isaac Sim beebot (isaac/standalone/isaac_sim.sh) — node on sim time (/clock)")
args = parser.parse_args()

FIELDS = ["n", "id", "tag", "class", "color", "dist_m", "ok", "reason", "skills", "seconds"]
out = Path(__file__).resolve().parent / "logs" / f"pick_coord_{time.strftime('%Y%m%d_%H%M%S')}.csv"
out.parent.mkdir(exist_ok=True)

os.environ.setdefault("FASTDDS_DEFAULT_PROFILES_FILE", str(Path(__file__).resolve().parents[2] / "ros2_ws" / "fastdds_local_only.xml"))
rclpy.init()
if args.use_sim:
    from rclpy.parameter import Parameter
    node = rclpy.create_node("pick_coord", parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)])
else:
    node = rclpy.create_node("pick_coord")
tf_buf = Buffer()
TransformListener(tf_buf, node)
state = {"objects": None}
seen_t = {}  # id -> wall time last seen
latest = {}  # id -> newest track seen — the remembered list is refreshed from it (11-15: snapshot taken before the
              #   posture was known kept a lying puck as 'puck' → pick on a lying puck → grasp_empty twice)


boxes_seen = []  # (x, y, size_x, size_y) of start_box detections in map — the put-back spot must lie on the real box


def on_objects(m):
    state.update(objects=m)
    for o in m.objects:
        if o.class_name == "start_box":
            boxes_seen.append((o.pose.pose.position.x, o.pose.pose.position.y, o.size.x, o.size.y))
            del boxes_seen[:-40]
        latest[o.id] = o
        seen_t[o.id] = time.time()


node.create_subscription(TrackedObjectArray, "/tracked_objects", on_objects, 5)
execute = ActionClient(node, ExecutePlan, "/task/execute_plan")
plan_id = 0
# which skills actually ran (the plan feedback only names the plan step — pick stays "pick" when the coordinator falls back
# to pick_far, inspect/pick may insert an approach): every new executing goal on /skills/<s>/_action/status
ran = {"on": False, "skills": [], "seen": set()}


def on_status(name):
    def cb(m):
        for st in m.status_list:
            gid = bytes(st.goal_info.goal_id.uuid)
            if st.status == GoalStatus.STATUS_EXECUTING and gid not in ran["seen"]:
                ran["seen"].add(gid)
                if ran["on"]:
                    ran["skills"].append(name)
                    print(f"    {name}")
    return cb


status_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
for s in ("navigate", "approach", "pose", "inspect", "pick", "pick_far", "stand", "stand_pick", "load"):
    node.create_subscription(GoalStatusArray, f"/skills/{s}/_action/status", on_status(s), status_qos)


def spin(sec):
    end = time.time() + sec
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.02)


def to_map(x, y):
    """base_footprint (x, y) -> map (x, y), None if no TF"""
    try:
        t = tf_buf.lookup_transform("map", "base_footprint", Time())
    except Exception:
        return None
    p, q = t.transform.translation, t.transform.rotation
    yaw = 2.0 * math.atan2(q.z, q.w)
    return p.x + x * math.cos(yaw) - y * math.sin(yaw), p.y + x * math.sin(yaw) + y * math.cos(yaw)


def run_plan(goals, timeout=600.0):
    """ExecutePlan -> (ok, reason, [skills that ran, in order])"""
    global plan_id
    plan_id += 1
    ran.update(on=True, skills=[])
    try:
        return _run_plan(goals, timeout)
    finally:
        spin(0.2)  # last status messages
        ran["on"] = False


def _run_plan(goals, timeout):
    fut = execute.send_goal_async(ExecutePlan.Goal(plan_id=plan_id, goals=goals))
    rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
    gh = fut.result()
    if gh is None or not gh.accepted:
        return False, "rejected", ran["skills"]
    res = gh.get_result_async()
    rclpy.spin_until_future_complete(node, res, timeout_sec=timeout)
    if not res.done():
        gh.cancel_goal_async()
        return False, "timeout", ran["skills"]
    r = res.result().result
    return r.success, r.reason, ran["skills"]


def detail(o, key):
    return next((kv.value for kv in o.details if kv.key == key), "")


def sg(skill, target="", relation=""):
    return SubGoal(skill=skill, target=target, relation=relation)


def stand_up(o):
    """stand the lying puck o, re-stand while perception still sees it lying (mission_manager stand_up, retries 2)"""
    x, y, skills, color = o.pose.pose.position.x, o.pose.pose.position.y, [], o.color
    for k in range(args.stand_tries):
        ok, why, ran_ = run_plan([sg("stand", str(o.id))])
        skills += ran_
        if not ok:
            return False, "stand_failed " + why, skills, None
        # judge from the look pose, not from where the hand lifted away: 11-15 bag — from the lift pose perception called a
        #   stood puck 'lying' three times (and a stand was sent to a standing puck); from stow/look the same track was 'standing'
        run_plan([sg("pose", "look")])
        spin(args.stand_check_s)
        # wait (≤ 3 s more) until the track near the spot has a posture: right after the look the puck is often a bare 'puck'
        #   (11-15: taken as 'stood' → pick on a lying puck → no_grasp; 6 s later perception said lying)
        o = None
        for _ in range(16):
            objs = state["objects"].objects if state["objects"] else []
            # 10 cm: a failed stand can throw the puck 6~7 cm (11-15: it landed 6.6 cm away, the old 6 cm search saw nothing,
            #   called it 'stood' and stand_pick closed on air — the grasp check passed at 0.0048 m · 0.09 A, same as a held puck)
            # same colour only: 11-15 a stand knocked the red neighbour over and the retry stood the red one instead of the green
            near = [p for p in objs if p.class_name.startswith("puck") and (not color or not p.color or p.color == color)
                    and math.hypot(p.pose.pose.position.x - x, p.pose.pose.position.y - y) <= 0.10]
            o = min(near, key=lambda p: math.hypot(p.pose.pose.position.x - x, p.pose.pose.position.y - y), default=None)
            if o is not None and (o.class_name == "puck_lying" or detail(o, "posture") == "standing"):
                break
            spin(0.2)
        if o is None:  # nothing seen from the look pose: do not grab blind — inspect the box again
            print("  no puck near the stand spot from the look pose — inspect again")
            return False, "stand_lost", skills, None
        if o.class_name != "puck_lying" and detail(o, "posture") != "standing":
            print("  posture of the stood puck unknown from the look pose — inspect again")
            return False, "stand_lost", skills, None
        if o.class_name != "puck_lying":
            return True, "", skills, o
        # still lying — but if another puck on the box stands now (e.g. a neighbour the stand set upright), pick that first
        up = [p for p in on_box_now() if p.class_name == "puck" and detail(p, "posture") == "standing"]  # posture known only
        #   (11-15: a lying red puck seen for a moment as 'puck' without posture deferred the stand four times)
        if up:
            print(f"  still lying — a standing puck is on the box: pick that first")
            return False, "stand_defer", skills, None
        print(f"  still lying — stand again ({k + 2}/{args.stand_tries})")
        x, y = o.pose.pose.position.x, o.pose.pose.position.y
    return False, "stand_still_lying", skills, None


if not execute.wait_for_server(timeout_sec=15.0):
    raise SystemExit("FAIL: /task/execute_plan not up — is the coordinator running?")
box = None
t0 = time.time()
while box is None and time.time() - t0 < 15.0:  # TF listener only fills while the node spins
    spin(0.5)
    box = to_map(args.box_x, args.box_y)
if box is None:
    raise SystemExit("FAIL: no TF map -> base_footprint after 15 s (coordinator plans are in map)")
BAG_NODE = "pick_coord_rec"
bag = None
if args.bag != "off":
    bag_out = str(out.with_suffix("")) + "_bag"
    bag = subprocess.Popen([str(Path(__file__).resolve().parent / "manipulator-record.sh")] + (["nocam"] if args.bag == "nocam" else []),
                           env=dict(os.environ, BAG_OUT=bag_out, BAG_NODE=BAG_NODE, BAG_SIM="1" if args.use_sim else ""),
                           stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    if node.create_client(Pause, f"/{BAG_NODE}/pause").wait_for_service(timeout_sec=15.0):
        spin(1.0)  # recorder subscribes to the topics after its services are up
        print(f"bag: recording -> {bag_out}")
    else:
        print("bag: recorder did not come up — not recorded")


def stop_bag():
    if bag and bag.poll() is None:
        bag.send_signal(signal.SIGINT)  # bash exec'd ros2 bag -> same pid; SIGINT closes the MCAP cleanly
        try:
            bag.wait(timeout=15.0)
        except subprocess.TimeoutExpired:
            bag.kill()


inspect_target = f"{box[0]:.4f},{box[1]:.4f},{args.support + args.puck_h:.4f}"
print(f"box centre in map: {box[0]:.3f}, {box[1]:.3f}")

failed = []  # map (x, y) of pucks that failed — skipped afterwards
put_back = []  # map (x, y) where a puck was put back after a failed load — counted per spot (--load-retries)


def on_box_now():
    """pucks perception tracks on the box right now, minus failed spots"""
    objs = state["objects"].objects if state["objects"] else []
    return [o for o in objs if o.class_name.startswith("puck")
            and math.hypot(o.pose.pose.position.x - box[0], o.pose.pose.position.y - box[1]) <= args.box_r
            and all(math.hypot(o.pose.pose.position.x - fx, o.pose.pose.position.y - fy) > 0.04 for fx, fy in failed)]


def put_back_spot(held_id, held_xy):
    """free spot on the box near the robot for a puck whose load failed — close to the body, easy to pick (11-15: putting it
    back where it was, among the other pucks, made the next pick fail). Box near edge row, centre first, ≥ 7 cm from other pucks"""
    # every puck seen on the box this run (newest position per id), not only what the camera sees now — during a load the
    #   hand camera looks at the basket, so the box looked empty and the puck went onto a lying neighbour (11-15)
    others = [(p.pose.pose.position.x, p.pose.pose.position.y) for p in latest.values()
              if p.class_name.startswith("puck") and p.id != held_id and p.pose.pose.position.z < 0.15
              # old ids of the puck now in the hand (11-15: ids 61/65 of the held green puck blocked the centre spots)
              and math.hypot(p.pose.pose.position.x - held_xy[0], p.pose.pose.position.y - held_xy[1]) > 0.06
              and time.time() - seen_t.get(p.id, 0) < 60.0  # old ids of a puck that was picked or moved
              and math.hypot(p.pose.pose.position.x - box[0], p.pose.pose.position.y - box[1]) <= args.box_r]
    # the real box (median of start_box detections near the assumed box): 11-15 the assumed centre (0.22, 0) was 6 cm off in y,
    #   the +8 cm spot was 4 cm past the box edge and the puck fell off. Spots must be ≥ 3 cm inside the measured box
    near = [b for b in boxes_seen if math.hypot(b[0] - box[0], b[1] - box[1]) < 0.12 and 0.15 < min(b[2], b[3]) and max(b[2], b[3]) < 0.30]
    bx, by, sx, sy = ([sorted(v)[len(v) // 2] for v in zip(*near)] if near else (box[0], box[1], 0.20, 0.20))
    # spot row: put_back_dx nearer the robot than the box centre, along robot → box (sideways steps across it)
    r = to_map(0.0, 0.0) or (box[0] - args.box_x, box[1] - args.box_y)
    ux, uy = bx - r[0], by - r[1]
    n = math.hypot(ux, uy) or 1.0
    ux, uy = ux / n, uy / n
    # centre of the box first, then around it (toward the robot first) — 11-15 user: centre first, avoid other pucks
    for dx, dy in ((0.0, 0.0), (args.put_back_dx, 0.0), (0.0, -0.04), (0.0, 0.04), (args.put_back_dx, -0.04),
                   (args.put_back_dx, 0.04), (-0.04, 0.0), (0.0, -0.08), (0.0, 0.08)):
        m = (bx - ux * dx - uy * dy, by - uy * dx + ux * dy)
        inside = abs(m[0] - bx) <= sx / 2 - 0.03 and abs(m[1] - by) <= sy / 2 - 0.03  # box roughly square to map axes
        # no place inside a 45 mm-diameter zone around another puck's centre (11-15 user) — the placed standing puck
        #   (30 mm square, half diagonal 21 mm) must stay out of it: centres ≥ 22.5 + 21 ≈ 45 mm apart (--put-back-clear)
        if inside and all(math.hypot(m[0] - x, m[1] - y) >= args.put_back_clear for x, y in others):
            return m
    return None


def far_from(o, p, r=0.04):
    return math.hypot(o.pose.pose.position.x - p.pose.pose.position.x, o.pose.pose.position.y - p.pose.pose.position.y) > r


with open(out, "w", newline="") as fh:
    w = csv.DictWriter(fh, FIELDS)
    w.writeheader()
    n = loaded = 0
    goal_n = None  # pucks the first inspect saw on the box — done once that many are loaded
    # like mission_manager locate: after a load go straight to the next puck the stack still knows (perception now, else
    # the list from the last inspect — the coordinator remembers lost tracks remember_s) — stow + inspect only when none is
    # known, after a failure, or when the coordinator no longer knows the id (*_no_target). Saves ~5 s per puck (11-15)
    known, need_inspect, unknown_looked = [], True, False
    try:
        while n < args.max and (goal_n is None or loaded < goal_n):
            if need_inspect:
                ok, why, _ = run_plan([sg("pose", "stow"), sg("inspect", inspect_target, "box")])
                if not ok:
                    print(f"inspect FAIL {why}")
                    if why in ("manual_mode", "manual_teach_mode", "explore_mode"):
                        raise SystemExit("coordinator is not in AUTO mode (bringup operation:=AUTO)")
                spin(0.5)
                on_box = known = on_box_now()
            else:
                now = on_box_now()
                known = [latest.get(k.id, k) for k in known]  # newest posture / position of each remembered puck
                on_box = now or [k for k in known if all(math.hypot(k.pose.pose.position.x - fx, k.pose.pose.position.y - fy) > 0.04
                                                         for fx, fy in failed)]
                if any(o.class_name == "puck" and not detail(o, "posture") for o in on_box):
                    print("  posture of a remembered puck unknown — inspect first")
                    need_inspect = True
                    continue
                print(f"next puck without inspect ({'perception' if now else 'remembered from the last inspect'})")
            if not on_box:
                if not need_inspect:
                    need_inspect = True
                    continue
                print("box empty (or only failed pucks left)")
                break
            if goal_n is None:
                goal_n = len(on_box)
                print(f"{goal_n} pucks on the box — stop after {goal_n} loaded")
            # standing pucks first (a stand can knock its neighbours), then nearest the box centre
            # posture known standing first, then unknown, then lying (11-15: 'puck' without posture was taken as standing)
            o = min(on_box, key=lambda o: (0 if detail(o, "posture") == "standing" else 2 if o.class_name == "puck_lying" else 1,
                                           math.hypot(o.pose.pose.position.x - box[0], o.pose.pose.position.y - box[1])))
            if o.class_name == "puck" and not detail(o, "posture") and not unknown_looked:
                print(f"  puck {o.id}: posture unknown — inspect again before picking")
                unknown_looked = True
                need_inspect = True
                continue
            unknown_looked = False
            n += 1
            lying = o.class_name == "puck_lying"
            r = to_map(0.0, 0.0)
            d = math.hypot(o.pose.pose.position.x - r[0], o.pose.pose.position.y - r[1]) if r else float("nan")
            print(f"\n[{n}] puck {o.id} tag {detail(o, 'tag') or '?'} {o.color} {o.class_name} {d:.3f} m from the robot")
            t = time.time()
            if lying:  # mission_manager stand_up: stand -> wait -> still lying = stand again, else pick the stood puck
                ok, why, skills, up = stand_up(o)
                if ok:  # 11-15: plain pick of the stood puck seen from the look pose (perception · hand cam · policy)
                    ok, why, more = run_plan([sg("pick", str(up.id)), sg("load", "next")])
                    skills += more
                elif why == "stand_lost":  # puck not seen after the stand: inspect and choose again (not counted as failed)
                    n -= 1
                    need_inspect = True
                    continue
                elif why == "stand_defer":  # a standing puck appeared: choose again (standing first) from what is seen now
                    n -= 1
                    continue
            else:
                ok, why, skills = run_plan([sg("pick", str(o.id)), sg("load", "next")])
            # load failed with the puck still in hand (not load_released_*: then it is in the slot): put it back on the box where
            #   it was picked, look again and choose again (11-15: a jammed load was pushed in / failed twice with the puck held)
            if not ok and skills and skills[-1] == "load" and not why.startswith("load_released"):
                px, py = o.pose.pose.position.x, o.pose.pose.position.y
                k = sum(1 for bx, by in put_back if math.hypot(bx - px, by - py) < 0.04)
                if k < args.load_retries:
                    spot = put_back_spot(o.id, (px, py)) or (px, py)
                    print(f"  load failed ({why}) — put the puck on the box near the robot ({spot[0]:.3f}, {spot[1]:.3f}), look, pick again")
                    pok, pwhy, more = run_plan([sg("place", f"{spot[0]:.4f},{spot[1]:.4f},{args.support:.4f}")])  # next: stow + inspect
                    skills += more
                    print(f"  put back: {'ok' if pok else 'FAIL ' + pwhy}")
                    if pok:
                        put_back.append((px, py))
                        put_back.append(spot)  # count the retry against the new spot too
                        w.writerow(dict(n=n, id=o.id, tag=detail(o, "tag"), **{"class": o.class_name}, color=o.color,
                                        dist_m=round(d, 3), ok=False, reason=why + " -> put back", skills=" ".join(skills),
                                        seconds=round(time.time() - t, 1)))
                        fh.flush()
                        n -= 1
                        need_inspect = True
                        continue
            # the coordinator forgot the remembered id: rejected at plan check ("invalid_plan:target N not tracked" — 11-15: the
            #   look after a stand re-tracked the far puck under a new id) or failed in the step (*_no_target)
            lost = why.endswith("_no_target") or "not tracked" in why
            if not ok and not need_inspect and lost and len(skills) <= 1:
                print(f"  {why}: the coordinator lost this id — inspect and try again")
                n -= 1
                need_inspect = True
                continue
            print(f"  {'ok' if ok else 'FAIL ' + why}  ({' -> '.join(skills)}, {time.time() - t:.0f} s)")
            known = [k for k in known if far_from(k, o)]  # this puck is done (loaded or failed)
            need_inspect = not ok  # after a failure the hand may hold the puck and the box changed: stow + inspect
            w.writerow(dict(n=n, id=o.id, tag=detail(o, "tag"), **{"class": o.class_name}, color=o.color, dist_m=round(d, 3),
                            ok=ok, reason=why, skills=" ".join(skills), seconds=round(time.time() - t, 1)))
            fh.flush()
            loaded += ok
            if not ok:
                failed.append((o.pose.pose.position.x, o.pose.pose.position.y))
                if "basket" in why or "full" in why:
                    print("basket full / no slot — stop")
                    break
                run_plan([sg("pose", "stow")])  # hand may still hold the puck — stow, the next inspect decides (need_inspect)
        # skill_load load_end_pose over leaves the hand over the basket (11-15) — fold the arm when the run ends
        run_plan([sg("pose", "stow")])
    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        stop_bag()
print(f"\n{n} pucks tried, {loaded} loaded -> {out}")
node.destroy_node()
rclpy.shutdown()
