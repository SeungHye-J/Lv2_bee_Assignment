"""Isaac Sim startup for the team: open usda/scene_sim.usda with the physx variant and start
opencr_bridge.py (virtual OpenCR on /tmp/opencr) so the ROS side can connect right away.
Viewport starts with colliders hidden.
While playing, puck ground truth goes to /tmp/beebot_sim_pucks.json (loops/pick_loop.py use_sim=true logs it).
Same result on every machine, no personal layout file.
Window: Kit raises and focuses its own window several times while loading (GLFW XRaiseWindow/XSetInputFocus, which
GNOME's focus-new-windows setting does not stop), so typing in a terminal gets interrupted. BEE_START_MINIMIZED=1
(default off) keeps the window minimized until the stage has loaded + KEEP_MIN_S, then leaves it alone
(open it from the dock, then press Play). The first raises come from Kit before this script runs, so they still
happen (11-13) - only ISAAC_HEADLESS=1 avoids them. No effect headless.
Sensor realism, BEE_SIM_NOISE (default 1, 0 = all off, shared with camera_adapter.py on the ROS side): RTX motion
blur on the colour images, mild on purpose (exposure 0.3 frame) - unmeasured values stay weaker than the real camera
so the sim never fails for reasons the robot would not. Depth is not blurred (checked 11-13).

    isaacsim --exec isaac_startup.py        (isaac_sim.sh does this)
"""
import asyncio
import json
import math
import os
import re
import time

import carb.settings
import omni.kit.app
import omni.physics.tensors
import omni.timeline
import omni.usd
from pxr import Gf, PhysxSchema, Usd, UsdGeom

HERE = os.path.dirname(os.path.abspath(__file__))
SCENE = os.environ.get("ISAAC_SCENE") or os.path.join(HERE, "..", "..", "usda", "scene_sim.usda")  # isaac_sim.sh <scene>
RESET_FILE = "/tmp/beebot_isaac_reset"  # isaac_sim.sh와 같은 경로
TRUTH_FILE = "/tmp/beebot_sim_pucks.json"
TRUTH_EVERY = 15  # frames (~4 Hz at 60 fps)
BASE = "/World/beebot/Geometry/base_footprint/base_link"  # 1 cm above base_footprint, same x y (URDF base_joint)
PUCK = re.compile(r"puck_(box|cyl)_(red|green)_\d+$")
PUCK_H = 0.06
ARM = "/World/beebot/Geometry/base_footprint/base_link/arm_base_link/shoulder_link/upper_arm_link/forearm_link/wrist_link/gripper_link"
CAMS = {"gripper_cam": ARM + "/gripper_cam_link/gripper_cam_color_optical_frame"}


_phys = {}  # physics tensor view of [base_link] + pucks — USD transforms stay at the file values while playing (Fabric)


def _poses(stage, paths):
    """world matrices from PhysX (the USD prims are not written back while playing). Rebuilt after a reset."""
    if "view" not in _phys:
        _phys["sim"] = omni.physics.tensors.create_simulation_view("numpy", omni.usd.get_context().get_stage_id())
        _phys["view"] = _phys["sim"].create_rigid_body_view(paths)
    v = _phys["view"]
    out = {}
    for path, t in zip(v.prim_paths, v.get_transforms()):  # px py pz qx qy qz qw
        out[path] = Gf.Matrix4d().SetTransform(Gf.Rotation(Gf.Quatd(float(t[6]), float(t[3]), float(t[4]), float(t[5]))),
                                               Gf.Vec3d(float(t[0]), float(t[1]), float(t[2])))
    return out


def write_truth(stage, pucks):
    """Puck centres in base_link (x forward, y left) [m], yaw of the long/up axis [deg], tilt from upright [deg].
    The puck prim origin is its bottom-face centre, so the centre is origin + up * PUCK_H/2 (matters when lying)."""
    w = _poses(stage, [BASE] + pucks)
    inv = w[BASE].GetInverse()
    out = {}
    for path in pucks:
        m = w[path] * inv  # puck -> base_link
        up = m.TransformDir(Gf.Vec3d(0, 0, 1)).GetNormalized()
        c = m.Transform(Gf.Vec3d(0, 0, PUCK_H / 2))
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, up[2]))))
        out[path.rsplit("/", 1)[1]] = {"x": c[0], "y": c[1], "z": c[2], "tilt": tilt,
                                       "yaw": math.degrees(math.atan2(up[1], up[0])) if tilt > 45 else None}
    inv = omni.usd.get_world_transform_matrix(stage.GetPrimAtPath(BASE)).GetInverse()  # cams: USD (file pose of the arm links)
    cams = {}  # optical frames (x right, y down, z forward) in base_link — compare with TF base_footprint -> *_color_optical_frame
    for name, path in CAMS.items():
        prim = stage.GetPrimAtPath(path)
        if prim.IsValid():
            m = omni.usd.get_world_transform_matrix(prim) * inv
            cams[name] = {"t": list(m.ExtractTranslation()), "R": [list(m.TransformDir(Gf.Vec3d(*a))) for a in ((1, 0, 0), (0, 1, 0), (0, 0, 1))]}
    with open(TRUTH_FILE + ".tmp", "w") as f:
        json.dump({"t": time.time(), "pucks": out, "cams": cams}, f)
    os.replace(TRUTH_FILE + ".tmp", TRUTH_FILE)


CASE_FILE = "/tmp/beebot_pick_case.json"  # scene_pick_bot: {"box": 1..6, "side": "-y"|"+y", "jitter": [[dx, dy, dyaw_deg], ...]}
DOCK_X = 0.1025 + 0.085  # [m] box centre ahead of base_footprint when docked (plan.yaml box_half + approach gap)
BOX_R = 0.16  # [m] pucks within this of a box centre (xy) belong to that box


def apply_case(stage, orig):
    """Move start box N and its pucks so the robot is docked on that box side, square to it (session layer, never saved).
    orig: world matrices read before the first move. Other boxes go back to where the file has them.
    side "-y": the robot faces the box from its -y side (+y side: the box is turned 180 deg first).
    jitter: optional per-puck (in box order of distance) offsets in the robot frame (x forward, y left) [m, m, deg] for robustness runs."""
    try:
        case = json.load(open(CASE_FILE))
    except (OSError, ValueError):
        return
    box = f"/World/start_boxes/start_{int(case['box'])}"
    robot = omni.usd.get_world_transform_matrix(stage.GetPrimAtPath(BASE))
    c = orig[box].ExtractTranslation()
    yaw = math.degrees(math.atan2(orig[box][0][1], orig[box][0][0]))  # box x axis in world (row-vector matrices)
    turn = 90.0 if case.get("side", "-y") == "-y" else -90.0  # box +y axis (-y side faces it) onto the robot's forward
    rx = robot.TransformDir(Gf.Vec3d(1, 0, 0))
    ryaw = math.degrees(math.atan2(rx[1], rx[0]))
    target = robot.Transform(Gf.Vec3d(DOCK_X, 0, 0))
    # world -> box frame -> placed in front of the robot (box +y or -y axis = robot forward)
    move = (Gf.Matrix4d().SetTranslate(Gf.Vec3d(-c[0], -c[1], 0)) * Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(0, 0, 1), -yaw))
            * Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(0, 0, 1), ryaw - turn)) * Gf.Matrix4d().SetTranslate(Gf.Vec3d(target[0], target[1], 0)))
    mine = sorted((p for p in orig if PUCK.match(p.rsplit("/", 1)[1])
                   and math.hypot(orig[p].ExtractTranslation()[0] - c[0], orig[p].ExtractTranslation()[1] - c[1]) < BOX_R),
                  key=lambda p: math.hypot(orig[p].ExtractTranslation()[0] - c[0], orig[p].ExtractTranslation()[1] - c[1]))
    jit = case.get("jitter") or []
    for path, m in orig.items():
        w = m
        if path == box or path in mine:
            w = m * move
            if path in mine and mine.index(path) < len(jit):
                dx, dy, dyaw = jit[mine.index(path)]
                p = w.ExtractTranslation()
                w = (w * Gf.Matrix4d().SetTranslate(-p) * Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(0, 0, 1), dyaw))
                     * Gf.Matrix4d().SetTranslate(p + robot.TransformDir(Gf.Vec3d(dx, dy, 0))))
        prim = stage.GetPrimAtPath(path)
        parent = omni.usd.get_world_transform_matrix(prim.GetParent())
        # translate · orient · scale ops, not one matrix op: PhysX writes the simulated pose back only into these, so with
        #   xformOp:transform the rendered puck stayed where it was put while its body moved (11-15: ghost puck on the box)
        t = Gf.Transform(w * parent.GetInverse())
        xf = UsdGeom.Xformable(prim)
        xf.ClearXformOpOrder()
        xf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(t.GetTranslation())
        xf.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(t.GetRotation().GetQuat()))
        xf.AddScaleOp(UsdGeom.XformOp.PrecisionFloat).Set(Gf.Vec3f(t.GetScale()))
    print(f"[isaac_startup] case: box {case['box']} side {case.get('side', '-y')} pucks {[p.rsplit('/', 1)[1] for p in mine]}", flush=True)


KEEP_MIN_S = 15.0  # [s] after the stage has loaded (late layout windows still raise the app)


async def _keep_minimized(app, state):
    """Minimize the main window whenever Kit raises it, until state['until'] (wall time) has passed."""
    try:
        import carb.windowing
        import omni.appwindow
        win = omni.appwindow.get_default_app_window().get_window()
        wi = carb.windowing.acquire_windowing_interface()
    except Exception as e:  # noqa: BLE001 - headless or API change: never block the startup
        print(f"[isaac_startup] window stays as is ({e!r})", flush=True)
        return
    if win is None:
        return
    while time.time() < state["until"]:
        if not wi.is_window_minimized(win):
            wi.minimize_window(win)
        await app.next_update_async()
    print("[isaac_startup] window no longer kept minimized - open it from the dock", flush=True)


async def _setup():
    app = omni.kit.app.get_app()
    minimized = {"until": time.time() + 600.0}  # until the stage has loaded (set below)
    if os.environ.get("BEE_START_MINIMIZED", "0") == "1" and not os.environ.get("ISAAC_HEADLESS"):
        asyncio.ensure_future(_keep_minimized(app, minimized))
    else:
        minimized["until"] = 0.0
    for _ in range(60):  # let the default windows come up first
        await app.next_update_async()
    await omni.usd.get_context().open_stage_async(os.path.abspath(SCENE))
    minimized["until"] = min(minimized["until"], time.time() + KEEP_MIN_S)
    # Viewport: no collider debug draw, no guide-purpose prims (the robot's collision meshes are guides).
    # Set after opening the stage so per-user persistent settings don't win.
    # Render mode stays at the default RTX - Real-Time: /rtx/rendermode is global, so it also drives the ROS camera images.
    settings = carb.settings.get_settings()
    settings.set_int("/persistent/physics/visualizationDisplayColliders", 0)
    settings.set_bool("/persistent/app/hydra/displayPurpose/guide", False)
    noise = float(os.environ.get("BEE_SIM_NOISE", "1"))
    settings.set_bool("/rtx/post/motionblur/enabled", noise > 0)  # global: viewport + both ROS colour cameras
    if noise > 0:
        settings.set_float("/rtx/post/motionblur/exposureFraction", 0.3 * noise)  # of one rendered frame (estimate)
        settings.set_float("/rtx/post/motionblur/maxBlurDiameterFraction", 0.05)
        settings.set_int("/rtx/post/motionblur/numSamples", 8)
    print(f"[isaac_startup] sensor noise BEE_SIM_NOISE={noise:g} (motion blur {'on' if noise > 0 else 'off'})", flush=True)
    stage = omni.usd.get_context().get_stage()
    # scene_sim.usda selects no Physics variant ("none" = no articulation, opencr_bridge finds nothing).
    # Pick physx in the session layer: never saved into the file.
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        stage.GetPrimAtPath("/World/beebot").GetVariantSet("Physics").SetVariantSelection("physx")
        # CPU PhysX: one robot + a few pucks is too small for GPU dynamics, whose per-step GPU<->CPU sync
        # (opencr_bridge reads joints via the numpy tensor API every step) cost more than the solve.
        # Measured 2026-10-30: 26 -> 39 fps, sim speed 0.43x -> 0.65x real time.
        scene = PhysxSchema.PhysxSceneAPI.Apply(stage.GetPrimAtPath("/PhysicsScene"))
        scene.CreateEnableGPUDynamicsAttr().Set(False)
        scene.CreateBroadphaseTypeAttr().Set("MBP")
        # Physics rate = Isaac Lab training (SimulationCfg dt 1/200): the scene default is 60 Hz.
        # Also lets opencr_bridge send S lines every 10 ms like the firmware. BEE_PHYSICS_HZ=60 if the sim gets too slow.
        scene.CreateTimeStepsPerSecondAttr().Set(int(os.environ.get("BEE_PHYSICS_HZ", "200")))
    pucks = [str(p.GetPath()) for p in stage.Traverse() if PUCK.match(p.GetName())]
    orig = {}  # scene_pick_bot: world poses as the file has them, for apply_case
    if os.path.basename(SCENE) == "scene_pick_bot.usda" and os.path.exists(CASE_FILE):  # only that scene (other sims share /tmp)
        orig = {str(p.GetPath()): omni.usd.get_world_transform_matrix(p) for p in stage.GetPrimAtPath("/World/start_boxes").GetChildren()}
        orig.update({p: omni.usd.get_world_transform_matrix(stage.GetPrimAtPath(p)) for p in pucks})
        with Usd.EditContext(stage, stage.GetSessionLayer()):
            apply_case(stage, orig)
    bridge = os.path.join(HERE, "opencr_bridge.py")  # creates the pty now; it attaches to the robot on the first physics step
    exec(compile(open(bridge).read(), bridge, "exec"), {"__name__": "__main__", "__file__": bridge})
    timeline = omni.timeline.get_timeline_interface()
    if os.environ.get("ISAAC_AUTOPLAY") == "1":  # 시연: Play를 자동으로
        timeline.play()
    frame = 0
    while True:  # reset.sh: Stop(장면 처음 자세로) → Play — ROS 재시작은 isaac_sim.sh
        await app.next_update_async()
        frame += 1
        if pucks and frame % TRUTH_EVERY == 0 and timeline.is_playing():
            try:
                write_truth(stage, pucks)
            except Exception as e:  # noqa: BLE001 - never stop the reset loop over the truth file
                _phys.clear()
                if frame % (TRUTH_EVERY * 240) == 0:
                    print(f"[isaac_startup] puck truth not written: {e!r}", flush=True)
        if os.path.exists(RESET_FILE):
            timeline.stop()
            _phys.clear()
            for _ in range(30):
                await app.next_update_async()
            if orig:
                with Usd.EditContext(stage, stage.GetSessionLayer()):
                    apply_case(stage, orig)
                for _ in range(5):
                    await app.next_update_async()
            timeline.play()
            os.remove(RESET_FILE)


asyncio.ensure_future(_setup())
