"""Traffic light blink sequence for the dice-memory mission (Isaac Sim Script Editor).

Open scene_sim.usda, press Play, paste this file into the Script Editor and Run.
Edit SEQUENCE below: a fixed string like "RGRRGRGGYGYG" (public practice Q1), or None for a
random valid sequence (R/Y/G each 1..6, total 12) drawn from SEED, so the same SEED always
gives the same sequence. Random sequences are limited to counts the pucks under /World/pucks can
solve (same rule as mission_solver allocate); a fixed SEQUENCE that cannot be solved only warns.
Running again cancels the previous run first.
To abort: set STOP = True and Run (all lights off).

Rule: each blink = one color ON 1 s + all OFF 1 s; after the 12th blink all lights stay OFF
for 3 s (the trailing 1 s off plus END_OFF_S more), then the same sequence repeats forever.
Every run starts with all lights OFF for 3 s before the first blink.
Timing follows timeline (sim) time, so it pauses with the simulation and matches use_sim_time.
Light state is written to the session layer: Ctrl+S never saves a lit lamp into scene_sim.usda.

Comments/strings stay English: the Script Editor breaks Korean text.
"""
import asyncio
import builtins
import random
from collections import Counter

import omni.kit.app
import omni.timeline
import omni.usd
from pxr import Usd

# ---- settings ----
SEQUENCE = None  # e.g. "RGRRGRGGYGYG"; None = random from SEED
SEED = 43
STOP = False
# ------------------

SIGN = "/World/sign/light/"
PUCKS = "/World/pucks"  # children named puck_<shape>_<color>_<n>
LIGHTS = {"R": "red", "Y": "yellow", "G": "green"}
ON_S = 1.0
OFF_S = 1.0
END_OFF_S = 2.0  # extra darkness after the last OFF_S -> 3 s total
START_OFF_S = 3.0  # darkness before the first blink


def _set(color=None):
    """Turn on one color (R/Y/G) or all off (None)."""
    stage = omni.usd.get_context().get_stage()
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        for key, name in LIGHTS.items():
            prim = stage.GetPrimAtPath(f"{SIGN}{name}/CylinderLight")
            prim.GetAttribute("visibility").Set("inherited" if key == color else "invisible")


async def _wait(seconds):
    """Wait in timeline time; does not advance while paused/stopped."""
    tl = omni.timeline.get_timeline_interface()
    app = omni.kit.app.get_app()
    t0 = tl.get_current_time()
    while tl.get_current_time() - t0 < seconds:
        await app.next_update_async()
        if tl.get_current_time() < t0:  # timeline was stopped/rewound
            t0 = tl.get_current_time()


def puck_counts():
    """(color, shape) -> count from the prim names under PUCKS."""
    stage = omni.usd.get_context().get_stage()
    root = stage.GetPrimAtPath(PUCKS)
    if not root:
        raise RuntimeError(f"{PUCKS} not found, open scene_sim.usda")
    n = Counter()
    for p in root.GetChildren():
        _, shape, color, *_ = p.GetName().split("_")
        n[color, shape] += 1
    return n


def solvable(r, y, g, n):
    """mission_solver allocate rule: R uses red, G uses green, leftovers (Y) share one shape."""
    y_red = n["red", "box"] + n["red", "cyl"] - r
    y_green = n["green", "box"] + n["green", "cyl"] - g
    if y_red < 0 or y_green < 0 or y_red + y_green != y:
        return False
    return any(n["red", s] >= y_red and n["green", s] >= y_green for s in ("box", "cyl"))


def random_sequence(seed, n, total=12, lo=1, hi=6):
    """R/Y/G counts each in [lo, hi] summing to total and solvable with pucks n, shuffled order."""
    combos = [(r, y, total - r - y) for r in range(lo, hi + 1) for y in range(lo, hi + 1)
              if lo <= total - r - y <= hi and solvable(r, y, total - r - y, n)]
    if not combos:
        raise RuntimeError(f"no solvable R/Y/G counts for pucks {dict(n)}")
    rng = random.Random(seed)
    while True:
        r, y = rng.randint(lo, hi), rng.randint(lo, hi)
        g = total - r - y
        if (r, y, g) in combos:
            seq = list("R" * r + "Y" * y + "G" * g)
            rng.shuffle(seq)
            return "".join(seq)


async def _run(seq):
    try:
        _set(None)  # start dark, same 3 s gap as between loops
        print(f"[blink] all off {START_OFF_S:.0f} s before start")
        await _wait(START_OFF_S)
        loop = 0
        while True:
            loop += 1
            for i, c in enumerate(seq, 1):
                _set(c)
                print(f"[blink] loop {loop} {i:2d}/{len(seq)} {LIGHTS[c]}")
                await _wait(ON_S)
                _set(None)
                await _wait(OFF_S)
            print(f"[blink] loop {loop} end, 3 s off")
            await _wait(END_OFF_S)
    except asyncio.CancelledError:
        _set(None)
        print("[blink] stopped")
        raise


# Script Editor re-runs this file in a fresh namespace, so keep the task handle on builtins
old = getattr(builtins, "_bee_blink_task", None)
if old and not old.done():
    old.cancel()
builtins._bee_blink_task = None

if STOP:
    _set(None)
    print("[blink] all off")
else:
    pucks = puck_counts()
    seq = (SEQUENCE or random_sequence(SEED, pucks)).upper().replace(" ", "")
    bad = set(seq) - set(LIGHTS)
    if bad:
        raise ValueError(f"unknown colors {bad}, use R/Y/G")
    if not solvable(seq.count("R"), seq.count("Y"), seq.count("G"), pucks):
        print(f"[blink] WARNING: {seq} cannot be solved with pucks {dict(pucks)} (mission gives no_allocation)")
    src = "fixed" if SEQUENCE else f"seed {SEED}"
    print(f"[blink] sequence {seq} ({src})  R={seq.count('R')} Y={seq.count('Y')} G={seq.count('G')}")
    if not omni.timeline.get_timeline_interface().is_playing():
        print("[blink] timeline not playing: press Play, the sequence waits for it")
    builtins._bee_blink_task = asyncio.ensure_future(_run(seq))
