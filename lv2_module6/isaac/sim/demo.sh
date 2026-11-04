#!/usr/bin/env bash
# Demo: one manipulator cycle in Isaac Sim (window) on the mission scene usda/scene_lab.usda.
#   from the saved start pose: pick a puck from start_1 -> load into basket slot 5 -> drive to shelf_L1 -> take it out ->
#   place it on the centre cell (dice 1) with the trained residual policy (isaac/lab/policies/place_residual.pt)
#
#   isaac/sim/demo.sh                      # one cycle, window stays open at the end (close it to quit)
#   isaac/sim/demo.sh --runs 3             # three cycles in a row (scene resets between them)
#   isaac/sim/demo.sh --no_policy          # place with the scripted motion only (residual 0), for comparison
#   isaac/sim/demo.sh --fast               # place at 2x speed with the speed-randomized policy (place_residual_fast.pt)
#   isaac/sim/demo.sh --seed -1            # random perception noise (default seed 0 = the run checked before handing over)
#   isaac/sim/demo.sh --puck b --cell 1    # the other start_1 puck, near-row centre cell (cells 0..5: near row, middle row)
#   HEADLESS=1 isaac/sim/demo.sh --runs 20 # no window, prints the success rate
#   VIZ=newton isaac/sim/demo.sh           # Newton OpenGL viewer instead of the Kit window (newton_rtx, rerun, viser too)
#   isaac/sim/demo.sh --render RealTimePathTracing   # full RTX look (default: RTX - Minimal viewport, lighter)
#   DEVICE=cuda:0 isaac/sim/demo.sh        # GPU PhysX (default cpu: 1 robot + a few pucks, CPU PhysX ran ~5x faster)
#   ISAACLAB=<path> isaac/sim/demo.sh      # Isaac Lab elsewhere (default ~/pa-course/IsaacLab-6.1, uses its .venv)
#
# No ROS: arm, gripper and wheels are driven inside Isaac Lab (same physics settings as training).
# The terminal prints each step ([demo] 1 ... 9) and the place result (centre error, tilt, stillness for 2 s).
HERE=$(dirname "$(realpath "$0")")
LAB=${ISAACLAB:-$HOME/pa-course/IsaacLab-6.1}
PY="$LAB/.venv/bin/python"
[ -x "$PY" ] || { echo "Isaac Lab python not found: $PY (set ISAACLAB=<Isaac Lab folder>)"; exit 1; }
cd "$HERE/../lab" || exit 1
if [ "${HEADLESS:-0}" = 1 ]; then
  exec "$PY" demo_cycle.py --device "${DEVICE:-cpu}" "$@"
fi
exec "$PY" demo_cycle.py --device "${DEVICE:-cpu}" --viz "${VIZ:-kit}" --hold "$@"
