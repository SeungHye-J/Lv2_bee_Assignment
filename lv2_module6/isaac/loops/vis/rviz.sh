#!/usr/bin/env bash
# RViz for arm runs: MoveIt planned path + /tracked_objects puck boxes + rl_arm_correction ghosts (ref blue, ref+delta green, cmd orange).
#   ref = JTC target (plan before correction), target = ref + delta, cmd = ref + corr + delta + grav (what was sent).
#   Measured robot should settle on target (twist_state: true); its gap to cmd = sag the correction pre-bends for.
#
#   ./rviz.sh            # while manipulator-launcher.sh (sim or real) is up; close RViz to stop everything
#
# ROS env (domain, Fast DDS profile) is copied from the running move_group so RViz joins the same graph.
# SRDF goes in as a parameter too, so the Trajectory display works even on a move_group without
# publish_robot_description_semantic.
HERE="$(cd "$(dirname "$0")" && pwd)"
WS="$HERE/../../../ros2_ws"
source /opt/ros/lyrical/setup.bash
source "$WS/install/setup.bash"  # before set -u: colcon setup reads unset vars (COLCON_TRACE)
set -euo pipefail

MG_PID=$(pgrep -f -n "moveit_ros_move_group/move_group" || true)
[ -z "$MG_PID" ] && { echo "move_group not running — start manipulator-launcher.sh first" >&2; exit 1; }
for v in ROS_DOMAIN_ID FASTDDS_DEFAULT_PROFILES_FILE ROS_AUTOMATIC_DISCOVERY_RANGE; do
  val=$(tr '\0' '\n' < "/proc/$MG_PID/environ" | sed -n "s/^$v=//p")
  if [ -n "$val" ]; then export "$v=$val"; else unset "$v"; fi
done
SIM=false
SIM_PID=$(cat /tmp/beebot_isaac_sim.pid 2>/dev/null || true)
[ -n "$SIM_PID" ] && kill -0 "$SIM_PID" 2>/dev/null && SIM=true
echo "move_group pid $MG_PID, ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}, use_sim_time=$SIM"

PARAMS=$(mktemp --suffix=.yaml)
python3 - "$WS/src/ros_system/config/moveit/beebot.srdf" "$PARAMS" "$SIM" <<'EOF'
import sys, yaml
srdf, out, sim = sys.argv[1], sys.argv[2], sys.argv[3] == 'true'
yaml.safe_dump({'/**': {'ros__parameters': {'robot_description_semantic': open(srdf).read(),
                                            'use_sim_time': sim}}}, open(out, 'w'))
EOF

python3 "$HERE/correction_ghost.py" --ros-args -p use_sim_time:=$SIM &
GHOST=$!
python3 "$HERE/puck_box.py" --ros-args -p use_sim_time:=$SIM &
PUCKS=$!
trap 'kill -TERM $GHOST $PUCKS 2>/dev/null; wait $GHOST $PUCKS 2>/dev/null; rm -f "$PARAMS"' EXIT
rviz2 -d "$HERE/ghost.rviz" --ros-args --params-file "$PARAMS"
