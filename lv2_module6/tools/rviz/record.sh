#!/usr/bin/env bash
# 주행 디버깅용 rosbag 녹화 (PC) — 가벼운 토픽만 (영상·점구름 제외: Wi-Fi로 받으면 RViz·Jetson 송신이 막힘)
#   켜 두면 운행 시작을 기다렸다가 녹화: 새 /task/sub_goals(미션·approach 계획) 또는 첫 /plan이 오는 순간부터.
#   그 전(RViz만 켜 둔 시간)은 버림. Ctrl-C(또는 start.sh가 RViz 끌 때)로 끝내면 파일을 닫음.
#   ./record.sh                         # → ~/bee_bags/nav_<월일_시분초>/
#   RVIZ_BAG_DIR=/data/bags ./record.sh
#   start.sh가 RViz와 같이 띄움 (PC에서만 · RVIZ_RECORD=0이면 안 띄움)
set -eo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
WS="$(cd "$HERE/../../ros2_ws" 2>/dev/null && pwd || true)"
# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO_NAME:-lyrical}/setup.bash"
# shellcheck disable=SC1091
[ -n "$WS" ] && [ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash"

TOPICS=(/tf /tf_static /odom /odometry/filtered /cuvslam/odom /imu /joint_states
        /cmd_vel /cmd_vel_nav /cmd_vel_smoothed /plan /unsmoothed_plan /transformed_global_plan
        /local_costmap/costmap /global_costmap/costmap /local_costmap/published_footprint
        /collision_monitor_state /collision_monitor/triggering_points /polygon_approach
        /navigate_to_pose/_action/feedback /skills/navigate/_action/feedback
        /task/status /task/sub_goals /state/collision /state/operation_mode /arena/correction /tracked_objects /rosout)
OUT="${RVIZ_BAG_DIR:-$HOME/bee_bags}/nav_$(date +%m%d_%H%M%S)"
mkdir -p "$(dirname "$OUT")"

echo "[record] 운행 시작 기다림 (새 /task/sub_goals 또는 첫 /plan) → $OUT"
# 기다림: /task/sub_goals는 latched라 구독 직후 오는 옛 계획은 건너뜀 (2 s 안에 온 것)
python3 - <<'EOF' || exit 0
import time, rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import Path
from plan.msg import SubGoalArray
rclpy.init()
n = rclpy.create_node('rviz_bag_trigger')
t0, hit = time.time(), []
latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
n.create_subscription(SubGoalArray, '/task/sub_goals', lambda m: hit.append('sub_goals') if time.time() - t0 > 2.0 and m.goals else None, latched)
n.create_subscription(Path, '/plan', lambda m: hit.append('plan'), 10)
try:
    while not hit:
        rclpy.spin_once(n, timeout_sec=0.2)
except KeyboardInterrupt:
    raise SystemExit(1)
print(f'[record] {hit[0]} 받음 — 녹화 시작', flush=True)
EOF

# 녹화 — 끝낼 때 SIGINT를 받아야 mcap이 닫힘 (exec로 신호를 그대로 받음) · hidden: 액션 feedback (_action/feedback)
exec ros2 bag record --disable-keyboard-controls --include-hidden-topics -s mcap -o "$OUT" --topics "${TOPICS[@]}"
