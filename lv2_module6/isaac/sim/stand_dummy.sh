#!/usr/bin/env bash
# 세우기 + 제자리 다시 잡기 더미 — 실기 ROS 팔 경로(JTC → rl_arm_correction → 가상 OpenCR)로 한 번 돌린다.
#   isaac/sim/stand_dummy.sh                 # 창으로 보고, 끝나도 Isaac은 남는다 (닫으면 ROS도 꺼짐)
#   STAND_EXIT=1 isaac/sim/stand_dummy.sh    # 결과 줄을 찍고 전부 끈다
#   STAND_SEED=3 isaac/sim/stand_dummy.sh    # 인식 잡음 seed (기본 0)
# 장면: scene_lab.usda를 저장된 그대로 연다 (읽기만) — 로봇은 start_1을 바라보고, puck_box_green_6은 누워 있어야 한다.
#   더미가 먼저 직진해 학습 때 도킹 거리(상자 앞면 0.08 m)로 간다 (/cmd_vel → diff_drive_controller, 바퀴 odom).
# ROS: ros_system 중 ros2_control만 (stand_dummy.launch.py) — SLAM·Nav2·카메라·MoveIt·plan 노드 없음.
# 그리퍼 힘(0.4 N 집기 → 전력 쥐기)은 /tmp/opencr_grip_force 파일로 Isaac 쪽에서만 바꾼다 (펌웨어에 아직 명령 없음).
# 필요: ros2_ws의 ml 패키지 빌드 (rl_arm_correction reset_sag), Isaac Lab venv (경로 계획)
HERE=$(dirname "$(realpath "$0")")
PORT=/tmp/opencr
LAB=${ISAACLAB:-$HOME/pa-course/IsaacLab-6.1}
export FASTDDS_DEFAULT_PROFILES_FILE="$HERE/../../ros2_ws/fastdds.xml"
export STAND_PLAN=/tmp/stand_plan.json
source /opt/ros/lyrical/setup.bash
(cd "$HERE/../lab" && env -u PYTHONPATH "$LAB/.venv/bin/python" stand_plan.py --seed "${STAND_SEED:-0}" --out "$STAND_PLAN") || exit 1

OTHERS=$(ros2 node list --no-daemon --spin-time 3 2>/dev/null | grep -vE '^/(rviz|transform_listener_impl_|_ros2cli_)')
if [ -n "$OTHERS" ]; then
  echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}에 이미 노드가 떠 있음 — 끄고 다시 실행:"; echo "$OTHERS" | sed 's/^/  /'; exit 1
fi
fastdds shm clean >/dev/null 2>&1
rm -f "$PORT" /tmp/opencr_grip_force
# ROS's own pinocchio (ros-lyrical-pinocchio, built for numpy 1.x) sits on the sourced PYTHONPATH ahead of the copy Isaac
# bundles for its pink extension, and Isaac (numpy 2.x) then fails to start (2026-11-02). Put Isaac's copy first.
PINK_PY="${ISAACSIM_VENV:-$HOME/pa-course/.venv-isaacsim}/lib/python3.12/site-packages/isaacsim/exts/isaacsim.robot_motion.pink/pip_prebundle/cmeel.prefix/lib/python3.12/site-packages"
(export PYTHONPATH="$PINK_PY:$PYTHONPATH" && source "${ISAACSIM_VENV:-$HOME/pa-course/.venv-isaacsim}/bin/activate" && \
  BEE_SCENE=scene_lab.usda BEE_AUTOPLAY=1 exec isaacsim --exec "$HERE/scripts/isaac_startup.py") &
ISAAC_PID=$!
until [ -e "$PORT" ]; do kill -0 $ISAAC_PID 2>/dev/null || exit 1; sleep 1; done
source "$HERE/../../ros2_ws/install/setup.bash"
setsid ros2 launch "$HERE/scripts/stand_dummy.launch.py" &
ROS_PID=$!
cleanup() {
  trap - EXIT INT TERM HUP
  kill $ISAAC_PID 2>/dev/null
  kill -INT -- -$ROS_PID 2>/dev/null
  for _ in $(seq 30); do pgrep -g $ROS_PID >/dev/null || return; sleep 0.5; done
  kill -KILL -- -$ROS_PID 2>/dev/null
}
trap cleanup EXIT INT TERM HUP
python3 "$HERE/scripts/stand_dummy.py" --ros-args -p use_sim_time:=true
RC=$?
[ "${STAND_EXIT:-0}" = 1 ] && exit $RC
wait $ISAAC_PID
