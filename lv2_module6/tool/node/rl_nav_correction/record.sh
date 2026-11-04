#!/usr/bin/env bash
# rl_nav_correction 기록 — rosbag2 (mcap). 주행 전에 켜 두고 Ctrl+C로 끝낸다.
#   ./run record ~/bags/on
#   ROS_DOMAIN_ID=88 ./run record ~/bags/on
# ON/OFF 비교하려면 같은 출발점·같은 goal로 두 번 (toggle 서비스로 보정 끄기).
set -eo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
WS="$(cd "$HERE/../../../ros2_ws" 2>/dev/null && pwd || true)"
ROS_SETUP="/opt/ros/${ROS_DISTRO_NAME:-lyrical}/setup.bash"
OUT="${1:?사용법: ./run record <출력 폴더>}"

# shellcheck disable=SC1090
source "$ROS_SETUP"
if [ -n "$WS" ] && [ -f "$WS/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "$WS/install/setup.bash"
fi

# ROS_DOMAIN_ID가 bringup과 같아야 한다 (~/.bashrc 값 확인)
exec ros2 bag record -o "$OUT" --topics \
  /cmd_vel_nav /cmd_vel_corrected \
  /odometry/filtered /odom /imu \
  /plan /tf /tf_static \
  /rl_nav_correction/debug
