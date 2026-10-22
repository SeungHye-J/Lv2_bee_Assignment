#!/usr/bin/env bash
# 운영 확인용 RViz — 카메라/점구름을 Best Effort + queue 1로 구독한다.
# Jetson에서 실행하면 로컬, PC에서 실행하면 네트워크 너머의 Jetson을 본다.
#   ./run_rviz.sh
#   ROS_DOMAIN_ID=42 ./run_rviz.sh
set -eo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
WS="$(cd "$HERE/../ros2_ws" 2>/dev/null && pwd || true)"
ROS_SETUP="/opt/ros/${ROS_DISTRO_NAME:-lyrical}/setup.bash"

if [ ! -f "$ROS_SETUP" ]; then
  echo "FAIL: ROS 2 setup 없음: $ROS_SETUP" >&2
  exit 1
fi
# shellcheck disable=SC1090
source "$ROS_SETUP"

if [ -n "$WS" ] && [ -f "$WS/install/setup.bash" ]; then
  # RViz 자체는 커스텀 메시지를 쓰지 않지만, 같은 환경에서 띄울 때 맞춘다.
  # shellcheck disable=SC1091
  source "$WS/install/setup.bash"
fi

if ! command -v rviz2 >/dev/null; then
  echo "FAIL: rviz2 없음 — ROS 2 RViz를 설치하세요" >&2
  exit 1
fi

exec rviz2 -d "$HERE/rviz/operator.rviz" "$@"
