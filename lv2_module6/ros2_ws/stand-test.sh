#!/bin/bash
# 팔 작업 테스트(세우기 stand · 집기 pick)용 가벼운 실행 — SLAM·Nav2 없이 팔·카메라·인지·MoveIt·coordinator만
#   (무엇을 빼고 남기는지: src/bringup/launch/stand_test.launch.py)
#
#   ./stand-test.sh                         # 기본
#   ./stand-test.sh use_rl_policy:=true     # 잔차 정책 노드까지 (ros2 param set /coordinator rl_policy true와 함께)
#
# 저장소 가상환경(../../.venv — YOLO ultralytics · torch)이 있으면 켜고, start.sh의 환경 확인(OpenCR · 카메라 serial · MoveIt)을 그대로 거친다.
#
# 시작 전에 깨끗한 상태로 만든다 (끄려면 STAND_NO_CLEAN=1 ./stand-test.sh):
#   1) ./stop.sh — 떠 있는 ROS(launch · 노드 · ros2 CLI)와 ros2 daemon 끄기
#   2) /dev/shm의 Fast DDS 잔재 삭제 (노드당 35 MB — 강제 종료된 노드가 남기면 RAM을 먹고 SHM 포트 오류가 남)
#      ROS가 하나라도 안 죽었으면 지우지 않는다 (살아 있는 노드의 공유 메모리를 지우면 통신이 끊김)
# 같은 머신의 다른 ROS 작업도 같이 꺼진다 (stop.sh와 같음).
HERE="$(cd "$(dirname "$0")" && pwd)"
VENV="$HERE/../../.venv/bin/activate"
if [ -z "${STAND_NO_CLEAN:-}" ]; then
  "$HERE/stop.sh"
  # stop.sh와 같은 기준 — 실행 파일이 ROS 설치 폴더 아래인 프로세스 (패턴 글자를 [] 로 쪼개 자기 자신은 안 걸리게)
  if pgrep -f "(/opt/ros/[^/ ]+|$HERE/install|/ws/install)/[^ ]*li[b]/|bin/ros2 (launc[h]|ru[n]|topi[c]|service|para[m]|nod[e]|actio[n]|lifecycl[e]|contro[l])" >/dev/null; then
    echo "WARN: 안 죽은 ROS 프로세스가 있어 Fast DDS 공유 메모리는 지우지 않음"
  else
    n=$(ls /dev/shm 2>/dev/null | grep -c '^\(sem\.\)\?fastdds_')
    rm -f /dev/shm/fastdds_* /dev/shm/sem.fastdds_* 2>/dev/null
    echo "Fast DDS 공유 메모리 ${n}개 삭제"
  fi
fi
if [ -z "${VIRTUAL_ENV:-}" ] && [ -f "$VENV" ]; then
  # shellcheck disable=SC1090
  source "$VENV"
  echo "가상환경: $VIRTUAL_ENV"
fi
# Jetson 한 대 안에서만 도는 테스트라 SHM 전용 프로파일을 기본으로 (start.sh 기본은 fastdds.xml = UDP+SHM).
# 11/04: fastdds.xml로 뜬 스택에서 EKF가 TF를 못 받아 "Failed to meet update rate"로 막히고 odom TF가 안 나왔음.
# 다른 값으로 쓰려면 FASTDDS_DEFAULT_PROFILES_FILE=... ./stand-test.sh
export FASTDDS_DEFAULT_PROFILES_FILE="${FASTDDS_DEFAULT_PROFILES_FILE:-$HERE/fastdds_local.xml}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-88}"
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}"
LAUNCH_FILE=stand_test.launch.py exec "$HERE/start.sh" "$@"
