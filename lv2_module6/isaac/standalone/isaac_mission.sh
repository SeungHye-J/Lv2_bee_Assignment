#!/usr/bin/env bash
# scene_sim.usda에서 실기(Jetson start.sh)와 같은 bringup 인자로 미션 한 판 — 신호등 · 바코드 · 퍽을 전부 ROS 인지로 읽는다
# (isaac/mission/run_mission.py처럼 신호·바코드를 읽은 셈 치지 않음). 폴 뒤 카메라는 끔.
#
#   isaac/standalone/isaac_mission.sh map     # 처음 한 번: 장면 매핑 → ros2_ws/map/scene_sim/ (Isaac 창을 닫거나 Ctrl+C로 끄면 저장)
#   isaac/standalone/isaac_mission.sh         # 미션: 저장 지도 scene_sim으로 위치추정 · 신호등 자동 점멸 · Play 자동
#   BEE_BLINK_SEQ=RGYRGRYGRGYR BEE_BARCODE=P2-02 isaac/standalone/isaac_mission.sh   # 신호(기본 blink.py SEED) · 바코드(기본 장면 값 P2-01)
#   BEE_BLINK_SEED=7 isaac/standalone/isaac_mission.sh                               # 랜덤 신호 — 장면 퍽으로 풀리는 조합만 (blink.py)
#   다른 Isaac 환경변수(ISAAC_HEADLESS · BEE_SIM_NOISE · OPENCR_TWIST_SCALE …)는 isaac_sim.sh 그대로
#   BEE_RVIZ=0 isaac/standalone/isaac_mission.sh   # odom_nvblox RViz 안 띄움 (기본: 컨트롤러가 뜬 뒤 자동으로 띄움 · 녹화 끔은 RVIZ_RECORD=0)
#
# 같이 띄우는 것 (map · mission 둘 다): runtime/cm_kick.py — controller_manager가 /robot_description을 못 받고 멈추면
#   (11-16 4회 연속) 같은 URDF를 다시 보내 살림 → 그 뒤 isaac/loops/vis/odom_nvblox.sh. Isaac을 끄면 RViz도 끔
#
# 미션 시작은 실기처럼 운영 도구로 (다른 터미널 — Isaac과 같은 FASTDDS_DEFAULT_PROFILES_FILE이어야 보임 · 없으면 local_only):
#   tools/cli/start.sh mission start    # 상태: tools/cli/start.sh mission status · 그만: mission abort
#
# 실기 start.sh 기본 그대로: map:=saved map_name:=scene_sim map_dir:=ros2_ws/map perception:=true use_pole_back_cam:=false
#   operation:=AUTO (바로 미션 · 실기 MANUAL과 다름) · use_perception은 bringup 기본 auto (매핑이면 인지 끔)
#   arena_map:=none — 벽은 실기처럼 mission_manager가 plan.yaml 벽으로 그린 /arena/walls (장면 벽 지도 /arena/static 안 씀)
# Isaac이라 실기와 다른 것: port:=/tmp/opencr · use_cameras:=false(카메라 토픽은 Isaac) · use_sim_time · Fast DDS 프로파일(isaac_sim.sh)
# ros2_ws/map은 .gitignore — 여기 만든 시뮬 지도는 Jetson의 실기 지도(같은 이름 scene_sim)와 섞이지 않는다.
HERE=$(dirname "$(realpath "$0")")
MAP_DIR=$(realpath -m "$HERE/../../ros2_ws/map")
NAME=scene_sim
COMMON="map_dir:=$MAP_DIR map_name:=$NAME use_pole_back_cam:=false operation:=AUTO use_perception:=auto arena_map:=none"
export BEE_BACK_CAM=false  # camera_adapter도 앞 · 손 카메라만

case "${1:-mission}" in
  map)
    [ -f "$MAP_DIR/$NAME/nvblox.nvblx" ] && { echo "이미 있는 지도: $MAP_DIR/$NAME — 다시 만들려면 그 폴더를 지우고"; exit 1; }
    export ISAAC_ROS_ARGS="map:=slam $COMMON"
    ;;
  mission)
    [ -f "$MAP_DIR/$NAME/nvblox.nvblx" ] || { echo "저장 지도 없음: $MAP_DIR/$NAME/nvblox.nvblx — 먼저 $0 map"; exit 1; }
    export ISAAC_ROS_ARGS="map:=saved perception:=true $COMMON"
    export BEE_BLINK=1
    ;;
  *) sed -n '2,9p' "$0"; exit 1 ;;
esac
export ISAAC_AUTOPLAY="${ISAAC_AUTOPLAY:-1}"
echo "[isaac_mission] ${1:-mission} · ROS 인자: $ISAAC_ROS_ARGS"
# cm_kick · RViz는 Isaac 쪽과 같은 Fast DDS 프로파일로 (isaac_sim.sh와 같은 값). 자기 세션(setsid)이라 Ctrl+C는 아래 cleanup이 처리
(export FASTDDS_DEFAULT_PROFILES_FILE="${FASTDDS_DEFAULT_PROFILES_FILE:-$HERE/../../ros2_ws/fastdds_local_only.xml}" ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
 source /opt/ros/lyrical/setup.bash
 /usr/bin/python3 "$HERE/runtime/cm_kick.py"
 [ "${BEE_RVIZ:-1}" = 0 ] || exec setsid "$HERE/../loops/vis/odom_nvblox.sh") &
EXTRA=$!
cleanup() {  # exec setsid라 RViz 단계면 $EXTRA = start.sh(세션 리더) — rviz2만 끄면 start.sh가 녹화(record.sh)까지 끄고 끝남
  trap - EXIT INT TERM HUP
  if [ "$(ps -o sid= -p $EXTRA 2>/dev/null | tr -d ' ')" = "$EXTRA" ]; then pkill -TERM -x rviz2 -s $EXTRA
  else kill $EXTRA 2>/dev/null; pkill -f "$HERE/runtime/cm_kick.py"; fi
}
trap cleanup EXIT INT TERM HUP
"$HERE/isaac_sim.sh" sim
