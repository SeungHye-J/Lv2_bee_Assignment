#!/bin/bash
# Nav2 주행 정책(rl_nav_controller ONNX) faker 검증 (컨테이너 안에서 실행): test-nav-policy
#   faker launch (course 맵 · 진짜 Nav2) → ONNX 로드 확인 → toggle off/on →
#   제자리 근처 goal로 DWB·정책 주행 → /cmd_vel_nav · FollowPath/debug 확인
#   예) test-nav-policy
# NOTE: set -u 사용 금지 — setup.bash가 미설정 변수를 참조해서 죽음.
if [ -z "$BASH_VERSION" ]; then exec bash "$0" "$@"; fi   # sh(dash)로 실행해도 bash로 재실행
source /opt/ros/lyrical/setup.bash
BIN_DIR=$(dirname "${BASH_SOURCE[0]}")
[ "$BIN_DIR" = . ] && BIN_DIR="$PWD"   # 컨테이너(/usr/local/bin)·호스트(docker/) 모두 동작
bin_of() {  # 컨테이너는 확장자 없음, 레포는 .sh — 있는 쪽 반환, 둘 다 없으면 PATH에 기대고 bare 이름
  [ -x "$BIN_DIR/$1" ] && echo "$BIN_DIR/$1" && return 0
  [ -x "$BIN_DIR/$1.sh" ] && echo "$BIN_DIR/$1.sh" && return 0
  echo "$1"
}
if [ -z "$WS" ]; then
  if [ -d /ws/src ]; then WS=/ws; else WS="$BIN_DIR/../lv2_module6/ros2_ws"; fi   # 컨테이너면 /ws, 호스트면 레포 안 ros2_ws (머신 경로 하드코딩 없음)
fi
WS=$(realpath -m "$WS")   # ../ 정리
export ROS_AUTOMATIC_DISCOVERY_RANGE=${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}
export FASTDDS_DEFAULT_PROFILES_FILE=${FASTDDS_DEFAULT_PROFILES_FILE:-$WS/fastdds.xml}
cd "$WS" || { echo "FAIL: $WS 없음"; exit 2; }

KILL_PAT="($WS/install|/opt/ros/lyrical)/[^ ]*lib/|ros2 launch"
kill_all() {
  local pids
  [ -n "${ESDF_PID:-}" ] && kill "$ESDF_PID" 2>/dev/null
  pids=$(pgrep -f "$KILL_PAT" 2>/dev/null) || return 0
  # shellcheck disable=SC2086
  [ -n "$pids" ] && kill $pids 2>/dev/null
  sleep 2
  pids=$(pgrep -f "$KILL_PAT" 2>/dev/null) || return 0
  # shellcheck disable=SC2086
  [ -n "$pids" ] && kill -KILL $pids 2>/dev/null
  return 0
}
trap 'kill_all' EXIT INT TERM

# 이전에 죽은 실행이 남긴 FastDDS SHM 조각 정리 (가득 차면 discovery·토픽이 멈춤 — 이 테스트 전용 머신이라 전체 삭제)
kill_all 2>/dev/null
rm -f /dev/shm/fastdds_* 2>/dev/null
# 오래된 ros2 데몬 캐시 제거 (stale graph로 서비스·토픽 못 찾는 경우 방지)
pkill -f "ros2-daemon" 2>/dev/null
sleep 2

"$(bin_of build-ws)" || exit 1   # 빌드 + install (모델 share/ml/model 복사 포함)
# shellcheck disable=SC1091
source install/setup.bash

set -- "$@" "use_monitor:=false"   # 모니터 끔 (가벼운 실행 · PASS 판정에서 기록 검사 제외)
case " $* " in
  *\ mock_hardware:=*) : ;;
  *) [ -e /dev/opencr ] || set -- "$@" "mock_hardware:=true" ;;   # 호스트엔 가상 시리얼이 없어서 mock으로
esac
set -m
ros2 launch faker faker.launch.py "$@" > /tmp/nav-policy.log 2>&1 & LAUNCH_PID=$!

# EKF 오도메트리 대기 (fake_opencr → /joint_states → EKF)
timeout 60 ros2 topic echo --once /odometry/filtered nav_msgs/msg/Odometry > /dev/null 2>&1 \
  || { echo "FAIL: /odometry/filtered 없음"; tail -20 /tmp/nav-policy.log; exit 1; }
# Nav2 서버 active 대기 — controller_server(정책 플러그인 주인) + planner_server(경로 계산) + bt_navigator
for _ in $(seq 90); do
  CS="$(timeout 5 ros2 lifecycle get /controller_server 2>/dev/null)"
  PS="$(timeout 5 ros2 lifecycle get /planner_server 2>/dev/null)"
  BT="$(timeout 5 ros2 lifecycle get /bt_navigator 2>/dev/null)"
  [ "$(printf '%s\n%s\n%s' "$CS" "$PS" "$BT" | grep -o 'active \[3\]' | wc -l)" = 3 ] && break
  sleep 1
done
[ "$(printf '%s\n%s\n%s' "$CS" "$PS" "$BT" | grep -o 'active \[3\]' | wc -l)" = 3 ] \
  || { echo "FAIL: Nav2 서버가 active 아님"; grep -E "lifecycle_manager|ERROR" /tmp/nav-policy.log | tail -10; exit 1; }
# ONNX 로드 확인 (모델 없으면 바이패스 → 정책 테스트 불가)
grep -q "ONNX loaded" /tmp/nav-policy.log \
  || { echo "FAIL: ONNX 미로드 — share/ml/model/rl_nav_controller/policy.onnx 확인"; grep -iE "onnx|policy" /tmp/nav-policy.log | tail -10; exit 1; }
echo "OK: ONNX loaded"
# 토글 서비스 확인
timeout 10 ros2 service list 2>/dev/null | grep -q "/controller_server/FollowPath/toggle" \
  || { echo "FAIL: FollowPath/toggle 서비스 없음"; exit 1; }

# 점구름 소스: fake_camera가 시뮬레이터 프레임 또는 더미(장애물 없음)를 낸다 — 흐름 확인.
# 안 흘러오면 faker 배선 문제 — 테스트 진행 불가.
timeout 15 ros2 topic echo --once /pole_front_cam/depth/color/points_filtered sensor_msgs/msg/PointCloud2 > /dev/null 2>&1 \
  || { echo "FAIL: 점구름 흐름 없음 (fake_camera 확인)"; exit 1; }
echo "점구름: 흐름 확인"
# 코스트맵 발행 확인 (planner가 "Costmap timed out" 내지 않게 — map + voxel 초기화 대기)
timeout 30 ros2 topic echo --once /local_costmap/costmap nav_msgs/msg/OccupancyGrid > /dev/null 2>&1 \
  || { echo "FAIL: local costmap 없음"; exit 1; }
timeout 30 ros2 topic echo --once /global_costmap/costmap nav_msgs/msg/OccupancyGrid > /dev/null 2>&1 \
  || { echo "FAIL: global costmap 없음"; exit 1; }
echo "코스트맵: 흐름 확인"
# discovery 안정화 대기 (기동 직후 서비스 호출이 타임아웃 나지 않게)
sleep 20
# ESDF 더미: 컨트롤러는 nvblox ESDF 점구름(intensity=장애물까지 거리)을 본다.
# headless에선 nvblox가 안 돌므로 빈 점구름(intensity 필드 포함)을 깔아준다 = 전방 free.
# 없으면 sensor_ok=false → 정책이 한 번도 안 돌고 DWB 바이패스만 돈다 (테스트가 그걸 못 잡음).
# FRAME / 토픽명은 nav2_params FollowPath.points_topic과 같게 (기본 /nvblox_node/combined_esdf_pointcloud).
ESDF_TOPIC="${ESDF_TOPIC:-/nvblox_node/combined_esdf_pointcloud}"
HAS_ESDF=0
if timeout 8 ros2 topic echo --once "$ESDF_TOPIC" sensor_msgs/msg/PointCloud2 > /dev/null 2>&1; then
  echo "ESDF: 실점군 사용"
  HAS_ESDF=1
else
  cat > /tmp/esdf_dummy_pub.py <<'EOF'
import math
import struct
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
# 빈 점구름(width 0)은 on_cloud가 데이터 없음으로 버리므로, far-free 점 72개를 깐다.
# 반경 5m 링 · z 0.1m · intensity(esdf) 1.0 → r이 lidar_max(3.5)를 넘어 전부 스킵 = 전방 free.
N = 72
PTS = []
for i in range(N):
    a = 2.0 * math.pi * i / N
    PTS += [5.0 * math.cos(a), 5.0 * math.sin(a), 0.1, 1.0]
RAW = struct.pack('<%df' % len(PTS), *PTS)
class EmptyEsdf(Node):
    def __init__(self):
        super().__init__('test_empty_esdf')
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=5)
        self.pub = self.create_publisher(PointCloud2, '/nvblox_node/combined_esdf_pointcloud', qos)
        self.timer = self.create_timer(0.2, self.tick)
    def tick(self):
        m = PointCloud2()
        m.header = Header(stamp=self.get_clock().now().to_msg(), frame_id='base_footprint')
        m.height, m.width = 1, N
        m.fields = [PointField(name=n, offset=o, datatype=PointField.FLOAT32, count=1)
                    for n, o in (('x', 0), ('y', 4), ('z', 8), ('intensity', 12))]
        m.is_bigendian, m.point_step, m.row_step, m.is_dense = False, 16, 16 * N, True
        m.data = list(RAW)
        self.pub.publish(m)
def main():
    rclpy.init(); rclpy.spin(EmptyEsdf()); rclpy.shutdown()
main()
EOF
  timeout 600 python3 /tmp/esdf_dummy_pub.py > /tmp/esdf-dummy.log 2>&1 &
  ESDF_PID=$!
  sleep 5  # controller가 소스를 잡을 시간
  echo "ESDF: 빈 점구름 발행 (전방 free 가정)"
fi

GOAL_A="{pose: {header: {frame_id: 'map'}, pose: {position: {x: 0.5, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
GOAL_B="{pose: {header: {frame_id: 'map'}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
drive() {  # $1 = on|off (토글) · $2 = 라벨 · $3 = goal JSON
  local tries=0 ok=""
  while [ $tries -lt 3 ]; do
    if timeout 30 ros2 service call /controller_server/FollowPath/toggle std_srvs/srv/SetBool "{data: $([ "$1" = on ] && echo true || echo false)}" 2>/dev/null | grep -q "success=True"; then
      ok=1
      break
    fi
    tries=$((tries + 1))
    sleep 5
  done
  [ -n "$ok" ] || { echo "FAIL: toggle $1 실패"; return 1; }
  echo "--- [$2] toggle $1 ok, goal 주행 ---"
  timeout 120 ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "$3" > /tmp/nav-goal.log 2>&1 &
  local goal_pid=$!
  # status 한 방: compute가 돌면 무조건 찍힌다 (정책이면 rl_active=1, 바이패스면 0).
  # goal 끝나기 전에 바로 붙어야 해서 대기 없이 실행한다.
  timeout -s KILL 90 ros2 topic echo --once /controller_server/FollowPath/status std_msgs/msg/Float32MultiArray > /tmp/nav-status.log 2>&1
  if grep -q "data:" /tmp/nav-status.log 2>/dev/null; then
    echo "OK: [$2] FollowPath/status 수신"
    head -14 /tmp/nav-status.log | tail -13
    rl_active=$(grep -A1 "^data:" /tmp/nav-status.log | tail -1 | tr -d ' -')
    echo "rl_active=$rl_active (기대: $([ "$1" = on ] && echo 1 || echo 0))"
    if [ "$1" = on ] && [ "${rl_active%%.*}" != 1 ]; then
      echo "FAIL: [$2] 정책 미동작 (rl_active=$rl_active)"
      kill "$goal_pid" 2>/dev/null
      return 1
    fi
    if [ "$1" != on ] && [ "${rl_active%%.*}" != 0 ]; then
      echo "FAIL: [$2] DWB인데 정책 활성 (rl_active=$rl_active)"
      kill "$goal_pid" 2>/dev/null
      return 1
    fi
    echo "OK: [$2] 모드 일치 (rl_active=$rl_active)"
  else
    echo "FAIL: [$2] FollowPath/status 없음"
    kill "$goal_pid" 2>/dev/null
    return 1
  fi
  wait "$goal_pid"
  if grep -qi "succeeded" /tmp/nav-goal.log; then
    echo "OK: [$2] goal 도착"
  elif [ "$1" = on ]; then
    # 정책 leg 도착 실패는 모델 품질 문제일 수 있음 — 인프라는 위에서 검증됨. WARN으로 남기고 계속 간다.
    echo "WARN: [$2] goal 미도착 (정책 주행 품질 확인 필요)"; tail -5 /tmp/nav-goal.log
  else
    echo "FAIL: [$2] goal 미도착 (DWB 기준선 실패)"; tail -5 /tmp/nav-goal.log; return 1
  fi
}

drive off "DWB" "$GOAL_A" || exit 1

drive on "정책" "$GOAL_B" || exit 1

# 정책 ON 복원 확인 (다음 실행·수동 주행이 DWB로 남지 않게)
timeout 10 ros2 service call /controller_server/FollowPath/toggle std_srvs/srv/SetBool "{data: true}" 2>/dev/null \
  | grep -q "success=True" || { echo "FAIL: toggle on 복원 실패"; exit 1; }

echo "=========================="
echo "PASS: nav-policy (ONNX 로드 · toggle off/on · DWB/정책 주행 · rl_active 일치)"
