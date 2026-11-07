#!/bin/bash
# Nav2 주행 정책(rl_nav_controller ONNX) faker 검증 (컨테이너 안에서 실행): test-nav-policy
#   faker launch (course 맵 · 진짜 Nav2) → ONNX 로드 확인 → toggle off/on →
#   제자리 근처 goal로 DWB·정책 주행 → /cmd_vel_nav · FollowPath/debug 확인
#   예) test-nav-policy
# NOTE: set -u 사용 금지 — setup.bash가 미설정 변수를 참조해서 죽음.
if [ -z "$BASH_VERSION" ]; then exec bash "$0" "$@"; fi   # sh(dash)로 실행해도 bash로 재실행
SMOKE=0
if [ "${1:-}" = smoke ]; then SMOKE=1; shift; fi   # CI용: 배선 확인까지만, goal 완주 대기 안 함
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
case " $* " in
  *\ course:=*) : ;;
  *) set -- "$@" "course:=twin_closed_maze" ;;   # Nav2 정책 시험은 옛 미로 (기본값은 dice_maze)
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

# run-cli ab guide로 A/B 전체 수행 (토글·녹화·주행·리포트 한 번에)
GUIDE_LABEL="ci_$(date +%Y%m%d_%H%M%S)"
python3 /ws/tool/cli/operator_tool.py ab rl_nav_controller guide --label "$GUIDE_LABEL" || { echo "FAIL: ab guide 실패"; exit 1; }
echo "OK: ab guide 완료 (A=정책, B=DWB)"

# 리포트 확인 (guide가 BAG_DIR/report_<ts>.png 생성)
REPORT=$(ls -t /debug/tool/artifacts/ab/*/report_*.png 2>/dev/null | head -1)
[ -n "$REPORT" ] || { echo "FAIL: 리포트 없음"; exit 1; }
echo "리포트: $REPORT"

# 정책 leg bag에 FollowPath/debug가 있어야 실제 정책이 돈 것 (DWB 바이패스 아님)
ABAG=$(ls -td /debug/tool/artifacts/ab/*_r1_A /debug/tool/artifacts/ab/*_A 2>/dev/null | head -1)
[ -n "$ABAG" ] || { echo "FAIL: 정책 bag 없음"; exit 1; }
DCNT=$(python3 - "$ABAG" <<'PYEOF'
import sys
from pathlib import Path
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
uri = sys.argv[1]
sid = "mcap" if list(Path(uri).glob("*.mcap")) else "sqlite3"
r = SequentialReader()
r.open(StorageOptions(uri=uri, storage_id=sid), ConverterOptions("", ""))
n = 0
while r.has_next():
    topic, _, _ = r.read_next()
    if "FollowPath/debug" in topic:
        n += 1
print(n)
PYEOF
)
[ "${DCNT:-0}" -gt 0 ] 2>/dev/null || { echo "FAIL: 정책 bag에 debug 없음 (바이패스 의심)"; exit 1; }
echo "OK: 정책 bag에 debug 토픽 $DCNT개 (정책 실제 동작)"

echo "=========================="
echo "PASS: nav-policy (run-cli ab guide · 리포트 · debug 확인)"
