#!/bin/bash
# test-logger: 지금 떠 있는 ROS 그래프를 파일로 기록 (컨테이너 안에서 실행). launch는 안 함.
#   test-logger [초]   (기본 10초, 0이면 Ctrl+C까지 계속 기록)
# 띄우기는 따로: test-faker 0 (또는 ros2 launch faker faker.launch.py)
# 결과 (사람용 로그 — sim 실시간값은 monitor_manager가 output/topic/에):
#   output/topic/{토픽명}/echo   토픽 스트림 (계속 append)
#   topic/{토픽명}/info          ros2 topic info (pub/sub 상세, 첫 발견 때 1회)
#   topic/{토픽명}/type          ros2 topic type (첫 발견 때 1회)
#   topic/list                   ros2 topic list (2초마다 갱신)
#   node/list                    ros2 node list (2초마다 갱신)
#   node/{노드명}/info           ros2 node info (새 노드마다 1회, 백그라운드)
#   node/{노드명}/params         ros2 param dump (새 노드마다 1회, 백그라운드)
#   pkg/list                     ros2 pkg list (2초마다 갱신)
#   pkg/{패키지명}/prefix        ros2 pkg prefix (새 패키지마다 1회, 백그라운드)
#   pkg/{패키지명}/executables   ros2 pkg executables (새 패키지마다 1회, 백그라운드)
# 첫 토픽이 보일 때까지 최대 30초 기다리고, 기록 중에도 2초마다 새로 생긴 것을 추가한다.
# NOTE: set -u 사용 금지 — setup.bash가 미설정 변수를 참조해서 죽음.
# shellcheck disable=SC1091
source /opt/ros/lyrical/setup.bash
# shellcheck disable=SC1091
[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash

DUR=${1:-10}
OUT_ECHO=/debug/output/topic
OUT_INFO=/debug/topic
OUT_NODE=/debug/node
OUT_PKG=/debug/pkg
# logger가 쓰는 것만 지움 (monitor의 message·image.jpg는 건드리지 않음).
# info·nodes·topics는 예전 위치(output/topic/)에 남아 있을 수 있어서 같이 치움.
rm -rf "$OUT_INFO" "$OUT_NODE" "$OUT_PKG"
mkdir -p "$OUT_ECHO" "$OUT_INFO" "$OUT_NODE" "$OUT_PKG"
find "$OUT_ECHO" -type f \( -name echo -o -name info \) -delete
rm -f "$OUT_ECHO/nodes" "$OUT_ECHO/topics"

# /parameter_events·/rosout은 노드만 있으면 항상 있으니 제외. 숨김(/_*)도 제외.
user_topics() {
  ros2 topic list 2>/dev/null | grep -vxE '/parameter_events|/rosout|/_.*'
}

user_nodes() {
  ros2 node list 2>/dev/null | grep -vxE '/_*'
}

for _ in $(seq 30); do
  [ -n "$(user_topics)" ] && break
  sleep 1
done
[ -n "$(user_topics)" ] || { echo "FAIL: 기록할 토픽 없음 (먼저 test-faker 0 등으로 띄울 것)"; exit 1; }

STOP=0
trap 'STOP=1' INT TERM
declare -A ECHO_PIDS
declare -A INFO_PIDS
END=$((SECONDS + DUR))
[ "$DUR" = 0 ] || [ "$DUR" = inf ] && END=-1 && echo "기록 중... 종료는 Ctrl+C"

while [ "$STOP" = 0 ]; do
  # 타입까지 한 번에 (토픽마다 ros2 CLI를 더 띄우면 기동 직후 CPU가 몰린다)
  TYPED=$(ros2 topic list -t 2>/dev/null)
  for topic in $(user_topics); do
    [ -n "${ECHO_PIDS[$topic]}" ] && continue
    # 영상·점구름·고주파 통계는 echo하지 않는다 — 원본을 파이썬으로 역직렬화하면 CPU를 다 먹어 다른 노드가 밀린다.
    # (최신값은 faker monitor_manager가 output/topic/<토픽>/message로 남긴다)
    if echo "$TYPED" | grep -qE "^$topic \[sensor_msgs/msg/(Image|PointCloud2)\]"; then
      ECHO_PIDS[$topic]=skip
      continue
    fi
    case "$topic" in
      */statistics/*|*/introspection_data/*|*/controller_state)
        ECHO_PIDS[$topic]=skip
        continue
        ;;
    esac
    ed="$OUT_ECHO/${topic#/}"
    id="$OUT_INFO/${topic#/}"
    mkdir -p "$ed" "$id"
    ros2 topic info -v "$topic" > "$id/info" 2>&1
    ros2 topic type "$topic" > "$id/type" 2>&1
    # --truncate-length: 긴 배열(영상 픽셀)·문자열은 앞 16개만 → echo 파일이 초당 MB로 안 커짐.
    # --no-arr는 짧은 배열(JointState position 등)까지 지워서 echo에서 값이 안 보임
    ros2 topic echo --truncate-length 16 "$topic" >> "$ed/echo" 2>&1 &
    ECHO_PIDS[$topic]=$!
    echo "+ $topic"
  done
  ros2 topic list > "$OUT_INFO/list" 2>&1
  for node in $(user_nodes); do
    if [ -z "${INFO_PIDS[node:$node]}" ]; then
      nd="$OUT_NODE/${node#/}"
      mkdir -p "$nd"
      { ros2 node info "$node" > "$nd/info" 2>&1; ros2 param dump "$node" > "$nd/params" 2>&1; } &
      INFO_PIDS[node:$node]=$!
      echo "+ node $node"
    fi
  done
  ros2 node list > "$OUT_NODE/list" 2>&1
  for pkg in $(ros2 pkg list 2>/dev/null); do
    if [ -z "${INFO_PIDS[pkg:$pkg]}" ]; then
      pd="$OUT_PKG/$pkg"
      mkdir -p "$pd"
      { ros2 pkg prefix "$pkg" > "$pd/prefix" 2>&1; ros2 pkg executables "$pkg" > "$pd/executables" 2>&1; } &
      INFO_PIDS[pkg:$pkg]=$!
      echo "+ pkg $pkg"
    fi
  done
  ros2 pkg list > "$OUT_PKG/list" 2>&1
  [ "$END" -ge 0 ] && [ "$SECONDS" -ge "$END" ] && break
  sleep 2
done

# echo 종료: TERM 후 최대 3초 기다리고 남으면 강제 종료
# (ros2 topic echo가 TERM을 받고도 가끔 안 꺼짐 → 그냥 wait하면 logger가 고아로 남음)
for k in "${!ECHO_PIDS[@]}"; do [ "${ECHO_PIDS[$k]}" = skip ] && unset "ECHO_PIDS[$k]"; done
kill "${ECHO_PIDS[@]}" "${INFO_PIDS[@]}" 2>/dev/null
for _ in $(seq 10); do
  alive=0
  for pid in "${ECHO_PIDS[@]}" "${INFO_PIDS[@]}"; do kill -0 "$pid" 2>/dev/null && alive=1; done
  [ "$alive" = 0 ] && break
  sleep 0.3
done
kill -9 "${ECHO_PIDS[@]}" "${INFO_PIDS[@]}" 2>/dev/null
wait 2>/dev/null
echo "--- dump ---"
find "$OUT_ECHO" "$OUT_INFO" "$OUT_NODE" "$OUT_PKG" -type f | sort
echo "DONE: test-logger (${DUR}s)"
