#!/bin/bash
# 더미 전체 흐름 검사 (컨테이너 안에서 실행): test-faker [초|0] [launch 인자...]
#   faker launch = bringup(🛠 노드) + ros_system(controller_manager mock·EKF) + 대역(📦) + 파일 연동 → user_input 명령 한 번
#   /task_status 에 SUCCEEDED(state 1)가 오고 /robot_say 가 나오고,
#   controller_manager(mock)가 /joint_states(팔 5축 + 그리퍼)·EKF가 /odometry/filtered 를 내면 PASS
#   초: SUCCEEDED 뒤 더 돌릴 시간 (기본 1). 0이면 Ctrl+C까지 띄워 둠 (판정 없음)
#   예) test-faker                       정상 흐름
#       test-faker 1 pick_fail_count:=2   파지 실패 → 재계획 → 성공
#       test-faker 0                      띄워 두고 다른 터미널에서 ros2 topic echo …
# 실행 중엔 test-logger를 같이 띄워 /ws/debug/topic 에 토픽 echo를 기록한다.
# NOTE: set -u 사용 금지 — setup.bash가 미설정 변수를 참조해서 죽음.
# shellcheck disable=SC1091
source /opt/ros/lyrical/setup.bash
cd /ws || { echo "FAIL: /ws 없음"; exit 2; }

colcon build --symlink-install --event-handlers console_direct- || { echo "FAIL: colcon build"; exit 1; }
# shellcheck disable=SC1091
source install/setup.bash

KILL_PAT='lib/(perception|plan|ml|control|faker|controller_manager|robot_state_publisher|robot_localization)/'
pkill -f "$KILL_PAT" 2>/dev/null || true

DUR=${1:-1}
shift 2>/dev/null
set -m
ros2 launch faker faker.launch.py "$@" > /tmp/faker.log 2>&1 & LAUNCH_PID=$!
# controller_manager가 컨트롤러를 다 올린 뒤에 logger를 붙인다 — logger가 토픽마다 ros2 CLI를 띄우는 순간과
# 컨트롤러 spawn이 겹치면 CPU가 몰려 spawner 서비스 호출이 시간 초과된다
JS=$(timeout 60 ros2 topic echo --once /joint_states sensor_msgs/msg/JointState 2>/dev/null)
test-logger 0 > /tmp/test-logger.log 2>&1 & LOGGER_PID=$!

stop_all() {
  kill "$LOGGER_PID" 2>/dev/null
  timeout 10 tail --pid="$LOGGER_PID" -f /dev/null
  kill -INT "$LAUNCH_PID" 2>/dev/null
  timeout 15 tail --pid="$LAUNCH_PID" -f /dev/null
  pkill -9 -f "$KILL_PAT" 2>/dev/null || true
}

if [ "$DUR" = 0 ]; then
  echo "실행 중 (로그 /tmp/faker.log)... 종료는 Ctrl+C"
  trap 'echo' INT
  wait "$LAUNCH_PID" 2>/dev/null
  trap '' INT
  stop_all
  exit 0
fi

# 타입을 명시해야 /task_status가 아직 안 생겼을 때 바로 끝나지 않고 기다림. transient_local로 받아야 놓친 것도 옴
DONE=$(timeout 60 ros2 topic echo --once --filter 'm.state == 1' \
  --qos-reliability reliable --qos-durability transient_local /task_status plan/msg/TaskStatus 2>/dev/null)
ODOM=$(timeout 10 ros2 topic echo --once /odometry/filtered nav_msgs/msg/Odometry 2>/dev/null)
sleep "$DUR"
stop_all

echo "--- 흐름 (faker.log) ---"
grep -E '\] \[(user_input|vlm_task_planner|coordinator|grasp_scorer|rl_nav_tuner|controller_server|move_group|arm_safety_monitor|spawner_[a-z_]+)\]' /tmp/faker.log \
  | sed -E 's/^\[[^]]+\] //' | head -60
[ -n "$DONE" ] || { echo "FAIL: /task_status SUCCEEDED 없음 (60초)"; tail -n 40 /tmp/faker.log; exit 1; }
grep -q '🔊' /tmp/faker.log || { echo "FAIL: /robot_say 없음"; exit 1; }
echo "$JS" | grep -q gripper_left_joint || { echo "FAIL: /joint_states에 gripper_left_joint 없음 (controller_manager·BeebotSystemHardware)"; grep -E "controller_manager|spawner|ros2_control" /tmp/faker.log | tail -20; exit 1; }
[ -n "$ODOM" ] || { echo "FAIL: /odometry/filtered 없음 (EKF ← /odom·/imu)"; exit 1; }
[ -s /ws/debug/topic/task_status/message ] || { echo "FAIL: monitor_manager 파일 없음 (/ws/debug/topic/task_status/message)"; exit 1; }
echo "토픽 기록: /ws/debug/topic (호스트 lv2_module6/debug/topic)"
echo "PASS: faker (user_input → vlm → coordinator → nav·grasp·mtc → task_status SUCCEEDED · controller_manager mock · EKF · monitor_manager)"
