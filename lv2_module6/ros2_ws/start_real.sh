#!/bin/bash
# 실기 시험 시작용 — bringup의 실제 하드웨어 기본값을 명시해 start.sh로 넘긴다.
#
#   ./start_real.sh                                  # scene_sim 위치추정 + 인지 + Nav2
#   ./start_real.sh navigation_infer_hz:=2.0         # 인자 override 가능
#   ./start_real.sh map:=slam map_name:=new_test      # 새 nvblox 지도 생성
#   ./start_real.sh --build                          # 현재 Jetson에서 빌드 후 실행
#
# 기존 start.sh가 ROS 환경·Isaac ROS overlay·장치 확인·안전 종료를 맡는다.
set -eo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
build_args=()
if [ "${1:-}" = --build ]; then
  build_args=(--build)
  shift
fi

if [ "${1:-}" = -h ] || [ "${1:-}" = --help ]; then
  cat <<'EOF'
Usage: ./start_real.sh [--build] [ROS launch arguments...]

Defaults: mock_hardware=false, fake_slam=false, real cameras, Nav2,
          map:=saved map_name:=scene_sim, perception:=true,
          use_pole_back_cam:=false. gripper_cam uses auto detection and needs
          GRIPPER_CAM_SERIAL (or gripper_cam_serial:=...) for a manipulation test.
Any explicitly supplied launch argument overrides its default.
EOF
  exit 0
fi

launch_args=("$@")
has_arg() {
  local key="$1" arg
  shift
  for arg in "$@"; do
    case "$arg" in
      "$key":=*) return 0 ;;
    esac
  done
  return 1
}
default_arg() {
  local key="$1" value="$2"
  if ! has_arg "$key" "${launch_args[@]}"; then
    launch_args+=("$key:=$value")
  fi
}

default_arg mock_hardware false
default_arg fake_slam false
default_arg use_ros_system true
default_arg use_cameras true
default_arg use_nav2 true
default_arg use_pole_back_cam false
default_arg map_backend nvblox
default_arg map_dir "$HERE/map"

map_mode=saved
map_was_set=false
for arg in "${launch_args[@]}"; do
  case "$arg" in
    map:=*) map_mode="${arg#map:=}"; map_was_set=true ;;
  esac
done
if [ "$map_was_set" = false ]; then
  default_arg map saved
fi
if [ "$map_mode" = saved ]; then
  default_arg map_name scene_sim
  default_arg perception true
fi

# start.sh/bringup은 지도 파일이 없으면 지도 없이 계속 뜰 수 있다. 실기 시험 시작 스크립트에서는
# 저장 위치추정 지도 누락을 먼저 알려 잘못된 상태에서 미션을 시작하지 않게 한다.
map_name=scene_sim
map_dir="$HERE/map"
map_backend=nvblox
use_ros_system=true
for arg in "${launch_args[@]}"; do
  case "$arg" in
    map_name:=*) map_name="${arg#map_name:=}" ;;
    map_dir:=*) map_dir="${arg#map_dir:=}" ;;
    map_backend:=*) map_backend="${arg#map_backend:=}" ;;
    use_ros_system:=*) use_ros_system="${arg#use_ros_system:=}" ;;
  esac
done
map_dir="${map_dir/#\~/$HOME}"
if [ -z "$map_dir" ]; then map_dir="$HOME/.ros/beebot_maps"; fi
if [ "$map_mode" = saved ] && [ "$use_ros_system" = true ] && [ -n "$map_name" ]; then
  map_file=nvblox.nvblx
  [ "$map_backend" = static_2d ] && map_file=map.yaml
  if [ ! -f "$map_dir/$map_name/$map_file" ]; then
    echo "FAIL: 실기 저장 지도 없음: $map_dir/$map_name/$map_file"
    echo "      지도 확인: $HERE/../../tools/run-cli map list · 다른 지도: $HERE/start_real.sh map:=saved map_name:=<이름>"
    exit 1
  fi
fi

exec env LAUNCH_FILE=bringup.launch.py "$HERE/start.sh" "${build_args[@]}" "${launch_args[@]}"
