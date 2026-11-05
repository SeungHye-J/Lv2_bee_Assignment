#!/usr/bin/env bash
# 집기 시연 환경 + ROS (SLAM·Nav2 없음): 바닥 · 시작 상자 · 퍽 2개 · 상자에서 30 cm 로봇 (manipulator_env.py)
#   isaac/scripts/demo/pick_load_demo.sh · 시연: python3 -u isaac/scripts/demo/pick_load_play.py --clear-basket · 리셋: isaac/scripts/standalone/reset.sh
#   인자는 manipulator_env.py로 (--headless 등)
STANDALONE=$(dirname "$(realpath "$0")")/../standalone
# YOLO: ~/.venv-yolo의 torch·ultralytics만 얹음 — numpy·cv2는 시스템 것 (cv_bridge가 numpy 1.x, isaac_sim.sh YOLO_VENV 주석)
VENV=${YOLO_SOURCE_VENV:-$HOME/.venv-yolo}/lib/python3.12/site-packages
OVERLAY=$HOME/.local/share/beebot/yolo_overlay
if [ ! -d "$OVERLAY" ] && [ -d "$VENV" ]; then
  mkdir -p "$OVERLAY/lib/python3.12/site-packages"
  for f in "$VENV"/*; do
    case "$(basename "$f")" in
      numpy|numpy-*|numpy.libs|cv2|opencv*|__pycache__|pip|pip-*|setuptools*) ;;
      *) ln -s "$f" "$OVERLAY/lib/python3.12/site-packages/" ;;
    esac
  done
fi
export YOLO_VENV=$OVERLAY
export ISAACSIM_VENV=${MANIPULATOR_DEMO_ISAAC_VENV:-$HOME/.venv-isaacsim}  # ~/.bashrc의 ISAACSIM_VENV는 Isaac Lab용 — 지금까지 시연은 이 venv
export ISAAC_ROS_LAUNCH=isaac_manipulator_demo.launch.py
export ISAAC_STANDALONE="$STANDALONE/manipulator_env.py"
export ISAAC_STANDALONE_ARGS="$*"
exec "$STANDALONE/isaac_sim.sh"
