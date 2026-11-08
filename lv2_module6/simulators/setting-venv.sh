#!/usr/bin/env bash
# setting-venv.sh — simulators/.venv 만들기 + vazebo/requirements.txt 설치. 처음 한 번 (requirements가 바뀌면 다시).
#   ./setting-venv.sh              # 없으면 만들고 설치
#   ./setting-venv.sh --recreate   # 기존 .venv를 지우고 새로
#
# 시뮬레이터는 ROS를 쓰지 않는다 — 호스트 python3로 만든다 (PYTHON=...로 다른 인터프리터 지정 가능).
set -eo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
VENV="$HERE/.venv"
PY="${PYTHON:-python3}"

if [ "${1:-}" = "--recreate" ]; then
  echo "기존 venv 삭제: $VENV"
  rm -rf "$VENV"
fi

if [ ! -x "$VENV/bin/python" ]; then
  echo "venv 만들기: $VENV ($("$PY" --version 2>&1))"
  "$PY" -m venv "$VENV" || { echo "FAIL: venv 생성 실패 — python3-venv 패키지 확인" >&2; exit 1; }
fi

"$VENV/bin/python" -m pip install --upgrade pip -q
"$VENV/bin/python" -m pip install -r "$HERE/vazebo/requirements.txt"
"$VENV/bin/python" - <<'PY'
import cv2
import numpy
import torch
print(f"  opencv {cv2.__version__} · numpy {numpy.__version__} · torch {torch.__version__}")
PY
echo "OK: $VENV — simulators/vazebo/main.py가 이걸 씁니다"
