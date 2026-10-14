#!/bin/bash
# OpenCR 펌웨어 업로드 (Raspberry Pi arm64 / x86_64)
#
#   ./firmware-upload.sh --setup         # 최초 1회: 업로더 설치 (sudo 필요)
#   ./firmware-upload.sh [포트]          # 업로드. 포트 기본: /dev/opencr, 없으면 /dev/ttyACM0
#
# 업로드할 펌웨어:
#   - CI 배포 패키지 (./firmware/ 가 있음): 미리 컴파일된 firmware/build/<스케치>/*.ino.bin 을 업로드
#   - 저장소 (../firmware/ 사용): ../firmware/upload.sh 로 컴파일 + 업로드
#     (그 경우 최초 1회 ../firmware/upload.sh --setup 필요)
#   배포 패키지에서도 다시 컴파일하려면: ./firmware/upload.sh --setup && ./firmware/upload.sh [포트]
#
# 업로드 방식은 ../firmware/upload.sh와 같다:
#   ROBOTIS opencr_ld_shell로 .bin → .opencr 변환 후 업로드 (arm64는 opencr_ld_shell_arm, libc6:armhf 필요)
set -euo pipefail

OPENCR_UPDATE_URL=https://github.com/ROBOTIS-GIT/OpenCR-Binaries/raw/master/turtlebot3/ROS2/latest/opencr_update.tar.bz2
TOOLS_DIR="$HOME/.local/share/opencr_update"   # ../firmware/upload.sh 와 같은 위치
HERE="$(cd "$(dirname "$0")" && pwd)"
SUDO=$([ "$(id -u)" = 0 ] || echo sudo)

case "$(uname -m)" in
  aarch64 | armv7l) ARCH=arm; LD_SHELL="$TOOLS_DIR/opencr_ld_shell_arm" ;;
  x86_64) ARCH=x86; LD_SHELL="$TOOLS_DIR/opencr_ld_shell_x86" ;;
  *) echo "지원 안 하는 아키텍처: $(uname -m)"; exit 1 ;;
esac

setup() {
  # 미리 컴파일된 .bin만 올리므로 업로더만 설치한다 (arduino-cli·컴파일러 불필요)
  local pkgs=(curl ca-certificates bzip2)
  if [ "$ARCH" = arm ]; then
    # 64비트 Ubuntu에서 32비트 ARM 업로더를 돌리기 위한 armhf libc
    [ "$(uname -m)" = aarch64 ] && $SUDO dpkg --add-architecture armhf
    pkgs+=(libc6:armhf)
  fi
  $SUDO apt-get update
  $SUDO apt-get install -y --no-install-recommends "${pkgs[@]}"

  mkdir -p "$TOOLS_DIR"
  # NOTE: 이름만 .tar.bz2고 내용은 gzip → 파일로 받아 tar가 형식을 판별하게 (xf)
  curl -fsSL "$OPENCR_UPDATE_URL" -o "$TOOLS_DIR/opencr_update.tar"
  tar xf "$TOOLS_DIR/opencr_update.tar" -C "$TOOLS_DIR" --strip-components=1 \
    opencr_update/opencr_ld_shell_arm opencr_update/opencr_ld_shell_x86
  rm -f "$TOOLS_DIR/opencr_update.tar"
  chmod +x "$TOOLS_DIR"/opencr_ld_shell_*
  echo "OK: 업로더 설치 완료 ($LD_SHELL)"
  echo "시리얼 권한이 없으면: sudo usermod -aG dialout \$USER  (재로그인 필요)"
}

default_port() {
  if [ -e /dev/opencr ]; then echo /dev/opencr; else echo /dev/ttyACM0; fi
}

upload_bin() {
  local port=$1 bin=$2
  local name work
  name=$(basename "$bin" .ino.bin)
  name=${name%.bin}

  [ -x "$LD_SHELL" ] || { echo "업로더 없음 → 먼저 $0 --setup"; exit 1; }
  [ -e "$port" ] || { echo "포트 없음: $port (ls /dev/ttyACM* 로 확인, USB 연결?)"; exit 1; }
  # ROS의 controller_manager(BeebotSystemHardware)가 같은 포트를 잡고 있으면 업로드가 깨진다
  if pgrep -f ros2_control_node >/dev/null; then
    echo "ros2_control_node(controller_manager) 실행 중 → 포트 충돌. ROS(start.sh)를 먼저 종료하세요."; exit 1
  fi

  work=$(mktemp -d)
  trap 'rm -rf "$work"' EXIT
  cp "$bin" "$work/$name.bin"

  echo "=== .bin → .opencr ($bin)"
  (cd "$work" && "$LD_SHELL" make "$name.bin" "$name" "V$(date +%y%m%d%H%M)")

  echo "=== upload: $name.opencr → $port"
  # 보드가 응답 안 하면 opencr_ld_shell은 무한 대기 → 시간 제한 (정상 업로드는 수 초)
  timeout 120 "$LD_SHELL" "$port" 115200 "$work/$name.opencr" 1 || {
    echo "FAIL: 업로드 실패/응답 없음 ($port)"
    echo "  - 포트가 OpenCR가 맞는지 (ls -l /dev/opencr /dev/ttyACM*), 권한(dialout 그룹)"
    echo "  - 안 되면 OpenCR의 SW2(PUSH SW2)를 누른 채 RESET → 부트로더 모드에서 다시 실행"
    exit 1
  }
  echo "OK: 업로드 완료 ($name → $port)"
}

if [ "${1:-}" = --setup ]; then
  setup
  exit 0
fi
case "${1:-}" in
  -h | --help) sed -n '2,14p' "$0"; exit 0 ;;
esac

port=${1:-$(default_port)}
if [ -d "$HERE/firmware" ]; then
  # 배포 패키지: 미리 컴파일된 .bin
  bins=("$HERE"/firmware/build/*/*.ino.bin)
  [ -e "${bins[0]}" ] || { echo "FAIL: 컴파일된 펌웨어 없음 ($HERE/firmware/build/*/*.ino.bin)"; exit 1; }
  [ "${#bins[@]}" = 1 ] || { echo "FAIL: .bin이 여러 개: ${bins[*]}"; exit 1; }
  upload_bin "$port" "${bins[0]}"
elif [ -x "$HERE/../firmware/upload.sh" ]; then
  # 저장소: 스케치를 컴파일해서 업로드 (예전 build/ 결과를 잘못 올리지 않도록 항상 새로 컴파일)
  echo "저장소 스케치를 컴파일해서 업로드 (../firmware/upload.sh)"
  exec "$HERE/../firmware/upload.sh" "$port"
else
  echo "FAIL: 업로드할 펌웨어 없음 ($HERE/firmware/ 또는 ../firmware/upload.sh)"
  exit 1
fi
