#!/bin/bash
set -e
# shellcheck disable=SC1091
source /opt/ros/lyrical/setup.bash
if [ -f /ws/install/setup.bash ]; then
  # shellcheck disable=SC1091
  source /ws/install/setup.bash
fi

# 가상 시리얼 브릿지 (OpenCR 실기 없이 통신 로직 검증용 — module5와 같은 구성)
#   앱(BeebotSystemHardware·업로더) ↔ /dev/ttyV0 ↔ (socat 중계) ↔ /dev/ttyV1 ↔ serial-in/out 파일
#   /dev/ttyV0에 들어온 데이터 → /ws/debug/serial-out 에 append   (호스트가 보낸 C 명령, 업로드된 .bin)
#   /ws/debug/serial-in 에 쓴 데이터 → /dev/ttyV0 으로 전송         (OpenCR 응답 S 상태를 흉내)
rm -f /run/testbed-ready
mkdir -p /ws/debug
# 공유 폴더 권한: 컨테이너(root)와 호스트(일반 사용자)가 서로 만든 파일·폴더를 읽고 써야 함.
# 파일은 644여도 상대가 읽기만 하면 되지만, 디렉토리는 양쪽 다 rename(원자적 쓰기)으로 교체하므로 777 유지.
mkdir -p /ws/debug/input /ws/debug/input-live /ws/debug/topic /ws/debug/service /ws/debug/action \
  /ws/debug/interface /ws/debug/sim
chmod -R 777 /ws/debug
if command -v socat >/dev/null 2>&1; then
  # 시작할 때마다 로그 파일 초기화 (빈 파일 재생성)
  : > /ws/debug/serial-in
  : > /ws/debug/serial-out
  # 컨테이너(root)가 만든 파일이라 호스트 사용자도 쓸 수 있게 (시뮬레이터가 serial-in에 OpenCR 응답을 써 넣음)
  chmod 666 /ws/debug/serial-in /ws/debug/serial-out
  # 1. PTY 쌍 생성 (앱용 /dev/ttyV0, 브릿지용 /dev/ttyV1)
  socat pty,raw,echo=0,link=/dev/ttyV0 pty,raw,echo=0,link=/dev/ttyV1 &
  sleep 1
  # 실기 OpenCR 포트명(URDF port 기본값)도 가상 시리얼로 연결. 실물 보드를 넘겨받았으면 그대로 둠.
  [ -e /dev/ttyACM0 ] || ln -s /dev/ttyV0 /dev/ttyACM0
  # 2. 시리얼 → 파일 (앱이 보낸 데이터를 serial-out에 축적)
  socat -u /dev/ttyV1,raw,echo=0 OPEN:/ws/debug/serial-out,creat,append &
  # 3. 파일 → 시리얼 (serial-in에 추가된 내용을 앱으로 전송)
  # -u 단방향 필수: 양방향이면 이 프로세스가 /dev/ttyV1 읽기를 나눠가져 serial-out이 유실됨
  tail -c +1 -F /ws/debug/serial-in 2>/dev/null | socat -u - /dev/ttyV1,raw,echo=0 &
fi
# 준비 완료 표시: docker run -d 직후 바로 exec하는 쪽(CI)은 이 파일이 생길 때까지 기다린다
touch /run/testbed-ready

exec "$@"
