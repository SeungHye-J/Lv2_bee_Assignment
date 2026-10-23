# OpenCR 펌웨어 (beebot_opencr)

OpenCR 하나가 TurtleBot3 바퀴 2개, OpenMANIPULATOR-X(RM-X52-TNM) 4축 + 손목 roll + 그리퍼, 내장 IMU를 맡는다.
SBC(라즈베리파이·Jetson 등 로봇에 실린 컴퓨터)의 `control/BeebotSystemHardware`(ros2_control 하드웨어 플러그인)와 USB 시리얼로 짝을 이룬다.

```text
SBC /dev/opencr (udev → ttyACM*) ── USB CDC, ASCII 줄 ──> OpenCR USB Serial
OpenCR Serial3 ── DYNAMIXEL Protocol 2.0, 1,000,000 bps ──> 바퀴 ID 1·2 · 팔 ID 11~15 · 그리퍼 ID 16
OpenCR 내장 IMU (MPU9250, cIMU)
```

## 프로토콜

단위는 SI, 관절 0rad = 서보 중앙(2048 = 180°). 바꾸면 `ros2_ws/src/control/src/beebot_system_hardware.cpp`도 같이 바꾼다 (`test-hardware`가 확인).

| 방향 | 형식 | 주기 |
| --- | --- | --- |
| SBC → OpenCR | `C,<바퀴L rad/s>,<바퀴R rad/s>,<joint1>,<joint2>,<joint3>,<joint4>,<joint5 rad>,<그리퍼 m>\n` (8개) | 제어 주기 (100Hz) |
| OpenCR → SBC | `S,` 바퀴L pos·vel, 바퀴R pos·vel, joint1~5 pos·vel·전류A, 그리퍼 m·m/s·A, IMU qx·qy·qz·qw, gx·gy·gz rad/s, ax·ay·az m/s² `\n` (32개) | `STATE_PERIOD_MS` (25ms) |

- watchdog: 마지막 **유효한** `C`부터 `COMMAND_TIMEOUT_MS`(500ms)가 지나면 바퀴 속도 0, 팔·그리퍼는 현재 자세 유지 (한 번).
  형식이 틀린 줄·빈 줄은 무시하고 타이머도 갱신하지 않는다.
- 부팅 때 모터 하나라도 ping이 안 되면 `C`를 받지 않는다 (상태 `S`는 계속 보내 SBC가 원인을 볼 수 있게).
- 그리퍼는 전류 기반 위치 모드 — 퍽을 쥐면 전류 한계에서 멈춘다.

## 준비

- 모터 ID (DYNAMIXEL Wizard로 미리): 바퀴 왼쪽 **1** · 오른쪽 **2**, 팔 joint1~4 **11~14**, 손목 roll **15**, 그리퍼 **16**
  (OM-X 기본은 그리퍼가 15 — 손목을 끼우면서 16으로 옮긴다). 전부 1,000,000 bps · Protocol 2.0.
- `config.h`에서 바퀴 방향, 관절 부호(`ARM_DIR`), 그리퍼 변환(`GRIPPER_RAD_PER_M`)을 실기에서 확인한다 (TODO).

## 설치·컴파일·업로드

스크립트는 단계별로 나뉜다.

| 스크립트 | 역할 |
| --- | --- |
| `setup.sh` | arduino-cli·OpenCR 코어·컴파일러·업로더 설치 (최초 1회) |
| `compile.sh [스케치]` | 컴파일만 → `build/<스케치>/*.ino.bin` |
| `upload.sh [포트] [파일]` | `.bin`/`.opencr` 업로드 (기본: 최신 `.bin`, `/dev/opencr`) |
| `setup_compile_upload.sh [포트] [스케치]` | 설치 + 컴파일 + 업로드를 한 번에 |

```bash
# 최초 1회 또는 툴이 바뀌었을 때
./setup.sh

# 컴파일만
./compile.sh
./compile.sh beebot_opencr

# 업로드만 (기본 포트 /dev/opencr → 없으면 /dev/ttyACM0)
./upload.sh
./upload.sh /dev/ttyACM0
./upload.sh /dev/ttyACM0 build/beebot_opencr/beebot_opencr.ino.bin

# 설치부터 한 번에
./setup_compile_upload.sh /dev/ttyACM0 beebot_opencr
```

CI 배포 패키지에서 미리 컴파일된 `.bin`을 올릴 때:

```bash
../ros2_ws/firmware-upload.sh --setup
../ros2_ws/firmware-upload.sh /dev/ttyACM0
```

업로드 중에는 ROS(`start.sh`, controller_manager)를 끈다 — 같은 포트를 잡으면 업로드가 깨진다.
안 되면 OpenCR의 SW2를 누른 채 RESET → 부트로더 모드에서 다시. 공통 경로·업로더 설정은 [common.sh](common.sh)를 본다.

## 실기 시리얼 테스트

업로드 후 ROS 없이 OpenCR의 `S` 상태와 `C` 명령을 확인한다.  
스크립트는 `test-firmware`이다.

### 사용법

```bash
cd lv2_module6/firmware

chmod +x test-firmware          # 최초 1회 (Permission denied 방지)

./test-firmware                  # S 상태만 읽음 — 로봇 안 움직임
./test-firmware --raw            # 받은 S 줄 원문 출력
./test-firmware --port /dev/ttyACM0

./test-firmware --hold --yes     # 현재 자세 유지 명령
./test-firmware --arm joint1 0.05 --yes
./test-firmware --gripper 0.005 --yes
./test-firmware --wheel 0.3 --yes       # 양쪽 바퀴 0.3 rad/s (팔·그리퍼 현재 유지)
./test-firmware --l-wheel 0.3 --yes     # 왼쪽만 (오른쪽 정지)
./test-firmware --r-wheel 0.3 --yes     # 오른쪽만 (왼쪽 정지)
./test-firmware --send 'C,0,0,0,0,0,0,0,0' --yes
./test-firmware --send 'C,0.3,0.3,,,,,,' --yes  # 빈칸 = 유지 (아래 규칙)
```

`sh ./test-firmware`로 실행하면 Ubuntu의 `/bin/sh`(dash)가 되어
`set: Illegal option -o pipefail` 오류가 난다. 반드시 `./` 또는 `bash`로 실행한다.

```bash
./test-firmware
# 또는
bash ./test-firmware
```

### 옵션

| 옵션 | 의미 |
| --- | --- |
| `--port <포트>` | 기본 `/dev/opencr`, 없으면 `/dev/ttyACM0` |
| `--raw` | `S` 줄 원문 출력 |
| `--lines <n>` | `--raw`에서 보여줄 줄 수 (기본 3) |
| `--hold` | 현재 자세를 유지하는 `C` 명령 전송 |
| `--arm <joint> <rad>` | 현재 자세에서 해당 관절만 delta만큼 이동 |
| `--gripper <m>` | 그리퍼 폭을 m로 이동 |
| `--wheel <vl> [vr]` | 바퀴 속도 rad/s (vr 생략 시 양쪽 동일, 팔·그리퍼는 S 현재값 유지) |
| `--l-wheel <v>` / `--r-wheel <v>` | 한쪽 바퀴만 속도 (반대쪽은 정지) |
| `--count <n>` | `C` 전송 횟수, 1회 = 0.05초 (기본: `--hold`·`--send` 5회, `--arm`·`--gripper`·바퀴 20회, 최대 1200) |
| `--send 'C,...'` | `C` 명령을 직접 전송 (빈 필드 `,,`는 유지 — 아래 규칙) |
| `--yes` | 확인 프롬프트 없이 실행 |

- `--arm`, `--gripper`, `--wheel`, `--send`는 실제로 모터를 움직인다. 주변을 비우고 작은 값부터 사용한다.
- `C` 빈 필드는 **유지**다: 팔·그리퍼는 현재 목표 유지, 바퀴는 정지(0). 전부 빈칸이면 바퀴 정지 + 팔 유지. 예: `C,0.3,0.3,,,,,,` (바퀴만 구동), `C,1,,0.4,,,,,` (좌바퀴 1 · 우 정지 · joint1 0.4).
- 필드가 모자라면 **뒷부분을 빈칸(유지)으로 채운다** — `C,0.3,0.3`만 보내도 바퀴만 돌고 팔은 유지. 8개를 넘거나 숫자 뒤에 쓰레기가 있으면 줄 전체를 버린다. `--send`도 같은 규칙으로 자동 채운다.
- `--send 'C,0,0,...'`처럼 전부 숫자로 보내면 팔을 0 rad(중앙)으로 보내는 명령이므로, 현재 자세 유지는 `--hold`나 빈칸을 쓴다.
- 모터 8개(바퀴 1·2, 팔 11~15, 그리퍼 16)가 모두 1 Mbps / Protocol 2.0으로 연결되어 있지 않으면 `C`가 무시된다.
- 그리퍼는 Current-based Position의 Goal Current가 0이면 움직이지 않을 수 있다. (`setGoalCurrent()` 확인)

### 결과 해석

```text
S 32개 필드                 → beebot_opencr 정상
S 필드가 9개/8개            → 다른 펌웨어이거나, 시리얼 timeout으로 마지막 줄이 잘린 경우
S가 안 나옴                 → 포트·업로드·권한 문제
S만 나오고 C가 무시됨       → 모터 일부 미연결 또는 ID/baud 오류
```

> 시리얼을 timeout으로 끊으면 마지막 `S` 줄이 중간에서 잘릴 수 있다.
> `test-firmware`는 **32개 필드가 모두 있는 완전한 줄만** 사용하므로, 중간에 잘린 줄은 무시한다.

`S` 필드가 8~9개로 나오는 경우:

```bash
./test-firmware --raw
# 또는
stty -F /dev/ttyACM0 raw -echo 115200
timeout 2 cat /dev/ttyACM0 | head -3
```

`S,` 뒤 값이 8개뿐이면 `beebot_opencr`의 32값 프로토콜이 아니다.  
OpenCR에 올라간 펌웨어를 확인하고 `beebot_opencr`을 다시 업로드한다.

```bash
./setup_compile_upload.sh /dev/ttyACM0 beebot_opencr
```
