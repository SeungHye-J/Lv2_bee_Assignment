# Arduino CLI / OpenCR

OpenCR에 Arduino CLI로 예제를 업로드하고, Dynamixel 버스를 패스스루로 열거나 ID를 변경하는 방법을 정리합니다.

- DYNAMIXEL Wizard 사용법: [`../dynamixel-wizard2/README.md`](../dynamixel-wizard2/README.md)
- 대상 로봇 펌웨어: `lv2_module6/firmware/beebot_opencr/`
- DYNAMIXEL ID 기준: `lv2_module6/firmware/beebot_opencr/config.h`

## 현재 이 환경의 상태

| 항목 | 상태 |
| --- | --- |
| Arduino CLI | `/home/pank/install/arduino-cli/arduino-cli` |
| OpenCR 보드 코어 | `OpenCR:OpenCR 1.5.3` |
| OpenCR 포트 | `/dev/ttyACM0` |
| 32비트 런타임 | 설치됨 (`libc6:i386`, `libstdc++6:i386`, `libgcc-s1:i386`) |
| `dialout` 그룹 | `/etc/group`에는 등록됨 |
| `usb_to_dxl` 업로드 | **완료** |
| 현재 OpenCR 펌웨어 | `usb_to_dxl` 패스스루 |
| `beebot_opencr` 복구 | **아직 안 함** |

> 현재 OpenCR은 `beebot_opencr`이 아니라 `usb_to_dxl`이 올라간 상태입니다.  
> ID 설정이 끝나면 이 문서의 “원래 펌웨어 복구” 절차로 되돌려야 합니다.

---

## 1. Arduino CLI 준비

### 1-1. 압축 풀기

```bash
mkdir -p "$HOME/install/arduino-cli"
tar -xzf "$HOME/Downloads/arduino-cli_1.5.2-rc.1_Linux_64bit.tar.gz" \
  -C "$HOME/install/arduino-cli"

"$HOME/install/arduino-cli/arduino-cli" version
```

이후 명령에서 사용할 변수:

```bash
ARDUINO_CLI="$HOME/install/arduino-cli/arduino-cli"
FQBN="OpenCR:OpenCR:OpenCR"
PORT="/dev/ttyACM0"
```

### 1-2. OpenCR 보드 코어 설치

```bash
"$ARDUINO_CLI" config init
"$ARDUINO_CLI" config add board_manager.additional_urls \
  https://raw.githubusercontent.com/ROBOTIS-GIT/OpenCR/master/arduino/opencr_release/package_opencr_index.json

"$ARDUINO_CLI" core update-index
"$ARDUINO_CLI" core install OpenCR:OpenCR
"$ARDUINO_CLI" core list
"$ARDUINO_CLI" board listall | grep -i opencr
```

정상 출력:

```text
OpenCR Board    OpenCR:OpenCR:OpenCR
```

### 1-3. 32비트 툴체인 의존성

OpenCR 1.5.3의 `arm-none-eabi-g++`는 **32비트(i386) 바이너리**입니다.  
64비트 Ubuntu에 32비트 런타임이 없으면 다음과 같은 오류가 납니다.

```text
fork/exec .../arm-none-eabi-g++: no such file or directory
```

바이너리는 존재하지만 `/lib/ld-linux.so.2`가 없어서 생기는 오류입니다.

```bash
sudo dpkg --add-architecture i386
sudo apt update
sudo apt install libc6:i386 libstdc++6:i386 libgcc-s1:i386
```

추가로 `cc1plus` 오류가 나오면:

```bash
sudo apt install libncurses5:i386 libtinfo5:i386
```

확인:

```bash
"$HOME/.arduino15/packages/OpenCR/tools/opencr_gcc/5.4.0-2016q2/bin/arm-none-eabi-g++" --version
```

### 1-4. USB 권한

`/dev/ttyACM0`은 보통 `root:dialout` 소유입니다.

```bash
sudo usermod -aG dialout "$USER"
```

**로그아웃 후 다시 로그인**해야 현재 세션에 그룹이 적용됩니다.

확인:

```bash
groups | grep dialout
id
```

`getent group dialout`에는 사용자가 보이는데 `id`에는 안 보이면 세션이 아직 갱신되지 않은 것입니다.

```bash
getent group dialout
# dialout:x:20:pank

id
# groups=... 에 dialout이 없으면 아직 이전 세션
```

로그아웃 없이 현재 터미널에서만 적용하려면:

```bash
newgrp dialout
```

또는 명령 단위로:

```bash
sg dialout -c 'id; ls -l /dev/ttyACM0'
```

> 가능하면 데스크톱 세션을 완전히 로그아웃했다가 다시 로그인하는 것이 가장 확실합니다.

---

## 2. OpenCR에 `usb_to_dxl` 업로드

`usb_to_dxl`은 OpenCR의 USB 시리얼과 Dynamixel 버스를 그대로 연결하는 패스스루 예제입니다.  
이 예제를 올리면 DYNAMIXEL Wizard가 `/dev/ttyACM0`를 통해 모터를 검색할 수 있습니다.

### 2-1. 예제 위치

```bash
SKETCH=$(find "$HOME/.arduino15/packages/OpenCR/hardware" \
  -type d -name usb_to_dxl | head -1)

echo "$SKETCH"
```

### 2-2. 컴파일

```bash
"$ARDUINO_CLI" compile --fqbn "$FQBN" "$SKETCH"
```

정상 예:

```text
Sketch uses 112736 bytes (14%) of program storage space.
```

### 2-3. 업로드

> 업로드하면 OpenCR의 기존 `beebot_opencr` 펌웨어가 지워집니다.  
> 로봇을 움직이지 않는 상태에서 진행하고, ROS/제어 노드는 모두 내려 주세요.

```bash
"$ARDUINO_CLI" upload -p "$PORT" --fqbn "$FQBN" "$SKETCH"
```

현재 세션에 `dialout`이 아직 적용되지 않았다면:

```bash
sg dialout -c "'$ARDUINO_CLI' upload -p '$PORT' --fqbn '$FQBN' '$SKETCH'"
```

정상 업로드 예:

```text
Board Name : OpenCR R1.0
CRC OK ...
[OK] Download
jump_to_fw
New upload port: /dev/ttyACM0 (serial)
```

### 2-4. 스캔 중 OpenCR LED 점멸 의미

`usb_to_dxl`이 올라간 상태에서 Wizard가 스캔하면 OpenCR의 **USER1/USER2 LED**가 깜빡일 수 있습니다. 이는 정상 동작입니다.

- **TX LED 깜빡임**  
  Wizard의 요청이 USB → OpenCR → Dynamixel 버스로 나가고 있음
- **RX LED 깜빡임**  
  Dynamixel → OpenCR → USB 방향으로 응답이 돌아오고 있음

`usb_to_dxl` 예제는 USB 데이터가 있을 때 `tx_led_count = 3`,  
Dynamixel 응답이 있을 때 `rx_led_count = 3`을 설정하고,  
`update_led()`에서 50 ms 간격으로 LED를 토글합니다.

따라서:

- **TX만 깜빡이고 RX는 안 깜빡임**  
  명령은 나가지만 모터 응답이 없음  
  → 모터 12V 전원, Dynamixel 케이블, baud/protocol 확인
- **TX/RX 둘 다 깜빡임**  
  모터가 응답하기 시작한 상태  
  → Wizard에 ID가 나타날 가능성이 높음
- **LED가 전혀 안 깜빡임**  
  Wizard가 포트를 열지 못했거나, `usb_to_dxl`이 올라가 있지 않은 상태

기존 `beebot_opencr` 펌웨어 상태에서는 USB→Dynamixel 패스스루가 아니므로  
Wizard 스캔 중 이런 LED 점멸이 나타나지 않는 것이 정상입니다.

### 2-5. Wizard로 검색

자세한 Wizard 설정은 [`../dynamixel-wizard2/README.md`](../dynamixel-wizard2/README.md)를 참고합니다.

핵심 설정:

```text
Port     : /dev/ttyACM0
Baudrate : 1,000,000
Protocol : 2.0
ID       : 1
```

모터에는 **12V 전원**이 들어와야 합니다. USB 전원만으로는 모터가 응답하지 않을 수 있습니다.

---

## 3. Dynamixel ID 변경

목표 ID:

| 모터 | ID |
| --- | ---: |
| 왼쪽 바퀴 | 1 |
| 오른쪽 바퀴 | 2 |
| 팔 joint1 | 11 |
| 팔 joint2 | 12 |
| 팔 joint3 | 13 |
| 팔 joint4 | 14 |
| 손목 roll joint5 | 15 |
| 그리퍼 | 16 |

### 3-1. Wizard 사용

`usb_to_dxl` 업로드 후 Wizard의 `/dev/ttyACM0`에서 ID를 변경합니다.  
15번(손목)과 16번(그리퍼)이 중복되지 않도록 주의합니다.

### 3-2. Arduino만으로 ID 변경

Wizard가 포트를 잘못 잡거나 검색이 불안정하면 `c_ID_Change` 예제를 사용할 수 있습니다.

```bash
ID_SKETCH=$(find "$HOME/.arduino15/packages/OpenCR/hardware" \
  -type d -name c_ID_Change | head -1)

echo "$ID_SKETCH"
```

`c_ID_Change.ino` 수정:

```cpp
#define BAUDRATE  1000000   // 현재 Dynamixel 버스 속도
#define DXL_ID    1         // 현재 ID
#define NEW_DXL_ID 2        // 새 ID
```

- `BAUDRATE`는 USB 모니터 속도가 아니라 **Dynamixel 버스 속도**입니다.
- 현재 로봇은 1Mbps 구성입니다.
- 새 모터/공장 초기 상태라면 57600일 수 있으므로, 먼저 Wizard로 실제 버스 속도를 확인하는 편이 안전합니다.
- 한 번에 모터 하나씩 변경합니다.

```bash
"$ARDUINO_CLI" compile --fqbn "$FQBN" "$ID_SKETCH"
"$ARDUINO_CLI" upload -p "$PORT" --fqbn "$FQBN" "$ID_SKETCH"
"$ARDUINO_CLI" monitor -p "$PORT" -c baudrate=57600
```

`Serial.begin(57600)`은 시리얼 모니터용 설정입니다.

---

## 4. 원래 `beebot_opencr` 펌웨어 복구

ID 설정이 끝나면 OpenCR을 원래 로봇 펌웨어로 되돌립니다.

```bash
"$ARDUINO_CLI" lib install Dynamixel2Arduino

BEEBOT="$HOME/workspaces/pa/source/Lv2_bee_Assignment/lv2_module6/firmware/beebot_opencr"

"$ARDUINO_CLI" compile --fqbn "$FQBN" "$BEEBOT"
"$ARDUINO_CLI" upload -p "$PORT" --fqbn "$FQBN" "$BEEBOT"
```

업로드 후 ROS 제어를 다시 올리고, OpenCR이 모터 8개를 모두 ping하는지 확인합니다.  
`beebot_opencr`은 시작 시 모터가 하나라도 응답하지 않으면 `motors_ready=false`가 되므로, 업로드 후 상태 로그를 확인합니다.

---

## 5. 문제 해결

### `arm-none-eabi-g++: no such file or directory`

32비트 런타임이 없는 상태입니다. 1-3의 `libc6:i386` 등을 설치합니다.

### `Permission denied: /dev/ttyACM0`

```bash
sudo usermod -aG dialout "$USER"
```

로그아웃/로그인 후 다시 확인합니다.  
현재 세션만 임시로 쓰려면 `newgrp dialout` 또는 `sg dialout -c '...'`를 사용합니다.

### `No boards found`

```bash
sg dialout -c '"$ARDUINO_CLI" board list'
ls -l /dev/ttyACM0
lsusb | grep -i -E 'OpenCR|STM'
```

포트가 `Unknown`으로 보여도 업로드는 가능한 경우가 있습니다.  
포트가 아예 없으면 USB 케이블/OpenCR 전원을 확인합니다.

### 포트가 사용 중임

- DYNAMIXEL Wizard
- Arduino 시리얼 모니터
- ROS 제어 노드
- `beebot_opencr` 펌웨어

중 하나가 `/dev/ttyACM0`를 잡고 있으면 업로드/스캔이 실패합니다. 하나만 열어 둡니다.

### 업로드는 됐는데 모터가 안 보임

- OpenCR에 12V 전원이 들어왔는지 확인
- Dynamixel 3핀 케이블 방향/체결 확인
- `usb_to_dxl`이 실제로 올라갔는지 확인
- 한 모터만 연결해 먼저 ping 확인
- 모터가 57600bps이면 Wizard에서 baud를 바꿔 검색
