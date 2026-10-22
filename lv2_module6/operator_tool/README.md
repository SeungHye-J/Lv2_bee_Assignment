# operator_tool

운영 중 수동 조작을 위한 CLI입니다. 별도 ROS 노드를 새로 만들지 않고, 이미 떠 있는
`state_manager`·`manual_control`의 토픽과 `map_saver` 서비스를 사용합니다.

```text
operator_tool/
├── README.md
├── run.sh
└── operator_tool.py
```

전체 로봇은 먼저 띄워 둡니다.

```bash
cd ../ros2_ws
./start.sh
```

## 실행

```bash
cd operator_tool
./run.sh status
```

`run.sh`가 `/opt/ros/lyrical`과 `../ros2_ws/install/setup.bash`를 source합니다.
다른 워크스페이스를 쓰면 `ROS_DISTRO_NAME` 또는 `BEEBOT_MAP_DIR`를 지정합니다.

## 명령

### 상태

```bash
./run.sh status
./run.sh status --diagnostics
./run.sh status --topics
```

`/state/operation_mode`, `/state/map_mode`, `/state/map_name`, `/state/emergency`,
장치별 `/state/devices/<이름>`을 읽습니다.

### 자동/수동 전환

```bash
./run.sh mode manual
./run.sh mode auto
./run.sh mode explore       # MAPPING 중일 때만
```

### 비상정지

```bash
./run.sh estop on
./run.sh estop off
./run.sh estop status
```

`estop on`은 바퀴 명령을 막고, 현재 수동 주행 명령도 0으로 보냅니다.

### 수동 주행

먼저 `mode manual`이어야 합니다.

```bash
./run.sh drive forward --speed 0.15 --duration 1.0
./run.sh drive back --speed 0.10 --duration 0.5
./run.sh drive left --angular 0.6 --duration 0.5
./run.sh drive right --angular 0.6 --duration 0.5
./run.sh drive stop
```

터미널 키보드 조작:

```bash
./run.sh teleop
# w/s 전후진 · a/d 좌우회전 · space 정지 · q 종료
```

### 팔·그리퍼

```bash
./run.sh arm show
./run.sh arm set joint1=0.0 joint2=0.4 joint3=-0.2
./run.sh arm delta joint2=+0.05
./run.sh arm home

./run.sh gripper show
./run.sh gripper open
./run.sh gripper close
./run.sh gripper position 0.01
```

- `arm set`은 지정한 관절만 보내고, 나머지는 `manual_control`이 현재 각으로 채웁니다.
- `MAPPING` 중에는 팔 명령이 스캔 자세 고정 때문에 거절됩니다. 그리퍼는 가능합니다.
- 관절 한계는 `controller_manager`의 joint limiter가 최종 적용합니다.

### 지도

```bash
./run.sh map status
./run.sh map list
./run.sh map mode mapping
./run.sh map mode localization
./run.sh map switch 20261019_093353
./run.sh map save
./run.sh map save --file /tmp/my_map
./run.sh map save --map-dir /path/to/maps
```

`map save`는 `/map_saver/save_map` 서비스를 호출해 `.pgm`·`.yaml`을 저장합니다.
기본 경로는 `ros2_ws/map/<현재 지도 이름>/map`입니다.

### 재시작

```bash
./run.sh restart all --yes
./run.sh restart perception --yes
```

`--yes` 없이는 실행되지 않습니다. 전체 스택이 재시작되므로 주의합니다.

## 안전 규칙

- 수동 주행·팔·그리퍼는 `MANUAL` 모드에서만 동작합니다.
- 비상정지 중에는 수동 명령을 거부합니다.
- `drive`는 지정 시간이 끝나거나 Ctrl+C로 종료될 때 0 속도를 보냅니다.
- `teleop`은 종료 시 터미널 설정을 복원하고 0 속도를 보냅니다.
- `--force`는 상태 확인만 건너뛰며, 실제 `manual_control`이 명령을 무시할 수 있습니다.

## 문제 확인

```bash
# 상태 토픽이 안 보이면
ros2 node list | grep -E 'state_manager|manual_control'
ros2 topic echo /state/operation_mode --once \
  --qos-reliability reliable --qos-durability transient_local

# 수동 명령이 안 먹으면
ros2 topic info /cmd/manual/cmd_vel -v
ros2 topic echo /state/emergency --once \
  --qos-reliability reliable --qos-durability transient_local
```

`state_manager`가 없으면 `status`는 “상태 없음”을 출력합니다. 먼저 `bringup`을 띄우세요.

### 시험과 실기 분리

같은 네트워크에서 여러 로봇/시뮬레이터를 띄우면 `operator_tool`이 다른 스택의
`/state`를 읽거나 명령을 보낼 수 있습니다. 시험할 때는 `ROS_DOMAIN_ID`를 다르게
지정하고, 실행 전에 `ros2 node list`로 대상 스택이 하나인지 확인하세요.

```bash
export ROS_DOMAIN_ID=42
./run.sh status
```
