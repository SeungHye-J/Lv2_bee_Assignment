# lv2_module6 / ros2_ws

ROS 2 Lyrical 실기 워크스페이스 — 로봇 bringup, 지도·주행, 팔·그리퍼, 인지·계획 노드.

- 전체 설계: [`../../documents/ros2-design/README.md`](../../documents/ros2-design/README.md)
- 미션 설명: [`../../documents/MISSION.md`](../../documents/MISSION.md)
- 개발·도커·테스트: [`../develop/DEVELOP.md`](../develop/DEVELOP.md)

## 1. 빠른 시작

```bash
cd lv2_module6/ros2_ws
source /opt/ros/lyrical/setup.bash

# 처음이거나 코드가 바뀌었으면
./start.sh --build

# 이미 빌드되어 있으면
./start.sh
```

`start.sh`는 `/opt/ros/lyrical`을 source하고, `install/`이 없거나 `--build`면
`colcon build --symlink-install`을 실행한 뒤 `bringup` launch를 띄웁니다.

종료:

```bash
./stop.sh
```

## 2. 장치 설정

### OpenCR

기본 포트는 udev 고정 이름 `/dev/opencr`입니다.

```bash
sudo cp 99-opencr.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

`/dev/opencr`이 없으면 `/dev/ttyACM*`를 직접 지정할 수 있습니다.

```bash
./start.sh port:=/dev/ttyACM0
```

OpenCR 없이 노드만 확인:

```bash
./start.sh mock_hardware:=true
```

### RealSense 2대

RealSense ROS 노드는 `/dev/video*`가 아니라 **serial_no**로 카메라를 고릅니다.
serial은 환경변수로 넘깁니다.

```bash
export POLE_FRONT_CAM_SERIAL=<front_serial>
export GRIPPER_CAM_SERIAL=<gripper_serial>
./start.sh
```

확인:

```bash
source /opt/ros/lyrical/setup.bash
rs-enumerate-devices -s
# 안 잡히면:
/opt/ros/lyrical/bin/rs-enumerate-devices -s
```

한 대만 연결한 경우에는 그 카메라 serial만 설정하면 `start.sh`가 반대쪽을 자동으로 끕니다.

```bash
export POLE_FRONT_CAM_SERIAL=261822074109
./start.sh
# WARN: gripper_cam serial 없음 → use_gripper_cam:=false
```

직접 launch 인자로 넘길 수도 있습니다.

```bash
./start.sh \
  pole_front_cam_serial:=<front_serial> \
  gripper_cam_serial:=<gripper_serial>
```

카메라별 on/off:

```bash
./start.sh use_gripper_cam:=false       # front만
./start.sh use_pole_front_cam:=false    # gripper만
./start.sh use_cameras:=false           # 둘 다 끔
```

### 카메라 해상도·FPS

기본값은 **640×360 @ 15 fps**이고, RGB/depth에 같은 프로파일을 씁니다.

```bash
./start.sh \
  pole_front_cam_profile:=640x360x15 \
  gripper_cam_profile:=640x360x15
```

지원 프로파일은 D435/D435i 기준입니다. 예: `424x240x15`, `640x480x15`.
네트워크가 버티지 못하면 먼저 해상도/FPS를 낮추고 point cloud를 끕니다.

```bash
./start.sh \
  pole_front_cam_profile:=424x240x15 \
  gripper_cam_profile:=424x240x15 \
  gripper_cam_pointcloud:=false
```

- `pole_front_cam_pointcloud`: `body_filter`·Nav2용. 보통 켜 둠.
- `gripper_cam_pointcloud`: `grasp_scorer`용. 네트워크 절약 시 끌 수 있음.

현재 대역폭 확인:

```bash
ros2 topic bw /pole_front_cam/color/image_raw
ros2 topic bw /pole_front_cam/depth/color/points
ros2 topic hz /pole_front_cam/color/image_raw
```

Wi-Fi에서 raw Image/PointCloud2를 여러 개 보내면 쉽게 포화됩니다.  
가능하면 Jetson과 PC를 유선 Ethernet으로 연결하고, 원격 영상만 필요하면 compressed image transport를 쓰세요.

## 3. 지도·모드

기본은 `map:=saved`입니다. `ros2_ws/map/<이름>/rtabmap.db`가 없으면
launch가 죽지 않고 **RTAB-Map 없이** 뜹니다.

```bash
# 새 지도 작성
./start.sh map:=slam

# 저장된 지도로 위치추정 (기본)
./start.sh
./start.sh map:=saved map_name:=20261017_163731

# 테스트베드 코스 지도 (RTAB-Map 없음)
./start.sh map:=course
```

- 지도 폴더: `ros2_ws/map/` (launch가 `map_dir`를 자동 지정)
- 상태: `/state/map_mode`, `/state/map_name`, `/state/map_list`

조작 모드:

```bash
ros2 topic pub --once /cmd/operation_mode std_msgs/msg/String "{data: MANUAL}"
ros2 topic pub --once /cmd/operation_mode std_msgs/msg/String "{data: AUTO}"
```

## 4. 연결 장치 상태

`state_manager`가 장치 목록과 상태를 발행합니다.

### 장치 이름 목록

```bash
ros2 topic echo /state/devices --once \
  --qos-reliability reliable \
  --qos-durability transient_local \
  --field data
```

```text
pole_front_cam,gripper_cam,opencr,imu,wheels,arm,gripper
```

### 장치별 상세

```bash
ros2 topic echo /state/devices/pole_front_cam --once \
  --qos-reliability reliable \
  --qos-durability transient_local \
  --field data
```

```text
camera|active|/pole_front_cam/color/image_raw,...
```

상태 값:

```text
active       = 최근 1초 안에 실제 데이터가 들어옴
disconnected = 장치가 없거나 1초 넘게 데이터가 없음
```

### 전체 토픽

```bash
ros2 topic echo /state/topics --once \
  --qos-reliability reliable \
  --qos-durability transient_local \
  --field data
```

`/state/devices`에 없는 새 센서도 `/state/topics`에서 토픽 이름·타입·publisher 수를 확인할 수 있습니다.

## 5. bringup 주요 인자

| 인자 | 의미 |
| --- | --- |
| `--build` | `colcon build --symlink-install` 후 실행 |
| `mock_hardware:=true` | OpenCR 없이 제어상태를 mock으로 |
| `port:=/dev/ttyACM0` | OpenCR 포트 |
| `use_cameras:=false` | RealSense 전체 끔 |
| `use_pole_front_cam:=false` | front 카메라만 끔 |
| `use_gripper_cam:=false` | gripper 카메라만 끔 |
| `pole_front_cam_serial:=...` | front serial_no |
| `gripper_cam_serial:=...` | gripper serial_no |
| `pole_front_cam_profile:=640x360x15` | front RGB/depth 프로파일 |
| `gripper_cam_profile:=640x360x15` | gripper RGB/depth 프로파일 |
| `pole_front_cam_pointcloud:=false` | front point cloud 끔 |
| `gripper_cam_pointcloud:=false` | gripper point cloud 끔 |
| `map:=slam\|saved\|course` | 지도 모드 |
| `map_name:=<이름>` | 사용할 지도 폴더 |
| `learning_enabled:=false` | 학습 노드 비활성 |
| `use_ml:=false` | ML 노드 전체 끔 |
| `use_nav2:=false` | Nav2 끔 |
| `use_ekf:=false` | EKF 끔 |
| `use_moveit:=false` | MoveIt move_group 끔 |

### 패키지별 단독 launch

전체 로봇은 `bringup`으로 띄우고, 아래 launch는 해당 패키지 노드만 따로 시험할 때 씁니다.
`ros_system`(하드웨어·Nav2·RTAB-Map)·TF·다른 패키지 노드는 포함되지 않습니다.

```bash
ros2 launch control control.launch.py
ros2 launch manager manager.launch.py map_mode:=MAPPING
ros2 launch ml ml.launch.py learning_enabled:=false
ros2 launch perception perception.launch.py device:=cuda
ros2 launch plan plan.launch.py command:="빨간 퍽을 목표 구역으로 옮겨"
```

`BeebotSystemHardware`와 `rl_arm_correction`은 노드가 아니라 `controller_manager`가
URDF/컨트롤러 설정으로 올리는 플러그인이므로 패키지 launch에 포함되지 않습니다.

## 6. 빌드·테스트

```bash
cd lv2_module6/ros2_ws
source /opt/ros/lyrical/setup.bash

colcon build --symlink-install
colcon test
colcon test-result --verbose
```

패키지 하나만:

```bash
colcon build --packages-select manager
colcon test --packages-select manager
```

전체 클린 빌드:

```bash
./rebuild.sh
```

## 7. OpenCR 펌웨어

저장소에서:

```bash
cd ../firmware
./setup.sh                     # 최초 1회
./compile.sh                   # build/<스케치>/*.ino.bin
./upload.sh /dev/opencr        # 또는 /dev/ttyACM0
./test-firmware                # 시리얼 상태 테스트

# 설치부터 한 번에
./setup_compile_upload.sh /dev/ttyACM0 beebot_opencr
```

CI 배포 패키지에서:

```bash
./firmware-upload.sh --setup
./firmware-upload.sh /dev/opencr
```

## 8. CI 배포 패키지

```bash
cd lv2_module6/ros2_ws

./download-artifact.sh                  # RUN ID 목록
./download-artifact.sh <RUN ID>         # 현재 장치 OS·아키텍처용
```

- x86_64 Ubuntu 26.04: `ros2_ws-deploy-x86_64.tar.gz`
- aarch64 Ubuntu 26.04: `ros2_ws-deploy-aarch64.tar.gz`
- **Jetson Ubuntu 24.04**: `ros2_ws-deploy-aarch64-noble.tar.gz`

Jetson은 ROS 2 Lyrical이 `/opt/ros/lyrical`에 설치되어 있어야 합니다.
자세한 설치: [`../../documents/jetson-orin-nano/SETTING.md`](../../documents/jetson-orin-nano/SETTING.md)

## 9. 자주 막히는 것

### `/opt/ros/lyrical` 없음

```bash
source /opt/ros/lyrical/setup.bash
```

없으면 ROS 2 Lyrical을 먼저 설치합니다.

### `/dev/opencr` 없음

```bash
ls -l /dev/ttyACM* /dev/opencr
```

udev 규칙을 설치하거나 `port:=/dev/ttyACM0`을 씁니다.

### 한쪽 카메라만 잡힘

```bash
rs-enumerate-devices -s
```

serial을 환경변수로 지정하고, 한 대만 쓸 때는 반대쪽 `use_*_cam:=false`가 자동 적용되는지 로그를 확인합니다.

### 지도가 없다는 경고

```text
저장한 지도 없음 ... RTAB-Map 없이 시작
```

`map:=slam`으로 새로 만들거나, 지도 폴더에 `rtabmap.db`가 있는지 확인합니다.

### 네트워크가 터짐

- `gripper_cam_pointcloud:=false`
- `*_cam_profile`을 `424x240x15`로 낮춤
- `ros2 topic bw`로 어떤 토픽이 큰지 확인
- 가능하면 유선 Ethernet, 원격 영상은 compressed transport 사용

## 10. 관련 스크립트

| 스크립트 | 용도 |
| --- | --- |
| `start.sh` | 빌드 확인 후 bringup 실행 |
| `stop.sh` | ROS 프로세스 전체 종료 |
| `rebuild.sh` | `build/`·`install/`·`log/` 삭제 후 클린 빌드 |
| `monitor.sh` | `monitor_manager`만 실행해 debug/ 토픽 기록 |
| `download-artifact.sh` | CI 배포 패키지 다운로드 |
| `firmware-upload.sh` | 배포 패키지에서 OpenCR 펌웨어 업로드 |
| `99-opencr.rules` | OpenCR `/dev/opencr` udev 규칙 |
