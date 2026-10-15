# DEVELOP — 처음 들어온 개발자용

lv2_module6 워크스페이스에서 개발·빌드·테스트하는 방법입니다. 프로젝트 개요·패키지·담당은 [README.md](README.md), 설계는 [documents/ros2-design](../documents/ros2-design/README.md), 브랜치·커밋·PR 규칙은 [CONTRIBUTING.md](../CONTRIBUTING.md)를 봅니다.

- ROS 2 **Lyrical** (Ubuntu 26.04) · **C++** (ament_cmake, rclcpp)
- 개발·테스트는 **도커 테스트베드**에서 합니다 (로컬에 ROS를 설치하지 않아도 됨). CI도 같은 컨테이너에서 같은 검사를 돌립니다.

## 1. 처음 셋팅

필요한 것: Git, [Docker Desktop](https://www.docker.com/products/docker-desktop/) (또는 Linux Docker Engine + compose)

```bash
git clone <저장소>
cd Lv2_bee_Assignment
git switch develop                     # 작업은 develop에서 브랜치를 따서 (CONTRIBUTING.md)

cd docker
docker compose build                   # 이미지: ROS 2 Lyrical + ros2_control + EKF + arduino-cli(OpenCR) (처음 한 번, 몇 분)
docker compose up -d
docker compose exec lyrical bash       # 컨테이너 셸 — 이후 명령은 전부 여기서
test-all                               # 빌드 + 전체 검사. ALL PASS면 셋팅 끝
```

컨테이너는 소스만 마운트하고 빌드 결과(`/ws/build`, `/ws/install`)는 컨테이너 안에 둡니다.

| 호스트 | 컨테이너 | 용도 |
| --- | --- | --- |
| `lv2_module6/ros2_ws/src` | `/ws/src` | ROS 2 패키지 소스 (호스트에서 편집하면 바로 반영) |
| `lv2_module6/urdf` | `/urdf` (읽기 전용) | 로봇 모델 |
| `lv2_module6/firmware` | `/ws/firmware` | OpenCR 펌웨어 스케치 |
| `lv2_module6/debug` | `/ws/debug` | 토픽 기록 · 시뮬레이터 연동 · 가상 시리얼(serial-in/out) · 펌웨어 빌드 결과 (gitignore). **양방향**: 호스트→컨테이너(`input/` 명령·수동, `input-live/` 카메라, `service/*/request`·`action/*/goal·cancel`, `sim/objects.json`)와 컨테이너→호스트(`topic/`·`service/*/response·type|info`·`action/*/accept·feedback·result`, `serial-out`)가 같은 폴더를 쓴다. 소유권·권한은 아래 "공유 폴더 권한" 참조 |

Dockerfile·`docker/*.sh`를 고쳤으면 `docker compose build` 후 `docker compose up -d`로 다시 띄웁니다 (스크립트는 이미지에 복사돼 있음).

### 공유 폴더 권한 (`lv2_module6/debug`)

이미지에 `USER` 지정이 없어 컨테이너 안 프로세스는 uid 0(root)으로 돕니다. 바인드 마운트는 호스트 파일시스템을 그대로 공유하고 user namespace remapping을 쓰지 않으므로(기본값), 컨테이너가 만든 파일·디렉토리는 호스트에서 root 소유로 보입니다. `sudo`라서가 아니라 uid 문제입니다.

그래서 양쪽 다 쓸 수 있게 디렉토리는 777로 유지합니다:

- `docker/entrypoint.sh`: 기동 시 `input/`·`input-live/`·`topic/`·`service/`·`action/`·`interface/`·`sim/` 미리 생성 + `chmod -R 777 /ws/debug`
- `faker` 노드(`monitor_manager`·`service_action_monitor`·`service_action_input`): 디렉토리 생성 직후 `chmod 0777` — 실행 중 새로 생기는 `topic/<토픽>/`·`service/<이름>/`·`action/<이름>/`도 호스트가 rename(원자적 쓰기)으로 교체 가능
- 파일 자체는 644여도 된다: 읽는 쪽은 읽기만, 쓰는 쪽은 디렉토리 권한으로 rename 하므로

`PermissionError`가 나면 (예: 시뮬레이터 `save_truth`가 `debug/sim` 생성 실패) 컨테이너에서 소유권 회수 — 호스트 `sudo`는 비밀번호가 필요할 수 있으니 컨테이너(root) 경유가 확실:

```bash
cd docker && docker compose exec -u root lyrical bash -c "chown -R 1000:1000 /ws/debug && chmod -R 777 /ws/debug"
```

참고: `docker compose up -d`로 컨테이너를 재생성하면 `/ws/build`·`/ws/install`(컨테이너 내부, 마운트 아님)이 날아가므로 `colcon build`부터 다시 합니다.

## 2. 컨테이너 안 명령

| 명령 | 하는 일 |
| --- | --- |
| `test-all` | **push 전 필수.** CI와 같은 검사: test-serial → test-firmware → test-packages → test-hardware → test-bringup → test-faker(정상) → test-faker(파지 실패·재계획) |
| `test-serial` | 가상 시리얼(`/dev/ttyV0` = `/dev/ttyACM0`) ↔ `debug/serial-in`·`serial-out` 파일 왕복 |
| `test-firmware` | OpenCR 펌웨어 컴파일 → 가상 시리얼로 업로드 → 보낸 바이트가 .bin과 같은지 (결과 `debug/firmware/`) |
| `test-packages [패키지...]` | colcon build + 패키지별 테스트 (`colcon test`). 예) `test-packages plan` |
| `test-hardware` | controller_manager를 실기 모드(`mock_hardware:=false`)로 가상 시리얼에 붙여 펌웨어 프로토콜 확인 (C 명령 송신 · S 상태 → `/joint_states`) |
| `test-bringup` | 🛠 노드 8개가 모두 뜨는지 |
| `test-faker [초\|0] [launch 인자]` | 더미로 전체 흐름 → `/task_status` SUCCEEDED면 PASS. `0`이면 띄워 두기 (Ctrl+C 종료). `mock_hardware:=false`면 시뮬레이터 플랜트가 OpenCR 역할, `ml:=real`이면 ML 대역 대신 진짜 ml 노드 |
| `test-logger [초]` | 지금 떠 있는 토픽을 `/ws/debug/topic/<토픽>/echo`로 기록 |

```bash
# 띄워 두고 직접 보기 — 셸 두 개 (docker compose exec lyrical bash 를 하나 더)
test-faker 0                                 # 셸 1
ros2 topic list                              # 셸 2
ros2 topic echo /task_status
ros2 run plan user_input                     # 텍스트 명령 직접 입력
ros2 control list_controllers                # ros2_control 상태 (joint_state_broadcaster … gripper_controller)
ros2 topic echo /diagnostics_agg             # 노드별 상태 (OK · WARN · ERROR)
ros2 topic pub --once /manual/enable std_msgs/msg/Bool '{data: true}'      # 수동 모드
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true -r cmd_vel:=/manual/cmd_vel   # 키보드 조종
ros2 topic pub --once /e_stop std_msgs/msg/Bool '{data: true}'             # 비상정지 (false로 해제)
```

## 3. 빌드

```bash
cd /ws
colcon build --symlink-install                              # 전체
colcon build --symlink-install --packages-select plan       # 한 패키지만
colcon build --symlink-install --packages-up-to plan        # plan과 plan이 의존하는 것까지
source install/setup.bash                                   # 빌드 후 새 셸마다 (컨테이너 .bashrc가 자동으로 함)
```

- `--symlink-install`: launch·yaml·URDF는 다시 빌드하지 않아도 수정이 반영됩니다. C++은 다시 빌드해야 합니다.
- 의존 방향: `perception`, `control` ← `ml` ← `plan` ← `bringup` · `faker`. 아래쪽 패키지(메시지 포함)를 고치면 위쪽도 다시 빌드됩니다.

## 4. 커스텀 메시지

별도 interfaces 패키지 없이 **그 메시지를 내보내는 패키지**가 `rosidl_generate_interfaces`로 갖고 있습니다.

| 패키지 | 정의 | 내보내는 노드 |
| --- | --- | --- |
| perception | `msg/TrackedObject.msg` · `msg/TrackedObjectArray.msg` | perception → `/tracked_objects` |
| control | `msg/ArmSafetyEvent.msg` | arm_safety_monitor → `/arm_safety_event` |
| ml | `srv/ScoreGrasps.srv` | grasp_scorer (서버) ← coordinator |
| plan | `msg/SubGoal.msg` · `msg/SubGoalArray.msg` · `msg/TaskStatus.msg` | vlm_task_planner → `/sub_goals`, coordinator → `/task_status` |

```bash
ros2 interface list | grep -E "^(perception|control|ml|plan)/"   # 빌드된 커스텀 타입
ros2 interface show plan/msg/TaskStatus                           # 필드 확인
```

### 필드를 바꾸거나 메시지를 추가할 때

1. `.msg`/`.srv` 파일을 고치거나 만든다. 파일 이름은 CamelCase, 필드는 snake_case.
2. 새 파일이면 그 패키지 `CMakeLists.txt`의 `rosidl_generate_interfaces(...)` 목록에 추가. 다른 패키지 타입(예: `geometry_msgs/Pose`)을 쓰면 `DEPENDENCIES`와 `package.xml`에도 추가.
3. 빌드: `colcon build --symlink-install --packages-above <패키지>` — 그 메시지를 쓰는 패키지까지 다시 빌드 (메시지가 바뀌면 쓰는 쪽도 반드시 다시 빌드).
4. 받는 쪽·보내는 쪽 담당자에게 알린다 (PR 설명에 바뀐 필드).

### C++에서 쓰기

헤더 이름은 타입 이름을 snake_case로: `TrackedObjectArray` → `perception/msg/tracked_object_array.hpp`

```cpp
#include "perception/msg/tracked_object_array.hpp"
perception::msg::TrackedObjectArray msg;
```

| 어디서 쓰나 | package.xml | CMakeLists.txt |
| --- | --- | --- |
| **같은 패키지** 노드 | (이미 있음) | `rosidl_get_typesupport_target(msgs_cpp ${PROJECT_NAME} rosidl_typesupport_cpp)` 후 `target_link_libraries(<노드> ${msgs_cpp})` |
| **다른 패키지** 노드 | `<depend>perception</depend>` | `find_package(perception REQUIRED)` 후 `target_link_libraries(<노드> ${perception_TARGETS})` |

> Lyrical에서는 `ament_target_dependencies`가 없어졌습니다 — 항상 `target_link_libraries` + `<패키지>_TARGETS`.

## 5. 노드 작성 규칙

- 노드 하나 = 파일 세 개, 클래스는 `namespace <패키지>` 안:
  | 파일 | 내용 |
  | --- | --- |
  | `include/<패키지>/<노드>.hpp` | 노드 설명(입출력·담당·TODO) + 클래스 **선언**만 (멤버 함수 선언 · 멤버 변수) |
  | `src/<노드>.cpp` | 멤버 함수 **구현** (생성자 · 콜백 · 로직) |
  | `src/<노드>_main.cpp` | `main`만 — `rclcpp::spin(std::make_shared<<패키지>::<클래스>>())` |
  ros2_control 플러그인(BeebotSystemHardware · RlArmCorrection)도 hpp(선언) + cpp(구현 + PLUGINLIB_EXPORT_CLASS).
- 빌드: 패키지의 노드 구현(`src/<노드>.cpp`)은 라이브러리 `<패키지>_nodes` 하나로 묶고, 실행 파일(`src/<노드>_main.cpp`)과 gtest가 그걸 링크한다 — 테스트가 같은 클래스를 같은 프로세스에 띄울 수 있게.
- 새 노드: 위 세 파일 → `CMakeLists.txt`의 `add_library(${PROJECT_NAME}_nodes …)`에 `src/<노드>.cpp`, `foreach(node …)`·`install(TARGETS …)`에 이름 추가 →
  `bringup/launch/bringup.launch.py`에 `node('<패키지>', '<실행 파일>')` → `docker/test-bringup.sh`의 `NODES`에 노드 이름 → `bringup/config/diagnostics.yaml` 묶음에 이름.
- 토픽·메시지·QoS는 설계 문서 표를 따른다. 센서는 `rclcpp::SensorDataQoS()`(best_effort), 명령은 reliable, 가끔만 바뀌는 상태(`/sub_goals`, `/task_status`)는 reliable + transient_local.
- 학습 노드는 `enabled` 파라미터로 끌 수 있어야 하고, 꺼지거나 모델 파일이 없으면 기본 동작(Δ=0 · 규칙)으로 돌아야 한다.
- **도커·CI에는 GPU·모델이 없다** — 테스트베드는 진짜 ml 노드 대신 **faker ML 대역**(`fake_ml`: grasp_scorer · rl_nav_tuner · rl_nav_correction, 컨트롤러 `faker/PassthroughArmCorrection`)을 같은 이름·토픽으로 띄운다 (`ml:=fake` 기본, `bringup use_ml:=false`).
  진짜로 돌려 보려면 `test-faker 0 ml:=real`. ml 패키지의 gtest(`colcon test`)는 도커에서 돌므로 **CPU만으로 도는 테스트**로 둔다.
  ml 노드의 토픽·서비스·파라미터를 바꾸면 `faker/src/fake_ml.cpp`도 같이 맞춘다.
- **상태는 `/diagnostics`로** — 노드 클래스에 `diagnostic_updater::Updater diag_{this};` 를 두고 `diag_.add("상태", …)`에서
  `s.summary(OK|WARN|ERROR, "한 줄")` + `s.add("키", 값)`. 1초마다 발행되고 bringup의 `diagnostic_aggregator`가 묶는다
  (`bringup/config/diagnostics.yaml` — 새 노드는 이름을 해당 묶음 `startswith`에 추가). `test-bringup`이 노드마다 상태가 오는지 확인한다.
- **수동 모드를 존중** — 로봇을 움직이는 노드는 `/operation_mode`가 "MANUAL"이면 자기 명령을 멈춘다 (coordinator처럼). 바퀴는 twist_mux가 이미 수동을 우선한다.
- 담당자가 채울 곳은 `TODO(이름)` 주석으로 표시돼 있다.

## 6. 파라미터 (config yaml)

노드 파라미터 값은 코드가 아니라 **패키지 `config/<패키지>.yaml`** 에서 관리합니다 (ROS 2 관례: 노드 이름 → `ros__parameters`).

| 파일 | 노드 |
| --- | --- |
| `perception/config/perception.yaml` | perception |
| `control/config/control.yaml` | arm_safety_monitor |
| `ml/config/ml.yaml` | grasp_scorer · rl_nav_tuner · rl_nav_correction |
| `plan/config/plan.yaml` | user_input · vlm_task_planner · coordinator |
| `faker/config/faker.yaml` | pole_cam · gripper_cam · controller_server · move_group · monitor_manager · input_manager (더미) |
| `ros_system/config/controllers.yaml` | ros2_control 컨트롤러 (rl_arm_correction 포함) |
| `ros_system/config/ekf.yaml` | robot_localization EKF |

```yaml
coordinator:               # 노드 이름
  ros__parameters:
    max_grasp_tries: 2     # 이름은 코드의 declare_parameter("max_grasp_tries", …)와 같아야 한다
```

- 순서: 코드의 기본값 → yaml → launch 인자(`learning_enabled`, `command`, `fail_count` 등)가 덮어씀.
- 새 파라미터: 코드에 `declare_parameter<타입>("이름", 기본값)` + yaml에 같은 이름. **yaml 이름 오타는 ROS가 조용히 무시**하므로 `test-bringup`이 yaml 키가 노드에 실제로 선언돼 있는지 검사한다.
- yaml은 `--symlink-install`이라 다시 빌드 없이 노드만 다시 띄우면 반영. 실행 중 확인·변경: `ros2 param get /coordinator max_grasp_tries`, `ros2 param set …` (코드가 실행 중 값을 다시 읽는 파라미터만 효과 있음).

## 7. 테스트 작성

ROS 2 권장 방식대로 C++은 **gtest**(`ament_cmake_gtest`)를 `colcon test`로 돌립니다. 패키지마다 기본 테스트가 하나씩 있으니 복사해서 늘리면 됩니다.

| 패키지 | 테스트 | 확인하는 것 |
| --- | --- | --- |
| perception | `test/test_perception_node.cpp` | 영상이 들어오면 `/tracked_objects`가 나온다 |
| control | `test/test_arm_safety_monitor.cpp` | 전류 과다가 0.1초 넘게 이어지면 `/arm_safety_event` |
| control | `test/test_manual_control.cpp` | 수동 모드일 때만 `/manual/cmd_vel` → `/cmd_vel_manual`, `/operation_mode` 전환 |
| ml | `test/test_rl_nav_correction.cpp` | 보정 출력이 `/speed_limit` 상한으로 잘린다 |
| plan | `test/test_coordinator.cpp` | 허용 안 된 동작이 든 계획은 FAILED로 거부 · 수동 모드로 바뀌면 진행 중 계획을 멈춤(manual_override) |
| ros_system | `CMakeLists.txt` `test_urdf` | URDF가 xacro로 펼쳐지고 트리가 올바르다 (`check_urdf`) |
| bringup · faker | `CMakeLists.txt` `test_launch` | launch 파일이 오류 없이 읽힌다 |

gtest 하나의 모양: 노드 클래스와 `probe` 노드를 같은 프로세스에 띄우고, 입력을 넣으며 원하는 출력이 올 때까지 spin.

```cpp
TEST_F(RosTest, ClipsToSpeedLimit) {
  auto node = std::make_shared<RlNavCorrection>();           // 시험할 노드
  auto probe = std::make_shared<rclcpp::Node>("probe");      // 입력을 넣고 출력을 받는 쪽
  // probe에 publisher(입력) · subscription(출력) 만들기 …
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(probe);
  ASSERT_TRUE(spin_until(exec, [&]() { /* 입력 publish */ }, [&]() { /* 원하는 출력이 왔나 */ }));
}
```

새 테스트 파일을 추가하면 그 패키지 `CMakeLists.txt`의 `if(BUILD_TESTING)` 블록에 등록합니다:

```cmake
ament_add_gtest(test_foo test/test_foo.cpp TIMEOUT 60 ENV ROS_DOMAIN_ID=104)
target_link_libraries(test_foo <노드가 쓰는 타깃들>)
```

- `ROS_DOMAIN_ID`는 테스트마다 다르게 (perception 101 · control 102·105 · ml 103 · plan 104) — 다른 테스트나 띄워 둔 faker와 토픽이 섞이지 않게.
- 실행: `test-packages <패키지>` 또는 `colcon test --packages-select <패키지> && colcon test-result --verbose`
- 결과: `/ws/build/<패키지>/test_results/`

## 8. 로봇 모델 (URDF)

`lv2_module6/urdf/beebot.urdf.xacro` — TurtleBot3 Waffle Pi + 앞쪽 상판 위 OpenMANIPULATOR-X + 손목 roll + 그리퍼(위에 gripper_cam) + 뒤쪽 전원선 가이드 폴(pole_cam). 치수는 아직 더미.

```bash
xacro /urdf/beebot.urdf.xacro mock_hardware:=true > /tmp/beebot.urdf && check_urdf /tmp/beebot.urdf   # 트리 확인
```

- 구동 관절 6개: `joint1~4` (OM-X) · `joint5` (손목 roll, 추가) · `gripper_left_joint` (오른쪽 손가락은 mimic) + 바퀴 2개
- `ros_system`이 빌드할 때 `share/ros_system/urdf/`로 설치하고, launch가 xacro로 펼쳐 `robot_state_publisher`·`controller_manager`에 넘긴다.
- Isaac Sim USD: `xacro`로 펼친 URDF를 Isaac Sim **URDF Importer**로 변환.
- 메시: OM-X(RM-X52-TNM)는 `ros-lyrical-open-manipulator-description`(apt)의 STL을 쓸 수 있다 (TODO).

## 9. SIL 시뮬레이터 (`simulator/`)

실행·화면·파일 규격은 [simulator/README.md](simulator/README.md)를 본다. 요약만 두면:

- 호스트에서 `cd lv2_module6/simulator && .venv/bin/python main.py` (컨테이너 셸에서는 `test-faker 0` 또는 `test-faker 0 mock_hardware:=false`로 띄워 두기 — 컨테이너 접속은 §1 참조)
- 컨테이너와는 `lv2_module6/debug/` 파일로만 주고받는다 (호스트에 ROS 불필요)

## 10. OpenCR 펌웨어 (`firmware/`)

OpenCR 하나가 바퀴 2 · 팔 5축 · 그리퍼 · IMU를 맡고, 호스트의 `control/BeebotSystemHardware`와 USB 시리얼 ASCII 줄 프로토콜로 짝을 이룬다 — 형식·모터 ID·업로드는 [firmware/README.md](firmware/README.md).

- 프로토콜을 바꾸면 **펌웨어(`beebot_opencr.ino`)와 호스트(`beebot_system_hardware.cpp`)를 같이** 바꾼다. `test-hardware`가 호스트 쪽을, `test-firmware`가 컴파일을 확인한다.
- 컨테이너엔 OpenCR 보드가 없어서 업로드는 가상 시리얼로 보낸 바이트가 .bin과 같은지만 본다. 실물 업로드는 `firmware/upload.sh` 또는 배포 패키지의 `firmware-upload.sh`.

## 11. CI · 배포 패키지

`.github/workflows/ci.yml` (module5와 같은 구성):

| 단계 | 내용 |
| --- | --- |
| 트리거 | `develop`·`main` push, 그 둘로 가는 PR |
| testbed (x86_64 · aarch64) | 아키텍처별 러너(`ubuntu-26.04`, `ubuntu-26.04-arm`)에서 이미지 빌드 → 컨테이너에서 `test-all` |
| 배포 패키지 | `ros2_ws-deploy-<arch>.tar.gz` = `install/`(그 아키텍처용) + `firmware/`(스케치 · `build/*.bin`) + `ros2_ws/*.sh` + `ARCH` → Actions artifact (7일) |
| Release | `main`에 병합(push)될 때만 `run-<RUN ID>` Release에 두 패키지 첨부 (로그인 없이 wget) |
| 제출 태그 | `main` 병합 + 검사 통과 커밋에 `lv2-module6-submit` (마지막 병합 = 최종 제출) |
| 실패 시 | launch 로그 · 토픽 기록 · serial-in/out · 테스트 결과를 artifact `debug-<arch>`로 |

실기(라즈베리파이·PC)에서 배포 패키지 쓰기:

```bash
./download-artifact.sh                    # 받을 수 있는 RUN ID 목록
./download-artifact.sh <RUN ID>           # 이 장치 아키텍처용을 ./ros2_ws-<RUN ID>/ 에 풂 (토큰 없으면 Release에서)
cd ros2_ws-<RUN ID>
./firmware-upload.sh --setup && ./firmware-upload.sh     # OpenCR 펌웨어 (최초 1회 --setup)
./start.sh                                               # 장치 확인 → bringup (mock_hardware:=true 면 OpenCR 없이)
```

## 12. 실기

```bash
./ros2_ws/start.sh                          # 저장소에서 (install/ 없으면 빌드) — OpenCR · RealSense 확인 후 bringup
./ros2_ws/start.sh learning_enabled:=false  # 학습 노드 끄고 비교 기준
./ros2_ws/start.sh mock_hardware:=true use_cameras:=false   # 장치 없이
```

- 실기 PC에 필요한 apt: `ros-lyrical-{realsense2-camera,rtabmap-ros,nav2-bringup,robot-localization,controller-manager,joint-state-broadcaster,imu-sensor-broadcaster,diff-drive-controller,joint-trajectory-controller,parallel-gripper-controller,robot-state-publisher,xacro}` (+ MoveIt, 설정 패키지 준비 후)
- OpenCR 포트 기본 `/dev/ttyACM0` (`port:=` 로 변경). 업로드 중에는 ROS를 끈다 (같은 포트).

## 13. 문제 해결

| 증상 | 원인 · 해결 |
| --- | --- |
| 토픽이 두 번씩 오거나 이상한 값 | 이전 실행의 노드가 남음 → `pkill -f /ws/install/` 또는 `docker compose restart` |
| `Overrun detected!` 경고 (controller_manager) | 컨테이너는 실시간 스케줄링이 안 돼서 나는 경고 — 테스트에는 문제없음 |
| spawner `Could not contact service` | 기동 직후 CPU가 몰림. `test-faker`는 spawner를 2회까지 자동 재시도, 그래도 안 되면 다른 무거운 일(빌드 등)을 끝내고 다시 |
| `malformed launch argument` | `인자:=""`처럼 빈 값은 안 됨 — 인자를 빼거나 공백이 아닌 값 |
| 메시지 타입을 못 찾음 (`plan/msg/...`) | `source /ws/install/setup.bash` 안 함, 또는 메시지 패키지 빌드 전 |
| 호스트에서 `lv2_module6/debug` 파일을 못 만들거나 못 지움 (`PermissionError`) | 컨테이너(root)가 만든 디렉토리 — §1 "공유 폴더 권한"의 `chown` 명령으로 회수 |
| `docker compose build`에서 `not enough free space` | 다시 빌드할 때마다 이전 이미지가 이름 없이(dangling) 쌓인다 → `docker image prune -f && docker builder prune -f` |
| test-hardware `OpenCR 포트를 열 수 없음` | 가상 시리얼 브릿지가 안 떴음 → `docker compose restart` (entrypoint가 다시 만든다) |
| 시뮬레이터 카메라가 `waiting` | 컨테이너에서 `test-faker 0`이 떠 있어야 로봇 자세(`/odom`·`/robot_description`)가 온다 |
| 시뮬레이터 플랜트가 `대기` | 컨테이너가 `mock_hardware:=false`로 떠 있어야 C 명령이 온다 (기본 mock이면 플랜트는 쉰다) |
| 진단에 `controller_manager … bad periodicity · jitter` ERROR | 도커는 실시간 스케줄링이 안 돼서 — 실기에선 사라져야 정상 |
| 진단에 `ekf_filter_node … No events recorded` ERROR | robot_localization 자체 진단 — 도커에서 EKF 갱신 주기를 못 맞출 때. `/odometry/filtered`는 나오고 있으면 테스트엔 문제없음 |
| 수동 조작이 안 먹음 | `/operation_mode`가 MANUAL인지(`[수동 모드]`), `/e_stop`이 true로 남아 있지 않은지 |
