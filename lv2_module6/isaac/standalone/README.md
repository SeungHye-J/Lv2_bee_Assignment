# isaac_sim.sh 처음 실행하기

`isaac_sim.sh`는 Isaac Sim과 ROS 쪽(`runtime/isaac_system.launch.py`)을 함께 띄운다. 명령은 모두 `lv2_module6`에서 실행한다. 띄운 뒤의 사용법(reset, 팔만 시험)은 [시연](../docs/demo.md)을 본다.

## 1. 준비 (한 번만)

| 항목 | 확인 · 설치 |
| --- | --- |
| Isaac Sim venv | 기본 `~/pa-course/.venv-isaacsim`. 다른 곳이면 실행할 때 `ISAACSIM_VENV=<경로>`를 붙인다 |
| `depth_image_proc` | `sudo apt install ros-lyrical-depth-image-proc` |
| `ros2_ws` 빌드 | `ros2_ws/install/`이 있어야 한다. 없거나 develop을 머지했으면 `ros2_ws/rebuild.sh` |
| YOLO용 레포 `.venv` | 레포 루트 `.venv` (torch · ultralytics). 없으면 인식 결과가 빈 채로 돈다 |

develop을 머지한 뒤에는 꼭 다시 빌드한다. 새 설정 파일이 `install/`에 없으면 launch가 실패한다.

## 2. 실행

1. 새 터미널을 연다. Isaac venv(`isaacsim` 별칭 등)는 켜지 않아도 된다. 켜져 있으면 스크립트가 끈다.
2. 실행한다.

   ```bash
   isaac/standalone/isaac_sim.sh          # 기본 장면 usda/scene_sim.usda
   isaac/standalone/isaac_sim.sh approach # usda/scene_approach.usda — 고를 수 있는 것: sim · lab · approach · approach_matte (파일 이름 · 경로도 됨)
   ```

3. Isaac 창이 뜨고 장면이 열리면 **Play**를 누른다. Play를 눌러야 `/tmp/opencr`가 생기고, 그때 ROS 노드(약 40개)가 자동으로 뜬다.
4. 끌 때는 Isaac 창을 닫거나 실행한 터미널에서 Ctrl+C를 누른다. ROS 노드도 같이 꺼진다.

## 3. 실행 중 지킬 것

- **Stop은 누르지 않는다.** 시뮬 시계가 0으로 돌아가 TF · EKF · cuVSLAM · nvblox가 꼬인다. 잠깐 멈출 때는 Pause를 쓰고, 처음 자세로 되돌릴 때는 `isaac/standalone/runtime/reset.sh`를 쓴다.
- 다른 터미널에서 `ros2` 명령으로 토픽을 보려면 같은 Fast DDS 프로파일을 지정한다. `fastdds.xml`과 섞으면 서로 못 찾는다.

  ```bash
  export FASTDDS_DEFAULT_PROFILES_FILE=$PWD/ros2_ws/fastdds_local_only.xml
  ```

## 4. 자주 쓰는 변형

| 목적 | 명령 |
| --- | --- |
| 창 없이 + 자동 Play (GPU 부담 줄임) | `ISAAC_HEADLESS=1 ISAAC_AUTOPLAY=1 isaac/standalone/isaac_sim.sh` |
| 팔만 시연 (SLAM · Nav2 없음) | `ISAAC_ROS_LAUNCH=isaac_manipulator_demo.launch.py ISAAC_AUTOPLAY=1 isaac/standalone/isaac_sim.sh` |
| 센서 노이즈 끔 | `BEE_SIM_NOISE=0 isaac/standalone/isaac_sim.sh` |
| 관절 비틀림 모델과 그 보정 끔 | `OPENCR_TWIST_SCALE=0 isaac/standalone/isaac_sim.sh` |
| 실기 start.sh와 같은 인자로 미션 (인지 전부 ROS · 폴 뒤 캠 끔) | `isaac/standalone/isaac_mission.sh map`(처음 한 번 매핑) → `isaac/standalone/isaac_mission.sh` — 사용법은 스크립트 머리말 |

팔 시험 루프를 돌릴 때는 Isaac에서 Play를 누른 뒤 다른 터미널에서 `isaac/loops/manipulator-launcher.sh use_sim:=true`를 실행한다.

## 5. 문제가 생기면

| 증상 | 원인 · 조치 |
| --- | --- |
| `ROS_DOMAIN_ID=…에 이미 노드가 떠 있음` 후 종료 | 같은 도메인에 다른 실행(MoveIt mock 등)이 있다. 그 실행을 끄거나 `ROS_DOMAIN_ID`를 바꾼다. 무시하고 띄우려면 `ALLOW_OTHER_NODES=1` |
| `[isaac_sim] YOLO 사용 불가` | 레포 `.venv`에 torch · ultralytics가 없다. 시뮬은 돌지만 인식 결과가 비어 있다 |
| Play를 눌러도 컨트롤러가 안 뜸 | `ros2_ws`가 빌드되지 않았거나 낡았다. `ros2_ws/rebuild.sh`로 다시 빌드한다 |
| `장면 없음: <이름>` | `usda/scene_<이름>.usda`가 없다. 메시지에 고를 수 있는 장면 이름이 나온다 |
| Isaac이 시작하다 죽음 (빈 USD stage) | 일시적인 경우가 있다. 다시 실행한다 |

지난 실행의 launch · 고아 노드 · Fast DDS 공유 메모리 찌꺼기는 스크립트가 시작할 때 정리한다.
