# 3-1 지도 맵핑 실습 — 지도 만들고 저장하고 쓰기

실기(Jetson)에서 cuVSLAM + nvblox로 지도를 새로 만들어 저장하고, 저장한 지도로 위치추정·자율주행하는 순서입니다. 원리는 [04-nav.md](04-nav.md), 설치·시험 기록은 [CUVSLAM.md](../../jetson-orin-nano/CUVSLAM.md). 근거 코드: [bringup.launch.py](../../../lv2_module6/ros2_ws/src/bringup/launch/bringup.launch.py) · [start.sh](../../../lv2_module6/ros2_ws/start.sh) · [지도 메뉴](../../../lv2_module6/tools/cli/op/pages/mapmenu/__init__.py) · [bus.py `cmd_map`](../../../lv2_module6/tools/cli/op/bus.py).

## 지도 한 벌

폴더 하나 = 지도 하나: `<map_dir>/<map_name>/` 안에 `nvblox.nvblx`(위치추정용 정본) · `map.pgm` · `map.yaml`(사람이 보는 미리보기).
- `./start.sh`는 `map_dir:=<ros2_ws>/map`을 자동으로 붙입니다 → `lv2_module6/ros2_ws/map/<이름>/` (저장소에 날짜_시각 이름 폴더가 이미 여럿 있음). bringup 자체의 기본은 `~/.ros/beebot_maps`.
- CLI도 같은 폴더를 봅니다 (`BEEBOT_MAP_DIR` 환경변수로 바꿈). 지도 목록은 `nvblox.nvblx`가 있는 폴더만 (CLI `local_maps`).
- 보기(터미널 지도 화면)는 `map.yaml` + `map.pgm`이 있는 지도만 됩니다.

## 순서

1. **매핑 모드로 띄우기**
   ```bash
   cd lv2_module6/ros2_ws
   ./start.sh map:=slam map_name:=<이름>      # map_name 생략 = <날짜_시각>, 이미 있는 이름이면 거부
   ```
   `map:=slam`이면 perception은 기본 꺼짐(`perception:=auto`)입니다. 이미 떠 있으면 CLI에서 `./run-cli` → 지도/위치 메뉴 → **3 MAPPING** (확인 후 launch 전체가 다시 뜸, 수십 초).
2. **CLI 열기**: `lv2_module6/tools/run-cli` (대화형 메뉴) 또는 한 번 명령 `./run-cli map status`. 지도/위치 메뉴 `center`에 모드·이름·목록이 보입니다.
3. **수동으로 몰며 지도 쌓기**: 지도/위치 메뉴 `a`(텔레오퍼) 또는 `b`(수동 주행 메뉴). 조작 모드가 MANUAL이어야 합니다. 수동 속도는 `/cmd_vel_manual`(twist_mux 우선순위 100)로 나갑니다.
   - 천천히, 벽을 따라 한 바퀴 돌고 출발 자리로 돌아옵니다. 지도 원점은 **출발 자리 base_footprint**입니다.
4. **보면서 하기**: 메뉴 `9` 매핑 전용 RViz(`tools/rviz/mapping.rviz`), `d` nvblox 전용, `e` cuVSLAM 전용. 직접은 `RVIZ_CONFIG=mapping tools/rviz/start.sh`.
   - Jetson 자체에서 RViz를 같이 띄우지 않습니다 — 메모리 부족 (CUVSLAM.md 3절: `free -h` available 300 MiB 아래면 중단).
5. **저장**: 메뉴 `6` 지도 저장(경로 빈칸 = 기본 `<map_dir>/<이름>/map`) 또는 `./run-cli map save`. 미리보기(pgm/yaml)를 저장한 뒤 `nvblox.nvblx`는 `/nvblox_node/save_map` 서비스로 저장합니다. 이어서 LOCALIZATION 전환 여부를 묻습니다.
6. **위치추정으로 쓰기**
   ```bash
   ./start.sh map:=saved map_name:=<이름> operation:=AUTO
   ```
   `map_name`을 비우면 가장 최근 지도, 기본 `operation`은 `AUTO`입니다. 실행 중에는 메뉴 `5` 지도 전환 / `4` LOCALIZATION. **출발 자리·방향이 저장 때와 같아야** 합니다.

그 밖의 CLI `map` 동작: `status` · `list` · `mode <none|mapping|localization>` · `switch <이름>` · `save` · `rename` · `delete`. 지금 쓰는 지도는 이름 바꾸기·삭제가 거부됩니다.

## 품질 확인

| 확인 | 방법 | 정상 (CUVSLAM.md 3-1 기준) |
| --- | --- | --- |
| cuVSLAM 추적 | cuvslam_slam 로그 (10초마다) | `추적 N · 잃음 0`, N이 늘어남 |
| TF 안정 | `ros2 run tf2_ros tf2_echo map base_footprint` | 정지 시 1 cm · 1° 안 |
| depth 마스크 | `ros2 topic hz /pole_front_cam/aligned_depth_to_color/image_nvblox` | 15 Hz, 가운데 팔 부분 0 |
| /map | `ros2 topic echo /map --once --field info` | 해상도 0.05 |
| 되돌아온 위치 | 한 바퀴 돌고 출발 자리 도착 후 지도·TF 비교 | 벽이 이중으로 겹치지 않음, 출발 자리 원점과 근접 |
| 저장 확인 | `ls <map_dir>/<이름>/` | `nvblox.nvblx`, `map.pgm`, `map.yaml` |
| 불러오기 | `map:=saved`로 재기동, nvblox_map 로그 | `지도 불러오기 … 성공`, 시작부터 /map이 그려짐 |

주의 (저장소 기록 기반): 정지해 있으면 아는 칸이 줄던 문제로 `decay_tsdf_rate_hz: 0.0`(잊기 끔)이 되어 있어, 한번 들어간 가짜 장애물은 안 지워집니다 (nvblox.yaml 주석). 깊이 잡음으로 바닥이 찍히는 검은 부채꼴이 보이면 `esdf_slice_min_height`(현재 0.08)를 참고하세요.

## 확인 못 한 것

- 저장 시점: CUVSLAM.md는 "끌 때 nvblox가 `nvblox.nvblx`를 자동 저장"이라 하지만, CLI 코드 주석은 종료 시 자동 저장은 png/yaml만 쓰고 `.nvblx`는 `save_map` 서비스를 불러야 한다고 합니다. 두 설명이 다릅니다 — 확실히 하려면 저장 후 `ls`로 `.nvblx`를 확인하세요.
- 메뉴 번호는 코드의 `key` 값이며 화면에서 직접 눌러 보지는 않았습니다.
- `map.pgm`/`map.yaml` 저장(`map_saver`)이 어떤 토픽에서 만드는지, 한 바퀴 돌 때 권장 속도·시간은 문서화된 값을 찾지 못했습니다.
- "Jetson 작업폴더·DOMAIN" 같은 환경 값은 실기 상태를 확인하지 않았습니다 (CUVSLAM.md 예시는 `ROS_DOMAIN_ID=42`).
