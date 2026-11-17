# Isaac 전체 미션

Isaac Sim을 beebot 하드웨어로 두고, 실기와 같은 bringup 전체로 주사위 미션 한 판을 돌린다. 신호등과 시작 바코드는 읽었다고 치고 넣어 준다. 명령은 `lv2_module6`에서 실행한다.

| 파일 | 하는 일 |
| --- | --- |
| `make_map.py` | 장면 USD 벽 · 기둥 → `map/<장면>/map.pgm` · `map.yaml` (Nav2 벽 지도) |
| `mission.rviz` | RViz 화면: 지도 · costmap · Nav2 경로 · footprint · 미션 마커 · 점구름 · 검출 영상 |
| `run_mission.py` | `/cmd/mission start` → BARCODE·READ 단계에 바코드·신호 넣기 → DONE/ERROR까지 단계 출력 |
| `rviz.sh` | `mission.rviz`를 시뮬 시계로 띄움 |
| `plan_to_map.py` | `plan.yaml` 실측 벽(`arena_walls`) → 실기용 Nav2 지도. 출발 자리(start)가 원점 |
| `map_to_nvblox.py` | 2D 지도(`map.pgm` + `map.yaml`) → 실기 nvblox 지도 파일(`nvblox.nvblx`) — `map:=saved`로 바로 위치추정 |
| `map/<장면>/` | 장면별 벽 지도: `scene_sim` · `scene_approach` · `scene_approach_matte` (matte는 approach 사본) |

## 실행

```bash
isaac/standalone/isaac_sim.sh          # 터미널 1: Play까지 (자동 Play: ISAAC_AUTOPLAY=1)
python3 isaac/mission/run_mission.py   # 터미널 2: ROS 노드가 다 뜬 뒤
```

- 기본 신호 `RGYRGRYGRGYR`(R5 Y3 G4) · 바코드 `P2-01`(오름차순) → L1 Y · L2 G · L3 R.
- 바꾸기: `--seq <R/Y/G 12개> --code P2-02`. 적재함 장부가 남아 시작이 거부되면 `--clear-basket`.
- 보기: `isaac/mission/rviz.sh` (mission.rviz · 시뮬 시계)
- Ctrl+C는 `/cmd/mission abort`. 회차 기록은 `~/.ros/mission/<시각>/`.

## Isaac 쪽 설정 (isaac_system.launch.py)

`isaac_sim.sh`가 띄우는 `isaac_system.launch.py`가 미션을 바로 돌릴 수 있게 실기 bringup과 다음을 다르게 둔다.

- `use_perception:=true`, `operation:=AUTO`. bringup은 매핑(`map:=slam`)이면 인지를 끄고 MANUAL로 둔다. 그대로면 coordinator가 미션 단계를 `manual_mode`로 거절하고, `/tracked_objects`가 없어 SURVEY에서 멈춘다.
- 경기장 벽은 장면 지도(아래 절)를 쓴다.
- 카메라는 Isaac 토픽을 쓴다(`use_cameras:=false`). 뒤 카메라 `pole_back_cam`도 Isaac 토픽으로 nvblox에 들어간다. 끄려면 `BEE_BACK_CAM=false isaac/standalone/isaac_sim.sh`.
- Nav2 costmap은 실기와 같이 `nav2_params.yaml` 값을 받는다(`nav2_container_params:=true`, #240). Nav2 기본값으로 보려면 `BEE_NAV2_PARAMS=false`.

## 신호 조건 (scene_sim 퍽)

`scene_sim.usda` 퍽은 빨강 7개(box 4 · cyl 3), 초록 5개(box 2 · cyl 3)다. `mission_solver`의 `allocate`가 받는 신호는 다음과 같다.

- R은 빨간 퍽 수 이하, G는 초록 퍽 수 이하.
- 남는 퍽(Y)이 한 모양(전부 box 또는 전부 cyl).

되는 조합: R3 Y6 G3 · R3 Y5 G4 · R3 Y4 G5 (Y=box), R4 Y6 G2 · R4 Y5 G3 · R4 Y4 G4 · R4 Y3 G5 · R5 Y5 G2 · R5 Y4 G3 · R5 Y3 G4 · R5 Y2 G5 · R6 Y4 G2 · R6 Y3 G3 · R6 Y2 G4 · R6 Y1 G5 (Y=cyl). 공개 연습 Q1(R4 Y2 G6)은 초록이 모자라 `no_allocation`.

## Nav2 지도 (scene_sim 벽 · 기둥)

```bash
~/pa-course/.venv-isaacsim/bin/python isaac/mission/make_map.py   # → isaac/mission/map/scene_sim/map.pgm · map.yaml
```

다른 장면은 `make_map.py`의 `SCENE`·`--out`을 바꿔 `map/<장면 이름>/`에 만든다. 지금 있는 지도는 `scene_sim` · `scene_approach` · `scene_approach_matte`이고, `lab`은 지도가 없어 `/arena/walls`를 쓴다. `isaac_system.launch.py`는 `isaac_sim.sh`가 연 장면(`ISAAC_SCENE`) 이름으로 `map/<장면>/map.yaml`을 찾고, 없으면 실기처럼 `/arena/walls`를 쓴다.

`scene_sim.usda`의 경기장 벽 · 기둥(`/World/maze`)을 높이 0.02~0.47 m 띠로 잘라 바닥에 투영한 map_server 형식 지도다(0.05 m/칸). 원점은 USD의 beebot 자세(로봇을 켠 자리)라 cuVSLAM·nvblox의 map 프레임과 같다. 시작 박스 · 장소 받침대(상자)와 퍽은 넣지 않는다. 미션이 다가가 붙는 대상이라 정적 지도에 있으면 목표 자리가 막힌다. 장면을 고치면 다시 만든다.

`isaac_system.launch.py`가 이 지도를 Nav2 `/map`에 넣는다.

- `/map`은 `nvblox_map`이 벽 격자(`walls_in`) 위에 nvblox가 본 것을 겹쳐 낸다. 실기 벽 격자는 mission_manager의 `/arena/walls`(plan.yaml 벽 선분)다.
- Isaac에서는 `standalone/runtime/arena_map_pub.py`가 이 지도를 `/arena/static`으로 내고, `node_params.py`가 `nvblox_map`을 `walls_in:=/arena/static`으로 다시 띄운다. ros2_ws 패키지는 그대로다.
- 상자 · 퍽은 nvblox가 보는 대로 겹친다. 실기처럼 `/arena/walls`를 쓰려면 `ros2 launch isaac/standalone/runtime/isaac_system.launch.py arena_map:=''`.

## 실기 지도 (plan.yaml 실측 벽)

장면 벽은 `maze.usda`에서 묶음째만 실측 쪽으로 옮겨져(묶음 안 모양은 그대로) 실측과 최대 약 11 cm 어긋난다. 실기 저장 지도는 `plan.yaml` 실측 벽으로 직접 그린다. 저장 지도의 map 프레임은 로봇을 켠 자리라, 출발 자리가 바뀌면 지도도 다시 그린다.

```bash
python3 isaac/mission/plan_to_map.py ros2_ws/map/scene_sim                              # plan.yaml start_pose_arena 기준
python3 isaac/mission/plan_to_map.py ros2_ws/map/start_b --start 3.670,0.950,3.14159    # 다른 자리에서 켤 때
python3 isaac/mission/map_to_nvblox.py ros2_ws/map/start_b ros2_ws/map/start_b          # 그다음 nvblox 지도
```

그다음 bringup은 `map:=saved map_name:=<폴더 이름> start_pose:=<같은 x,y,yaw>`로 띄운다.
