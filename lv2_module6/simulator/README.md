# SIL 시뮬레이터

실기 대신 세상(코스·퍽·장애물·이동물체·카메라 장면)과 로봇 몸(OpenCR)을 흉내 내고, ROS 쪽은 실제 노드가 그대로 돈다. 호스트에서 띄우는 창이고, 컨테이너와는 `lv2_module6/debug/` 파일로만 주고받는다 (호스트에 ROS 불필요). 전체 개발 흐름은 [DEVELOP.md](../DEVELOP.md).

```bash
# 컨테이너 (저장소 루트에서 — 둘 중 하나로 띄워 두기)
cd docker && docker compose up -d && docker compose exec lyrical bash   # 컨테이너 셸
test-faker 0                          # mock 하드웨어: 바퀴·팔은 명령 그대로 — 카메라 · 명령 · 수동 조작 확인용
test-faker 0 mock_hardware:=false     # 시뮬레이터 플랜트가 OpenCR 역할 — 미끄럼 · 마찰 · 충돌 · 통신 끊김까지
# 호스트 (새 터미널)
cd lv2_module6/simulator
python3 -m venv .venv && .venv/bin/pip install -r requirements.txt   # 최초 1회
.venv/bin/python main.py
.venv/bin/python main.py --snapshot out.png                # 창 없이 한 장 (확인용)
```

| 시뮬레이터 기능 | 필요한 컨테이너 명령 | 안 맞으면 |
| --- | --- | --- |
| 카메라·명령·수동 조작·service/action 입력·상태/진단 보기 | `test-faker 0` (둘 중 아무거나) | 카메라 `waiting`, 진단은 `(진단 없음)` |
| 미끄럼·마찰·충돌·통신 끊김·팔 붙잡기 (플랜트) | `test-faker 0 mock_hardware:=false` | 플랜트 `대기`, 실제 로봇(채움) 안 뜸 |

## 호스트 CLI (`bee.py`) — 컨테이너에 안 들어가고 지시

```bash
.venv/bin/python bee.py user "빨간 퍽을 목표 구역으로 옮겨"   # 텍스트 명령
.venv/bin/python bee.py status                                # 작업·모드·위치·service/action 목록
.venv/bin/python bee.py estop on|off                          # 비상정지
.venv/bin/python bee.py manual on|off                         # 수동 모드
.venv/bin/python bee.py twist 0.1 0.0                         # 수동 바퀴 (수동 모드에서만)
.venv/bin/python bee.py service score_grasps '{"object_id": 3}'   # 응답 대기, exit 2=호출 실패
.venv/bin/python bee.py action navigate_to_pose '{"x": 1.0, "y": 0.5}'  # accept→result 대기
.venv/bin/python bee.py action navigate_to_pose '{...}' --no-wait     # accept까지만
.venv/bin/python bee.py cancel navigate_to_pose <goal_id>
.venv/bin/python bee.py snap out.png
```

service/action은 `request_id`·`goal_id` 일치로 자기 응답만 받는다 (stale 무시). 창(`main.py`)을 안 띄우고 명령·호출만 할 때 쓴다.

| 화면 | 내용 |
| --- | --- |
| 지도 | 코스 + **실제 로봇(채움, 플랜트)** · `/odom`(테두리 — 미끄럼만큼 어긋남) · `base_footprint`(초록점, `/tf` — `/odom`까지 점선·거리) · 경로(`/plan`) · 인지 물체(`/tracked_objects`, 보라 테두리) · 퍽(정답) · 바닥 장애물(회색) · 이동물체(파랑) · 두 카메라 시야. 퍽·장애물은 왼쪽 드래그로 이동, 로봇은 잡아끌어 순간이동(플랜트 즉시·mock 릴리스 때), 빈 곳 왼쪽 드래그 지도 이동, 휠 확대(커서 고정) · 오른쪽 드래그 이동 · [지도 리셋] |
| 로봇 3D | 지금 상태(URDF+관절값, 드래그 회전 · 휠 줌) + 주변 퍽·장애물·이동물체 — 바퀴 회오리(주황=회전 중) + map·odom·base_footprint 좌표 + `관절축 보기` 체크(X빨·Y초·Z파, 바퀴는 같이 돌아 회전 표시) — 수동 조작 창의 로봇 뷰를 여기로 옮김 |
| 퍽 · 배경 · 바닥 | [퍽 랜덤 배치] 개수 · 모양(원기둥·상자·구) · 색 무작위 / 360° 파노라마 배경 / 바닥 텍스처(`images/floor/`, 1타일=1m 반복, 없으면 격자) / 벽 텍스처(`images/wall/`, 가로 0.5m 반복, 없으면 단색) / [카메라 송출] 끄기 = 카메라 빠짐 |
| 환경 | 미끄럼(바퀴가 돈 만큼 덜 나감) · 마찰(가속 한계) · [이동물체 추가/삭제](미로 안 칸 사이를 돌아다님, 삭제는 나중 것부터) · [바닥 장애물 추가/삭제](삭제는 나중 것부터) · [신호등 추가/삭제](랜덤 배치, 평소 꺼짐 — 초록 5초 → 황 2초 → 빨강 5초, 사이 1초씩 꺼짐, 높이 벽 수준) · [로봇 위치 초기화](플랜트+odom 리셋) · OpenCR 통신 끊기 · 팔 붙잡기(과전류) |
| 수동 조작 | 우측 [수동 조작] 탭 (`manual_window.py`, 고르면 수동 모드 · 나가면 바퀴 정지 + 자동 모드): 바퀴 버튼 앞 · 뒤 · 멈춤 · 좌측 바퀴 앞/뒤 · 우측 바퀴 앞/뒤 · 전체 속도 ▲▼, 비상정지(mock 모드 벽 앞 자동 정지), **팔 3D**(노란 관절 마커를 잡아 위아래로 끌면 그 관절 회전 → 놓으면 `/manual/arm`, 목표 밝게 · 지금 어둡게, 팔/그리퍼 초기화, 그리퍼 열기/닫기) + 바퀴 단축키(`W/X` 앞뒤 · `S` 멈춤 · `Q/Z` 좌바퀴앞뒤 · `E/C` 우바퀴앞뒤 · `A/D` 회전 · `[`/`]` 속도, 1.5초 자동정지). 로봇 전체 3D(지금 상태)는 메인 창에 있음 |
| 카메라 2대 | ROS 영상(로봇이 실제로 발행한 토픽, 없으면 시뮬레이터 렌더) · 깊이 컬러맵 · 점구름(드래그로 회전) |
| 상태 · 진단 | 작업 · 계획 · 운영 모드 · 위치 · 팔 · 플랜트 / `/diagnostics_agg` 노드별 OK·WARN·ERROR |

주고받는 파일:

| 방향 | 파일 | ROS 쪽 |
| --- | --- | --- |
| 시뮬레이터 → ROS | `debug/input-live/<카메라>.png` · `<카메라>_depth.png` (16비트 mm) | faker `fake_camera` → RealSense 토픽 (컬러 · 정렬 depth · 점구름 · camera_info) |
| 시뮬레이터 → ROS | `debug/input/<토픽>/message` (`{"type", "data"}` JSON) | faker `input_manager` → `/user_command` · `/manual/*` · `/e_stop` |
| 플랜트 ↔ ROS | `debug/serial-out`(C 명령) 읽기 · `debug/serial-in`(S 상태) 쓰기 | 가상 시리얼 `/dev/ttyACM0` ↔ BeebotSystemHardware (`mock_hardware:=false`) |
| ROS → 시뮬레이터 | `debug/topic/<토픽>/message` · `image.jpg` · `depth.png` + `debug/topic/list.json` | faker `monitor_manager` (모든 토픽의 마지막 메시지 + 목록) |
| ROS → 시뮬레이터 | `debug/service/list.json` · `service/<이름>/type|info` · `debug/action/list.json` · `action/<이름>/type|info` | faker `service_action_monitor` (서비스·액션 목록·정의) |
| 시뮬레이터 → ROS | `debug/service/<이름>/request` (`{"request_id", "data"}`) → `response` (`{"request_id", "ok", "data"|"error"}`) | faker `service_action_input` → `/score_grasps` · `/clear_octomap` · `/apply_planning_scene` |
| 시뮬레이터 ↔ ROS | `debug/action/<이름>/goal` + `cancel` → `accept` · `feedback`(최신 1건) · `result` | faker `service_action_input` → `/navigate_to_pose` · `/execute_task_solution` |
| ROS → 시뮬레이터 | `debug/interface/list` + `debug/interface/<종류>/list` + `debug/interface/<종류>/<패키지>/<이름>/info` (예: `srv/ml/ScoreGrasps/info`, 프로젝트 타입은 자동 생성, 나머지는 `.../request` 쓰면 생성 — `bee.py interface <타입>`) | faker `service_action_monitor` (`ros2 interface list` 카탈로그) |
| 시뮬레이터 기록 | `debug/sim/objects.json` | 퍽 · 바닥 장애물 정답 (미로 좌표 + map 좌표) — 인지 결과와 비교용 |

- 파일: `main.py`(메인 창) · `manual_window.py`(수동 조작 탭 패널) · `robot_view.py`(URDF 3D 뷰) · `scene.py`(카메라 렌더) · `plant.py`(OpenCR + 로봇 몸) · `maze.py`(코스) · `bridge.py`(debug 파일 주고받기)
- 팔·물체 상호작용(`main.py` `grip_interact`, 매 렌더 스텝): 손끝 3cm 안에 + 높이도 맞는 퍽 + 그리퍼 닫힘 → 집어서 손끝 높이로 들어올림(지도의 `held`, 카메라도 공중 퍽 렌더), 벌리면 그 자리에서 떨어짐(트레이·데크·몸통 위면 안착, 아니면 바닥). 높이 안 맞으면 위를 지나가도 무시. 손끝이 물체 높이에 닿아 팔이 움직여야 밀려남(퍽·바닥장애물, 벽은 안 뚫음; 베이스 이동분은 제외). 바퀴가 바닥 퍽에 닿으면 막지 않고 밀어냄(벽에 끼면 막힘).
- 적재함(`urdf` `tray_link` + `main.py` `TRAY`, 뒤쪽 절반): 내려놓으면 바닥면(0.15m)에 안착, 가이드벽 4면 안쪽으로 가둠(흘러넘침 방지), 로봇이 움직이면 함께 운반. 데크(0.20m)·몸통 윗면(0.145m)도 받침으로 동작. 지도에 외곽 표시.
- 플랜트(`plant.py`): 바퀴는 명령 속도로 가속(한계 × 마찰), 엔코더는 바퀴 회전 그대로지만 바닥 이동은 (1 − 미끄럼)만큼 → `/odom`이 실제와 어긋난다 (rl_nav_correction·EKF가 다룰 문제).
  로봇 원(반경 0.18m)이 벽·장애물·이동물체와 겹치면 그 걸음은 막힌다(충돌). 팔은 관절 속도 한계로 따라가고, 팔 붙잡기면 안 움직이고 과전류(→ arm_safety_monitor).
  통신 끊기면 C를 안 읽고 S를 안 보낸다 → 호스트 진단 "OpenCR 상태(S) 수신 없음" ERROR, 펌웨어 watchdog처럼 0.5초 뒤 바퀴 정지.
- faker를 다시 띄우면 `/odom`이 0부터 시작한다 — [로봇 위치 초기화]로 플랜트·odom을 함께 맞춘다.
- 카메라 렌더: 로봇 URDF(`/robot_description`) 순기구학 + 관절값 + 로봇 위치(플랜트가 돌면 실제 자세, 아니면 `/odom`)로 두 카메라 자세를 구해 광선 추적 (`scene.py`, 2대 병렬로 약 0.3초 — 내부는 절반 해상도로 쏘고 320×240으로 확대).
  화면 타일은 SIM 렌더를 직접 보여준다 (ROS 왕복분 빠름, 타일에 SIM/ROS 표시). `input-live` 파일 → `fake_camera` → ROS 토픽 경로는 그대로 돌아간다.
  URDF visual 상자로 **자기 몸(팔·폴·상판)도 그린다** — 뒤쪽 상판에 실은 퍽 확인용. 렌더 중인 카메라 자체 마운트만 제외.
  화각 `HFOV_DEG`는 `faker/config/faker.yaml`의 `hfov_deg`와 같아야 한다.
- 코스는 저장소의 `course/twin_closed_maze/walls.json`을 읽는다 (네트워크 불필요) — [course/README.md](../course/README.md).
- map·odom 원점 = 미로 시작 칸으로 가정 (위치추정이 붙기 전까지). 퍽은 시작 칸이 있는 절반에만 놓는다 (가운데 벽이 위·아래를 막음).
- 실기 로봇에 붙여 보기: 로봇과 같은 네트워크의 PC에서 `ros2_ws/monitor.sh` (monitor_manager만) → 시뮬레이터가 상태·영상·진단을 표시

## 서비스·액션 입력 (파일로 호출·전송)

`bridge.py` 헬퍼가 `request_id`/`goal_id`(전문번호)를 자동 생성한다. 시작 전 잔재 파일은 무시, 서비스·액션마다 1건씩 순차 처리(처리 중 새 요청은 `busy` 에러).

```python
from bridge import send_service_request, read_service_response, send_action_goal, cancel_action_goal, read_action_file

qid = send_service_request('score_grasps', {'object_id': 3, 'max_candidates': 3})
resp, age = read_service_response('score_grasps')   # resp['request_id'] == qid 확인
# resp → {"request_id": ..., "ok": true, "data": {"scores": [0.87, ...], "grasps": [...]}}

gid = send_action_goal('navigate_to_pose', {'x': 1.0, 'y': 0.5})  # 간편형 x,y(+yaw) 또는 pose 전체
acc, _ = read_action_file('navigate_to_pose', 'accept')      # {"goal_id": ..., "accepted": True}
fb, _ = read_action_file('navigate_to_pose', 'feedback')      # 스트리밍 최신 1건 (faker는 안 냄, 실 Nav2만)
res, _ = read_action_file('navigate_to_pose', 'result')       # {"goal_id": ..., "status": "succeeded|aborted|canceled", ...}
cancel_action_goal('navigate_to_pose', gid)
```

- service `data`: `score_grasps`는 `object_id`(+`max_candidates`), `clear_octomap`은 `{}` (빈 요청), `apply_planning_scene`은 `{}` 또는 `{"is_diff": true}`
- action `data`: `navigate_to_pose`는 `{"x","y"[,"yaw"]}` 또는 `{"pose": {...}, "behavior_tree": ...}`, `execute_task_solution`은 `{"task_id": "..."}` (faker는 task_id만 보고 실행)
