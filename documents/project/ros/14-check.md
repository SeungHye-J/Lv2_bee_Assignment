# 체크리스트·로드맵

실기 시험 전에 확인할 것과, 지금까지 된 것·남은 것을 한곳에 모읍니다. 명령은 [PR #281 실기 시험 시작 안내](https://github.com/visualkhh/Lv2_bee_Assignment/pull/281)·[DEBUG.md](../../ros/DEBUG.md)·[lv2_module6/README.md](../../../lv2_module6/README.md)에서 가져왔습니다. 시나리오는 [13-scenario.md](13-scenario.md).

## 1. 실기 시험 전 체크리스트

| 구분 | 확인 | 어떻게 | 통과 기준 |
|---|---|---|---|
| 전원 | 로봇·OpenCR·팔 전원 | 켠 뒤 `/dev/opencr`가 보이는지 | OpenCR 장치 확인 |
| 배터리 | 충전 상태 | 방법 TODO: 작성 필요 (PR #44에 배터리 값 추가가 있으나 읽는 명령은 확인 못 함) | — |
| 도메인 | `ROS_DOMAIN_ID` 88 | `start.sh`가 기본 88로 설정 (`${ROS_DOMAIN_ID:-88}`), 다른 PC·TUI도 같은 값 | 같은 도메인에서 토픽이 보임 |
| 카메라 | serial 고정 · 뷰 | `rs-enumerate-devices -s` → `POLE_FRONT_CAM_SERIAL` · `GRIPPER_CAM_SERIAL` 설정, `./tools/run-rviz`로 두 카메라 영상 | pole_front·gripper 둘 다 보임 (pick·load 시험은 gripper 필수) |
| 지도 | 저장 지도 선택 | `run-cli map list` · `map status` (`map switch <이름>`). 실기 기본 `scene_sim` | `LOCALIZATION / scene_sim`, `map/scene_sim/nvblox.nvblx` 있음 (없으면 시작 거부) |
| 놓는 자리 | 출발 관측 위치 · 방향 | 출발 구역 (734, 950 mm)에서 −X | 다르면 `start_pose:=x,y,yaw` |
| 모드 | AUTO | `run-cli status` | AUTO, 장치 정상 |
| 위치 | cuVSLAM · TF | `ros2 topic hz /cuvslam/odom` · `ros2 run tf2_ros tf2_echo map base_footprint` | 들어오고 값이 안정 (C++판: `ps -ef \| grep '[c]uvslam_slam_cpp'`) |
| Nav2 | 서버 active | `run-cli node status planner_server` · `controller_server` | active |
| 충돌 감시 | 켜짐 | `run-cli collision status` (기본 auto: AUTO면 켬) → `/state/collision` | `on:auto` |
| 비상정지 | 시험 | `run-cli estop on` → 바퀴가 멈추는지 → `estop off` → `estop status` | on에서 자동·수동 바퀴 명령 차단, off로 복구 |
| 팔 | stow 상태 | TUI 팔 화면(자세 버튼 stow) · 주행 전 `pose stow` 자동 | 접힌 채 출발, 처짐 없음 (`arm_sag_rad` 0.25) |
| MoveIt | 활성 | 로그에서 move_group 확인 (미설치면 start.sh가 끔) | pick·load 전에 확인 |
| 적재함 | 장부·실물 비어 있음 | `run-cli mission status` · 필요하면 `basket_clear` | 장부 비고 실물도 빔 |
| 녹화 | bag | `run-cli bag start -l <라벨>` (상자로 가기 시작하면 TUI가 자동 녹화) | 저장 위치 `~/.cache/beebot/bags` |
| 주변 | 경기장 | 사람·물건 치움, 손 닿는 곳에 e-stop | — |
| 종료 | | 미션 종료 확인 → bringup `Ctrl+C` 한 번 | 노드 정상 종료 |

- Jetson 메모리·CPU: `run-cli`의 부하 화면(PR #220)에서 노드별 CPU·메모리를 봅니다. 11-15 실기에서 제어 주기가 최저 3.5 Hz까지 떨어졌습니다(T-23).
- 시험 직전 CI도 확인합니다. main 병합 직후 `test-all`이 plan.yaml 파싱 오류로 실패한 기록이 있습니다(T-26, 커밋 c9002372).

## 2. 로드맵 — 된 것

병합된 PR 제목 기준입니다. 근거는 [project2/results/test-results.csv](../../../project2/results/test-results.csv) 시험 ID.

| 영역 | 된 것 | PR | 시험 |
|---|---|---|---|
| SLAM | cuVSLAM + nvblox (정지 drift 0.5 mm, 추적 잃음 0) | #40 · #43 | T-01 |
| 인지 | 퍽 모델 v4 → v5, 신호등 12회 점멸 해석, 상자 인지, C++ 전환, TensorRT FP16, cuAprilTags | #63 · #144 · #45 · #164 · #246 · #226 | T-03 · T-04 · T-19 · T-21 |
| 미션 | mission_manager·mission_solver, ExecutePlan Action, 진단 모드(DIAG), 방문 순서 최적화(−21 %) | #49 · #195 · #202 · #146 | T-25 |
| 팔 skill | pose·load·unload·stand·inspect·pick_far·look 자세 | #48 · #192 · #80 · #287 | T-05 · T-06 · T-24 |
| 주행 | RL 주행 컨트롤러(ONNX), 25초 갇힘 감시, 인지 기반 상자 도킹, 경유점 지도·NavigateThroughPoses, collision_monitor 조정 | #82 · #145 · #203 · #260 · #274 · #243 | T-09 · T-11 · T-12 · T-14 |
| 부하 | 주행 중 인지 줄임, nav2 컨테이너·py_helpers 묶기, C++ 보조 노드 | #244 · #217 · #272 | T-22 · T-23 |
| 운영 | TUI 미션·부하·route 녹화 화면, bag 자동 녹화, field-test.py, 실기 시작 안내 | #199 · #220 · #264 · #237 · #173 · #281 | — |

## 3. 로드맵 — 남은 것 (open Issue)

`gh issue list --state open` 기준(2026-11-17). 담당은 Issue 지정자.

| 영역 | Issue | 내용 |
|---|---|---|
| 시연 | #251 | 시나리오에 맞게 진행 여부 체크 연습·시연 담당 |
| 검토(주행) | #190 · #191 · #189 | 벽 어깨빵 우려로 접근 못 하는 경우 · 지나가지 못할 자리 미리 막기 · Jetson 메모리 조치 |
| 계획·coordinator | #131 · #130 · #105 · #104 · #12 | 단계 타임아웃 확인 · skill 미기동 이유 구분 · 재계획 제거 뒤 연속 실패 · plan_next 실기 확인 · 플랜 처리 방안 |
| skills | #135 · #118 · #112 · #110 · #107 | skill 액션 서버 단위 테스트 없음 · inspect→load→pick 연속 · stand · load·unload 연동 · pick·unload |
| RL | #119 · #117 · #115 · #99 · #98 · #13 | 실기 pick 정책 off·on 비교 · 누운 퍽 세우기 학습 · 잔차 RL 단계 학습 · 먼 퍽 진입 · stand_grasp 실기 검증 · 보정·속도 학습 |
| 제어·도구 | #153 · #150 | 매니퓰레이터 떨림 원인 · TUI 실기 시험용 정리 |
| 인지·모델 | #28 · #26 · #114 | cuVSLAM 검토 · 지도 맵핑 실기기 테스트 · 로봇 모델·Isaac 장면 |

## 4. 아직 비어 있는 것 (시험 기록 기준)

- 실기에서 DONE까지 가 본 기록이 없습니다(T-07·T-08이 SURVEY 단계에서 막힌 기록, 이후 개선은 구간별 시험 T-09~T-12만).
- 인지 모델 v5(T-04)는 이미지 시험이고, 실기 주행 중 검출은 별도 기록 없음.
- 배터리 확인 방법·충전 기준은 TODO: 작성 필요.
