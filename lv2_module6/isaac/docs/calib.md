[목차](README.md) · [◀ USD](usd.md)

# 실기 측정 · 시험 반복

시험 스크립트는 `loops/`에 있다. 실기와 Isaac Sim(`use_sim`) 모두 같은 스크립트를 쓴다. 스크립트별 명령 · 인자 · 결과 파일은 [loops/README.md](../loops/README.md)가 기준이고, 시험 중에 바꾸는 skill 파라미터는 [loops/pick_load_stand_params.md](../loops/pick_load_stand_params.md)에 있다. 이 문서는 전체 그림만 적는다.

| 파일 | 역할 |
|---|---|
| `manipulator-launcher.sh` | 팔 · 카메라 2대 · 인지 · MoveIt · coordinator · skill 노드 bringup (SLAM · Nav2 없음). `use_sim:=true`면 Isaac 대상 |
| `manipulator-record.sh` | 팔 전용 rosbag 기록 |
| `pick_loop.py` · `pick_coord.py` | 집기(+ 적재) 반복 · coordinator를 거친 상자 비우기 |
| `stand_loop.py` | 누운 퍽 세우기 반복 (기본은 실제 인식, `--no-perception`이면 가짜 누운 퍽) |
| `place_loop.py` | 놓기 반복 (`--from slot` 적재함에서 꺼내서 · `--from pick` 상자에서 집어서) |
| `goto.py` · `slot_check.py` | 정해진 자세로 보내고 진동 측정 · 적재함 칸 자세 오차 기입 |
| `approach_loop.py` · `approach_batch.sh` · `approach_summary.py` | 상자까지 주행해 붙기 (Isaac 기본, `--real` 실기) · 반복 · 요약 |
| `arm_model.py` | URDF 기구학 · 중력 토크 (ROS 쪽 `rl_arm_correction`의 기준값) |
| `ros2_ws/src/sine_test` (빌드: `cd ros2_ws && colcon build`) | 사인 · 프로파일 응답 측정용 ROS 패키지 (`hw.launch.py`) |

## 공통 규칙

- 결과는 `loops/logs/`에 쌓인다(git 제외, 로컬에만 있음). 실기 approach 결과 `loops/real_logs/`만 git에 올린다.
- 정책 켜고 끄기는 스크립트 옵션이 아니라 `models/skills/<skill>/meta.yaml`의 `use:` + `colcon build`로 한다.
- 스크립트를 돌리는 셸의 `ROS_DOMAIN_ID`와 Fast DDS 프로파일이 스택과 같아야 한다.

## Isaac Sim 대상

`isaac/standalone/isaac_sim.sh`를 띄우고 Play한 뒤 `./manipulator-launcher.sh use_sim:=true`를 띄운다. 스크립트는 `--use_sim true`로 실행한다.

- 줄자 대신 Isaac 참값을 쓴다. `isaac_startup.py`가 재생 중 퍽 위치 · 기울기를 `/tmp/beebot_sim_pucks.json`에 쓰고, `pick_loop.py`가 인식한 퍽마다 가장 가까운 참값(6 cm 안)을 골라 `perc_err_mm`를 채운다. 자세(서 있음 · 누움 · 기울기)는 CSV `true_posture`에 남는다.
- 파일이 2초보다 오래됐거나 없으면 줄자 입력으로 돌아간다(`--auto` 제외). 옛 `isaac_startup.py`로 띄운 Isaac이면 이 파일이 없으니 `isaac_sim.sh`를 다시 띄운다.
- 먼 퍽(퍽 위 5 cm 수직 자세에 못 닿는 범위)과 대안은 `rl/scripts/pick_far/REPORT.md`.
