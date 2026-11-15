# isaac

Isaac Sim 장면·로봇 USD(`usda/`)와 Isaac Lab 학습(`rl/bee_isaac/tasks/`). 로봇은 `../../urdf/beebot_mesh/beebot.urdf`를 Isaac Sim URDF Importer로 가져와 만든다.

## 목차

1. [isaac_sim.sh 처음 실행](../standalone/README.md) — 준비 · 실행 · 문제 해결
2. [시연](demo.md) — Isaac Sim + ROS 실행 명령
3. [학습](lab.md) — 팔 RL 잔차, skill별 학습 환경, 결과
4. [USD](usd.md) — 장면·로봇 USD 구조, 퍽·상자·미로 물리, 텍스처
5. [시험 반복](calib.md) — launcher · pick/place/stand loop (실기 · Isaac). 스크립트별 실행법은 [loops/README.md](../loops/README.md), skill 파라미터는 [loops/pick_load_stand_params.md](../loops/pick_load_stand_params.md)
6. [Isaac 전체 미션](../mission/README.md) — 미션 한 판 · 신호 조건 · Nav2 벽 지도

## 폴더 구조

```
isaac/
├── pyproject.toml      패키지 bee_isaac (설치: 아래)
├── rl/                 RL 전부
│   ├── bee_isaac/      시연·학습 공용 코드
│   │   ├── constants.py    경로 값 (표준 라이브러리만 — ROS 쪽도 씀)
│   │   ├── common.py       IK·궤적·치수 (pinocchio)
│   │   ├── correction.py   실기 rl_arm_correction과 같은 팔 보정 (처짐 적분 + 잔차 + 중력 앞먹임) — 모든 skill 환경 공용
│   │   ├── stand_ik.py     5축 팔 torch 배치 FK · DLS (stand_grasp)
│   │   ├── assets/         로봇 설정 (beebot.py)
│   │   ├── tasks/          Isaac Lab 학습 환경 (*_env.py)
│   │   └── data/           slot_table
│   ├── scripts/        train.py · train_queue.sh · slot_table.py · stand_plan.py
│   │   └── pick_far/   먼 퍽 도달성 분석 (45° 기울여 진입 vs 수직 하강) — REPORT.md · 스크립트 · 그림
│   ├── policies/       학습한 정책 <skill>/vNN/ (.pt · .onnx · manifest.yaml) · export_onnx.py — 버전 폴더는 git에 올림
│   └── logs/           rsl_rl 학습 런 (git 제외)
├── learning/           plan_next(다음 skill 고르기) 학습 — 가상 미션 · MaskablePPO (learning/plan/) · skills.yaml
├── standalone/         Isaac Sim 실행: isaac_sim.sh (진입점, README.md) · runtime/ (isaac_startup · opencr_bridge · launch · camera_adapter · arena_map_pub · node_params · reset.sh · inspect_pose · inspect_cover)
├── loops/              시험 반복 (README.md · manipulator-launcher/record · pick/place/stand_loop · pick_coord · goto · slot_check · approach_loop/batch · vis/ RViz · ros2_ws/sine_test — logs/는 git 제외, real_logs/는 실기 approach 결과라 git에 올림)
├── mission/            Isaac 전체 미션 (README.md · run_mission · 장면 벽 지도 map/ · plan_to_map · map_to_nvblox · RViz)
├── scripts/demo/       옛 시연 pick_load_play.py (쓰던 manipulator_env.py가 지워져 지금은 안 돎)
├── usda/               USD 파일 (usd.md)
├── source/             장면용 이미지 (HDRI · texture) — usda가 ../source로 참조
└── docs/               문서 (README.md가 목차)
```

**실행 환경이 둘이다.** `tasks/`·학습은 Isaac venv (numpy 2), `camera_adapter.py`·launch는 시스템 python + ROS (numpy 1.26). `constants.py`만 양쪽에서 import한다.

## 설치

쓰는 venv마다 한 번 (editable — 코드 수정은 바로 반영):

```bash
uv pip install --python <isaaclab venv python 경로> --no-deps -e lv2_module6/isaac
uv pip install --python ~/pa-course/IsaacLab-6.1/.venv/bin/python --no-deps -e lv2_module6/isaac
```

워크트리를 옮기거나 지우면 다시 설치한다.
