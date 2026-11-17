[목차](README.md) · [◀ 노드 사전](10-catalog.md)

## 학습 설계

원칙은 [00-overview.md](00-overview.md)와 같습니다. 학습 모델은 **앞에서 자문하거나 뒤에서 보정만** 하고, 확정 실행은 Nav2·MoveIt·ros2_control이 합니다. 모든 학습 컴포넌트는 꺼도(바이패스) 기준 동작으로 돌아갑니다.

### 모델 파일 위치와 설치

| 모델 | 저장소 위치 | 설치 위치 (colcon build 때 복사) |
| --- | --- | --- |
| RL 주행 | `lv2_module6/models/rl_nav_controller/policy.onnx` (+ `policy.onnx.data`) | `share/ml/model/rl_nav_controller/` (`ml/CMakeLists.txt`) |
| skill 정책 12개 폴더 | `lv2_module6/models/skills/<skill>/` — `meta.yaml` · `<skill>.onnx` · `<skill>.policy` | `share/skills/model/` (`skills/CMakeLists.txt`) |
| plan_next | `lv2_module6/models/plan/plan_next.onnx` · `plan_next.policy` | `share/ml/model/plan/` |
| YOLO | `lv2_module6/models/yolo/` — `bee_yolo11n_v5.onnx` · `bee_yolo11n_v6.onnx` · `.pt` 여러 개 | `share/perception/model/` (`perception/CMakeLists.txt`; ml은 yolo를 중복 설치 안 함) |
| grasp_scorer | `lv2_module6/models/grasp_scorer/` (비어 있음 — 학습 모델 없음) | — |

정책을 켜고 끄거나 모델을 바꾸면 `colcon build`가 필요합니다(`install/`로 복사되므로). 학습 로그·가중치 원본은 `lv2_module6/isaac/rl/policies/<skill>/vNN/`(`.pt` · `.onnx` · `manifest.yaml`)에 버전별로 있습니다.

### 요약

| 컴포넌트 | 학습 방식 | 켜고 끄는 법 | 바이패스 시 |
| --- | --- | --- | --- |
| RL 주행 `ml::RlNavController` | Isaac Lab 학습 정책 (ONNX) | `nav2_params.yaml` `FollowPath.enable` · 서비스 `~/FollowPath/toggle` | inner controller (`fallback_controller`: 현재 `dwb`) |
| RL 팔 보정 `ml::RlArmCorrection` | skill별 정책 + 처짐 적분·중력 앞먹임 | `controllers.yaml` `rl_arm_correction.enabled` · launch `learning_enabled` · skill별 `meta.yaml use` | Δ = 0 (정책 없음) |
| skill 정책 (pick 등) | Isaac Lab PPO (rsl_rl) | `models/skills/<skill>/meta.yaml`의 `use`, `contact_mode` | 기준 동작만 / MoveIt만 |
| `grasp_scorer` | 지금은 규칙 (모델 없음) | `ml.yaml` `grasp_scorer.enabled` | 규칙 기반 점수 (둘 다 규칙) |
| `plan_next` | MaskablePPO (가상 미션) | `plan.yaml` `planner: rule \| plan_next` | 규칙 순서 (기본) |
| YOLO | 별도 학습 (이 저장소엔 학습 코드 없음) | `perception.yaml` `model_path` | `""`이면 팀 기본 `model.pt` |

### 1. RL 주행 — `rl_nav_controller`

- 코드: `ml/src/rl_nav_controller.cpp`, `ml/include/ml/nav_obs.hpp`, 플러그인 `ml/rl_nav_controller.xml`. Nav2 `controller_server`가 `FollowPath` 플러그인으로 로드합니다.
- 입력(관측 375): 웨이포인트 5×2 + 가상 스캔 360(nvblox 2D ESDF height slice `/nvblox_node/combined_esdf_pointcloud`를 base_footprint 기준 360방향 최소거리로) + v · w · gyro + 이전 행동 2. 정규화 값(`wp_scale` · `lidar_max` · `max_lin` · `max_ang`)은 학습과 같아야 합니다.
- 출력: 행동 [-1,1]² → 선속도 `a0 × max_lin`, 각속도 `a1 × max_ang` (`extra_lin_vel` · `extra_ang_vel`로 추가 가능, 현재 0).
- 바이패스: `enable: false`(현재 `nav2_params.yaml` 값 — 11-16 시뮬에서 정책이 회전 반경이 커서 DWB가 낫다고 판단), 모델 파일 없음, 점구름이 `cloud_timeout`(1.5 s)보다 오래됨 → inner controller. 실행 중 `~/FollowPath/toggle`(SetBool) · `~/FollowPath/set_fallback`로 바꿉니다.
- 모델 경로: `model_path: "rl_nav_controller/policy.onnx"` (share/ml/model 기준).
- 테스트: `ml/test/test_nav_obs.cpp` · `test_nav_policy_onnx.cpp` · `test_fallback_controller.cpp`.
- 학습 환경: 주행 환경은 `isaac/rl/bee_isaac/tasks/drive_env.py`(`--task drive`)로 문서에 적혀 있으나([lab.md](../../../lv2_module6/isaac/docs/lab.md)), 이 정책(375 입력)을 어느 환경으로 학습했는지는 코드로 확인하지 못했습니다.

### 2. RL 팔 보정 — `rl_arm_correction`

- 코드: `ml/src/rl_arm_correction.cpp`, `ml/include/ml/rl_arm_correction.hpp`, `ml/include/ml/arm_obs.hpp`. 하드웨어 앞에 있는 chainable controller라 `arm_controller → rl_arm_correction → 하드웨어` 순서입니다([05-arm.md](05-arm.md)).
- 입력: `arm_controller`의 reference + `/skill_context`(`ml/msg/SkillContext`, skill 노드의 `arm_io.cpp`가 발행 — 정책 파일 경로·layout·단계 phase 포함). 정책 관측은 layout별로 `place` 39 · `pick` 41 · `pick_box` 67 · `pick_far` 68 (`arm_obs.hpp`, `meta.yaml` 주석).
- 출력: `policy_hz`(50) 마다 a ∈ [-1,1] → Δ = `residual_rad × a`, 한 스텝 변화 `max_delta_step`, 상한 `max_delta`(0.05 rad). 닿는 구간에서만 정책을 쓰고 그 밖에는 Δ = 0.
- 정책이 없어도 도는 부분: 처짐 적분(`sag_enabled`, 관절별 `sag_joints`)과 중력 앞먹임(`gravity_enabled`)은 정책과 별개로 항상 적용됩니다.
- 켜고 끄기: `controllers.yaml`의 `rl_arm_correction.enabled`, launch `learning_enabled`(bringup → ml), skill마다 `meta.yaml`의 `use`. 테스트베드는 `arm_correction_type:=faker/FakeRlArmCorrection`으로 대역 플러그인을 씁니다(`faker.launch.py`의 `rl_arm_correction:=fake` 기본).
- 학습 환경: `isaac/rl/bee_isaac/correction.py`가 실기 `rl_arm_correction`과 같은 보정(처짐 적분 + 잔차 + 중력 앞먹임)을 모든 skill 환경에 공유합니다.

### 3. skill 정책 (pick · load · stand 등)

- 로드: 각 skill 노드(`skills/src/skill_node.cpp`의 `load_model`)가 `model_dir`(기본 `share/skills/model`)의 `<skill>/meta.yaml`을 읽고, `use: true`이며 파일이 있을 때만 정책으로 씁니다. `runtime: onnx`(기본) 또는 `policy`(같은 정책의 텍스트 MLP). 불러올 때 검산합니다(`skills/include/skills/model.hpp`).
- 종류: `kind: arm`은 `rl_arm_correction`이 계산, `kind: base`는 `skill_approach`가 계산(meta 주석). `/skill_context`로 정책 경로가 전달됩니다.
- `contact_mode` (`skills/params/arm.yaml`, 현재 `sim`): `sim` = 퍽·칸에 닿는 구간은 시뮬과 같은 기준 동작 + 학습 보정, `moveit` = 예전처럼 MoveIt만 (`pick_skill.cpp` · `place_skill.cpp` · `arm_io.cpp`).
- 현재 폴더별 `use` (`models/skills/*/meta.yaml`):

| `use: true` | `use: false` |
| --- | --- |
| `pick_standing_puck` (v03, `pick_box` 67) · `place_cell` · `stand_grasp` · `drive_straight` · `drive_arc` · `dock_box` · `park_home` | `load_slot` · `unload_slot` · `regrasp_after_stand` · `pick_standing_far` (11-13 실기에서 퍽 윗부분으로 가 놓침) · `pregrasp` (정책 파일 없음) |

- 시뮬 성공률(`meta.yaml` · [lab.md](../../../lv2_module6/isaac/docs/lab.md) 표): pick_standing_puck 기준 동작만 53.1% → 정책 98.7%(1024판, seed 7). meta의 `note`처럼 학습 `delta_step_rad`(0.004)와 `rl_arm_correction.max_delta_step`(실기 0.003)을 맞춰 돌려야 합니다.
- 학습 환경: `lv2_module6/isaac/rl/bee_isaac/tasks/*_env.py` (`pick_env` · `slot_env`(load/unload) · `right_env`(stand) · `stand_grasp_env` · `place_env`/`pip_env` · `reach_env` · `drive_env`). 실행은 `isaac/rl/scripts/train.py --task <이름>`(rsl_rl PPO), 큐는 `train_queue.sh`. Isaac Lab 6.1 venv(`~/pa-course/IsaacLab-6.1/.venv`)에서 돕니다(ROS 안 씀). 버전별 산출물은 `isaac/rl/policies/<skill>/vNN/`, ONNX 변환은 `isaac/rl/policies/export_onnx.py`, 텍스트 `.policy` 변환은 `ml/scripts/export_policy.py`.
- skill 목록·기준선·보상은 `isaac/learning/skills.yaml`(method: rl · yolo · rule · measured).

### 4. 잡기 채점 — `grasp_scorer`

- 서비스 `/score_grasps`(`ml/srv/ScoreGrasps`). 입력 `/tracked_objects`(퍽 위치·크기·방향), `/gripper_cam/depth/color/points`. 출력은 후보 손 방향 ψ와 점수.
- 현재는 규칙(`rule_grasps`)입니다. 헤더 주석에 "학습 모델이 생기면 enabled일 때 모델 점수"라는 TODO가 있고 `models/grasp_scorer/`는 비어 있어 `enabled`를 켜도 규칙만 돕니다(`grasp_scorer.cpp`: `res.message = "rule: …"`).
- 호출: `skill_pick` · `skill_pick_far`. 도달 가능성은 검사하지 않고 실패 시 coordinator가 다음 후보로 넘어갑니다.

### 5. 학습된 플래너 — `plan_next`

- 코드: `plan/include/plan/plan_next.hpp`, 호출은 `plan/src/mission_manager.cpp`. 입력 167(퍽 12개 × [상태 one-hot 8 + 장소 3 + 가운데 칸 1 + 시도] + 로봇 자리 10 + 시간), 출력 97(퍽 12 × skill 8 + 복귀). 시작 조건이 안 맞는 행동과 연달아 3번 변화 없던 행동은 막습니다.
- 토글: `plan.yaml`의 `planner: "rule"`(기본, 배정 순서대로) | `"plan_next"`, `plan_model: ""`이면 `share/ml/model/plan/plan_next.onnx`(`.policy` 경로도 가능). 불러올 때 입력 167·출력 97을 검산하고 실패하면 "규칙 순서로" 경고 후 규칙으로 돌아갑니다. 박스마다 퍽 2개가 아니어도 규칙으로 돌아갑니다.
- 학습: `isaac/learning/plan/` — 가상 미션 `mission_sim.py` + `train_plan.py`(sb3-contrib MaskablePPO, CPU), 기준선 `baseline.py` · `solver.py`. 로봇 ONNX 평가는 `train_plan.py eval --model …/plan_next.onnx`. C++과 Python 결과 일치는 `plan/test/test_plan_next.cpp`.

### 6. YOLO 인지

- 노드: `perception_node` (C++, ONNX Runtime). `perception.yaml`의 `model_path`는 현재 `bee_yolo11n_v6.onnx`(퍽 + 상자, 입력 384×640 고정). `bee_yolo11n_v5.onnx`(상자 없는 퍽 모델)·`.pt`·`""`(팀 기본 `model.pt`)로 되돌릴 수 있습니다.
- 출력: `/tracked_objects` 등 ([10-catalog.md](10-catalog.md)). 클래스는 v5 기준 퍽(빨강·초록 × 사각·원기둥) · 신호등 상태 4 · person · obstacle, v6은 여기에 상자.
- 추론 장치: launch `device` 기본 `cuda`(안 되면 CPU), `cpu_threads: 2`. `tensorrt: true`면 `~/.cache/beebot/trt/<모델>_fp16.engine`이 있을 때만 TensorRT(없으면 ONNX Runtime). 엔진은 그 Jetson에서 `trtexec --onnx=… --saveEngine=… --fp16`로 만들어야 합니다(저장소에 안 올림). `tensorrt_cuda_graph: true`는 TensorRT 추론을 CUDA 그래프 하나로 띄웁니다.
- 학습: 이 저장소에 YOLO 학습 코드는 없습니다(`perception.yaml` 주석에 "yolo_puck runs"만 언급).

### ONNX Runtime · TensorRT 메모

출처: [ONNXRUNTIME.md](../../jetson-orin-nano/ONNXRUNTIME.md), `ml/src/onnx_policy.cpp`.

- 정책 추론(`ml/src/onnx_policy.cpp`)은 CUDA EP가 있으면 GPU, 실패하면 CPU로 폴백합니다. 로그 `ONNX loaded: … device=CUDA|CPU`. 환경변수 `ML_ONNX_CUDA=0`이면 CPU 고정(작은 MLP는 CPU가 더 빠를 수 있음).
- `ros-lyrical-onnxruntime-vendor`는 CPU 전용입니다. GPU를 쓰려면 ONNX Runtime 1.24.3을 CUDA EP로 직접 빌드해야 하고(Jetson Orin `sm_87`, `CMAKE_CUDA_ARCHITECTURES=87`), `ml`의 `ML_ONNX_RPATH` / `ONNXRUNTIME_ROOT` 설정으로 그 라이브러리를 보게 합니다.
- 정책 MLP에는 TensorRT EP를 쓰지 않습니다(문서: 이득이 거의 없음). TensorRT는 YOLO(`perception_node`)에서만 직접 엔진으로 씁니다.

### 확인 못 한 것

- 요청에 있던 "바퀴 쏠림(drift) 보정"을 하는 학습 컴포넌트는 코드에서 찾지 못했습니다. `rl_nav_controller`는 경로 추종 정책이고, 바퀴 보정 상수는 `controllers.yaml`의 diff_drive 값(`wheel_separation_multiplier` 등)입니다.
- RL 주행 정책 `policy.onnx`를 학습한 정확한 환경·커밋, YOLO 학습 데이터·스크립트 위치.
- `inhand_ox` · `inhand_yolo` 정책(`isaac/rl/policies/`)이 로봇 쪽 어디에 쓰이는지(`models/`에 없음).
- 현재 배포 `use` 값이 실기 최신인지는 `meta.yaml` 기준일 뿐이며 [lab.md](../../../lv2_module6/isaac/docs/lab.md)의 11-15 표와 일부(예: load_slot·unload_slot) 다를 수 있습니다.
