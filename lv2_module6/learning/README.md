# learning — 학습용 skill 구성과 플랜 학습

미션을 풀기 위해 로봇 능력을 학습할 수 있는 작은 skill로 나누고, skill마다 학습합니다.

- **설명 페이지**: https://junekunst.github.io/bee/skill-learning.html — 왜 이렇게 나눴는지, skill마다 상세, 학습 순서
- **실행 코드와의 관계**: 학습 쪽 코드입니다. 로봇에서 실행하는 쪽(coordinator·mission_manager)은 `ros2_ws/src/plan`에 있습니다.

| 파일 | 내용 |
|---|---|
| [skills.yaml](skills.yaml) | skill 37개 목록. 영역, 학습 방식, 기준선, 시작 조건·성공 판정(계약), 보상·손실 항목. task → skill 연결 |
| [plan/mission_sim.py](plan/mission_sim.py) | 가상 미션(gymnasium). `plan_next` 학습용으로, 물리 없이 skill 결과를 확률로 흉내 냅니다 |
| [plan/baseline.py](plan/baseline.py) | 기준선. 지금 mission_manager 규칙을 가상 미션에서 그대로 흉내 냅니다 |
| [plan/solver.py](plan/solver.py) | **계산기 플래너**. 학습 없이 매 걸음 남은 일의 최단 순서를 정확히 계산해 다음 skill을 고릅니다 |
| [plan/train_plan.py](plan/train_plan.py) | `plan_next` 학습(MaskablePPO)과 기준선 비교 |
| [plan/test_mission_sim.py](plan/test_mission_sim.py) · [plan/test_skills_yaml.py](plan/test_skills_yaml.py) | 확인용 |

## 다음 skill 고르기 — 계산기와 학습 모델 두 가지

| | 계산기 `solver.py` | 학습 모델 `plan_next` |
|---|---|---|
| 방법 | 남은 박스·장소를 모두 돌고 복귀하는 최단 경로를 정확히 계산 (부분집합 DP), 매 걸음 다시 계산 | 가상 미션에서 강화학습 (MaskablePPO) |
| 보는 것 | 거리만 (skill은 다 성공한다고 봄) | 성공률·시간까지 (확률로 학습) |
| 장점 | 학습 없음 · 바로 씀 · 결과를 설명할 수 있음 | 실패가 잦을 때 계산기보다 나을 수 있음 |
| 같이 쓰기 | 학습 모델이 처음에 계산기를 따라 하게 함 (`--teacher solver`) | |

둘 다 같은 가상 미션·같은 평가 문제로 비교합니다. 목표(퍽마다 장소·칸)는 둘 다 규칙(mission_solver)이 정합니다.

## plan_next — 다음 skill 고르기

- **범위**: 계산기(할당)가 끝난 뒤부터입니다. 수집 → 배치 → 복귀를 다룹니다. 퍽마다 목표(장소·칸)는 계산기 결과로 주어집니다.
- **행동**: skill 하나와 대상 퍽입니다.
  - 퍽마다 고를 수 있는 skill: 집기, 세우기, 싣기, 꺼내기, 놓기, 밀어 맞추기, 다시 집기, 놓인 퍽 세우기
  - 전체에 하나: 복귀
  - 시작 조건이 안 맞는 행동은 가려서 고를 수 없습니다.
- **보상**: 채점표를 그대로 씁니다. 장소별 주사위 눈 +10, 복귀 +10, 여기에 걸린 시간만큼 작은 벌점을 줍니다. 배터리(3600초)를 넘기면 복귀 점수가 없습니다.
- **확률과 시간**: `PROBS`·`SKILL_S`는 **임시값**입니다. skill별 Isaac 평가 결과가 나오면 그 값으로 바꿉니다.

```bash
pip install gymnasium stable-baselines3 sb3-contrib pyyaml   # CPU만 있으면 됨
cd lv2_module6/learning/plan
python3 test_mission_sim.py && python3 test_skills_yaml.py
python3 train_plan.py eval --episodes 1000                          # 기준선 · 계산기
python3 train_plan.py train --steps 300000 --out plan_next.zip      # 기준선 따라 하기 → 학습 + 비교
python3 train_plan.py train --teacher solver --out plan_next.zip    # 계산기 따라 하기 → 학습 + 비교
```

## 첫 결과 (2026-11-03, 임시 확률, 평가 1000판)

| | 점수 평균 (40점 중) | 만점 비율 | 시간 평균 |
|---|---|---|---|
| 기준선 (지금 규칙) | 38.14 | 82.6% | 1011초 |
| **계산기 (최단 순서)** | **38.11** | **82.4%** | **934초** |
| 강화학습만 | 10.0 | 0% | — (바로 복귀에 갇힘) |
| 기준선 따라 하기 → 강화학습 30만 걸음 | 37.57 | 77.5% | 1180초 |
| 같은 방법, 학습률 낮춤, 100만 걸음 | 37.55 | 77.7% | 1159초 |
| 계산기 따라 하기 → 강화학습 30만 걸음 | 37.15 | 75.6% | 1114초 |

- **학습한 플래너가 아직 기준선을 넘지 못했습니다.** 기준선이 만점을 놓친 174판을 보면, **169판이 퍽을 잃어서**(상자 밖으로 떨어뜨림) 실패했습니다. 잃은 퍽은 어떤 플래너도 되돌리지 못하므로, 이 확률에서는 플래너가 더 올릴 여지가 0.5%뿐입니다.
- **뜻**: 지금은 플래너 학습보다 **skill의 되돌릴 수 없는 실패를 줄이는 것**이 점수에 더 큽니다. skill 학습 우선순위를 정할 때 이 가상 미션을 바로 쓸 수 있습니다. `PROBS`를 바꿔 가며 점수를 다시 재면 됩니다.
- **계산기**는 점수는 기준선과 같고 시간은 8% 짧습니다. 지금은 계산기를 쓰고, 학습 모델은 Isaac에서 나온 실제 성공률로 다시 학습해서 계산기를 넘을 때만 바꿉니다.
- 학습 시간은 30만 걸음에 CPU로 약 80초입니다.
