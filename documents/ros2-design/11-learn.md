[목차](README.md) · [◀ 노드 사전](10-catalog.md) · [Isaac 파이프라인 ▶](12-isaac.md)

## 학습 개념: 지도학습 vs 강화학습

지도학습은 정답이 붙은 문제를 보고 맞히도록 배우고, 강화학습은 직접 행동하며 받은 점수로 더 좋은 행동을 찾습니다. **정답을 만들 수 있으면 더 쉽고 안정적인 지도학습을 먼저** 씁니다.

| 기준 | 지도학습 | 강화학습 |
| --- | --- | --- |
| 배우는 것 | 입력 → 정답 예측 | 상황 → 행동 (정책) |
| 신호 | 정답 라벨 | 보상 |
| 데이터 | 미리 모은 데이터 | 학습 중 환경에서 생성 |
| 결과 | 시도 한 번에 결과 한 번 | 여러 행동이 이어진 뒤 쌓임 |
| Isaac의 역할 | 데이터 공장 (Isaac Sim) | 연습장 (Isaac Lab, 학습 내내 실행) |
| 이 시스템 | perception, grasp_scorer | rl_nav_correction, rl_arm_correction, (rl_nav_tuner) |

**파지는 왜 지도학습인가** — 잡기 한 번에 성공·실패가 한 번 확정되고, 모델은 후보의 결과만 예측할 뿐 선택은 coordinator가 한다. 바퀴 보정은 초당 20번 Δ를 주고 경로 이탈의 원인을 여러 Δ에 나눠 따져야 하므로 강화학습이다.

## 노드별 학습 설계

학습 대상은 다섯 개, VLM은 프롬프트로만 씁니다. 학습 후 ONNX로 내보내 ROS 2 노드(또는 컨트롤러) 안에서 추론합니다.

| 노드 | 방법 | 도구 | 입력 | 출력 | 정답 · 보상 |
| --- | --- | --- | --- | --- | --- |
| perception | 지도학습 | Isaac Sim Replicator → YOLO 등 파인튜닝 + 실사진 수백 장 | RGB | 클래스, 박스(·마스크) | 자동 라벨 |
| grasp_scorer | 지도학습 | Isaac Sim 파지 시도 → PyTorch 분류 | 퍽 포인트클라우드, **IK 가능한** 파지 후보 | 성공 확률 0~1 | 들어 흔든 뒤 유지 1, 낙하 0 |
| rl_nav_correction | RL (residual) | Isaac Lab + PPO | 기본 명령, 엔코더 속도, IMU 각속도, 경로 횡·방향 오차, 직전 Δ | Δv, Δω | 경로·속도 추종 오차 −, 보정량 −, 떨림 − |
| rl_arm_correction | RL (residual) | Isaac Lab + PPO | 목표·현재 관절각·속도, 추종 오차, 파지 여부(그리퍼 state interface로 판단), 직전 Δ | 관절별 Δ | 추종 오차 −, 보정량 −, 떨림 − |
| rl_nav_tuner | **규칙으로 시작**, 필요할 때 RL | 규칙 → (Isaac Lab + PPO) | 통로 폭, 최근접 장애물, 목표 거리, 현재 속도 | 프리셋 A/B/C | 빠른 도착 +, 충돌·근접 큰 −, 출렁임 − |
| vlm_task_planner | 학습 없음 | 사전학습 VLM + 프롬프트 | 명령, /tracked_objects, 이미지 | /sub_goals (JSON) | — |

### residual 보정이 배우는 것

목표 정보는 기본 명령 안에 이미 들어 있습니다. 보정 정책은 **명령과 실제 움직임의 차이**만 메웁니다. 예) 미끄러운 바닥에서 엔코더는 0.30rad/s인데 IMU는 0.24rad/s → 미끄러짐으로 보고 각속도를 조금 더 준다. 위치는 맵 좌표 대신 **경로 대비 상대 오차**로 넣어 특정 장소를 외우지 않게 합니다.

```python
obs = [cmd_base_v, cmd_base_w, wheel_v, wheel_w, imu_w,
       lateral_err, heading_err, last_action_v, last_action_w]
delta = action * [0.05, 0.10]                       # action ∈ [-1, 1]
cmd   = clip(cmd_base + delta, -v_limit, v_limit)   # 프리셋·speed_limit 상한으로 재클리핑
reward = (- 2.0 * lateral_err**2
          - 1.0 * heading_err**2
          - 0.5 * (actual_vel - cmd)**2              # 속도 추종: 기본 제어기 종류에 덜 민감
          - 0.1 * (action**2).sum()
          - 0.05 * ((action - last_action)**2).sum())
```

> **비교 기준을 정직하게:** 미끄러짐 보정의 확정적 기본은 EKF(엔코더+IMU 융합)와 Nav2 제어기의 폐루프다. "EKF 켠 상태" 대비 RL 효과를 재야 부풀려지지 않는다.

### rl_nav_tuner 프리셋 — 실행 중 바뀌는 값만 쓴다

DWB의 **critic 가중치(PathDist.scale 등)는 초기화 때 읽혀** 실행 중 바꿔도 반영되지 않을 수 있습니다. 그래서 프리셋은 실행 중 반영되는 두 경로만 씁니다.

| 값 | 경로 | A: 넓은 공간 | B: 좁은 통로 | C: 목표 직전 |
| --- | --- | --- | --- | --- |
| 속도 상한 | /speed_limit (nav2_msgs/SpeedLimit, 백분율) | 100% | 60% (≈0.15m/s) | 40% (≈0.10m/s) |
| FollowPath.max_vel_theta | set_parameters (DWB kinematic) | 1.82 | 1.0 | 0.8 |
| FollowPath.acc_lim_x | set_parameters | 2.5 | 1.5 | 1.0 |
| FollowPath.decel_lim_x | set_parameters | −2.5 | −1.5 | −1.0 |

- 값은 실행 중인 controller_server 메모리에만 반영되고 yaml은 안 바뀐다. 재시작하면 기본값.
- 쓰기 전 `ros2 param set /controller_server FollowPath.max_vel_theta 1.0` 후 실제 동작이 바뀌는지 먼저 확인한다.
- 가중치까지 바꿔야 하면 controller_server에 `FollowPath`·`FollowPathNarrow` 두 플러그인을 등록하고 BT에서 controller_id로 고르게 한다(정적 설정 두 벌).
- rl_nav_correction도 /speed_limit을 구독해 같은 상한으로 클리핑한다 → 보정이 프리셋을 넘지 않는다.

### 랜덤화 범위 예

| 대상 | 랜덤화 항목 | 범위 |
| --- | --- | --- |
| perception | 조명, 배경, 질감, 카메라 노이즈·노출 | 장면마다 |
| grasp_scorer | 퍽 모양·크기·위치·회전, 마찰, 질량 | 원기둥·상자·구 퍽, 마찰 0.3~1.0, 0.1~0.4kg |
| rl_nav_correction | 바닥 마찰, 바퀴 반경, 질량, 명령 지연, IMU 노이즈, **기본 제어기 게인** | 0.3~1.0, ±3%, ±20%, 0~60ms |
| rl_arm_correction | 관절 마찰, 물건 무게, 모터 게인, 지연, 백래시 | 무게 0~0.3kg 등 |
| rl_nav_tuner | 장애물 배치, 통로 폭, 목표 위치 | 통로 0.7~2.0m |

---

[목차](README.md) · [◀ 노드 사전](10-catalog.md) · [Isaac 파이프라인 ▶](12-isaac.md) · [원본 페이지](https://dooboostore.github.io/ros2?concept=design&sec=learn)
