# 4 팔·MoveIt — coordinator skill 정의

coordinator가 실행하는 skill(`SubGoal.skill` 한 단계)의 정의입니다. 구현은 [coordinator.cpp](../../../lv2_module6/ros2_ws/src/plan/src/coordinator.cpp)이고, 아직 없는 것은 **(예정)**으로 표시했습니다.

근거:
- 팀원 팔 스펙 (2026-11-02): 자세 stow·carry, skill 6개, reset_sag, 성공 판정
- 적재함 자세표: [plan/config/basket.yaml](../../../lv2_module6/ros2_ws/src/plan/config/basket.yaml). feature/isaac-lab의 `isaac/lab/slot_table.yaml` 값을 옮긴 것으로, 원본은 `slot_table.py`가 URDF로 오프라인 계산합니다.
- 인지 출력: [API.md TrackedObject](../../API.md#api-msg-trackedobject) (PR #45 — 퍽 `shape`·`yaw`, `location_box`·`start_box`)

## 1. 원칙

1. **팔 동작은 전부 MoveIt을 거칩니다.** coordinator → move_group(계획, pick_ik) → ExecuteTrajectory → `arm_controller`(표준 joint_trajectory_controller) → `rl_arm_correction`(학습 Δ 보정, ±0.05 rad) → 하드웨어.
   - 정해진 관절 자세(stow, carry, 적재함 칸)는 **관절 목표**로 계획합니다.
   - 물체를 기준으로 한 목표(집기, 놓기, 보기)는 **손끝 자세 목표**(end_effector_link, 위에서 아래로)로 계획합니다.
   - JTC 토픽으로 바로 보내는 예외는 두 가지뿐입니다: 기동 때 `initial_pose`, 사람이 조작하는 `manual_control`.
   - "경로표 + JTC 직접"은 보류입니다. 속도나 반복성이 문제가 되면 표를 move_group ExecuteTrajectory로 실행하거나 Pilz planner를 씁니다.
2. **skill 하나는 한 가지 일만 합니다.** 단계 순서(흐름)는 계획을 보내는 쪽이 정합니다. 다만 적재함 싣기·꺼내기는 칸마다 carry 방향이 정해져 있어서, 그 경유를 skill 안에 넣었습니다([5절](#5-흐름)).
3. **판단은 인지 값으로 합니다.** 퍽 위치·모양·방향과 상자 윗면·크기·방향은 실행하는 순간 `/tracked_objects`에서 다시 읽습니다. 계획을 세운 당시의 값은 쓰지 않습니다.
4. **실패하면 이유를 붙여 바로 끝냅니다.** `/task/status`에 FAILED와 `reason`을 냅니다. 다시 할지는 계획 쪽이 정합니다. coordinator는 같은 단계를 `max_step_failures`번까지만 다시 받습니다.

## 2. 자세

| 이름 | 관절값 joint1..5 [rad] | 쓰임 | 값 |
|---|---|---|---|
| `stow` | `[0, -1.91, 1.3701, 0.5952, 0]` | 기본·주행 자세. 기동 때 `initial_pose`도 이 값 | basket.yaml `pose_stow`, SRDF `stow`, plan.yaml `initial_pose` |
| `carry_left` | `[1.5708, -0.4650, 0.1023, 1.9334, 1.5708]` | 퍽을 들고 적재함 왼쪽 칸으로 가는 경유 | basket.yaml `pose_carry_left` |
| `carry_right` | `[-1.5708, -0.4650, 0.1023, 1.9334, -1.5708]` | 퍽을 들고 적재함 오른쪽 칸으로 가는 경유 | basket.yaml `pose_carry_right` |
| `scan` | `[2.618, 0.681, -1.414, 1.082, 0]` | 지도 만들 때 gripper_cam을 두 번째 카메라로 씀. coordinator가 알아서 감 | `ros_system/config/scan_pose.yaml` |

carry 방향은 **칸이 정합니다**. basket.yaml `basket_carry`에서 칸 0·1·4·5·8·9는 right, 2·3·6·7·10·11은 left입니다.

## 3. skill 목록

`SubGoal {step, skill, target, relation, offset_m}`의 `target`은 문자열입니다.

| skill | target | 하는 일 | 끝 자세 |
|---|---|---|---|
| `navigate` | 물체 id · `"x,y"` | Nav2로 이동 (`relation front`와 `offset_m`을 주면 물체 앞으로) | — |
| `inspect` | 퍽 id | 손끝을 퍽 윗면 `inspect_height_m`(0.15m) 위로 옮겨 gripper_cam이 16mm 태그를 읽게 함. 집지 않음 | 그 자리 |
| `say` | 말할 내용 | `/robot/say` | — |
| `pose` | `stow` · `carry_left` · `carry_right` | 이름 있는 관절 자세로 이동 | 그 자세 |
| `pick` | 퍽 id | 위에서 집기: 접근 → 벌림 → 내려감 → 닫기 → **잡기 판정** → 들어올림. 파지 후보는 grasp_scorer가 냄 | 들어올린 자리 |
| `load` | 칸 `0`~`11` · `next` | 손에 든 퍽을 적재함 칸에 싣기: **carry(칸 방향) → q_over → q_low → 벌림 → q_lift → stow** | `stow` |
| `unload` | 칸 `0`~`11` · `last` | 적재함 칸의 퍽 꺼내기: **carry(칸 방향) → 벌림 → q_over → q_grasp → 닫기 → 잡기 판정 → q_over → carry** | `carry_<칸 방향>` |
| `place` | `location_box` id (다른 물체도 됨) | 손에 든 퍽을 상자 윗면에 놓기 ([4절](#place--상자-값)) | 상자 위 |
| `right` | 퍽 id (`puck_lying`) | 누운 퍽을 세워 놓음 | **(예정 — 세우는 방법 미정)** |
| `right_pick` | 퍽 id | 세운 퍽을 다시 집기 (`right` 바로 뒤) | **(예정)** |

- **적재함 12칸**: 칸 위치는 URDF 링크 `basket_slot_0`~`11`(3줄 × 4칸)입니다. `load`·`unload`는 basket.yaml에서 칸과 퍽 모양(`box`·`cyl`)에 맞는 `q_over`·`q_low`·`q_lift`·`q_grasp`를 관절 목표로 씁니다. 적재함은 로봇에 고정돼 있어 매번 같은 값입니다.
  - 손 안 퍽 위치가 기준(`held_bottom_m`)과 다를 때 쓰는 보정(slot_table의 `P_*`)은 아직 쓰지 않습니다. **(예정 — 손 안 퍽 위치를 잴 수 있을 때)**
- **칸 고르기**: `next`는 빈 칸 중 가장 낮은 번호, `last`는 마지막에 실은 칸입니다.
  - coordinator가 칸마다 실린 퍽 모양을 기억해 `/diagnostics` "적재함"에 냅니다(예: `3/12 bc·b········`).
  - 노드가 다시 뜨면 모든 칸을 비었다고 봅니다.
- **손에 든 퍽 모양**: `pick`은 퍽의 `details.shape`(모르면 `box`)를, `unload`는 칸에 기억한 모양을 기억합니다. `load`가 이 모양으로 표를 고릅니다.
- **옛 `tray` (호환)**: `place tray`는 `load next`, `pick tray`는 `unload last`와 같습니다. 그래서 vlm_task_planner와 ai-service의 기존 계획이 그대로 돌아갑니다. 적재함 용량(`tray_capacity`)은 4에서 12로 바꿨습니다.

## 4. skill 안에서 쓰는 값

### 시작할 때: 처짐 보정 초기화
팔 skill(`pick`, `place`, `load`, `unload`)을 **시작할 때 한 번** `/rl_arm_correction/reset_sag`(std_srvs/Trigger)를 부릅니다. skill 중간에는 부르지 않습니다. 다가가기(`approach`)가 끝나고 팔이 움직이기 직전에 부릅니다.
- 서비스가 없으면 WARN을 한 번 남기고 계속합니다. 지금 develop에는 없고 feature/isaac-lab에만 있습니다.
- `right`·`right_pick`도 구현할 때 같은 방식으로 부릅니다.

### 잡기 판정 (`pick`, `unload`)
닫은 뒤 `/joint_states`의 `gripper_left_joint` 위치와 전류(effort)로 판정합니다.
- **위치 ≥ `grasp_min_pos_m`(0.003)**: 퍽이 끼어 손가락이 끝까지 닫히지 않은 상태입니다. 실측으로 퍽에 닿는 곳이 약 5mm(`gripper_left_joint` ≈ 0.005)입니다.
- **전류 |effort| ≥ `grasp_min_effort`(0.25A)**: 실측값은 원기둥 0.33~0.42A, 팀 기준 0.3~0.4A(약 22~29N), 한계 0.6A입니다. effort가 오지 않으면(NaN) 위치만 봅니다.
- **실패 처리**: `pick`은 그리퍼를 벌리고 다음 파지 후보로 넘어갑니다. 후보를 다 써도 못 잡으면 `grasp_empty`로 실패합니다. `unload`는 바로 `grasp_empty`로 실패합니다.
- **기준값**: 문턱은 plan.yaml에 있고 실기에서 다시 맞춥니다.
- **끄기**: launch 인자 `grasp_check:=false`로 끕니다. 테스트베드 faker는 그리퍼가 퍽 없이 끝까지 닫혀 늘 빈손이므로 꺼 둡니다.
- **집는 힘**: 힘을 바꾸는 명령은 펌웨어 변경 뒤에 skill 단계별로 보냅니다. **(예정)**

### pick: 인지 값
| 값 | 출처 | 쓰는 법 |
|---|---|---|
| 퍽 위치 | `pose.position` | grasp_scorer 후보의 기준. 퍽이 `start_box` 위에 있으면 z가 상자 높이(약 0.07m)만큼 높은 그대로 씀 (고정 높이 아님) |
| 모양 | `details.shape` (`box` · `cyl`) | 손에 든 퍽 모양으로 기억해 적재함 표를 고름 |
| 방향 | `pose.orientation`(yaw) · `details.yaw_period` | **(예정)** 사각 퍽은 손 수직축 회전(joint5)을 퍽 yaw(90° 주기, 지금 손 방향에서 가장 가까운 것)에 맞춤. 지금은 수직축 회전을 풀어 둠 |
| 자세 | `class_name` `puck` / `puck_lying`, `details.posture` | **(예정)** `puck_lying`이면 `pick` 대신 `right` → `right_pick` |

### place: 상자 값
| 값 | 출처 | 쓰는 법 |
|---|---|---|
| 윗면 | `location_box` `pose.position.z` · `size.z` | 윗면 높이는 둘 중 큰 값을 씁니다. 윗면을 못 찾고 태그만 보인 경우 z가 태그 높이라서 `size.z`(바닥~윗면)를 씁니다. 손끝 목표 = 윗면 + `held_bottom_m`(손 안 퍽 바닥, 0.0606) + `place_release_m`(0.02). 이 위치로 가서 바로 벌립니다 |
| 크기 | `size` (기본 0.205×0.205×0.07) | 여러 개 놓을 때 비키는 자리(`place_spacing` 0.08)를 상자 가장자리에서 `box_margin_m`(0.03) 안쪽으로 자름 |
| 방향 | `pose.orientation` · `details.yaw_period` | 비키는 배열을 상자 축에 맞춰 돌립니다. 360이면 태그 법선, 90이면 태그 방향을 못 구해 윗면 변 방향입니다. 상자가 정사각형이라 어느 쪽이든 축은 맞습니다. 방향이 없으면 돌리지 않습니다 |

`location_box`가 아닌 대상(구역 등)은 예전처럼 대상 위치 0.10m 위에서 벌립니다.

## 5. 흐름

계획을 보내는 쪽(미션 흐름, VLM)이 단계를 다 적어 보냅니다. coordinator는 단계를 끼워 넣지 않습니다. `load`·`unload` 안의 carry 경유만 예외입니다(칸이 방향을 정하므로).

| 흐름 | 단계 (`/task/sub_goals`) | 팀원 스펙의 표기 |
|---|---|---|
| 집어 싣기 | `pose stow` → `navigate <퍽>` → `pick <퍽>` → `load next` | stow → pick → carry → load → stow |
| 세워 싣기 (예정) | `pose stow` → `navigate <퍽>` → `right <퍽>` → `right_pick <퍽>` → `load next` | stow → right → right-pick → carry → load → stow |
| 꺼내 놓기 | `navigate <상자>` → `unload last` → `place <상자>` → `pose stow` | stow → unload → carry → place |

- 퍽 여러 개: 퍽마다 "집어 싣기"를 반복 → `navigate <location_box>` → "꺼내 놓기" × N.
- 주행(`navigate`) 동안 팔은 `stow`입니다(`load`가 stow로 끝남). 꺼내 놓기 뒤의 `pose stow`는 다음 주행 전에 필요해서 넣었습니다.

## 6. 실패 이유 (`TaskStatus.reason`)

| reason | 언제 |
|---|---|
| `invalid_plan:<이유>` | 계획 검증 실패. 모르는 skill, 모르는 자세 이름(`pose <이름>`), 없는 물체 id |
| `arm_unavailable` · `pick_unavailable` · `nav_unavailable` | MoveIt·그리퍼·grasp_scorer·Nav2 서버가 `server_wait_s` 안에 안 뜸 |
| `nav_failed` · `approach_failed` | 이동·다가가기 실패 |
| `no_grasp` · `grasp_failed_all` · `grasp_empty` · `arm_contact` | 파지 후보 없음 · 후보 다 실패 · 닫았는데 빈손 · 팔 접촉 |
| `pose_failed` | 그 자세로 못 감 |
| `slot_invalid` · `slot_full` · `slot_empty` | 칸 번호가 0~11 밖 · `load` 칸이 차 있음(`next`면 다 참) · `unload` 칸이 빔(`last`면 실린 게 없음) |
| `load_failed` · `unload_failed` | 칸 경유 자세 중 하나를 못 감 |
| `basket_table_missing` | basket.yaml 표가 안 들어옴 (launch 확인) |
| `place_failed` · `place_no_target` · `inspect_no_target` · `inspect_failed` | 놓기·보기 실패, 대상 없음 |
| `manual_*` · `explore_*` · `estop*` · `cancelled` | 모드 전환·비상정지·정지 요청 |
| `right_failed` · `right_not_lying` | **(예정)** 세우기 실패 · 대상이 누운 퍽이 아님 |

옛 `tray_full`·`tray_empty`·`tray_place_failed`·`tray_pick_failed`는 없어졌습니다. 같은 경우에 `slot_full`·`slot_empty`·`load_failed`·`unload_failed`가 나옵니다.

## 7. 남은 것

| 할 것 | 조건 |
|---|---|
| `right`·`right_pick` | 세우는 방법이 정해진 뒤 ([8절](#8-정할-것-팀원-확인) 1) |
| 사각 퍽 집을 때 손 방향을 yaw에 맞춤 | MoveIt 방향 제약에 90° 주기 후보 넣기 |
| 손 안 퍽 위치 보정 (`P_*`) | 손 안 퍽 위치를 잴 수 있을 때 |
| 집는 힘 단계별 명령 | 펌웨어 변경 뒤 |
| 실기 확인 | 칸 12개 load·unload 실측, 잡기 판정 문턱, 상자 위 놓는 높이 |

## 8. 정할 것 (팀원 확인)

1. **`right` 세우는 방법**: 손으로 어떻게 세우는지(밀기, 집어서 돌려 놓기), 경유점을 표로 줄지 pick_ik로 풀지.
2. **꺼내 놓기 뒤 자세**: `place` 뒤 `stow`로 돌아가는 게 맞는지. 스펙은 place에서 끝납니다.
3. **`reset_sag` 단위**: skill마다 한 번(지금 구현)인지, 흐름마다 한 번인지.
4. **잡기 판정 기준**: "다시 잡은 뒤"의 뜻(닫고 한 번 더 조이는지)과 위치·전류 문턱값.
5. **이름 표기**: 스펙의 `right-pick`을 다른 skill과 맞춰 `right_pick`으로 씁니다.

담당 밖이라 따라가기만 할 것: 정책 Δ 전달 방식(50Hz 토픽 vs ONNX), 집는 힘 펌웨어, cuMotion 범위.
