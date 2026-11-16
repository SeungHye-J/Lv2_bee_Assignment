# loops — 실기·시뮬 반복 시험 실행법

이 폴더의 시험 스크립트를 어떻게 띄우는지 모았다. 모든 명령은 이 폴더(`isaac/loops`)에서 실행한다고 가정한다.
각 스크립트의 자세한 옵션은 `python3 <스크립트> --help`(파일 맨 위 설명)에 있다. 시험 중에 바꾸는 skill 파라미터는 [pick_load_stand_params.md](pick_load_stand_params.md)에 정리돼 있다.

## 공통 준비

| 대상 | 먼저 띄울 것 |
| --- | --- |
| 실기 팔 시험 (pick · load · stand · place · goto) | 터미널 1에서 `./manipulator-launcher.sh` (OpenCR · 카메라 2대 · 인식 · MoveIt · skill 노드, SLAM·Nav2 없음) |
| 시뮬 팔 시험 | `../standalone/isaac_sim.sh`를 띄운 뒤 `./manipulator-launcher.sh use_sim:=true`, 스크립트에는 `use_sim=true` |
| 시뮬 approach 시험 | `../standalone/isaac_sim.sh <scene>`이 Play 중이고 coordinator가 AUTO |

- `manipulator-launcher.sh`는 시작할 때 `ros2_ws/stop.sh`로 **자기 `ROS_DOMAIN_ID`의 ROS만** 끈다 (`STOP_DOMAIN`, 11-15). 다른 도메인에서 도는 시뮬은 그대로 둔다. `stop.sh`를 손으로 그냥 돌리면 모든 도메인이 꺼지니 `STOP_DOMAIN=<id> ../../ros2_ws/stop.sh`로 쓴다.
- 스크립트를 돌리는 셸의 `ROS_DOMAIN_ID`와 Fast DDS 프로파일이 스택과 같아야 한다. 스크립트 설명에 있는 `ROS_DOMAIN_ID=88` 접두어는 셸이 이미 같은 값을 쓰면 빼도 된다.
- 학습 정책 켜고 끄기는 스크립트 옵션이 아니라 `models/skills/<skill>/meta.yaml`의 `use:`와 colcon build로 한다.
- 결과는 모두 `logs/`에 쌓인다(git 제외). 대부분 라운드마다 bag을 함께 남긴다(`--bag off`로 끔).
- 팔은 skill 노드나 JTC가 직접 움직이므로, 첫 실행은 손을 비상정지 가까이에 두고 본다.

## 팔 시험

### pick_loop.py — 집기 (+ 적재)

인식으로 상자 위 퍽을 찾아 집고, 인식 오차와 손 안 어긋남을 기록한다. `--slot`을 주면 집은 퍽을 적재함 칸에 차례로 싣는다.

```bash
python3 pick_loop.py --box-x 0.22 --box-y 0.0 --rounds 0          # 집기만, q / Ctrl-C까지 반복
python3 pick_loop.py --pucks 2 --slot 0                           # 라운드마다 상자 비우기: 칸 0, 1, ...
python3 pick_loop.py --pucks 2 --slot 0 --auto                    # 멈춤 없이 한 판, 질문은 끝에 한 번
```

| 인자 | 기본값 | 뜻 |
| --- | --- | --- |
| `--box-x` | 0.22 | 상자 가운데, base_footprint 앞쪽 [m] |
| `--box-y` | 0.0 | 상자 가운데, base_footprint 왼쪽 [m] |
| `--support` | 0.07 | 상자 윗면 높이 [m] |
| `--puck-h` | 0.06 | 세운 퍽 높이 [m] |
| `--pucks` | 1 | 상자 위에 있을 퍽 수. inspect가 이만큼 보면 멈춤 |
| `--rounds` | 0 | 반복 횟수. 0 = q / Ctrl-C까지 |
| `--slot` | -1 | 집은 퍽을 이 칸(0~11)부터 차례로 load. -1 = 하나만 집고 상자 위에서 손을 엶 |
| `--auto` | 끔 | `--slot`과 같이: 한 판을 멈춤 없이 pick → load 반복, 질문은 끝에 |
| `--bag` | `cam` | 라운드마다 bag. `cam` · `nocam`(영상 없음) · `off` |
| `--use_sim` | false | Isaac Sim 로봇. 노드와 bag이 sim time(`/clock`) |

- 상자 가운데를 base_footprint 기준 `--box-x`/`--box-y`(기본 0.22, 0)에 둔다. base_footprint에서 0.30 m보다 먼 퍽은 `approach_failed`.
- 라운드마다 인식 위치를 보여 주고 자로 잰 값(mm)을 묻는다. Enter는 건너뛰기.
- 결과: `logs/pick_loop_<ts>.csv`, `logs/pick_loop_<ts>_bags/round_NN/`.

### pick_coord.py — coordinator를 거친 상자 비우기

접근이 끝난 상자를 coordinator(`/task/execute_plan`)로 비운다. 모든 단계가 coordinator를 거치므로 실제 미션과 같은 대체 동작이 들어간다: pick이 못 닿으면 pick_far, 누운 퍽은 stand → stand_pick, `load next`가 칸을 고르고 적재함 장부를 적는다. coordinator가 AUTO여야 한다 (아니면 첫 inspect에서 멈추고 알림).

```bash
python3 pick_coord.py --box-x 0.21 --box-y -0.015            # 상자 가운데를 인식 값에 맞춤
python3 pick_coord.py --box-x 0.25 --box-y 0.03 --max 3
python3 pick_coord.py use_sim=true
```

| 인자 | 기본값 | 뜻 |
| --- | --- | --- |
| `--box-x` | 0.22 | 상자 가운데, base_footprint 앞쪽 [m] |
| `--box-y` | 0.0 | 상자 가운데, base_footprint 왼쪽 [m] |
| `--support` | 0.07 | 상자 윗면 높이 [m] |
| `--puck-h` | 0.06 | 세운 퍽 높이 [m] |
| `--box-r` | 0.15 | 상자 가운데에서 이 안의 퍽만 상자 위로 셈 [m] |
| `--stand-tries` | 3 | 누운 퍽 하나당 stand 시도 횟수 (미션 retries 2 + 1) |
| `--stand-check-s` | 1.0 | stand 뒤 섰는지 다시 볼 때까지 기다림 [s] (plan.yaml `stand_check_s`) |
| `--max` | 12 | 퍽 이만큼 처리하면 멈춤 |
| `--load-retries` | 1 | 쥔 채 load가 실패하면 상자에 되돌려 놓고 다시 집는 횟수 (퍽마다) |
| `--bag` | `cam` | 실행 전체를 bag 하나로 (`manipulator-record.sh`). `cam` · `nocam` · `off` |
| `--use_sim` | false | Isaac Sim 로봇. 노드가 sim time |

퍽마다 도는 순서:

1. 첫 퍽 · 실패 뒤 · 아는 퍽이 없을 때만 `pose stow` → `inspect box`로 상자를 다시 본다. 그 밖에는 싣고 나서 look으로 가지 않고 바로 다음 퍽으로 간다 (미션 `locate`와 같음, 퍽당 약 5 s 절약): 지금 인식에 상자 위 퍽이 있으면 그것, 없으면 마지막 inspect 목록에서 처리한 퍽을 뺀 것 (coordinator가 안 보이는 트랙을 `remember_s` 동안 기억). coordinator가 그 id를 잊었으면(계획 검증 `invalid_plan:target N not tracked` · 단계 실패 `*_no_target`) inspect하고 같은 퍽을 다시 한다.
2. 선 퍽부터, 그다음 상자 가운데에서 가까운 순으로 하나를 고른다 (세우다 옆 퍽을 칠 수 있어서 누운 퍽은 나중에 · 미션도 같음). 앞에서 실패한 퍽(그 자리 4 cm 안)은 뺀다.
3. 선 퍽: `pick <id>` → `load next`. 팔이 수직으로 못 닿으면 coordinator가 `pick_far`로 넘긴다.
4. 누운 퍽 (`puck_lying`): `stand <id>` → `pose look` → `--stand-check-s` 기다림 → 누운 가운데 10 cm 안의 같은 색 퍽을 본다: 누움이면 (상자 위에 선 퍽이 생겼으면 그것부터 집고 나중에 다시) `stand` 다시 (`--stand-tries`까지), 섰으면 그 퍽을 일반 `pick <id>` → `load next`, 아무것도 없으면 빈손으로 집지 않고 inspect부터 다시 (`stand_lost`, 실패로 안 셈). 11-15: 실패한 stand가 퍽을 6.6 cm 던졌는데 6 cm로 찾다 "섰음"으로 보고 `stand_pick`이 빈손으로 닫음 — 잡기 판정이 빈손(0.0048 m · 0.09 A)과 쥔 퍽(0.0050~0.0056 m · 0.09~0.11 A)을 못 가림. 미션도 같음 (`plan.yaml` `stand_find_m` 0.10).
5. 퍽을 쥔 채 load가 실패하면 (`arm_jam` 재시도까지 실패 · `load_released_*`는 칸에 놓인 것이라 제외) 그 퍽을 상자 위 빈 자리(인식한 상자 가운데 먼저 → 로봇 쪽 4 cm → 옆 ±4 · 8 cm, 상자 가장자리 3 cm 안쪽, 다른 퍽 가운데에서 `--put-back-clear` 45 mm 이상)에 `place`로 세워 놓고, stow → `inspect box`(look)로 다시 본 뒤 다시 고른다 (`--load-retries`, 기본 1번 · 실패로 안 셈 · CSV에 `… -> put back`). 그래도 안 되면 실패로 두고 stow.
6. 그 밖의 실패는 stow (손에 퍽이 남았을 수 있음). 다음 inspect가 판단한다.

멈추는 조건: 첫 inspect에서 본 퍽 수만큼 실음 · 상자가 빔(실패한 퍽만 남음 포함) · 적재함이 참 · `--max`.

- 섰는지는 인식의 자세 판정(`puck_lying`인지, YOLO 박스 안 depth로 윗면 높이)으로 **look 자세에서** 본다 (미션 `stand_up`도 같음). 11-15: 손을 뺀 자리에서 보면 인식이 선 퍽을 세 번 다 `puck_lying`으로 봐서 선 퍽에 stand를 또 보냈다 — 같은 트랙이 stow/look에서는 standing.
- 방향이 모호한 퍽(누운 퍽 긴 축 없음 · 선 사각 퍽 면 없음)은 coordinator가 pick·stand 전에 한 번 그 퍽 바로 위에서 다시 보고(`inspect <id>`, 로그 "방향 모름 — 퍽 위에서 다시 봄") 같은 단계를 다시 한다 (11-15). 다시 봐도 모르면 원래대로 (stand는 `stand_no_yaw`, pick은 yaw 0°). 다시 보기가 실패해도 계획은 이어진다.
- stand가 실패로 끝나면 (`stand_no_yaw` · `stand_grip_timeout` · `arm_contact` · IK) 다시 하지 않고 그 퍽을 실패로 두고 다음 퍽으로 간다 (미션 `stand_up`과 같음). 다시 세우는 건 "stand는 끝났는데 아직 누움"일 때만이다.
- 퍽이 넘어가 6 cm 밖으로 가면 "섰음"으로 보고 `stand_pick`을 시도한다 — 잡기 판정에서 실패한다.
- 상자 위 퍽이 `reach_m`(0.20)보다 멀어도 pick은 `pick_far_max_m`(0.29), stand는 `stand_box_max_m`(0.26) 안이면 다가가지 않는다. 이 스택엔 Nav2가 없어 다가가기는 `nav_unavailable`로 실패한다.
- 정책 켜고 끄기는 여기서 안 한다: `models/skills/<skill>/meta.yaml`의 `use:` + colcon build.

결과: `logs/pick_coord_<ts>.csv` (퍽마다 id · tag · 자세 · 로봇에서 거리 · 성공 여부 · 이유 · 실제로 돈 skill 순서 · 시간), bag은 `logs/pick_coord_<ts>_bag/`.

### stand_loop.py — 누운 퍽 세우기

```bash
python3 stand_loop.py                                             # 인식한 실제 누운 퍽 세우기 반복 (기본)
python3 stand_loop.py --x 0.22 --y 0.0 --preview                  # 쥘 자리 위에 손만 띄워 위치 확인
python3 stand_loop.py --pick                                      # 세운 뒤 depth가 "섰음"이면 집기
python3 stand_loop.py --no-perception --axis-deg 70 --rounds 0    # 가짜 누운 퍽(축 직접 지정) — 인식 노드를 끔
python3 stand_loop.py --auto --x 0.22 --y 0.0 --slot 0             # 세우기 → 재시도 → 집기 → 칸 0 적재, 질문 없음
```

| 인자 | 기본값 | 뜻 |
| --- | --- | --- |
| `--x` | 0.22 | 퍽 가운데, base_footprint 앞쪽 [m] (팔 도달 0.13~0.31) |
| `--y` | 0.0 | 퍽 가운데, base_footprint 왼쪽 [m] |
| `--axis-deg` | 0.0 | 퍽 긴 축 방향, 로봇 앞에서 [deg] (0 = 앞뒤로 누움) |
| `--support` | 0.07 | 퍽이 놓인 면 높이 [m] |
| `--shape` | `box` | 퍽 모양 `box` · `cyl` |
| `--rounds` | 0 | 반복 횟수. 0 = Ctrl-C까지 |
| `--pause` | 6.0 | `--no-ask`일 때 라운드 사이 퍽을 다시 놓을 시간 [s] |
| `--preview` | 끔 | 쥘 자리 위에 손만 띄움 |
| `--perception` / `--no-perception` | 켬 | 인식한 실제 누운 퍽. 끄면 `/tracked_objects`에 가짜 퍽(`--x` `--y` `--axis-deg`) |
| `--match-m` | 0.08 | `--x`/`--y`에서 이 안의 퍽만 고름 [m] (인식 모드) |
| `--pick` | 끔 | 깊이 확인이 "섰음"이면 `/skills/pick`으로 집기 |
| `--auto` | 끔 | 누운 퍽 하나: stand → (재시도) → pick → load, 질문 없음 |
| `--slot` | 0 | `--auto`: 집은 퍽을 넣을 칸 0~11. -1 = load 없음, Enter로 손 엶 |
| `--retries` | 3 | `--auto`: 첫 시도 뒤 stand 재시도 횟수 |
| `--ask` / `--no-ask` | 켬 | 라운드마다 섰는지 물음 (답이 다음 라운드 시작). 끄면 `--pause` |
| `--bag` | `cam` | 라운드마다 bag. `cam` · `nocam` · `off` |
| `--use_sim` | false | Isaac Sim 로봇. 노드와 bag이 sim time |

- 기본은 인식 모드다 (11-14부터). inspect 뒤 `--x`/`--y`에서 `--match-m` 안의 누운 퍽 중 긴 축 방향(`yaw_period`)이 붙은 트랙을 기다려 고른다. 대기 시간 안에 방향이 안 붙으면 방향 없는 트랙을 보내고 coordinator가 `stand_no_yaw`로 끝낸다.
- 퍽은 로봇 앞 0.19~0.26 m에 둔다. coordinator는 대상이 `reach_m`(0.20)을 넘으면 navigate로 다가가는데, 이 스택엔 Nav2가 없어 `nav_unavailable`로 30 s 뒤 실패한다. 상자 위 퍽 stand는 `stand_box_max_m`(0.26) 안이면 다가가지 않는다 (11-15 추가).
- `--no-perception`은 인식 노드를 끈다 (같은 토픽을 내므로). 끝나면 launcher를 다시 띄운다.
- 라운드 사이 `ros2 param set /skill_stand stand_...`로 원호 값을 바꾸면 CSV에 같이 남는다 (`stand_contact_h_box_m` · `stand_release_deg`는 안 남음).
- 기본값은 `ros2_ws/src/skills/config/skills.yaml`의 `skill_stand` (11-15: `stand_contact_h_box_m` 0.035 · `stand_arc_k` 0.5 · `stand_tilt_over_m` 0.02 · `stand_arc_vel_scale` 2.0 · `stand_release_deg` 95).
- `stand_release_deg`: 호가 이 각도를 지날 때 그리퍼를 벌리고 팔은 호 끝(`stand_arc_end_deg`)까지 간다. 7.5° 경유점 단위라 최대 7.5° 늦게 벌린다. ≤ 0이면 호 끝에서 벌림.
- 쥠 판정: 손가락이 빈손 위치(`stand_empty_m` 3 mm)까지 안 닫히고 0.5 s 멈추면 전류가 없어도 쥠으로 본다. 목표 `stand_pinch_m` 0.0062가 퍽 폭(간격 30 mm)과 같아 쥐어도 전류가 거의 안 생겨서다.
- 결과: `logs/stand_loop_<ts>.csv`(`--auto`는 `stand_auto_<ts>.csv`), 라운드별 bag.

### place_loop.py — 놓기

퍽을 손에 넣고(`--from slot`: 칸에서 unload, `--from pick`: 상자에서 pick) `/skills/place`로 정한 자리에 세운다.

```bash
python3 place_loop.py --from slot --slot 0 --shape box box box cyl cyl --to 0.22 -0.05 0.07
python3 place_loop.py --from pick --to 0.22 0.05 0.07
python3 place_loop.py use_sim=true --from slot --shape box box box cyl cyl --dice 5 --to 0.185 0 0.07 --auto
```

| 인자 | 기본값 | 뜻 |
| --- | --- | --- |
| `--from` | (필수) | 퍽 가져올 곳: `slot`(칸에서 unload) · `pick`(상자에서 pick) |
| `--slot` | 0 | `--from slot`: 첫 칸 0~11. 라운드마다 +1 |
| `--shape` | `box` | `--from slot`: 칸마다 퍽 모양, `--slot`부터 차례로 (한 개면 전부 같음). 예: `box box box cyl cyl` |
| `--to X Y Z` | 0.22 -0.05 0.07 | 놓을 자리, base_footprint [m]. z = 놓을 면 높이 (`--dice`면 무늬 가운데) |
| `--dice` | 0 | 1~6: 주사위 눈 무늬로 놓기. 라운드 k가 눈 k로 (먼 줄부터). 0 = 끔 |
| `--pitch` | 0.045 | `--dice` 눈 간격 [m] |
| `--box-x` | 0.22 | `--from pick`: 상자 가운데 앞쪽 [m] |
| `--box-y` | 0.0 | `--from pick`: 상자 가운데 왼쪽 [m] |
| `--support` | 0.07 | `--from pick`: 상자 윗면 높이 [m] |
| `--puck-h` | 0.06 | 세운 퍽 높이 [m] |
| `--rounds` | 0 | 반복 횟수. 0 = q / Ctrl-C까지 |
| `--auto` | 끔 | 라운드 사이 Enter 없이 unload → place 반복. 첫 실패에서 멈춤, 자 값은 끝에 한 번 |
| `--bag` | `motion` | 라운드마다 팔 동작 bag (`--auto`면 한 개). `motion` · `off` |
| `--use_sim` | false | Isaac Sim 로봇. 노드와 bag이 sim time |

- `--to x y z`의 z는 놓을 면 높이(퍽 바닥이 닿는 곳).
- 놓은 뒤 자로 잰 중심(mm)과 섰는지를 묻는다.
- 결과: `logs/place_loop_<ts>.csv`, 팔 동작 bag.

### goto.py — 정해진 자세로 보내기 + 진동 측정

FollowJointTrajectory goal을 `/arm_controller`에 바로 보낸다(MoveIt·충돌 확인 없음). 도착한 뒤 `--hold` 동안 관절 흔들림을 잰다.

```bash
python3 goto.py --named look                                       # stow carry_left carry_right look gaze
python3 goto.py --joints 0 -20 40 70 0                             # joint1..5 [deg]
python3 goto.py --pose 0.22 0.0 0.15 --pitch 30                    # 손끝 위치 (base_footprint, m)
python3 goto.py --slot 0                                           # 적재함 칸 0 위 (q_over)
python3 goto.py --slot 3 --stage low --shape cyl --back            # 칸 3 q_low, 같은 길로 돌아옴
python3 goto.py --slot 3 --stage low --auto                        # 같은 길, Enter 없이 carry·q_over에서 안 멈추고 지나감
python3 goto.py --named look --from stow --rounds 5 --back --tag sag_off
```

| 인자 | 기본값 | 뜻 |
| --- | --- | --- |
| `--joints` | (택1) | 목표 joint1..5 [deg] |
| `--pose X Y Z` | (택1) | 목표 손끝 면 가운데, base_footprint [m]. 기본은 손 수직 아래 |
| `--named` | (택1) | 이름 붙은 자세: `stow` `carry_left` `carry_right` (basket.yaml) · `look` `gaze` (arm.yaml) |
| `--slot` | (택1) | 적재함 칸 0~11. `carry_<칸 방향>` → `q_over` → `--stage` 순서로 감. |
| `--stage` | `over` | `--slot`: 마지막 목표 자세 `over` · `low` · `lift` · `grasp` |
| `--shape` | `box` | `--slot`: 퍽 모양 표 `box` · `cyl` |
| `--pitch` | 0.0 | `--pose`: 수직에서 손 기울기 [deg]. + = 손끝이 로봇 바깥쪽 |
| `--roll` | 0.0 | `--pose`: joint5 [deg] |
| `--time` | 0.0 | 구간 하나 이동 시간 [s]. 0 = `--speed`로 계산 |
| `--speed` | 30.0 | `--time` 0일 때 가장 많이 도는 관절의 평균 속도 [deg/s] |
| `--hold` | 3.0 | 도착 뒤 흔들림을 재는 시간 [s] |
| `--tol` | 0.2 | 안정 판정 폭 [deg] |
| `--rounds` | 1 | 반복 횟수 |
| `--back` | 끔 | 목표에 간 뒤 같은 길을 거꾸로 시작 자세로 돌아옴 (돌아오는 구간도 잼) |
| `--from` | 지금 자세 | 시작 자세 이름 (`--named`와 같은 이름). 먼저 그리로 감 (안 잼) |
| `--tag` | 없음 | CSV에 남길 꼬리표 (예: 어떤 설정을 켰는지) |
| `--bag` | 끔 | 실행 전체를 motion bag 하나로 |
| `--yes` | 끔 | 첫 이동 전 Enter 안 기다림 |
| `--auto` | 끔 | Enter 없이, 경유 자세에서 멈추지 않고 지나감. 방향마다 JTC goal 하나 (시작 → 경유 → 목표), 흔들림은 끝에서만 잼 |

- `--slot`은 skill_load와 같은 경로로 간다: `carry_<칸 방향>` → `q_over` → `q_<stage>`(`over` · `low` · `lift` · `grasp`). 기본은 경유 자세마다 멈추고 잰다. `--auto`면 멈추지 않고 지나간다.
- 이미 그 자세에 있으면 "start = target"으로 멈춘다. `--from stow`처럼 시작 자세를 준다.
- 첫 실행은 `--speed 15`처럼 느리게. 퍽을 쥔 채 `low`·`grasp`로 보내면 칸 벽에 닿을 수 있다.
- 결과: `logs/goto_<ts>.csv`(원 샘플), `logs/goto_<ts>_summary.csv`(구간 × 관절).

### slot_check.py — 칸 자세 오차 기입

`goto.py --auto`로 칸마다 보낸다(`carry_<칸 방향>` → `q_over` → `--stage`, 경유점에서 안 멈춤). 도착하면 잰 오차를 입력한다. 입력하면 다음 칸으로 간다. 첫 이동만 Enter를 기다린다.

```bash
python3 slot_check.py                              # 칸 0~11, q_low, box 표
python3 slot_check.py --slots 0 1 4 5 --stage over
python3 slot_check.py --shape cyl --speed 10
```

| 인자 | 기본값 | 뜻 |
| --- | --- | --- |
| `--slots` | 0~11 | 갈 칸, 이 순서대로 |
| `--stage` | `low` | 멈출 칸 자세 `over` · `low` · `lift` · `grasp` |
| `--shape` | `box` | 퍽 모양 표 `box` · `cyl` |
| `--speed` | 15.0 | `goto.py --speed` [deg/s] |
| `--hold` | 1.0 | `goto.py --hold`, 칸에서 흔들림 재는 시간 [s] |

- 입력 형식: `dx dy [dz] [# 메모]`, 단위 mm. 손(또는 손 안 퍽) 위치에서 칸 가운데를 뺀 값이다. 로봇 기준으로 dx = 앞, dy = 왼쪽, dz = 위. 예: `3 -1.5`, `2 0 -4 # 벽에 닿음`.
- Enter = 건너뜀(skip으로 기록), `r` = 이 칸으로 다시 이동, `q` = 끝.
- 결과: `logs/slot_check_<ts>.csv` (칸마다 한 줄). 칸마다 `goto.py`가 남기는 `logs/goto_<ts>*.csv`도 있다.

## 주행 시험 (approach)

### approach_loop.py — 상자까지 가서 붙기

미션과 같은 계획(stow → Nav2로 변 앞 standoff → `/skills/approach dock_box`)을 coordinator에 보내고, 끝난 자리를 참값 상자와 비교한다.

```bash
python3 approach_loop.py 1                          # 시뮬, start_1, 가장 가까운 빈 변
python3 approach_loop.py 3 --side north             # 변 고정: west | east | south | north
python3 approach_loop.py 1 --plan-error 0.04 -0.02  # 상자 힌트를 일부러 틀리게
python3 approach_loop.py 3 --real --exit 0.165      # 실기, plan.yaml start box 3
python3 approach_loop.py L2 --real                  # 실기, location box L1~L3
```

| 인자 | 기본값 | 뜻 |
| --- | --- | --- |
| `box` | (필수) | start box 번호 (`start_<n>`). `--real`이면 `L1`~`L3`도 됨 |
| `--side` | 가장 가까운 빈 변 | 붙을 변 `west` · `east` · `south` · `north` (map 좌표) |
| `--scene` | 실행 중 Isaac의 `ISAAC_SCENE` | 장면 usda |
| `--policy` | `dock_box` | approach 정책 (goal.target) |
| `--half` | 0.1025 | 상자 반변 길이 [m] (plan.yaml `box_half`) |
| `--real` | 끔 | 실기: plan.yaml 상자·벽, 인식한 퍽을 참값으로 |
| `--start-pose X Y YAW` | plan.yaml `start_pose_arena` | `--real`: 로봇을 켠 경기장 자세 |
| `--exit` | 0.0 | 붙은 뒤 곧게 이만큼 뒤로 빠짐 [m]. 0 = 끔, 음수 = Nav2 목표까지 |
| `--nav-turn` | 0.55 | Nav2 목표를 상자 가운데에서 이 거리 안으로 [m] |
| `--standoff` | 0.25 | Nav2가 변에서 이만큼 떨어져 멈춤 [m] |
| `--gap` | 0.085 | approach가 base_footprint와 변 사이를 이만큼까지 좁힘 [m] |
| `--wall-clearance` | 0.22 | 벽에 이보다 가까운 변은 건너뜀 [m] |
| `--timeout` | 240.0 | 계획 전체 제한 시간 [s] |
| `--wait-ready` | 90.0 | 시작 전 Nav2·lifecycle·skill 서버·TF를 기다림 [s]. 0 = 안 봄. 실패면 exit 3 |
| `--bag` | `on` | `on` · `full`(`/map`·costmap 포함, 큼) · `off` |
| `--plan-error DX DY` | 0 0 | 로봇에 주는 상자 힌트를 일부러 옮김 [m] (map 좌표). 참값 상자는 그대로 |

- 시뮬은 한 번 붙은 뒤 `../standalone/runtime/reset.sh`로 되돌리고 다음 시도.
- 결과: `logs/approach_<ts>/result.json` + bag.

### approach_batch.sh · approach_summary.py — 반복과 요약

```bash
./approach_batch.sh 10 1                            # 시뮬 10판, start box 1 (매판 reset.sh)
./approach_batch.sh 10 1 --plan-error 0.04 -0.02    # 뒤 인자는 approach_loop.py로
python3 approach_summary.py logs/approach_<ts> ...  # 이미 있는 결과 폴더 요약
```

| 인자 | 기본값 | 뜻 |
| --- | --- | --- |
| `N` (첫째) | (필수) | 반복 판 수. 판마다 `reset.sh` |
| `BOX` (둘째) | (필수) | start box 번호 |
| 나머지 | 없음 | 그대로 `approach_loop.py`로 넘어감 |
| `approach_summary.py` 인자 | (필수) | 요약할 `approach_loop.py` 결과 폴더들 |

성공 기준은 위치 오차 10 mm 이하(yaw는 따로 표시). 요약: `logs/approach_batch_<ts>.txt`.

## 보조 도구

| 파일 | 용도 |
| --- | --- |
| `manipulator-launcher.sh` | 팔 시험용 스택 (위 "공통 준비"). `perception_device:=cuda`, `use_sim:=true` |
| `manipulator-record.sh` | 팔 시험용 bag. 단독으로 쓸 때: `./manipulator-record.sh` · `nocam` · `motion` (Ctrl-C로 끝) |
| `arm_model.py` | URDF 기반 FK·IK·중력 토크 (goto.py 등이 import, 직접 실행하지 않음) |
| `arm_vib_test.py` | `goto.py`의 앞선 판 — 관절 목표 · 손 자세 · 이름 자세 하나로 보내고 진동 측정 (칸 자세는 `goto.py`) |
| `sys_log.py` | Jetson과 ROS 프로세스별 메모리 · CPU · GPU를 1초마다 CSV로 (`system.csv` · `procs.csv`). `approach_loop.py --real`이 스스로 띄움 |
| `vis/rviz.sh` | 팔 시험용 RViz: MoveIt 경로 + `/tracked_objects` 퍽 상자(`puck_box.py`) + `rl_arm_correction` 유령 로봇 3개(`correction_ghost.py`, `ghost.rviz`). launcher가 떠 있을 때 실행 |
| `vis/odom_nvblox.sh [domain]` | 주행용 RViz(`odom_nvblox.rviz`) — `tools/rviz/start.sh`로 띄우고 bag도 같이 (`RVIZ_RECORD=0`이면 bag 없음). 첫 인자가 숫자면 `ROS_DOMAIN_ID` |
