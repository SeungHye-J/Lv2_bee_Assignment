# Perception 시험·증빙 안내

> 기준: `develop` 커밋 `48cc6b2b` (2026-11-17).
> 이 문서는 **시험 방법**이다. 여기 적힌 것만으로 "검증 완료"가 되지 않는다. 결과는 원본 증빙(사진·영상·로그·rosbag)과 함께 [project2/results/test-results.csv](../../../../project2/results/test-results.csv)에 쓴다.
> 미션 규칙 근거: [documents/mission/README.md](../../../../documents/mission/README.md) (원문 [dice-mission.pdf](../../../../documents/mission/dice-mission.pdf)). 다르면 PDF가 맞다.
> 통과 기준 근거: 팀 노션 「프로젝트(2) 발제」의 **5️⃣ 평가표** (아래 "발제 5️⃣ 평가표"). 저장소에는 이 페이지 사본이 없다. 원문과 다르면 원문이 맞다.

## 발제 5️⃣ 평가표 중 인지 관련 항목

| 구분 | 항목 | 배점 · 기준 |
|---|---|---|
| 기본 | 퍽 ID | 6점 = 실물 퍽 12개(ID 0~11) × 0.5점 |
| 기본 | 장소 ID | 3점 = 실물 목적지 3개(ID 12~14) × 1점 |
| 기본 | 바코드 | 4점 = 실제 출력 4종(P2-01~P2-04) × 1점 |
| 결합 | 신호 | 6점 = 점멸 경계 구분 2 + 12회 횟수 계산 2 + 종료(마지막 3초 소등) 확인 2. 같은 점등을 여러 프레임에서 봐도 1회. 누락·못 읽은 신호는 추측하지 않음 |
| 결합 | 오류 입력 | 3점 = 누락·불명 신호, 미등록 코드, 통신 끊김 → 안전 정지 각 1점 |
| 원점수 | 신호 순서 · 색상별 횟수 · 바코드 해석 | 12점(시점당 1) · 9점(R·Y·G 각 3) · 4점(문자열 + 정렬 방향) |
| 원점수 | 이동 후 재검증 | 8점 = 새 관측으로 장소별 ID·위치 6 + 전체 중복·누락 2. 가려진 퍽은 미확인 |
| 요구사항 평가표 | No.3 표식 인식 · No.4 신호 기억 · No.7 오류 감지·안전 정지 · No.8 새 관측 재검증 | 실물/모의 구분 필수. No.4는 `signal.json` + 신호 관측 영상 + 시각별 로그 |
| 도전 A (선택) | 신호 인식 안정화 | 2설정 × 3조건 × 5회 = 30회 |

표식 규격 (같은 발제): tag36h11, 퍽 태그 16 mm(여백 20 mm), 목적지 태그 48 mm(여백 60 mm). 퍽 바닥 Z=70, 윗면 Z=130 mm. 신호등 (80, 950) mm, 바코드 중심 (80, 650, 450) mm, 로봇은 (734, 950)에서 −X를 보고 관측. 신호는 점등 1초·소등 1초, 12회 뒤 3초 소등.

## 0. 공통 규칙

- **장비를 구분해서 적는다**: 실물(Jetson + 실제 카메라) / 녹화 재생(rosbag) / 모의(Isaac Sim, 도커 faker).
- **정답을 먼저 적는다**: 퍽 색·모양·태그 번호·자세, 신호 12개 순서, 바코드 값을 시험 전에 종이에 적고 사진으로 남긴다.
- **실패도 그대로 남긴다.** 횟수를 고르지 않는다.
- **PR 본문 수치 ≠ 다시 잰 값.** 원본이 없는 수치는 "PR 보고값"이라고 적는다.
- **증빙 폴더**: Jetson `~/evidence_<날짜>/` 아래 시험마다 폴더를 만든다. 끝나면 노트북으로 옮긴다.
  ```bash
  scp -r bee@<젯슨 주소>:~/evidence_<날짜> ~/Downloads/
  ```
- **증빙 스크립트** (실습실 안내에서 만든 것, **저장소에는 없음**):
  - `~/evidence_<날짜>/grab.py <이름> <초>`: `/perception/<cam>/debug_image`를 1초마다 jpg로, `/tracked_objects`를 1초마다 `tracked.json`으로 저장
  - `~/evidence_<날짜>/record_video.py`: 로봇 카메라 영상을 동영상 파일로 저장 (신호등 영상용)
  - 같은 일을 저장소 도구만으로 하려면 아래 `ros2 bag record`를 쓴다.

### 실물 준비 (전체 로봇)
```bash
cd ~/Lv2_bee_Assignment/lv2_module6/ros2_ws
POLE_FRONT_CAM_SERIAL=<폴 카메라 시리얼> GRIPPER_CAM_SERIAL=<손 카메라 시리얼> ./start.sh
# 확인: 인지 노드가 떴는지 · 어떤 장치로 추론하는지
ros2 node list | grep -E 'perception|signal_decoder'
ros2 topic echo /diagnostics --once | grep -A2 -E 'YOLO|AprilTag|바코드'
```
- 기본 실행(`start.sh` 인자 없음)은 저장 지도 + `perception:=true`다 (`start.sh` 머리말).
- 시험 중에는 다른 사람이 같은 `ROS_DOMAIN_ID`로 명령을 보내지 않게 한다.

### 공통 녹화 (가벼운 토픽만)
```bash
ros2 bag record -o ~/evidence_<날짜>/bag_<시각> \
  /tracked_objects /mission/signal /mission/signal_status /mission/qr /diagnostics \
  /perception/pole_front_cam/detections /perception/gripper_cam/detections
```
카메라 원본(`/<cam>/color/image_raw`)도 넣으면 나중에 다시 돌려 볼 수 있다. 다만 용량이 크다 (640×360 컬러 기준 수 MB/s).

## 1. 요구사항별 시험

미션 채점(`documents/mission/README.md` §7)과 인지 담당 범위를 기준으로 나눴다. "현재 기록" 열은 `test-results.csv` 그대로이고, 다시 재현한 기록이 아니다.

| # | 요구사항 | 미션 근거 | 현재 기록 |
|---|---|---|---|
| P1 | 퍽 검출 · 색(빨강/초록) · 모양(원기둥/직육면체) | §1 퍽 · §7 색상·모양 선택 | T-04 퍽 727/767 (94.8%) — **PR #144 보고값**, 원본 없음, 이미지 단위 · **11-17 실물 직접 시험 T-38: 퍽 12개 색·모양 12/12** ([증빙](../../../../project2/results/perception_1117/README.md)) |
| P2 | 퍽 자세 (서 있음/누움) | 집기·놓기 전제 | T-06 Isaac 22→100%, 실물 영상 88→99% — **PR #95 보고값**, 원본 없음 · **11-17 실물 T-38·T-42: 가까운 쪽 맞음(누운 퍽 3개 포함) · 상자 먼 끝 서 있는 퍽 2/2 누움 오판** |
| P3 | 퍽 태그 번호 (AprilTag 0~11) | §5 ID 등록 · §7 검증 기록 | T-05 39/40 — **모의(Isaac)**, PR #80 보고값. (11-16까지 실물 기록 없음) · **11-17 실물 T-38: ID 0~11 12/12** (사용자 스티커 확인) |
| P4 | 퍽 위치 (Depth → map) 정확도 | §7 놓기 오차 10 mm의 전제 | 기록 없음 · **11-17 실물 T-40: 25 cm 이동마다 거리 0.239·0.256·0.253 m 줄어듦 (줄자 기준 없음)** |
| P5 | 바코드 (Code 128) 해독 | §3 · §7 바코드 4점 | T-07 실물 **실패** (`barcode_missing` 7회) — PR #202 보고값 · **11-17 실물 T-37: P2-01~04 4/4 (손 카메라 · 폴 카메라로는 못 읽음)** |
| P6 | 신호등 12회 점멸 순서·횟수 | §3 · §7 신호 12점 + 횟수 9점 | T-04 신호등 상태 579/582 (이미지 단위, PR 보고값). (11-16까지 12회 해독 실물 기록 없음) · **11-17 실물 T-36: 사이클 31번 중 30번 정답 (R4·Y5·G3), 1번 한 칸 오독 · 순서 1가지만** |
| P7 | 시작 상자 · 장소 상자(L1~L3, 태그 12~14) 인지 | §2 장소 · 도킹 | 기록 없음 · **11-17 실물 T-39·T-40: 장소 태그 12·13·14 3/3 (14는 손 카메라 look 추가 시험) · 태그 놓치면 `start_box`로 전환 · 멀리서 상자 중복** |
| P8 | 처리 시간 · CPU (Jetson) | 실시간 동작 | T-19 CPU 108→23%, T-21 21→7 ms — **PR #164·#246 보고값** · **11-17 실물 T-43: CPU 평균 42.6 % (코어 1개 = 100 %) · YOLO 5 ms · `/tracked_objects` 9.4 Hz (rosbag)** |
| P9 | ROS 출력이 미션으로 전달됨 · 새 관측 재검증 입력 | §6 수행 순서 · 발제 No.8 | T-27 모의 미션 (주입값) — 인지 출력은 대역 없음 · **11-17 실물 rosbag: `/tracked_objects`·`/mission/qr`·`/mission/signal` 발행 기록. 추가 T-44에서 `/mission/qr` 미등록 문자열을 미션이 받아 거부·오류 종료하는 연동 확인 (BUILD·VERIFY 새 관측 재검증은 별도)** |
| P10 | 신호·바코드 오류 입력을 오류로 알림 (추측 안 함) | 발제 결합·오류 입력 · No.7 | 단위 테스트(`test_signal.cpp`, plan `test_mission_solver.cpp`) 및 실물 시험. 11-17 T-44 미등록 문자열 거부·unknown_barcode 오류 종료·종료 후 속도 명령 0 확인. 신호 가림 시험도 11-17 17:51~17:53 수행했다. 수집한 sigocc2_status.txt에는 정상 사이클 3개가 있고 오류·정지 이벤트는 기록되지 않았다 |

> T-24(먼 퍽 집기)는 조작(skill) 시험이라 인지 요구사항 표에서 뺐다. 인지는 그 시험에서 퍽 위치를 주는 역할만 했다.

### P1·P2 퍽 검출·색·모양·자세
- **장비**: 실물. 상자 위에 퍽 2개를 둔다. 서 있음/누움, 빨강/초록, 원기둥/직육면체를 섞는다.
- **명령**:
  ```bash
  python3 ~/evidence_<날짜>/grab.py puck_pole 20      # 폴 카메라로 (상자 앞 약 50 cm)
  python3 ~/evidence_<날짜>/grab.py puck_gripper 20   # 손 카메라로 (팔을 상자 위 내려다보기 자세로)
  # 저장소 도구만 쓸 때
  ros2 topic echo /tracked_objects --once
  ```
- **볼 것**: `class_name`(`puck`/`puck_lying`), `color`, `details.shape`, `details.posture`가 정답과 같은지 본다.
- **통과 기준 (팀이 정할 값)**: 배치를 바꿔 N회 반복하고, 퍽마다 맞음/틀림을 센다. 기준 비율은 팀이 먼저 정하고 적는다. 이 문서에서 정하지 않는다.
- **증빙**: 디버그 사진(jpg), `tracked.json`, 정답 종이 사진, 집계표.

### P3 퍽 태그 번호
- **장비**: 실물, 손 카메라가 퍽 윗면을 내려다보는 자세 (태그 16 mm, 위에서 봐야 읽힘).
- **명령**: P1과 같다. `details.tag` 값을 본다.
- **기준**: 정답 번호와 같음. 틀린 번호(오독)는 따로 센다. 발제 기본 점수는 **실물 퍽 12개(ID 0~11) 각각**을 읽었는지로 본다 → 12개 모두 한 번 이상 시험한다.
- **참고 (11-16 실기 미션 감시 로그)**: S5 퍽 태그 #3·#6이 읽혔다. 로그 원본은 Jetson `~/.ros/log`에 있을 수 있고, 아직 저장소로 옮기지 않았다.

### P4 퍽 위치 정확도
- **방법**: 로봇과 상자를 고정하고, 줄자로 로봇 중심 → 퍽 중심 거리를 잰다. `details.distance`(로봇 중심 ~ 물체 바닥 위 거리, `perception_node.cpp:1385`)와 비교한다.
- **기준**: 팀이 정한 허용 오차 (예: 놓기 정밀도 10 mm 미션 기준을 참고). 3회 이상 잰다.
- **증빙**: 줄자 사진, `tracked.json`, 표 (실제 / 인지 / 차이).

### P5 바코드
- **장비**: 실물. 바코드를 경기 위치(중심 (80, 650, 450) mm)에 두고 로봇은 출발 자리에 둔다.
- **주의**: 바코드는 미션 시작 명령 뒤 180초(`barcode_window_s`) 동안만 찾는다 (`perception_node.cpp:390-400`, `:1410`). 미션 없이 시험하려면 `start:BARCODE`를 보내거나 `barcode_always:=true`로 띄운다.
- **명령**:
  ```bash
  ros2 topic pub --once -w 1 --keep-alive 3 /cmd/mission std_msgs/msg/String "{data: 'start:BARCODE'}"
  timeout 60 ros2 topic echo /mission/qr | tee ~/evidence_<날짜>/barcode.txt
  ```
- **기준**: 실제 코드와 같은 문자열이 나온다. 발제 기본 점수는 **실제 출력 4종(P2-01~P2-04) 각각**이라 네 장을 다 시험한다.
- **증빙**: `barcode.txt`, 손 카메라 사진, `/diagnostics`의 `바코드` 항목(`zxing-cpp` 있음/없음). 실패하면 바코드가 화면에 보였는지 사진으로 남긴다.

### P6 신호등 12회 점멸
- **장비**: 실물. 로봇은 출발 자리(734, 950)에서 −X를 본다. 신호등은 (80, 950)에 있다.
- **정답**: 운영 신호 순서 12개를 미리 적는다.
- **명령** (3분 = 약 6사이클):
  ```bash
  cd ~/evidence_<날짜>
  timeout 180 ros2 topic echo /mission/signal_status | tee signal_status.txt &
  timeout 180 ros2 topic echo /mission/signal | tee signal.txt
  # 로봇 카메라 영상 (휴대폰 아님) — 신호등을 보는 폴 카메라
  ros2 bag record -o signal_video /pole_front_cam/color/image_raw /tracked_objects /mission/signal_status
  # 또는 실습 스크립트: python3 record_video.py (저장소 밖)
  ```
- **볼 것**:
  - `signal_status`의 `raw`(12개 순서)
  - `red/yellow/green` 횟수
  - `ok`, `error` (`count`·`unknown`·`range`·`unconfirmed`)
- **기준** (발제 결합·신호 + 원점수):
  - 점멸 경계를 구분한다. 같은 점등을 여러 프레임에서 봐도 1회로 센다.
  - 12회 횟수가 맞고, 12개 순서도 시점마다 맞다.
  - 마지막 3초 소등으로 종료를 확인한다. `signal_decoder`는 꺼짐이 `gap_s` 2.5초 이어지면 사이클 끝으로 본다.
  - 누락·못 읽은 신호는 추측하지 않고 `error`로 나온다.
  - 사이클마다 같은 답이 나오는지도 센다 (예: 6사이클 중 몇 번 일치).
- **영상은 휴대폰이 아니라 로봇 카메라 영상**으로 남긴다. 발제 No.4가 "신호 관측 영상·시각별 로그"를 요구한다. 로봇이 실제로 본 화면이어야 근거가 된다.
- **알아둘 것**: 지금 설정은 `confirm: 1`이다. 12개를 한 번 다 보면 바로 확정한다 (`config/perception.yaml`, `include/perception/signal_decoder.hpp:51`).
- **증빙**: `signal_status.txt`, `signal.txt`, rosbag(폴 카메라 영상 포함), 정답 종이 사진.

### P7 시작 상자 · 장소 상자
- **장비**: 실물. 시작 상자 1개, 장소 상자 1개 앞에 로봇을 세운다.
- **명령**: `python3 ~/evidence_<날짜>/grab.py box_start 15` · `python3 … grab.py box_L2 15`
- **볼 것**:
  - `class_name`이 `start_box`인지 `location_box`인지
  - 장소 상자는 `details.tag`(12~14)가 나오는지
  - `face_dist_m`
  - 상자 하나가 트랙 둘로 잡히지 않는지
- **기준**: 발제 기본 점수는 **실물 목적지 3개(ID 12~14) 각각** → L1·L2·L3를 모두 시험한다.
- **증빙**: 사진, `tracked.json`, 줄자로 잰 로봇 앞면 ~ 상자 면 거리.

### P8 처리 시간 · CPU
- **장비**: 실물 Jetson, 전체 bringup이 떠 있는 상태.
- **명령**:
  ```bash
  timeout 120 top -b -d 5 > ~/evidence_<날짜>/top.txt
  timeout 20 ros2 topic echo /diagnostics | grep -A40 'perception: 상태' > ~/evidence_<날짜>/diag.txt
  ```
- **볼 것**:
  - `top`의 `perception_node` CPU 사용률
  - 진단의 `YOLO` 장치(TensorRT / CUDA / CPU)
  - `처리 시간 ms 평균/최대 <cam>`
- **조건을 같이 적는다**: 정지 중인지 주행 중인지(주행 중 절전 2 Hz), 다른 노드가 켜져 있는지.

### P9 미션으로 전달
- **장비**: 실물 미션 한 판, 또는 그 판의 rosbag.
- **볼 것**:
  - 미션 기록 `~/.ros/mission/<시각>/`의 `signal.json`, `allocation.json` (`plan.yaml` `log_dir`)
  - `mission_manager` 로그의 `퍽 N: 색 모양 (박스 k …)`, `tag_unread`
- **기준**: 인지가 낸 값(태그·색·모양·신호·바코드)이 미션 기록에 그대로 들어갔는지 본다. 재검증(발제 No.8)은 **새 관측**이어야 한다. 이전 기억을 복사한 로그는 인정되지 않는다. 가려진 퍽은 "미확인"으로 남긴다.

### P10 오류 입력 → 오류로 알림
- **11-17 추가 시험(T-44)**: 미등록 문자열 거부, `unknown_barcode` 오류 종료, 종료 후 속도 명령 0 확인. [성공 증빙](../../../../project2/results/perception_1117/unknown_barcode/README.md).
- **인지 쪽 범위**: 신호가 12개가 아니거나 모르는 색이 있으면 `signal_status`에 `count`·`unknown`·`range`가 나와야 한다. 바코드가 미등록 형식이면 고정하지 않는다 (`barcode_once_pattern` `P2-0[1-4]`, `perception_node.cpp:336`, `:1450`).
- **안전 정지 자체**는 `mission_manager` 담당이다 (`signal_error_cycles`, `barcode_timeout_s`). 인지 시험에서는 "오류를 올바르게 알렸나"까지만 본다.
- **시험 예**:
  - 신호 도중 신호등을 가린다 → 누락 → `count` 오류가 나와야 한다.
  - 미등록 문자열의 바코드를 보여 준다 → `/mission/qr`에 원문은 나오지만 고정되지 않아야 한다.
- **증빙**: `signal_status.txt`, 로봇 카메라 영상, 미션 로그(정지했다면 `error` json).

## 2. 결과 적는 법

`test-results.csv` 한 줄에 아래를 적는다.

- 시험 ID, 날짜, 시험 대상(P1~P9), 장비(실물/녹화/모의)
- 입력 조건, 기대 결과, 실제 결과, 판정, 시도 횟수, 오차
- 증빙 경로

원본 파일이 없는 수치는 판정 칸에 "PR 보고값"이라고 적는다. 저장소에 올리는 사진·로그는 `project2/results/` 아래에 둔다. 용량이 크면 링크만 둔다.
