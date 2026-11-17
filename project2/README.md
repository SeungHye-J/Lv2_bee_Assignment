# 프로젝트 2 — 주사위 기억 로봇 (BEE 팀)

> 제출 요구사항(발제문) 기준 목차. 이미 문서화된 내용은 새로 쓰지 않고 링크로 연결하고, **작성 필요**만 이 폴더에서 채운다.

## 1. 프로젝트 목표
- 미션 정의·규칙·배점: [documents/mission/README.md](../documents/mission/README.md)
- 목표: 출발 구역에서 신호등·바코드를 읽고, 배치된 주사위 칸을 파악해 상자에 담은 뒤 다시 확인하고 출발 구역으로 복귀하는 모바일 매니퓰레이터(TurtleBot3 Waffle Pi + OpenMANIPULATOR-X + RealSense)를 만든다.
- 구현한 범위: 신호 인식·바코드·할당·주행(Nav2)·집기·놓기·재검증·복귀 코드와 상태기계(`mission_manager`/`mission_solver`)는 있다.
- 못 한 범위: 기존 통합 미션 기록에서는 SURVEY 단계를 넘지 못했다 ([PR #202](https://github.com/visualkhh/Lv2_bee_Assignment/pull/202)). 11-17 인지 단위 시험에서는 정상 바코드 4종·퍽 ID 12개·장소 ID 3개를 실제 판독했지만, 이 결과를 실물 BUILD 이후 단계 완주로 집계하지 않는다. 공식 1·2회차 결과는 기록이 없어 **미확인**이다.

## 2. 장비·소프트웨어 버전
- 로봇·팔·카메라·제어보드: [lv2_module6/README.md — 미션 · 코스 · 로봇](../lv2_module6/README.md#미션--코스--로봇)

| 항목 | 버전 | 출처 |
|---|---|---|
| 보드 | Jetson Orin Nano (JetPack 7.2 / L4T r39.2) | [documents/jetson-orin-nano/](../documents/jetson-orin-nano/) |
| CUDA · cuDNN · TensorRT | 13.2 · 9 · 10.16 | 같은 문서 |
| ONNX Runtime | 1.24.3 | 같은 문서 |
| OS · ROS 2 | Ubuntu 24.04 · ROS 2 Lyrical (ros-core 0.13.0) | 개발 PC `apt` 확인 (bee 실기 재확인 안 함) |
| Nav2 · MoveIt | 1.5.0 · 2.15.0 | 개발 PC `apt` 확인 (bee 실기 재확인 안 함) |
| cuVSLAM | 17.0.1+51631ba | bee 노드 로그 |
| nvblox | 미확인 (bee에서 확인하지 않음) | — |
  - Jetson ONNX Runtime 설치 기록: [documents/jetson-orin-nano/ONNXRUNTIME.md](../documents/jetson-orin-nano/ONNXRUNTIME.md)

## 3. 설치 · 실행 · 정지 · 초기화
- 설치·실행·정지: [lv2_module6/README.md — 실행](../lv2_module6/README.md#실행) (`start.sh` · `stop.sh` · `run-cli` · 테스트베드)
경기 초기화 순서 (코드의 `start.sh`·`stop.sh`·`run-cli` 기준, 실기에서 한 번 더 따라 해 확인하지는 않음 — 확인되면 이 문장 삭제):
1. 로봇을 출발 구역에 두고 바구니(`/coordinator/basket_clear`)를 비운다.
2. 이전 실행이 있으면 `./ros2_ws/stop.sh`로 종료한다.
3. `./tools/run-cli`에서 지도를 선택한다.
4. `./ros2_ws/start.sh`로 bringup을 띄운다.
5. `./tools/run-cli mission start`로 미션을 시작한다.

## 4. 시스템 구성
[docs/architecture.md](docs/architecture.md) (상태도·완료 조건·오류 처리) · [lv2_module6/README.md — 전체 아키텍처](../lv2_module6/README.md#전체-아키텍처) · [documents/project/ros](../documents/project/ros/README.md)
- 인지: [perception/README.md](../lv2_module6/ros2_ws/src/perception/README.md) (구조·토픽·설정) · [FUNCTION_GUIDE.md](../lv2_module6/ros2_ws/src/perception/FUNCTION_GUIDE.md) (함수별) · [VALIDATION_GUIDE.md](../lv2_module6/ros2_ws/src/perception/VALIDATION_GUIDE.md) (시험·증빙)

## 5. 시험 결과
[results/test-results.csv](results/test-results.csv) · [results/scorecard.md](results/scorecard.md) · 팔 skill 반복 시험·RL 평가: [results/arm-rl-tests/](results/arm-rl-tests/README.md).
[results/test-results.csv](results/test-results.csv) **총 44건(T-01~T-44)**. 기존 T-01~T-35의 출처는 다음과 같다: 26건은 git 기록(PR 본문·커밋·CI)에서, T-27은 이번에 직접 돌린 모의 실행에서, T-28~35는 팔 skill 반복 시험 로그(원본은 저장소 밖, [arm-rl-tests](results/arm-rl-tests/README.md)에 요약·도표)에서 모았다. T-01~26은 **원본 로그·영상이 없고** PR 본문에 적힌 내용을 옮긴 것이다. 장비 구분은 행마다 적었다 (실물 / 모의).
- 기존 T-01~T-35 판정: 성공 20 · 실패 9 · 부분 성공 6. 공식 1·2회차는 이 표에 없고 모의 결과로 대체했다. 날짜는 PR 병합 시각(UTC) 기준이라 T-11·T-13은 한국 시간으로 하루 늦다.
- 실패 9건: 실기 미션 SURVEY 통과 못 함(바코드 7회·박스 접근 11회, T-07·08), 상자 이동 충돌감시 정지(T-09)·TF 끊김(T-10), 젯슨 CPU 과부하(T-22·23), main CI 실패(T-26), 모의 미션 한 판이 퍽 0개로 끝남(T-27, [sim-run](results/sim-run/)), 실기 누운 퍽 세우기(T-30). T-10·22·23은 수정이 반영된 기록이다.
- 공식 1·2회차: 기록이 없어 **모의 결과로 대체**했다 ([scorecard.md](results/scorecard.md), [sim-run](results/sim-run/)). 모의 1판에서 확인되는 점수는 0, 신호·바코드 25점 항목은 주입값이라 평가 제외.
- 공식 회차와 미션 완주는 실기 기록이 없고, 성공 항목 상당수는 모의(Isaac Sim·도커)나 이미지 단위 수치다. 다만 **실물 로봇에서 팔 단위로 반복한 실측**은 있다: pick → load 618회(11-14 pick 94 %), 정책 끔·켬 비교(자로 잰 손 안 어긋남 6.0 → 2.2 mm), 놓기 53/55 (T-28·29·31). 실기 누운 퍽 세우기는 15~47 %로 가장 약하다 (T-30).

### 5-1. 인지 실물 시험 (2026-11-17, T-36~T-44)

Jetson과 로봇 카메라 2대로 관측했다. 정상 입력 시험은 인지 출력 기록이며, 미등록 바코드는 자동 모드에서 BARCODE 단계의 거부·오류 종료까지 확인했다. 공식 경기 점수 및 전체 미션 완주와 구분한다.

| 항목 | 실물 확인 결과 | 시험 |
| --- | --- | --- |
| 정상 바코드 | P2-01~04 **4/4 판독** (손 카메라) | T-37 |
| 퍽 ID·색·모양 | ID 0~11 **12/12 판독**, 색·모양 확인 | T-38 |
| 장소 ID | 태그 12·13·14 **3/3 판독** (14는 손 카메라 look) | T-39 |
| 신호 관측 | 31사이클 중 30사이클 정답 일치 · 실물 순서·횟수·경계 기록 | T-36 |
| 거리 변화 | 25 cm씩 접근할 때 인지 거리가 0.239·0.256·0.253 m 감소 (줄자 절대 오차 시험은 별도) | T-40 |
| 사람·장애물 | 실물 검출 확인 | T-41 |
| 처리 부하 | CPU 평균 42.6% · YOLO 평균 5 ms · 객체 발행 평균 9.4 Hz | T-43 |
| 미등록 바코드 | 미등록 문자열 거부 → `unknown_barcode` 오류 종료 → 종료 후 속도 명령 모든 축 0 | T-44 |

- **사진·실물 로그:** [인지 시험 증빙과 사진](results/perception_1117/README.md) · [미등록 바코드 오류 처리](results/perception_1117/unknown_barcode/README.md).
- **실물 신호 기록:** [signal_real.json](results/perception_1117/signal_real.json). 모의 실행의 주입 신호와 구분한다.
- **상세 기준·진행 현황:** [인지 검증 가이드](../lv2_module6/ros2_ws/src/perception/VALIDATION_GUIDE.md) · [개인 기여 기록](docs/contributions.md).
- 저장소에 사진 25장과 선별 로그를 포함했다. 원본 MP4·rosbag은 노트북에 별도 보관한다. 자세 판정 등 상세 시험 범위는 증빙 문서에 기록했다.

## 6. 실패 원인과 수정 전후 비교
| 증상 | 원인 | 수정 | 전 → 후 |
| --- | --- | --- | --- |
| 웨이포인트·단일 목표가 3초 안에 취소됨 (`bt_navigator`: `Goal failed error_code:102 'Failed to transform end pose to global frame'`) | `controller_server`가 `map→odom`을 요청 시각으로 찾는데 최신 TF가 **3.4 ~ 8.9 ms 늦음** (`Lookup would require extrapolation into the future. Requested time …091.379904 but the latest data is at time …091.371045`). 같은 때 컨트롤러 루프 **1.39 Hz**(목표 20 Hz), load average **19~23**, 가용 메모리 약 **200 MB** → 부하로 TF 지연 | `bt_navigator`·`controller_server`에 `transform_tolerance: 0.5` (`69c0466`), 미션 노드를 끄는 `use_mission:=false` (`477fd30`) | 목표가 반복 취소됨 → 부하 load 약 9로 내려가고 두 목표가 `Goal succeeded` |
| 단일 목표가 시작 즉시 실패 (`error_code:100 'Failed to find goal checker name: '`) | goal checker를 하나 더 추가(`general_goal_checker`)한 뒤 `navigate_to_pose` BT가 빈 이름을 넘김. 컨트롤러 로그: `FollowPath called with goal_checker name in parameter 'current_goal_checker', which does not exist. Available goal checkers are: goal_checker general_goal_checker` | `navigate_to_pose.xml`의 `FollowPath`에 `goal_checker_id`·`progress_checker_id`·`path_handler_id` 전달 (`fa6c5e5`) | 시작 즉시 실패 → 같은 종류 목표가 11초 만에 `Reached the goal!` |
| 웨이포인트 근처를 맴돌다 `Failed to make progress` (오류 105)로 끊김 | 도착 판정 `stateful: false`라 위치·방향을 매 순간 둘 다 만족해야 함 → 방향을 맞추다 위치가 10 cm 밖으로 밀려 되풀이. 기록: 목표 시작 후 약 20초 맴돌다 105, 속도 기록에서 도착 직전 약 4초간 선속도 0.012 m/s·각속도 +0.15~0.4 rad/s | 도착 판정 `stateful: true` (`2a879b3`) | 105로 끊기고 재시도 → 중단 없이 약 11초 만에 `Goal succeeded` |
| 그리퍼 카메라로 본 책상·상자 높이가 실제보다 **약 20 mm 높게** 인식됨 (팀 노션 트러블슈팅, 11-11) | 11/11 팔 교체 후 joint4·joint5 서보 영점을 측정하지 않았고(실제 +2.08°, +1.53° 더 숙임), 카메라 브래킷(CB_A) 교체 후 `gripper_cam` 장착각(약 +2°)이 반영되지 않아, TF상 카메라가 실제보다 덜 숙여 있었음 | `gripper_cam` depth로 책상 평면을 여러 자세에서 재고 줄자로 확인해 실측값 반영 — joint2~5 서보 영점(joint4 −0.036, joint5 −0.027 rad) + `gripper_cam` pitch 20° → 22° (URDF · Isaac USD, 커밋 `461ef27` · `3f2fb65`) | 높이 약 20 mm 높게 인식 → 실측 보정 반영 (보정 후 오차 값은 노션에 없음) |
| 먼 퍽을 못 집음 — skill `pick-far` (팀 노션 트러블슈팅, 11-10) | 퍽 위 5 cm 수직 자세는 손 높이 때문에 쥐는 자세보다 약 30 mm 덜 닿음(x 235~260 mm vs 270~290 mm). 수평(0°) 진입은 관절 한계(joint4 −97.4°, joint2 −110°, joint3 +80°)로 불가 | 먼 퍽만 새 skill `pick_standing_far`(`/skills/pick_far`)로: 45° 기울여 진입 → 피치 다운 → 수직 닫기, 가까운 퍽은 기존 `pick` ([PR #192](https://github.com/visualkhh/Lv2_bee_Assignment/pull/192)) | 도달 면적 **448 → 533 cm² (+19 %)**, 반경 +26 mm |
| OpenCR 응답 지연 · 그리퍼 방향 문제 (팀 노션 트러블슈팅, 10-24) | 펌웨어가 서보 8개를 개별 패킷으로 읽고 쓰고, 들어온 C를 전부 처리한 뒤에야 S를 보내는 구조. 그리퍼 방향·영점·변환 상수가 실측 전 값 | 펌웨어에 BulkWrite·SyncRead 도입 (커밋 `2b7c45a4`), 펌웨어 S 43필드 ([PR #44](https://github.com/visualkhh/Lv2_bee_Assignment/pull/44)) | 개인 일지 기준 수정 전 갱신 약 17 Hz · 지연 약 100 ms. 수정 후 수치는 기록 없음 |
| 매니퓰레이터 떨림 (팀 노션 트러블슈팅, 11-11) | 노션에 원인 기록 없음 | **미해결** (노션 상태 `해결 필요`) | [Issue #153](https://github.com/visualkhh/Lv2_bee_Assignment/issues/153) 열림 상태, 수정 기록 없음 |
| cuVSLAM·nvblox 주행이 안 됨 (노션, 11-08~11) · planner 교체 (노션, 11-12) | 노션에는 "config 수정 완료 정상" · "navigation planner change smac"만 있음 | `smac` planner로 교체 ([PR #177](https://github.com/visualkhh/Lv2_bee_Assignment/pull/177)) 등 | 원인·전후 수치 기록 없음 (PR #177 본문에 수치 없음). nvblox 튜닝 커밋: `1ed3db55`(최대 적분 거리 4.5→2.5 m), `d11cabf3`(TSDF decay off) |

> 앞 3건은 2026-11-11~12 bee 실기 시험 중 `~/.ros/log`(`bt_navigator`·`controller_server`·`planner_server`·`collision_monitor`)와 속도 체인 기록에서 읽은 것이고, 아래 5건은 **팀 노션 트러블슈팅**(날짜·원인·해결)에서 옮긴 것이다.
> 원본 노드 로그는 이 저장소로 복사해 두지 않았다 (근거는 PR [#165](https://github.com/visualkhh/Lv2_bee_Assignment/pull/165)와 커밋뿐이며 원본 로그 없음).

### 6-1. 예상 실패 모드 (⚠️ 예상 · 실측 아님)

> **이 표는 코드 설계와 지금까지 겪은 비슷한 문제를 바탕으로 한 위험 분석(가설)이다. 공식 경기나 시험에서 관측한 결과가 아니며, 상태 열이 `미시험`인 것은 아직 확인하지 않았다는 뜻이다. 바코드 미등록 입력 행은 T-44 실물 시험 결과로 갱신했다.** 시험하면 [results/test-results.csv](results/test-results.csv)에 결과를 쓰고 이 표의 상태를 바꾼다. 감지·대응은 `mission_manager`·`skills`의 설계 기준이다 ([documents/project/ros/03-plan.md](../documents/project/ros/03-plan.md)).

| 단계 | 예상 실패 | 원인 가설 | 감지 | 설계상 대응 | 상태 |
| --- | --- | --- | --- | --- | --- |
| 신호 읽기 | 소등을 점멸로 세거나 같은 색 연속을 한 번으로 셈 | 조명 변화, 켜짐/소등 경계 임계값, 카메라 노출 | 사이클마다 12개·1~6회·모르는 색을 검사해 `count`·`range`·`unknown` (`confirm: 1`이라 한 사이클 결과로 확정 — 사이클 간 비교는 안 함) | 확정될 때까지 대기, 같은 오류가 `signal_error_cycles`번 이어지면 `ERROR`로 안전 정지 (추정 금지) | 11-17 실물 가림 시험 수행 · 수집 로그에 오류·정지 이벤트 없음 |
| 신호 읽기 | 12회를 다 못 세고 시간 초과 | 신호등이 화면 밖, 거리·각도 | `signal_timeout_s`(150초) | `ERROR`(`signal_timeout`) | 미시험 |
| 바코드 | 미등록 문자열 입력 | 정상 등록값 P2-01~04 밖의 문자열 | 정상 코드로 채택하지 않고 거부·재탐색, 마지막 대기 후 `unknown_barcode` | `ERROR` 종료 · 종료 후 속도 명령 0 | **실물 검증 (T-44)** · [증빙](results/perception_1117/unknown_barcode/README.md) |
| 바코드 | 끝까지 판독값 없음 | 거리·초점, 반사 | `barcode_timeout_s`(20초) 및 접근·팔 관측 조건 | `barcode_missing` 오류 종료 | 이 문서의 실물 추가 시험 범위 밖 |
| 조사(SURVEY) | 퍽 태그를 못 읽어 12개 중 일부가 등록 안 됨 | 상자 윗면 가림, 거리, 조명 | 할당 단계에서 퍽 수 부족 | 상자별 다시 보기(look 자세), 못 실은 퍽은 시작 박스에서 직접 집기 | 미시험 |
| 주행 | 목표 앞에서 멈추거나 맴돎 (`Failed to make progress`) | 벽 근처 감속, 위치 흔들림, 부하로 인한 TF 지연 | Nav2 오류 105, `navigate` 진척 감시(25초 동안 거리가 안 줄면 `nav_no_progress`) | 복구(후진·회전·대기) 뒤 재계획, 다음 접근 면으로 | 미시험 |
| 주행 | 장애물을 못 보고 벽에 붙음 | 앞 카메라만 있어 뒤를 못 봄 | 충돌 감시(`collision_monitor`), `approach_blocked` | 후진 거리를 짧게(0.10 m), 막힌 접근 면을 처음부터 제외 | 미시험 |
| 집기 | 파지 실패 또는 놓침 | 퍽이 누워 있음, 정렬 오차, 그리퍼 힘 | 그리퍼 위치·전류로 판정(`grasp_check`) | 파지 후보 `max_grasp_tries`번, 누운 퍽은 세워서(`stand`) 다시 | 미시험 |
| 놓기 | 퍽이 눕거나 칸에서 10 mm 이상 벗어남 | 벌림 높이, 장소 보정 오차 | 내려다보기 재검증(10 mm 기준) | 다시 집어 놓기, 그래도 실패하면 그 퍽만 실패로 기록하고 진행 | 미시험 |
| 재검증 | 같은 퍽을 두 번 세거나 빠뜨림 | 이전 기억 재사용 | 새로 본 퍽만으로 판정, 12개 중복·누락 점검 | 이번에 본 관측만 사용 | 미시험 |
| 복귀 | 출발 구역 안에 차체가 다 안 들어감 | 정지 위치 오차, 도면과 실제 어긋남 | 차체 외곽(`footprint`) 구역 판정 | 한 번 보정 후 3초 정지, 실패하면 `ERROR` | 미시험 |
| 공통 | 통신 끊김·OpenCR 이상 | USB 시리얼, 전원 | 상태바 `joint/odom` 끊김, 진단 | 비상정지(`/cmd/e_stop`)로 바퀴 명령 차단 | 미시험 |

## 7. 자료 위치
- 공식 경기 영상·로그·판정표: [results/scorecard.md](results/scorecard.md)
- 재현 시험 기록: [results/reproduction.md](results/reproduction.md)
- 인지 실물 사진·로그: [results/perception_1117/](results/perception_1117/README.md)
- 개인 기여 자료: [docs/contributions.md](docs/contributions.md)
- 발표 자료: [docs/presentation.md](docs/presentation.md)
- 문제 패키지: [problem/](problem/)
- 협업 규칙(브랜치·커밋·PR·리뷰): [CONTRIBUTING.md](../CONTRIBUTING.md)

## 8. 제출
- 제출 순서 (문서 작업이 끝난 뒤에 진행): (1) `docs/report`를 PR로 `main`에 병합(승인 1명 이상, 병합은 팀장 — [CONTRIBUTING.md](../CONTRIBUTING.md)) → (2) 저장소가 Public인지 확인 → (3) 태그 고정: `git tag project2-problem-submit && git tag project2-submit && git push origin project2-problem-submit project2-submit` → (4) 과제/프로젝트 > 프로젝트 (2)에 **GitHub 링크 · 프로젝트명 · 팀 노션 링크** 제출 → (5) 제출 메시지: `팀명 / 저장소 / 태그·커밋 / 문제·배치 / 실행 안내 / 영상·로그 / 개인 근거 / 접근 권한 확인`.
- 남은 항목 목록: [REMAINING.md](REMAINING.md)
