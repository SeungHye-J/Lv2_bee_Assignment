# 프로젝트 2 — 주사위 기억 로봇 (BEE 팀)

> 제출 요구사항(발제문) 기준 목차. 이미 문서화된 내용은 새로 쓰지 않고 링크로 연결하고, **작성 필요**만 이 폴더에서 채운다.

## 1. 프로젝트 목표
- 미션 정의·규칙·배점: [documents/mission/README.md](../documents/mission/README.md)
- TODO: 우리 팀의 목표와 범위 3~5줄 — **만드는 법:** 발제문 "프로젝트 2 — 주사위 기억 로봇"과 [documents/mission/README.md](../documents/mission/README.md)를 읽고, 우리가 구현한 범위(신호 인식·바코드·할당·주행·집기·놓기·재검증·복귀 중 어디까지)와 못 한 범위를 적는다.

## 2. 장비·소프트웨어 버전
- 로봇·팔·카메라·제어보드: [lv2_module6/README.md — 미션 · 코스 · 로봇](../lv2_module6/README.md#미션--코스--로봇)
- TODO: 실제로 쓴 버전 표 — **만드는 법:** 로봇에서 `cat /etc/nv_tegra_release` · `nvcc --version` · `ros2 doctor --report`(ROS) · `python3 -c "import onnxruntime"`로 확인하거나, 이미 정리된 [documents/jetson-orin-nano/](../documents/jetson-orin-nano/)에서 옮긴다. ONNX Runtime 1.24.3 · CUDA 13.2 · JetPack L4T R39는 이 저장소 문서에 있다. cuVSLAM·nvblox·Nav2·MoveIt 버전은 확인해서 적는다.
  - Jetson ONNX Runtime 설치 기록: [documents/jetson-orin-nano/ONNXRUNTIME.md](../documents/jetson-orin-nano/ONNXRUNTIME.md)

## 3. 설치 · 실행 · 정지 · 초기화
- 설치·실행·정지: [lv2_module6/README.md — 실행](../lv2_module6/README.md#실행) (`start.sh` · `stop.sh` · `run-cli` · 테스트베드)
- TODO: 경기 초기화 절차 — **만드는 법:** 경기 직전에 실제로 하는 순서를 번호로 적는다. 로봇을 출발 구역(−X를 보게)에 두기 → 이전 `bringup` 종료(`./ros2_ws/stop.sh`) → 지도 선택(`./tools/run-cli`) → `./ros2_ws/start.sh` → 미션 시작(`./tools/run-cli mission start`). 실제 쓰는 명령과 순서가 맞는지 한 번 따라 해 보고 고친다.

## 4. 시스템 구성
[docs/architecture.md](docs/architecture.md) (상태도·완료 조건·오류 처리) · [lv2_module6/README.md — 전체 아키텍처](../lv2_module6/README.md#전체-아키텍처) · [documents/project/ros](../documents/project/ros/README.md)

## 5. 시험 결과
[results/test-results.csv](results/test-results.csv) · [results/scorecard.md](results/scorecard.md).
- TODO: 시험 결과 요약 5~10줄 — **만드는 법:** `results/test-results.csv`의 성공·실패·미확인 건수를 세어 적고, 공식 1·2회차 점수와 완주 여부를 `scorecard.md`에서 옮긴다. 성공한 것만 쓰지 말고 실패·미확인도 쓴다.

## 6. 실패 원인과 수정 전후 비교
| 증상 | 원인 | 수정 | 전 → 후 |
| --- | --- | --- | --- |
| 웨이포인트·단일 목표가 3초 안에 취소됨 (`bt_navigator`: `Goal failed error_code:102 'Failed to transform end pose to global frame'`) | `controller_server`가 `map→odom`을 요청 시각으로 찾는데 최신 TF가 **3.4 ~ 8.9 ms 늦음** (`Lookup would require extrapolation into the future. Requested time …091.379904 but the latest data is at time …091.371045`). 같은 때 컨트롤러 루프 **1.39 Hz**(목표 20 Hz), load average **19~23**, 가용 메모리 약 **200 MB** → 부하로 TF 지연 | `bt_navigator`·`controller_server`에 `transform_tolerance: 0.5` (`69c0466`), 미션 노드를 끄는 `use_mission:=false` (`477fd30`) | 목표가 반복 취소됨 → 부하 load 약 9로 내려가고 두 목표가 `Goal succeeded` |
| 단일 목표가 시작 즉시 실패 (`error_code:100 'Failed to find goal checker name: '`) | goal checker를 하나 더 추가(`general_goal_checker`)한 뒤 `navigate_to_pose` BT가 빈 이름을 넘김. 컨트롤러 로그: `FollowPath called with goal_checker name in parameter 'current_goal_checker', which does not exist. Available goal checkers are: goal_checker general_goal_checker` | `navigate_to_pose.xml`의 `FollowPath`에 `goal_checker_id`·`progress_checker_id`·`path_handler_id` 전달 (`fa6c5e5`) | 시작 즉시 실패 → 같은 종류 목표가 11초 만에 `Reached the goal!` |
| 웨이포인트 근처를 맴돌다 `Failed to make progress` (오류 105)로 끊김 | 도착 판정 `stateful: false`라 위치·방향을 매 순간 둘 다 만족해야 함 → 방향을 맞추다 위치가 10 cm 밖으로 밀려 되풀이. 기록: 목표 시작 후 약 20초 맴돌다 105, 속도 기록에서 도착 직전 약 4초간 선속도 0.012 m/s·각속도 +0.15~0.4 rad/s | 도착 판정 `stateful: true` (`2a879b3`) | 105로 끊기고 재시도 → 중단 없이 약 11초 만에 `Goal succeeded` |
| 그리퍼 카메라로 본 책상·상자 높이가 실제보다 **약 20 mm 높게** 인식됨 (팀 노션 트러블슈팅, 11-11) | 11/11 팔 교체 후 joint4·joint5 서보 영점을 측정하지 않았고(실제 +2.08°, +1.53° 더 숙임), 카메라 브래킷(CB_A) 교체 후 `gripper_cam` 장착각(약 +2°)이 반영되지 않아, TF상 카메라가 실제보다 덜 숙여 있었음 | `gripper_cam` depth로 책상 평면을 여러 자세에서 재고 줄자로 확인해 실측값 반영 — joint2~5 서보 영점(joint4 −0.036, joint5 −0.027 rad) + `gripper_cam` pitch 20° → 22° (URDF · Isaac USD, 커밋 `461ef27` · `3f2fb65`) | 높이 약 20 mm 높게 인식 → 실측 보정 반영 (보정 후 오차 값은 노션에 없음 — TODO) |
| 먼 퍽을 못 집음 — skill `pick-far` (팀 노션 트러블슈팅, 11-10) | 퍽 위 5 cm 수직 자세는 손 높이 때문에 쥐는 자세보다 약 30 mm 덜 닿음(x 235~260 mm vs 270~290 mm). 수평(0°) 진입은 관절 한계(joint4 −97.4°, joint2 −110°, joint3 +80°)로 불가 | 먼 퍽만 새 skill `pick_standing_far`(`/skills/pick_far`)로: 45° 기울여 진입 → 피치 다운 → 수직 닫기, 가까운 퍽은 기존 `pick` ([PR #192](https://github.com/visualkhh/Lv2_bee_Assignment/pull/192)) | 도달 면적 **448 → 533 cm² (+19 %)**, 반경 +26 mm |
| OpenCR 응답 지연 · 그리퍼 방향 문제 (팀 노션 트러블슈팅, 10-24) | 펌웨어가 서보 8개를 개별 패킷으로 읽고 쓰고, 들어온 C를 전부 처리한 뒤에야 S를 보내는 구조. 그리퍼 방향·영점·변환 상수가 실측 전 값 | TODO: 해결 방법이 노션에 비어 있음 — 실제 수정(펌웨어 읽기·쓰기 방식, 실측 상수 반영)과 PR을 찾아 적는다 ([PR #44](https://github.com/visualkhh/Lv2_bee_Assignment/pull/44) 펌웨어 S 43필드 등이 후보) | TODO |
| 매니퓰레이터 떨림 (팀 노션 트러블슈팅, 11-11) | 노션에 원인 기록 없음 | **미해결** (노션 상태 `해결 필요`) | TODO: 지금 상태 확인 후 갱신 |
| cuVSLAM·nvblox 주행이 안 됨 (노션, 11-08~11) · planner 교체 (노션, 11-12) | 노션에는 "config 수정 완료 정상" · "navigation planner change smac"만 있음 | `smac` planner로 교체 ([PR #177](https://github.com/visualkhh/Lv2_bee_Assignment/pull/177)) 등 | TODO: 원인과 전후 수치를 PR 본문·로그에서 보충 |

> 앞 3건은 2026-11-11~12 bee 실기 시험 중 `~/.ros/log`(`bt_navigator`·`controller_server`·`planner_server`·`collision_monitor`)와 속도 체인 기록에서 읽은 것이고, 아래 5건은 **팀 노션 트러블슈팅**(날짜·원인·해결)에서 옮긴 것이다.
> TODO: 원본 로그를 이 저장소로 복사해 링크한다 — **만드는 법:** 로봇에서 `scp -r bee@<주소>:~/.ros/log/ project2/results/nav-logs/` (11-11~12 시각대 파일만 골라도 됨, `bt_navigator_*.log`·`controller_server_*.log`). 로그가 이미 지워졌으면 이 표의 근거를 PR 본문(#165)과 커밋으로만 남기고 "원본 로그 없음"이라고 쓴다. 이 표에 더 넣을 실패(주행 외 인지·집기·미션)가 있으면 같은 형식으로 추가한다.

### 6-1. 예상 실패 모드 (⚠️ 예상 · 실측 아님)

> **이 표는 코드 설계와 지금까지 겪은 비슷한 문제를 바탕으로 한 위험 분석(가설)이다. 공식 경기나 시험에서 관측한 결과가 아니며, 상태 열이 `미시험`인 것은 아직 확인하지 않았다는 뜻이다.** 시험하면 [results/test-results.csv](results/test-results.csv)에 결과를 쓰고 이 표의 상태를 바꾼다. 감지·대응은 `mission_manager`·`skills`의 설계 기준이다 ([documents/project/ros/03-plan.md](../documents/project/ros/03-plan.md)).

| 단계 | 예상 실패 | 원인 가설 | 감지 | 설계상 대응 | 상태 |
| --- | --- | --- | --- | --- | --- |
| 신호 읽기 | 소등을 점멸로 세거나 같은 색 연속을 한 번으로 셈 | 조명 변화, 켜짐/소등 경계 임계값, 카메라 노출 | 두 사이클 결과가 다르면 `unconfirmed` | 확정될 때까지 대기, 같은 오류가 `signal_error_cycles`번 이어지면 `ERROR`로 안전 정지 (추정 금지) | 미시험 |
| 신호 읽기 | 12회를 다 못 세고 시간 초과 | 신호등이 화면 밖, 거리·각도 | `signal_timeout_s`(150초) | `ERROR`(`signal_timeout`) | 미시험 |
| 바코드 | 문자열 오독 또는 못 읽음 | 손 카메라 거리·초점, 반사 | `barcode_timeout_s`(20초) 안에 이번에 본 바코드 없음 | `ERROR`로 정지 | 미시험 |
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
- 개인 기여 자료: [docs/contributions.md](docs/contributions.md)
- 발표 자료: [docs/presentation.md](docs/presentation.md)
- 문제 패키지: [problem/](problem/)
- 협업 규칙(브랜치·커밋·PR·리뷰): [CONTRIBUTING.md](../CONTRIBUTING.md)

## 8. 제출
- TODO: 제출 마무리 — **만드는 법:** (1) `docs/report`를 PR로 `main`에 병합(승인 1명 이상, 병합은 팀장 — [CONTRIBUTING.md](../CONTRIBUTING.md)) → (2) 저장소가 Public인지 확인 → (3) 태그 고정: `git tag project2-problem-submit && git tag project2-submit && git push origin project2-problem-submit project2-submit` → (4) 과제/프로젝트 > 프로젝트 (2)에 **GitHub 링크 · 프로젝트명 · 팀 노션 링크** 제출 → (5) 제출 메시지: `팀명 / 저장소 / 태그·커밋 / 문제·배치 / 실행 안내 / 영상·로그 / 개인 근거 / 접근 권한 확인`.
- 남은 TODO 목록: [TODO.md](TODO.md)
