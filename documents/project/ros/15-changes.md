# 검토 반영 — 현장 시험·리뷰 뒤 바뀐 설계

시험·리뷰에서 문제가 나와 설계를 바꾼 기록입니다. 날짜는 PR 병합일(작업일이 다르면 따로 적음), 이유는 PR 본문에서 옮겼습니다. 시험 수치는 [test-results.csv](../../../project2/results/test-results.csv)의 시험 ID로 이어집니다. 전체 목록은 `gh pr list --state merged`.

PR 링크: `https://github.com/visualkhh/Lv2_bee_Assignment/pull/<번호>`

| 날짜 | 바꾼 것 | 이유 (PR·commit 본문) | 링크 |
|---|---|---|---|
| 10-29 | SLAM을 RTAB-Map에서 **cuVSLAM(위치) + nvblox(지도)** 로 교체 | Jetson Orin 실기에서 정지 drift 0.5 mm · 0.02°, 1 m 왕복 추적 잃음 0, 지도 생성 확인 (T-01). 지도 정본 `rtabmap.db` → `nvblox.nvblx` | [#43](https://github.com/visualkhh/Lv2_bee_Assignment/pull/43) · [#40](https://github.com/visualkhh/Lv2_bee_Assignment/pull/40) |
| 11-04 | 퍽 인지 모델 v3 → v4, 기억을 통합 방식으로 | 로봇 카메라 시험 퍽 검출률 58% → 91.5% (T-03) | [#63](https://github.com/visualkhh/Lv2_bee_Assignment/pull/63) |
| 11-04 | Nav2 전역 경로를 A\*로 (NavFn `use_astar`) | PR 제목 외 이유 기록 없음 (TODO: 작성 필요) | [#58](https://github.com/visualkhh/Lv2_bee_Assignment/pull/58) |
| 11-05 | RL 주행 컨트롤러·휠 드리프트 보정 적용, `rl_nav_correction`·`rl-nav-turner` 노드 제거, scan 모드에서도 팔을 stow로 | 팔 자세가 무게중심에 영향 (PR 본문 "무게중심 이슈") | [#65](https://github.com/visualkhh/Lv2_bee_Assignment/pull/65) |
| 11-07 | RlNavController를 ONNX 정책으로 로드·토글, DWB와 A/B 비교 가능하게 | `test-nav-policy`(ONNX 로드 → toggle off/on → DWB/정책 주행)를 `test-all` 게이트에 추가해 검증 | [#82](https://github.com/visualkhh/Lv2_bee_Assignment/pull/82) |
| 11-09 | 퍽 자세(서 있음/누움)를 **윗면 높이**로 판단 | 실물 그리퍼캠 퍽 221개에서 88% → 99% (T-06) | [#95](https://github.com/visualkhh/Lv2_bee_Assignment/pull/95) |
| 11-09 | 시작 상자 inspect를 퍽 자리마다가 아니라 **상자 한 번 보기**(look 자세 → 다시 담기)로 | 지금 자세에서 다 보이면 팔을 안 움직이고 끝내 시간 단축 | [#80](https://github.com/visualkhh/Lv2_bee_Assignment/pull/80) |
| 11-11 | navigate 진척 감시: 남은 거리가 25초 동안 안 줄면 `nav_no_progress` | 11-10 Isaac에서 벽 앞 회복·재계획 반복으로 navigate가 4~5분 끝나지 않음 → 26초 만에 판단 (T-14) | [#145](https://github.com/visualkhh/Lv2_bee_Assignment/pull/145) |
| 11-11 | 경기장 벽을 처음부터 알게 하고, 벽에 막힌 다가설 면 제외 | SURVEY 첫 이동에서 로봇이 출발 구역 옆 벽에 박고 몇 분씩 멈춤 (벽-박스 틈 0.27 m, 11-10 Isaac) | [#141](https://github.com/visualkhh/Lv2_bee_Assignment/pull/141) |
| 11-11 | place 벌림 36 → 46 mm, MoveIt 시작 허용 오차 0.05 | 놓은 퍽이 늘 누움(약 5 cm 어긋남), `load_failed` 4회 — 사각 퍽(대각선 42 mm)이 기울면 손가락에 낌 | [#149](https://github.com/visualkhh/Lv2_bee_Assignment/pull/149) |
| 11-11 | 퍽·신호등 모델 v5 | 퍽 727/767(94.8%) · 신호등 상태 579/582 (T-04) | [#144](https://github.com/visualkhh/Lv2_bee_Assignment/pull/144) |
| 11-12 | **팔 scan 자세 제거** (미션 GO_OBSERVE는 navigate만, 팔 stow). 코드 제거 커밋은 11-09, 병합은 11-12 | 실기 11-12: 팔이 왼쪽 뒤 150°로 뻗은 채 회전·주행. 11-09 Isaac에서도 관성으로 26° 밀리고 무게중심 쏠림. nvblox는 pole_front_cam 하나만 사용 | [#168](https://github.com/visualkhh/Lv2_bee_Assignment/pull/168) |
| 11-12 | 박스·장소 방문 순서를 도면 벽 A\*로 정함 | 번호 순서는 배치를 무시. 33.6 m → 26.6 m (−21 %, T-25). 11-14에 진입 면까지 넣어 재계산 | [#146](https://github.com/visualkhh/Lv2_bee_Assignment/pull/146) |
| 11-12 | **인지 노드 C++ 전환** (YOLO ONNX GPU · AprilTag · 바코드) | Jetson 인지 CPU 약 108% → 약 23% (T-19). AprilTag 후보 최소 둘레로 태그 찾기 시간 약 1/5 | [#164](https://github.com/visualkhh/Lv2_bee_Assignment/pull/164) |
| 11-13 | mission → coordinator 계획 전달을 topic에서 **ExecutePlan Action**(`/task/execute_plan`)으로 | goal id로 결과를 요청에 묶고 feedback·result(`success`·`final_step`·`reason`)를 받기 위해. `/task/sub_goals`는 legacy 입구로 유지 | [#195](https://github.com/visualkhh/Lv2_bee_Assignment/pull/195) |
| 11-13 | 현장 진단 모드(DIAG), 단계 시간 기록(`steps.jsonl`) | 실기 미션이 SURVEY를 못 넘음(`barcode_missing` 7회 · `approach_blocked` 11회, T-07·T-08). 관측 하나 실패에 ERROR로 멈춰 자료가 적음 | [#202](https://github.com/visualkhh/Lv2_bee_Assignment/pull/202) |
| 11-13 | 상자 도킹을 도면이 아니라 **인지한 상자** 기준으로, 상자별 접근 면·exit 후진 | 도면을 믿지 않고 Nav2는 상자 근처까지만, `skill_approach`가 카메라로 붙음 | [#203](https://github.com/visualkhh/Lv2_bee_Assignment/pull/203) |
| 11-13 | 신호 확정 전에는 팔을 접고 기다림, 모터 에러·팔 처짐이면 미션 정지 | 실기 11-14: gaze로 2분 넘게 뻗다 joint2 과부하 → 팔이 처져 `barcode_missing` (신호는 R4 Y2 G6으로 확정) | [#204](https://github.com/visualkhh/Lv2_bee_Assignment/pull/204) |
| 11-14 | 코너 회전 뒤 위치 틀어짐 수정 (cuVSLAM rig 수평), 도착 뒤 목표 방향 회전 | 카메라가 19° 숙인 채 rig 기준이 되어 `planar_constraints`가 기울어진 평면에 묶임 (EKF 아님). Isaac 정답과 일치 확인 (T-13) | [#225](https://github.com/visualkhh/Lv2_bee_Assignment/pull/225) |
| 11-15 | AprilTag **cuAprilTags(GPU)** · C++ depth gate | 11-14 perception이 코어 하나 100%, py_helpers 60%대, load 20/6, 메모리 바닥으로 재부팅 | [#226](https://github.com/visualkhh/Lv2_bee_Assignment/pull/226) |
| 11-15 | **collision_monitor** 감시 구역을 몸 모양 사각형으로 (Approach 앞 0.22 · 좌우 ±0.17, `time_before_collision` 1.0 → 0.6 s, SlowZone 앞쪽 사각형) | 11-15 GOTO S3·S6에서 approach 51번 · slowdown 27번 → `nav_no_progress`. 문틈 0.57 m에서 원 0.30 · 반원 ±0.20이 늘 걸림 (T-09) | [#243](https://github.com/visualkhh/Lv2_bee_Assignment/pull/243) |
| 11-15 | 주행 중 인지 끔(`navigation_infer_hz` 0이면 navigate 중 YOLO 건너뜀), 도착 1 m 전부터 켬 | 11-15 실기 제어 주기 최저 3.5 Hz, EKF 갱신 실패 74번 (T-23). 주행 자체는 YOLO를 안 씀. 이후 2 Hz로 완화 | [#244](https://github.com/visualkhh/Lv2_bee_Assignment/pull/244) · [#278](https://github.com/visualkhh/Lv2_bee_Assignment/pull/278) |
| 11-15 | **YOLO TensorRT FP16** 엔진 (엔진 파일 있을 때만, 없으면 ONNX Runtime) | 11-15 Jetson 주행 중 CPU 부하 점검에서 나옴. 전처리·후처리는 그대로 (T-21) | [#246](https://github.com/visualkhh/Lv2_bee_Assignment/pull/246) |
| 11-15 | `cuvslam_slam` `map_tf_tolerance` 0.5 → 1.0, bag 녹화 부하 낮춤 | 미션 시작 직후 자동 bag 녹화가 CPU를 몰아 cuVSLAM이 1초 멈춤 → TF 끊김으로 약 2초 만에 `nav_failed`. 수정 뒤 map→odom 최악 0.35 s 미래 (T-10) | [#252](https://github.com/visualkhh/Lv2_bee_Assignment/pull/252) |
| 11-15 | 문 경유점 skill (문 앞에서 축 맞추고 통과) | 경유점 없이는 같은 자리 정지가 재현됨 (위치 오차 3 mm, T-12) | [#235](https://github.com/visualkhh/Lv2_bee_Assignment/pull/235) |
| 11-15 | 사람이 찍은 **경유점 지도**로 경로 선택, 끼임 풀기(후진 → 회전), EKF에서 바퀴 odom 제외(시험) | 4·5·3번 상자 도킹 성공, 오차 along ≤ 8.7 mm · lateral ≤ 24.7 mm · yaw ≤ 1.95° (T-11, 시뮬) | [#260](https://github.com/visualkhh/Lv2_bee_Assignment/pull/260) |
| 11-15 | 경유점 길을 **NavigateThroughPoses 한 번**으로 | 경유점마다 NavigateToPose를 보내 도착 판정 6 cm에서 매번 감속·정지·재계획. 0.40 m 안이면 지난 점으로 보고 서지 않음 | [#274](https://github.com/visualkhh/Lv2_bee_Assignment/pull/274) |
| 11-15 | 답지(teach & repeat) 재생 · route_tables · 몸 모양 collision_monitor를 develop(C++) 위에 다시 | #265~#271을 #272(cuvslam C++/GPU 등) 위에 한 번에 재적용 | [#273](https://github.com/visualkhh/Lv2_bee_Assignment/pull/273) |
| 11-16 | 끼임 풀기 방향(가까운 장애물 쪽 판단) · 막은 구역 안 갇힘 · 적재 실패 되돌리기 · RETURN 재시도 | 11-16 시뮬: 막은 구역 안으로 들어가 이후 모든 이동이 0 s 만에 `nav_no_path_zone`, RETURN이 한 번 실패로 ERROR | [#286](https://github.com/visualkhh/Lv2_bee_Assignment/pull/286) |
| 11-16 | inspect: 가까운 퍽은 look 자세로 **물러나며 보기** | 상자에 퍽이 하나 남았을 때 너무 가까워 `inspect_failed`(IK 밖)·`inspect_partial`로 다음 상자로 넘어감 | [#287](https://github.com/visualkhh/Lv2_bee_Assignment/pull/287) |
| 11-17 | main 병합 직후 `test-all` 실패 확인 | `plan.yaml` 163행 ParserError (`start_box_retreat` 키 누락, 커밋 c9002372) → `/perception` 노드 없음. 369 tests는 0 failures (T-26) | Actions run 43994595225 |

## 읽는 법

- "이유" 열은 PR 본문의 "왜/문제/현장 문제" 문장을 줄인 것이고, 수치는 본문 그대로입니다.
- 실기와 Isaac 시뮬 구분은 본문 표기를 따랐습니다(괄호에 11-10 Isaac 등).
- 이유를 PR 본문에서 못 찾은 행(#58)은 TODO로 남겼습니다.
- 관련 시험 결과: T-01·T-03·T-04·T-06·T-09~T-14·T-19~T-23·T-25·T-26.
