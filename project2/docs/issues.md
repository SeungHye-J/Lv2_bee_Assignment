# GitHub Issue 목록 (발견된 오류와 처리 현황)

`gh issue list --state all`로 뽑은 **66건** (열림 25 · 처리됨 41). 목록·제목·담당자·상태·날짜는 GitHub 기록 그대로이고, 사람이 읽고 채워야 하는 칸은 마지막 열이다.

- TODO: 마지막 열 `원인 · 검토자`를 채운다 — **만드는 법:** 오류성 이슈(예: [#153](https://github.com/visualkhh/Lv2_bee_Assignment/issues/153) 매니퓰레이터 떨림, [#190](https://github.com/visualkhh/Lv2_bee_Assignment/issues/190) 벽 앞 접근 불가)부터 이슈 본문과 연결된 PR을 열어 `원인(한 줄) / 검토자(PR을 승인한 팀원)`를 적는다. 기능 작업 이슈는 비워 둬도 된다. 발제문은 "발견된 오류, 발생 조건, 원인, 수정 담당자, 검토자, 처리 현황"을 요구하므로, **오류가 아닌 작업 이슈와 오류 이슈를 구분**해 두면 좋다 (제목 접두어 `[control]`·`[plan]` 등은 영역이다). 다 채우면 이 `TODO`를 지운다.
- 이슈 담당자(assignee)를 "수정 담당자"로 보았다. 실제 수정한 사람이 다르면 고친다.

## 다시 뽑는 법
```bash
gh issue list --repo visualkhh/Lv2_bee_Assignment --state all --limit 300 \
  --json number,title,state,assignees,url,createdAt,closedAt
```

## 표 (최신순)

| 이슈 | 제목 (발견된 오류·작업) | 생성 | 담당 | 처리 | 닫힌 날 | 원인 · 검토자 |
| --- | --- | --- | --- | --- | --- | --- |
| [#251](https://github.com/visualkhh/Lv2_bee_Assignment/issues/251) | [시연] 시나리오에 맞게 진행 여부 체크 연습  및 시연 담당 | 11-15 | 전승혜, 권형중 | 열림 |  | |
| [#191](https://github.com/visualkhh/Lv2_bee_Assignment/issues/191) | [검토] 맵 제작시 검토, 못지나갈자리 미리 막아두기 | 11-13 | 권형중 | 열림 |  | |
| [#190](https://github.com/visualkhh/Lv2_bee_Assignment/issues/190) | [검토] 네비게이션 주행모드시 벽에 어깨빵당할까봐 접근조치못하는경우 | 11-13 | 권형중 | 열림 |  | |
| [#189](https://github.com/visualkhh/Lv2_bee_Assignment/issues/189) | [검토] jetson memory 많이잡아먹으면 조치해볼만할것들 | 11-13 | 권형중 | 열림 |  | |
| [#162](https://github.com/visualkhh/Lv2_bee_Assignment/issues/162) | coordinator는 아직 기존처럼 /navigate_to_pose 사용중  -> 다중웨이포인트로	/navigate_through_poses 변경 | 11-12 | 김현하, 권형중 | 처리됨 | 11-12 | |
| [#153](https://github.com/visualkhh/Lv2_bee_Assignment/issues/153) | [control] 매니퓰레이터 떨림 원인 파악 및 수정 | 11-10 | - | 열림 |  | |
| [#150](https://github.com/visualkhh/Lv2_bee_Assignment/issues/150) | [tools/cli] TUI 실기 미션 시험용 정리 — deprecated 6건 · 실시간 정보 추가 16건 | 11-10 | - | 열림 |  | |
| [#138](https://github.com/visualkhh/Lv2_bee_Assignment/issues/138) | [perception] 박스의 면 인지 확인 | 11-10 | 전승혜 | 처리됨 | 11-10 | |
| [#137](https://github.com/visualkhh/Lv2_bee_Assignment/issues/137) | [skills] 원거리 퍽 pick을 위한 스킬노드 작성 | 11-10 | 전승혜 | 처리됨 | 11-13 | |
| [#136](https://github.com/visualkhh/Lv2_bee_Assignment/issues/136) | [docker][faker] test-faker의 작업 흐름 검사 복구 (mission_manager 시작 → coordinator → skill → SUCCEEDED) | 11-10 | 김현하 | 처리됨 | 11-10 | |
| [#135](https://github.com/visualkhh/Lv2_bee_Assignment/issues/135) | [skills] skill 액션 서버 단위 테스트가 없음 (navigate·approach·pose·inspect·pick·place·load·unload·stand) | 11-10 | 권형중 | 열림 |  | |
| [#134](https://github.com/visualkhh/Lv2_bee_Assignment/issues/134) | [plan][coordinator] 모드 취소 이유의 `explore_*` 잔재 정리 (mission_manager 중단 사유 매칭과 함께) | 11-10 | - | 처리됨 | 11-10 | |
| [#133](https://github.com/visualkhh/Lv2_bee_Assignment/issues/133) | [plan][coordinator] 테스트 보강: 실행 규칙 2·3·4와 장부 | 11-10 | - | 처리됨 | 11-10 | |
| [#132](https://github.com/visualkhh/Lv2_bee_Assignment/issues/132) | [plan][coordinator] 옛 tray 호환(`pick tray`·`place tray`) 제거와 mission_manager의 tray_* 이유 정리 | 11-10 | - | 처리됨 | 11-10 | |
| [#131](https://github.com/visualkhh/Lv2_bee_Assignment/issues/131) | [plan][coordinator] skill이 결과를 안 줄 때를 막는 단계 타임아웃이 없음 — 확인·추가 | 11-10 | 권형중 | 열림 |  | |
| [#130](https://github.com/visualkhh/Lv2_bee_Assignment/issues/130) | [plan][coordinator] skill 서버 미기동 시 reason이 nav/pick/arm 3종뿐 — 어느 skill인지 구분 | 11-10 | 권형중 | 열림 |  | |
| [#129](https://github.com/visualkhh/Lv2_bee_Assignment/issues/129) | [docs][mission_manager] DEBUG.md의 "신호를 아직 해석하지 않는다" 설명이 코드와 다름 | 11-10 | - | 처리됨 | 11-10 | |
| [#128](https://github.com/visualkhh/Lv2_bee_Assignment/issues/128) | [ros_system][cuvslam·nvblox] 매핑 품질: 먼 거리 depth 잡음·제자리 회전 25~30%·Grid/RangeMax 시험 | 11-10 | 김현하 | 처리됨 | 11-10 | |
| [#127](https://github.com/visualkhh/Lv2_bee_Assignment/issues/127) | [ros_system][nav2] 실기 확인: stateful goal checker, 허용 오차, 파라미터 이름 오타 | 11-10 | 김현하 | 처리됨 | 11-10 | |
| [#126](https://github.com/visualkhh/Lv2_bee_Assignment/issues/126) | [ros_system][move_group] pick_ik·octomap 센서(pole_front_cam 점구름) 연결 | 11-10 | 전승혜 | 처리됨 | 11-10 | |
| [#125](https://github.com/visualkhh/Lv2_bee_Assignment/issues/125) | [ros_system][moveit] beebot.srdf 충돌 제외(Never) 쌍을 실기 치수로 다시 생성 | 11-10 | - | 처리됨 | 11-10 | |
| [#124](https://github.com/visualkhh/Lv2_bee_Assignment/issues/124) | [ros_system][moveit] 관절 가속도 한계 실측 (joint_limits.yaml) | 11-10 | - | 처리됨 | 11-10 | |
| [#123](https://github.com/visualkhh/Lv2_bee_Assignment/issues/123) | [ros_system][ekf] 실기 공분산·imu0_remove_gravitational_acceleration 튜닝 | 11-10 | 김현하 | 처리됨 | 11-10 | |
| [#122](https://github.com/visualkhh/Lv2_bee_Assignment/issues/122) | [control][manual_control] collision_monitor 자동 끄기(MANUAL·MAPPING) 실기 확인과 CLI 토글 우선순위 정리 | 11-10 | 김현하 | 처리됨 | 11-10 | |
| [#121](https://github.com/visualkhh/Lv2_bee_Assignment/issues/121) | [control][BeebotSystemHardware] 관절 비틀림 모델(중력 토크·강성) 사본 3곳 동기화 | 11-10 | 김현하 | 처리됨 | 11-10 | |
| [#120](https://github.com/visualkhh/Lv2_bee_Assignment/issues/120) | [control][arm_safety_monitor] 퍽 든 상태의 정상 전류 실측 → 관절별 effort_threshold | 11-10 | 김현하 | 처리됨 | 11-10 | |
| [#119](https://github.com/visualkhh/Lv2_bee_Assignment/issues/119) | [rl] 실기 pick 정책 off·on 비교 | 11-10 | 전승혜, 권형중, 송정혁 | 열림 |  | |
| [#118](https://github.com/visualkhh/Lv2_bee_Assignment/issues/118) | [skills] inspect 상자 보기와 load→pick 연속 동작 | 11-10 | 전승혜, 권형중, 송정혁 | 열림 |  | |
| [#117](https://github.com/visualkhh/Lv2_bee_Assignment/issues/117) | [rl] 누운 퍽 세우기 학습 (stand C·stand_grasp) | 11-10 | 권형중, 송정혁 | 열림 |  | |
| [#116](https://github.com/visualkhh/Lv2_bee_Assignment/issues/116) | [control] 팔 처짐 보정 twist_state 및 calib 도구 | 11-10 | 권형중, 송정혁 | 처리됨 | 11-11 | |
| [#115](https://github.com/visualkhh/Lv2_bee_Assignment/issues/115) | [rl] 잔차 RL 학습 환경 및 단계 학습 (place·pick·load·unload) | 11-10 | 권형중, 송정혁 | 열림 |  | |
| [#114](https://github.com/visualkhh/Lv2_bee_Assignment/issues/114) | [urdf] 로봇 모델·Isaac 장면 셋업 | 11-10 | 전승혜 | 열림 |  | |
| [#113](https://github.com/visualkhh/Lv2_bee_Assignment/issues/113) | [skills][skill_pick·place·load·unload·stand] 예전(moveit) 경로 정리 여부 결정 | 11-09 | - | 처리됨 | 11-10 | |
| [#112](https://github.com/visualkhh/Lv2_bee_Assignment/issues/112) | [skills][skill_stand] | 11-09 | 권형중 | 열림 |  | |
| [#111](https://github.com/visualkhh/Lv2_bee_Assignment/issues/111) | [skills][skill_stand] 경유점 ik / table 성공률 비교 (Isaac · 실기) | 11-09 | - | 처리됨 | 11-10 | |
| [#110](https://github.com/visualkhh/Lv2_bee_Assignment/issues/110) | [skills][skill_load·skill_unload] 연동확인 | 11-09 | 권형중 | 열림 |  | |
| [#109](https://github.com/visualkhh/Lv2_bee_Assignment/issues/109) | [skills][skill_load·skill_unload] 적재함 12칸 실측 | 11-09 | - | 처리됨 | 11-10 | |
| [#108](https://github.com/visualkhh/Lv2_bee_Assignment/issues/108) | [skills][skill_place] 상자 위 놓는 높이 실기 확인 | 11-09 | 김현하, 전승혜 | 처리됨 | 11-10 | |
| [#107](https://github.com/visualkhh/Lv2_bee_Assignment/issues/107) | [skills][skill_pick·skill_unload] | 11-09 | 권형중 | 열림 |  | |
| [#106](https://github.com/visualkhh/Lv2_bee_Assignment/issues/106) | [skills][skill_pick] 퍽·주변 장애물pick | 11-09 | - | 처리됨 | 11-10 | |
| [#105](https://github.com/visualkhh/Lv2_bee_Assignment/issues/105) | [plan][coordinator] 재계획 제거 뒤 연속 실패 처리 확인 | 11-09 | 전승혜, 권형중 | 열림 |  | |
| [#104](https://github.com/visualkhh/Lv2_bee_Assignment/issues/104) | [plan][mission_manager] plan_next(학습 플래너) 실기·시뮬레이터 확인 | 11-09 | 권형중 | 열림 |  | |
| [#103](https://github.com/visualkhh/Lv2_bee_Assignment/issues/103) | [plan][coordinator] /robot/say를 받는 쪽 결정 (user_input 삭제 후) | 11-09 | 김현하 | 처리됨 | 11-10 | |
| [#102](https://github.com/visualkhh/Lv2_bee_Assignment/issues/102) | [perception][signal_decoder] MissionSignal.msg의 옛 3단계 규칙 칸(color·count·sum·dest_tag) 삭제 | 11-09 | - | 처리됨 | 11-10 | |
| [#101](https://github.com/visualkhh/Lv2_bee_Assignment/issues/101) | [plan][mission_manager] 운영자 CLI에 미션 시작 버튼 추가 | 11-09 | 김현하 | 처리됨 | 11-11 | |
| [#100](https://github.com/visualkhh/Lv2_bee_Assignment/issues/100) | [plan][mission_manager] 실기 값 맞추기 (경기장 좌표·바코드 자세·다가가기 거리·칸 보정) | 11-09 | - | 처리됨 | 11-10 | |
| [#99](https://github.com/visualkhh/Lv2_bee_Assignment/issues/99) | [rl] 먼 거리 퍽: 수평 진입 후 퍽 중심 축으로 손목 올리기 | 11-09 | 전승혜 | 열림 |  | |
| [#98](https://github.com/visualkhh/Lv2_bee_Assignment/issues/98) | [rl] stand_grasp 보정 실기 검증 | 11-09 | 송정혁 | 열림 |  | |
| [#93](https://github.com/visualkhh/Lv2_bee_Assignment/issues/93) | [TODO] 실기 교육장 검증 — OpenCR/cuVSLAM/initialpose/DWB/ONNX/GPU/bag | 11-08 | 김현하 | 처리됨 | 11-10 | |
| [#61](https://github.com/visualkhh/Lv2_bee_Assignment/issues/61) | [rl] 주행control,  nav2 control 강화학습 노드 | 11-04 | 김현하 | 처리됨 | 11-11 | |
| [#60](https://github.com/visualkhh/Lv2_bee_Assignment/issues/60) | [rl] wheel rl drift  onnx 적용 | 11-04 | 김현하 | 처리됨 | 11-06 | |
| [#51](https://github.com/visualkhh/Lv2_bee_Assignment/issues/51) | [control] 그립퍼 최소 전류 2.69mA | 11-02 | 김현하 | 처리됨 | 11-02 | |
| [#42](https://github.com/visualkhh/Lv2_bee_Assignment/issues/42) | [control] 펌웨어: 그리퍼 전류 명령 | 11-02 | 김현하 | 처리됨 | 11-02 | |
| [#28](https://github.com/visualkhh/Lv2_bee_Assignment/issues/28) | [perception] cuVSLAM 검토 | 10-26 | 전승혜 | 열림 |  | |
| [#27](https://github.com/visualkhh/Lv2_bee_Assignment/issues/27) | [perception] 미션 전달 내용 인지 | 10-26 | 전승혜 | 처리됨 | 11-10 | |
| [#26](https://github.com/visualkhh/Lv2_bee_Assignment/issues/26) | [perception] 지도 맵핑 실기기 테스트 | 10-26 | 전승혜 | 열림 |  | |
| [#25](https://github.com/visualkhh/Lv2_bee_Assignment/issues/25) | [perception] yolo 학습 및 테스트 (실제데이터+합성데이터) | 10-26 | 전승혜 | 처리됨 | 11-10 | |
| [#24](https://github.com/visualkhh/Lv2_bee_Assignment/issues/24) | [perception] gripper 점구름은 필터가 없음 | 10-26 | 전승혜 | 처리됨 | 11-10 | |
| [#23](https://github.com/visualkhh/Lv2_bee_Assignment/issues/23) | [perception] 지도 보정 바뀌면 물체 위치/공분산 | 10-26 | 전승혜 | 처리됨 | 11-10 | |
| [#22](https://github.com/visualkhh/Lv2_bee_Assignment/issues/22) | [perception] 트레이에 뭐가 들었는지 알 방법이 없음 | 10-26 | 전승혜 | 처리됨 | 11-10 | |
| [#21](https://github.com/visualkhh/Lv2_bee_Assignment/issues/21) | [perception] tracked_object topic 메시지구조에 맞는 데이터 필요 | 10-26 | 전승혜 | 처리됨 | 11-10 | |
| [#20](https://github.com/visualkhh/Lv2_bee_Assignment/issues/20) | [perception] body_filter가 안 돼서 자기 팔이 장애물로 잡힘 | 10-26 | 전승혜 | 처리됨 | 11-10 | |
| [#14](https://github.com/visualkhh/Lv2_bee_Assignment/issues/14) | 제어: firmware 및 주기 체크 및 보정 | 10-25 | 김현하 | 처리됨 | 10-26 | |
| [#13](https://github.com/visualkhh/Lv2_bee_Assignment/issues/13) | 학습-(보정:매니플레이터,휠), 그립팡계획, 속도limit 학습 | 10-25 | 송정혁 | 열림 |  | |
| [#12](https://github.com/visualkhh/Lv2_bee_Assignment/issues/12) | 플랜-미션에 맞는 플랜처리 방안 | 10-25 | 권형중 | 열림 |  | |
| [#11](https://github.com/visualkhh/Lv2_bee_Assignment/issues/11) | [perception] 카메라에 잡힌 오브젝트외 이떄동안 인지한 오브젝트까지 돌려줘야한다 | 10-25 | 전승혜 | 처리됨 | 11-10 | |
