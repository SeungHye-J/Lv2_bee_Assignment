# 개인 기여 기록

팀원 전원이 쓴다. 작업 목록만 쓰지 말고 **PR·시험·리뷰·설명의 링크와 결과**를 연결한다.

- 팀원별 담당 영역: [루트 README — 담당자](../../README.md#담당자) · [lv2_module6/README.md — 담당자](../../lv2_module6/README.md#담당자)
- PR 운영 규칙(승인·병합·리뷰): [CONTRIBUTING.md](../../CONTRIBUTING.md)
- 참고: 루트 README의 GitHub·Notion·Slack 통계는 CI가 자동 집계한 **업무량 대리 지표**라서, 기여 평가 근거로 쓰지 말고 PR 목록 같은 사실 자료의 위치로만 쓴다.
- 개인 작업 일지·팀 논의·의사결정 기록은 팀 노션에 있다 (README 상단 노션 링크).

## 김현하
- 담당: 제어 (`control`, `manager`, `firmware/`)
- TODO: 그 밖에 대행한 영역 — **만드는 법:** 실제로 맡아 한 통합·주행·계획 쪽 작업이 있으면 적는다 (예: Nav2 파라미터, 주행 오류 원인 분석, bringup 옵션).
- 구현 — 후보 PR (main 기준 `gh pr list --author visualkhh --state merged` 38건 중 의미 있는 것만 추림. 제목·설명은 각 PR 본문에서 옮김):

  | PR | 병합일 | 무엇을 | 규모 |
  | --- | --- | --- | --- |
  | [#44](https://github.com/visualkhh/Lv2_bee_Assignment/pull/44) | 11-02 | 펌웨어 S 43필드(모터별 위치·속도·전류·온도·배터리), C 9번째 칸=그리퍼 쥐는 힘, 부저·모터 재부팅, CLI 전류·온도 표시 | +1033/−307 |
  | [#65](https://github.com/visualkhh/Lv2_bee_Assignment/pull/65) | 11-05 | RL 주행 컨트롤러(`rl_nav_controller`)와 휠 드리프트 보정, scan 모드에서 팔을 stow 자세로(무게중심 이슈) | +3644/−768 |
  | [#82](https://github.com/visualkhh/Lv2_bee_Assignment/pull/82) | 11-07 | RlNavController ONNX 정책 로드·토글·바이패스, ONNX gtest, docker e2e `test-nav-policy` | +4991/−1656 |
  | [#87](https://github.com/visualkhh/Lv2_bee_Assignment/pull/87) | 11-08 | run-cli 터미널 지도 화면·팔 조작 화면, 키 꾹 누를 때 렉 수정 | +2242/−49 |
  | [#92](https://github.com/visualkhh/Lv2_bee_Assignment/pull/92) | 11-08 | 테스트베드 fake SLAM(MAPPING/LOCALIZATION), CLI 구조 정리, ONNX CUDA/CPU 자동 선택·Jetson GPU 빌드 문서 | +3151/−9722 |
  | [#142](https://github.com/visualkhh/Lv2_bee_Assignment/pull/142) ~ [#159](https://github.com/visualkhh/Lv2_bee_Assignment/pull/159) | 11-10~11 | `control-nav2-control-rl` 묶음 — 웨이포인트(`/navigate_through_poses`) 전송 경로 등 | 여러 건 |
  | [#165](https://github.com/visualkhh/Lv2_bee_Assignment/pull/165) | 11-12 | 주행 오류(TF 지연 102·goal checker 100·맴돌기 105) 수정, bringup `use_mission`·`use_perception` 옵션 | +60/−21 |
  | [#171](https://github.com/visualkhh/Lv2_bee_Assignment/pull/171) | 11-12 | 상단 `충돌감시` 칸 토글(현재/적용 확인 후 즉시 적용), 충돌 상태를 state_manager로 이관 | +660/−42 |
  | [#178](https://github.com/visualkhh/Lv2_bee_Assignment/pull/178) · [#181](https://github.com/visualkhh/Lv2_bee_Assignment/pull/181) | 11-12~13 | through-poses 복구를 Spin→BackUp 순서로, 전역 지도 기준 충돌 검사, 도착 허용치 조정 | +240/−43 |
  | [#187](https://github.com/visualkhh/Lv2_bee_Assignment/pull/187) | 11-13 | 테스트베드 `FollowPath.set_fallback` 브리지, cancel 중 죽던 `service_action_bag_input` 수정 | +86/−38 |

  - TODO: 위 후보에서 **3~5개만 골라**, 선택한 PR마다 `변경 효과(전 → 후)`를 한 줄씩 적는다 — **만드는 법:** PR 본문의 "문제/원인" 문장과 시험 결과(`test-results.csv`의 시험 ID 또는 로그 링크)를 근거로 쓴다. 나머지 표 행은 지운다. (#142~#159 묶음은 제목이 같아 PR 본문을 열어 무엇을 했는지 확인하고 쓴다.)
- TODO: 검증 — **만드는 법:** 직접 한 시험마다 `조건 / 기대값 / 실제 결과 / 영상·로그 링크 / 한계와 개선`. [results/test-results.csv](../results/test-results.csv)의 시험 ID를 같이 적는다. 실물과 모의를 구분한다.
- 리뷰 — 후보 PR (`gh search prs --reviewed-by visualkhh`로 검색된 타인 PR 9건, 모두 병합됨): [#156](https://github.com/visualkhh/Lv2_bee_Assignment/pull/156) · [#71](https://github.com/visualkhh/Lv2_bee_Assignment/pull/71) · [#70](https://github.com/visualkhh/Lv2_bee_Assignment/pull/70) · [#64](https://github.com/visualkhh/Lv2_bee_Assignment/pull/64) · [#9](https://github.com/visualkhh/Lv2_bee_Assignment/pull/9) · [#8](https://github.com/visualkhh/Lv2_bee_Assignment/pull/8) · [#7](https://github.com/visualkhh/Lv2_bee_Assignment/pull/7) · [#6](https://github.com/visualkhh/Lv2_bee_Assignment/pull/6)
  - TODO: 위 중 **의미 있는 리뷰 1~3건**을 골라 `PR 링크 / 내가 확인한 문제(코드 위치·조건) / 수정 반영 여부와 후속 확인`을 적는다 — **만드는 법:** 각 PR의 Files changed·리뷰 댓글(내가 남긴 것)을 열어 구체적 지적이 있는 것만 쓴다. 승인만 누른 리뷰는 근거가 약하다. 이 검색은 **리뷰 기능으로 남긴 것**만 잡으므로, 댓글로만 의견을 남긴 PR이 있으면 직접 찾아 추가한다.
- TODO: 설명 — **만드는 법:** 신호 읽기부터 복귀까지 전체 흐름을 내 말로 5~10줄, 그리고 내가 맡지 않은 부분(인지·계획·학습 중 최소 1곳)을 이해한 근거(그 코드·설정·시험 결과를 읽고 설명한 내용 또는 관련 PR 리뷰)를 쓴다.

## 송정혁
- 담당: 설계, 학습 (`ml`) — 실제로는 시뮬레이션·URDF·실기 시험·주행까지 폭넓게 작업
- GitHub: `cowsjh` — 아래 후보는 `gh pr list --author cowsjh --state merged`와 `gh search prs --reviewed-by cowsjh`로 자동으로 뽑은 것이라 **본인이 확인하고 고쳐 쓴다.**
- 구현 — 후보 PR (병합 34건 중 주제별 13건):

  | PR | 병합일 | 무엇을 (제목 기준) | 규모 |
  | --- | --- | --- | --- |
  | [#1](https://github.com/visualkhh/Lv2_bee_Assignment/pull/1) | 10-15 | 시뮬레이션 초기 구성 (Feature/simulate) | +18177 |
  | [#6](https://github.com/visualkhh/Lv2_bee_Assignment/pull/6) | 10-21 | beebot_v00 URDF·USD 초기 셋업 | +1743/−156 |
  | [#46](https://github.com/visualkhh/Lv2_bee_Assignment/pull/46) | 11-02 | Isaac Sim 최종 URDF·USDA 셋업 | +13551/−464 |
  | [#75](https://github.com/visualkhh/Lv2_bee_Assignment/pull/75) | 11-06 | Isaac 재구성 · 시뮬 집기 시연 · 팔 처짐 보정(twist_state) · calib 도구 | +7363/−5399 |
  | [#83](https://github.com/visualkhh/Lv2_bee_Assignment/pull/83) | 11-09 | load 뒤 바로 다음 pick (기억 좌표 · carry 베지어) + 정책 입력 확장 (#96 동일 제목) | +854/−204 |
  | [#139](https://github.com/visualkhh/Lv2_bee_Assignment/pull/139) | 11-10 | Isaac 팔 시험 환경 정리(loops·rl·standalone) + 먼 퍽 도달성 분석 | +6381/−3789 |
  | [#161](https://github.com/visualkhh/Lv2_bee_Assignment/pull/161) | 11-12 | v02 매니퓰레이터 URDF·팔 장착·영점·손목 카메라 실측 반영 (#172 USD 브래킷) | +236/−227 |
  | [#203](https://github.com/visualkhh/Lv2_bee_Assignment/pull/203) | 11-13 | 인지 기반 상자 도킹 · 상자별 접근 면 · exit 후진 | +455/−59 |
  | [#223](https://github.com/visualkhh/Lv2_bee_Assignment/pull/223) | 11-14 | 실기 pick·stand·load 반복 시험 수정 | +15576/−3638 |
  | [#225](https://github.com/visualkhh/Lv2_bee_Assignment/pull/225) | 11-14 | 코너 회전 뒤 위치 틀어짐(cuVSLAM rig 수평) · 도착 뒤 방향 회전 | +122/−3 |
  | [#260](https://github.com/visualkhh/Lv2_bee_Assignment/pull/260) | 11-15 | 사람이 찍은 경유점 지도 · 끼임 풀기 · 문 경유점 보완 · EKF 바퀴 뺌 | +426/−41 |
  | [#272](https://github.com/visualkhh/Lv2_bee_Assignment/pull/272) | 11-15 | cuVSLAM C++/GPU · Python 노드 C++판 · 노드 컨테이너 묶기 | +3894/−251 |
  | [#286](https://github.com/visualkhh/Lv2_bee_Assignment/pull/286) | 11-16 | 끼임 풀기 벽 반대쪽 · 적재 실패 되돌리기 · RETURN 재시도 | +110/−18 |

  - TODO: **3~5개만 골라** 선택한 PR마다 `변경 효과(전 → 후)`를 한 줄씩 적고 나머지 행은 지운다 — **만드는 법:** PR 본문의 "문제/원인" 문장과 시험 결과(`results/test-results.csv`의 시험 ID 또는 로그 링크)를 근거로 쓴다. 본인이 한 것이 맞는지(담당 영역·PR 작성자) 확인한다.
- TODO: 검증 — **본인이 쓴다.** 직접 한 시험마다 `조건 / 기대값 / 실제 결과 / 영상·로그 링크 / 한계와 개선` ([results/test-results.csv](../results/test-results.csv)의 시험 ID와 같이). 실물과 모의를 구분한다.
- 리뷰 — 후보 PR (타인 PR, 모두 병합됨): [#43](https://github.com/visualkhh/Lv2_bee_Assignment/pull/43) · [#45](https://github.com/visualkhh/Lv2_bee_Assignment/pull/45) · [#48](https://github.com/visualkhh/Lv2_bee_Assignment/pull/48) · [#50](https://github.com/visualkhh/Lv2_bee_Assignment/pull/50) · [#59](https://github.com/visualkhh/Lv2_bee_Assignment/pull/59) · [#62](https://github.com/visualkhh/Lv2_bee_Assignment/pull/62) · [#63](https://github.com/visualkhh/Lv2_bee_Assignment/pull/63) · [#78](https://github.com/visualkhh/Lv2_bee_Assignment/pull/78)
  - TODO: 의미 있는 리뷰 1~3건을 골라 `PR 링크 / 내가 확인한 문제(코드 위치·조건) / 수정 반영 여부와 후속 확인`을 적는다 — **만드는 법:** 각 PR의 리뷰 댓글(내가 남긴 것)을 열어 구체적 지적이 있는 것만 쓴다. 승인만 누른 리뷰는 근거가 약하다. 이 검색은 리뷰 기능으로 남긴 것만 잡는다.
- TODO: 설명 — **본인이 쓴다.** 신호 읽기부터 복귀까지 전체 흐름을 내 말로 5~10줄, 그리고 내가 맡지 않은 부분을 이해한 근거(그 코드·설정·시험 결과를 읽고 설명한 내용 또는 관련 PR 리뷰).

## 권형중
- 담당: 통합 (`plan`, `ros_system`, `bringup`, `faker`)
- GitHub: `JuneKunst` — 아래 후보는 `gh pr list --author JuneKunst --state merged`와 `gh search prs --reviewed-by JuneKunst`로 자동으로 뽑은 것이라 **본인이 확인하고 고쳐 쓴다.**
- 구현 — 후보 PR (병합 90건 중 주제별 16건):

  | PR | 병합일 | 무엇을 (제목 기준) | 규모 |
  | --- | --- | --- | --- |
  | [#43](https://github.com/visualkhh/Lv2_bee_Assignment/pull/43) | 11-02 | SLAM을 cuVSLAM(위치) + nvblox(지도)로 교체 | +751/−499 |
  | [#48](https://github.com/visualkhh/Lv2_bee_Assignment/pull/48) | 11-02 | coordinator 팔 skill — pose·load·unload(적재함 12칸)·잡기 판정·처짐 보정 | +1836/−104 |
  | [#49](https://github.com/visualkhh/Lv2_bee_Assignment/pull/49) | 11-04 | 주사위 미션 흐름 — mission_manager · mission_solver | +3740/−566 |
  | [#55](https://github.com/visualkhh/Lv2_bee_Assignment/pull/55) | 11-03 | 학습 정책 배포 — 보정 컨트롤러·도킹 시뮬 기준 동작·plan_next | +1668/−56 |
  | [#59](https://github.com/visualkhh/Lv2_bee_Assignment/pull/59) | 11-04 | skill 노드 구조 + Isaac 학습 환경 + isaac-lab 통합 | +12428/−1940 |
  | [#62](https://github.com/visualkhh/Lv2_bee_Assignment/pull/62) | 11-04 | Isaac 디지털 트윈 미션 시뮬 — 무작위 출제·바코드·출발 자리 | +786/−10 |
  | [#141](https://github.com/visualkhh/Lv2_bee_Assignment/pull/141) | 11-11 | 경기장 벽을 처음부터 알기 · 벽에 막힌 다가설 면 빼기 | +122/−2 |
  | [#145](https://github.com/visualkhh/Lv2_bee_Assignment/pull/145) | 11-11 | navigate 진척 감시 — 남은 거리가 25초 동안 안 줄면 nav_no_progress | +73/−1 |
  | [#146](https://github.com/visualkhh/Lv2_bee_Assignment/pull/146) | 11-12 | 박스·장소 방문 순서 파라미터 — 도면 벽 A*로 정한 최단 순서 (−21 %) | +100/−5 |
  | [#173](https://github.com/visualkhh/Lv2_bee_Assignment/pull/173) | 11-12 | field-test.py — 실기 단계별 시험 runner (rosbag + 이벤트 기록) | +816/−0 |
  | [#195](https://github.com/visualkhh/Lv2_bee_Assignment/pull/195) | 11-13 | mission 계획을 ExecutePlan action으로 실행 | +635/−87 |
  | [#202](https://github.com/visualkhh/Lv2_bee_Assignment/pull/202) | 11-13 | 현장 진단 시험 모드 — 단계 시간 기록 · 실패해도 계속(DIAG) | +817/−33 |
  | [#217](https://github.com/visualkhh/Lv2_bee_Assignment/pull/217) | 11-13 | low-memory integration — static 2D map backend 등 | +1062/−84 |
  | [#227](https://github.com/visualkhh/Lv2_bee_Assignment/pull/227) | 11-15 | AUTO에서 충돌 감시 유지 · 경기장 보정 토픽·벽 맞춤 · Nav2 수정 | +1023/−77 |
  | [#256](https://github.com/visualkhh/Lv2_bee_Assignment/pull/256) | 11-15 | 가르친 경로 (teach & repeat) — 수동 경로를 AUTO 경유점으로 | +272/−1 |
  | [#273](https://github.com/visualkhh/Lv2_bee_Assignment/pull/273) | 11-15 | 답지 재생 · route_tables · 몸 모양 collision_monitor | +839/−99 |

  - TODO: **3~5개만 골라** 선택한 PR마다 `변경 효과(전 → 후)`를 한 줄씩 적고 나머지 행은 지운다 — **만드는 법:** PR 본문의 "문제/원인" 문장과 시험 결과(`results/test-results.csv`의 시험 ID 또는 로그 링크)를 근거로 쓴다. 본인이 한 것이 맞는지(담당 영역·PR 작성자) 확인한다.
- TODO: 검증 — **본인이 쓴다.** 직접 한 시험마다 `조건 / 기대값 / 실제 결과 / 영상·로그 링크 / 한계와 개선` ([results/test-results.csv](../results/test-results.csv)의 시험 ID와 같이). 실물과 모의를 구분한다.
- 리뷰 — 후보 PR (타인 PR, 모두 병합됨): [#3](https://github.com/visualkhh/Lv2_bee_Assignment/pull/3) · [#45](https://github.com/visualkhh/Lv2_bee_Assignment/pull/45) · [#46](https://github.com/visualkhh/Lv2_bee_Assignment/pull/46) · [#75](https://github.com/visualkhh/Lv2_bee_Assignment/pull/75) · [#83](https://github.com/visualkhh/Lv2_bee_Assignment/pull/83) · [#95](https://github.com/visualkhh/Lv2_bee_Assignment/pull/95) · [#192](https://github.com/visualkhh/Lv2_bee_Assignment/pull/192) · [#193](https://github.com/visualkhh/Lv2_bee_Assignment/pull/193) · [#246](https://github.com/visualkhh/Lv2_bee_Assignment/pull/246)
  - TODO: 의미 있는 리뷰 1~3건을 골라 `PR 링크 / 내가 확인한 문제(코드 위치·조건) / 수정 반영 여부와 후속 확인`을 적는다 — **만드는 법:** 각 PR의 리뷰 댓글(내가 남긴 것)을 열어 구체적 지적이 있는 것만 쓴다. 승인만 누른 리뷰는 근거가 약하다. 이 검색은 리뷰 기능으로 남긴 것만 잡는다.
- TODO: 설명 — **본인이 쓴다.** 신호 읽기부터 복귀까지 전체 흐름을 내 말로 5~10줄, 그리고 내가 맡지 않은 부분을 이해한 근거(그 코드·설정·시험 결과를 읽고 설명한 내용 또는 관련 PR 리뷰).
- 참고: 병합 PR이 90건으로 많아 위 16건은 제목 기준으로 추린 것이다. 같은 GitHub 계정에 현장 진단·튜닝 PR이 많으니, 그중 **본인이 설명할 수 있는 것**만 고른다.

## 전승혜
- 담당: 인지 (`perception`)
- GitHub: `SeungHye-J` — 아래 후보는 `gh pr list --author SeungHye-J --state merged`와 `gh search prs --reviewed-by SeungHye-J`로 자동으로 뽑은 것이라 **본인이 확인하고 고쳐 쓴다.**
- 구현 — 후보 PR (병합 12건 전부):

  | PR | 병합일 | 무엇을 (제목 기준) | 규모 |
  | --- | --- | --- | --- |
  | [#9](https://github.com/visualkhh/Lv2_bee_Assignment/pull/9) | 10-22 | 퍽 기억·퍽 판단 및 새 YOLO 모델 추가 | +522/−29 |
  | [#45](https://github.com/visualkhh/Lv2_bee_Assignment/pull/45) | 11-02 | 신호등 12회 점멸 해석, 퍽 인지 개선, 상자 인지 | +825/−178 |
  | [#95](https://github.com/visualkhh/Lv2_bee_Assignment/pull/95) | 11-09 | 퍽 자세를 윗면 높이로 판단 | +81/−12 |
  | [#144](https://github.com/visualkhh/Lv2_bee_Assignment/pull/144) | 11-11 | 퍽 모델 v5 · 긴 기억 삭제 · API.md 수정 | +1294/−581 |
  | [#152](https://github.com/visualkhh/Lv2_bee_Assignment/pull/152) | 11-12 | MoveIt octomap 센서 연결 (pole_front_cam 점구름) | +78/−7 |
  | [#164](https://github.com/visualkhh/Lv2_bee_Assignment/pull/164) | 11-12 | 인지 노드 C++ 전환 · 젯슨 CPU 줄이기 | +3212/−142 |
  | [#180](https://github.com/visualkhh/Lv2_bee_Assignment/pull/180) | 11-12 | 퍽+상자 YOLO v6 · 상자 도킹 정보 · 인지 CPU 옵션 | +305/−31 |
  | [#192](https://github.com/visualkhh/Lv2_bee_Assignment/pull/192) | 11-13 | pick_far 먼 퍽 집기 | +757/−13 |
  | [#193](https://github.com/visualkhh/Lv2_bee_Assignment/pull/193) | 11-13 | 신호등 하나로 추적 · 켜진 신호등 우선 | +666/−96 |
  | [#243](https://github.com/visualkhh/Lv2_bee_Assignment/pull/243) | 11-15 | 충돌 감시 구역 — 문틈에서 감속·정지 줄임 | +12/−6 |
  | [#246](https://github.com/visualkhh/Lv2_bee_Assignment/pull/246) | 11-15 | YOLO TensorRT FP16 추론 추가 | +243/−23 |
  | [#284](https://github.com/visualkhh/Lv2_bee_Assignment/pull/284) | 11-16 | 조사 때 상자 윗면 안의 태그 퍽을 모두 등록 | +38/−11 |

  - TODO: **3~5개만 골라** 선택한 PR마다 `변경 효과(전 → 후)`를 한 줄씩 적고 나머지 행은 지운다 — **만드는 법:** PR 본문의 "문제/원인" 문장과 시험 결과(`results/test-results.csv`의 시험 ID 또는 로그 링크)를 근거로 쓴다. 본인이 한 것이 맞는지(담당 영역·PR 작성자) 확인한다.
- TODO: 검증 — **본인이 쓴다.** 직접 한 시험마다 `조건 / 기대값 / 실제 결과 / 영상·로그 링크 / 한계와 개선` ([results/test-results.csv](../results/test-results.csv)의 시험 ID와 같이). 실물과 모의를 구분한다.
- 리뷰 — 후보 PR (타인 PR, 모두 병합됨): [#47](https://github.com/visualkhh/Lv2_bee_Assignment/pull/47)
  - TODO: 의미 있는 리뷰 1~3건을 골라 `PR 링크 / 내가 확인한 문제(코드 위치·조건) / 수정 반영 여부와 후속 확인`을 적는다 — **만드는 법:** 각 PR의 리뷰 댓글(내가 남긴 것)을 열어 구체적 지적이 있는 것만 쓴다. 승인만 누른 리뷰는 근거가 약하다. 이 검색은 리뷰 기능으로 남긴 것만 잡는다.
- TODO: 설명 — **본인이 쓴다.** 신호 읽기부터 복귀까지 전체 흐름을 내 말로 5~10줄, 그리고 내가 맡지 않은 부분을 이해한 근거(그 코드·설정·시험 결과를 읽고 설명한 내용 또는 관련 PR 리뷰).
