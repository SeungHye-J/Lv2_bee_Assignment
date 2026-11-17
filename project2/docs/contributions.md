# 개인 기여 기록

팀원 전원이 쓴다. 작업 목록만 쓰지 말고 **PR·시험·리뷰·설명의 링크와 결과**를 연결한다.

- 팀원별 담당 영역: [루트 README — 담당자](../../README.md#담당자) · [lv2_module6/README.md — 담당자](../../lv2_module6/README.md#담당자)
- PR 운영 규칙(승인·병합·리뷰): [CONTRIBUTING.md](../../CONTRIBUTING.md)
- 참고: 루트 README의 GitHub·Notion·Slack 통계는 CI가 자동 집계한 **업무량 대리 지표**라서, 기여 평가 근거로 쓰지 말고 PR 목록 같은 사실 자료의 위치로만 쓴다.
- 개인 작업 일지·팀 논의·의사결정 기록은 팀 노션에 있다 (README 상단 노션 링크).
- **구현 목록 검증 기준 (2026-11-17)**: PR 작성자뿐 아니라 **커밋 작성자**를 `gh pr view <번호> --json commits`로 확인했다. PR만 열고 커밋이 전부 타인인 경우 구현 목록에서 빼고 `제외`로 사유를 적는다.

## 김현하
- 담당: 제어 (`control`, `manager`, `firmware/`)
- 그 밖에 대행한 영역: 통합·주행·계획 쪽 — Nav2 파라미터, 주행 오류 원인 분석, bringup 옵션(`use_mission`·`use_perception`·`use_mission_manager`), 미션 문서 정리. 근거 PR: [#165](https://github.com/visualkhh/Lv2_bee_Assignment/pull/165)
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

  - 변경 효과 (PR 본문 수치 · 시험 ID는 [results/test-results.csv](../results/test-results.csv)):
    - [#165](https://github.com/visualkhh/Lv2_bee_Assignment/pull/165): 주행 오류 3종 — 수정 전 3초 만에 취소·시작 즉시 실패·도착점 근처 맴돌다 105 → 수정 후 다중 웨이포인트 2회 도착, 단일 목표 11초 도착 (실물, T-17)
    - [#178](https://github.com/visualkhh/Lv2_bee_Assignment/pull/178)·[#181](https://github.com/visualkhh/Lv2_bee_Assignment/pull/181): through-poses 복구·도착 허용치 조정 후 단일 NavigateToPose 도착 (모의, T-15)
    - [#187](https://github.com/visualkhh/Lv2_bee_Assignment/pull/187): 테스트베드 `FollowPath.set_fallback` 브리지로 DWB 단일 goal 도착, recoveries 0 (모의, T-16)
    - [#44](https://github.com/visualkhh/Lv2_bee_Assignment/pull/44): 펌웨어 S 43필드로 모터별 위치·속도·전류·온도·배터리를 CLI에서 확인 가능 (수치 비교 기록 없음)
    - 위 표의 나머지 행은 후보로 남긴 것이며 효과 수치를 확인하지 않았다.
- 검증: 시험 기록은 [results/test-results.csv](../results/test-results.csv)의 T-15 · T-16 · T-17. 실물·모의 구분과 근거(PR 본문, 원본 로그 없음)는 그 표에 있다. 본인이 직접 수행한 시험인지는 본인 확인이 필요하다.
- 리뷰 — 후보 PR (`gh search prs --reviewed-by visualkhh`로 검색된 타인 PR; 문서 작성 시 9건, 2026-11-17 현재 23건. 아래는 그중 예시 8건, 모두 병합됨): [#156](https://github.com/visualkhh/Lv2_bee_Assignment/pull/156) · [#71](https://github.com/visualkhh/Lv2_bee_Assignment/pull/71) · [#70](https://github.com/visualkhh/Lv2_bee_Assignment/pull/70) · [#64](https://github.com/visualkhh/Lv2_bee_Assignment/pull/64) · [#9](https://github.com/visualkhh/Lv2_bee_Assignment/pull/9) · [#8](https://github.com/visualkhh/Lv2_bee_Assignment/pull/8) · [#7](https://github.com/visualkhh/Lv2_bee_Assignment/pull/7) · [#6](https://github.com/visualkhh/Lv2_bee_Assignment/pull/6)
  - 리뷰 내용: 8건 모두 APPROVED. 코멘트는 #8 "gooooood", #71·#70·#6 "good", #7은 승인 2회, 나머지는 본문 없음 — 구체적인 지적 기록은 없다. 근거로 내세울 리뷰는 없다.
- 설명: 신호 읽기부터 복귀까지: 출발 구역에서 바코드(Code 128)와 신호등 12회 점멸을 읽어 색 순서와 R·Y·G 횟수를 기억하고(`BARCODE`·`READ`), 바코드가 오름/내림차순을 정한다. 시작 박스 6곳을 돌며 퍽을 확인해 적재함에 싣고 12개를 L1~L3 그룹에 한 번에 할당한다(`SURVEY`). 퍽마다 장소 앞으로 가서 주사위 칸에 놓고(`BUILD`), 새 관측으로 칸별 판정과 중복·누락을 재검증한 뒤(`VERIFY`) 출발 구역으로 돌아와 3초 정지한다(`RETURN`). 관측 오류·복귀 실패는 `ERROR`로 안전 정지하고 퍽 단위 실패는 기록하고 이어간다 ([architecture.md](architecture.md), `mission_solver.cpp`의 `next_phase` 기준). 맡지 않은 계획 쪽: 주행 오류를 고치던 중 `mission_solver.cpp`의 누락된 `}`를 수정했고(커밋 `f0b993c`) `test_mission_solver.cpp`로 상태 전이를 확인했다. 단계별 완료 조건과 실패 이유는 plan.yaml 값과 함께 [architecture.md](architecture.md)에 정리했다.

## 송정혁
- 담당: 설계, 학습 (`ml`) — 실제로는 시뮬레이션·URDF·실기 시험·주행까지 폭넓게 작업
- GitHub: `cowsjh` — 아래 후보는 `gh pr list --author cowsjh --state merged`와 `gh search prs --reviewed-by cowsjh`로 자동으로 뽑은 것이라 **본인이 확인하고 고쳐 쓴다.**
- 구현 — 후보 PR (병합 34건 중 **본인 커밋이 있는 12건**):

  | PR | 병합일 | 무엇을 (제목 기준) | 규모 |
  | --- | --- | --- | --- |
  | [#1](https://github.com/visualkhh/Lv2_bee_Assignment/pull/1) | 10-15 | 시뮬레이션 초기 구성 중 URDF 최신화 | +2312/−289 |
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
  | [#286](https://github.com/visualkhh/Lv2_bee_Assignment/pull/286) | 11-16 | 끼임 풀기 벽 반대쪽 · 적재 실패 되돌리기 · RETURN 재시도 | +110/−18 |

  - **제외**: [#272](https://github.com/visualkhh/Lv2_bee_Assignment/pull/272) — 커밋 8개가 전부 JuneKunst·claude(본인 0). PR만 열고 병합한 건이라 구현 목록에서 제외. (PR 본문: "11-16 실기에서 한 작업, develop 위에 커밋 8개")
  - **규모**: [#1](https://github.com/visualkhh/Lv2_bee_Assignment/pull/1)은 PR 전체 +18177줄이지만 커밋 3개 중 2개(초기 구조 +13320/−3, 시뮬레이터 수정 +3105/−271)가 visualkhh 것이라, 위 표에는 본인 커밋(URDF 최신화 +2312/−289)만 적었다.

  - 변경 효과 (PR 본문 수치 · 시험 ID는 [results/test-results.csv](../results/test-results.csv)):
    - [#225](https://github.com/visualkhh/Lv2_bee_Assignment/pull/225): 코너 회전 뒤 위치 정확도 — 기존 3/6 성공(오차 114~976 mm) → `BEE_CUVSLAM_LEVEL=1` 4/6 성공(성공 시 5~15 mm) (모의, T-13)
    - [#260](https://github.com/visualkhh/Lv2_bee_Assignment/pull/260): 사람이 찍은 경유점 지도로 4·5·3번 상자 도킹 3/3 성공, 앞뒤 ≤ 8.7 mm·옆 ≤ 24.7 mm·yaw ≤ 1.95° (모의, T-11)
    - [#223](https://github.com/visualkhh/Lv2_bee_Assignment/pull/223) 전후 실기 팔 반복 시험: pick → load 618회, pick 성공 11-08 58 % → 11-14 94 %, 퍽 1개 주기 20.7 s → 12~14 s (실물, T-28). 누운 퍽 세우기 15~47 %, coordinator 경유 상자 비우기 선 퍽 68 % · 누운 퍽 19 % (실물, T-30 · T-32)
    - 잔차 RL: 실기 pick 정책 켬으로 손 안 어긋남 6.0 → 2.2 mm (실물, T-29). Isaac Lab 평가 stand_grasp v1 61.6 → 83.4 %, 실기 배포한 pick v02 100 % · load v04 99.9 % (모의, T-35)
    - 요약·도표: [results/arm-rl-tests/](../results/arm-rl-tests/README.md)
    - 위 표의 나머지 행은 후보로 남긴 것이며 효과 수치를 확인하지 않았다.
- 검증: 시험 기록은 [results/test-results.csv](../results/test-results.csv)의 T-11 · T-13 · T-28 ~ T-35 (T-28 ~ 35는 본인이 직접 돌린 반복 시험 로그에서 집계). 실물·모의 구분과 근거(PR 본문, 원본 로그 없음)는 그 표에 있다. 본인이 직접 수행한 시험인지는 본인 확인이 필요하다.
- 리뷰 — 후보 PR (타인 PR, 모두 병합됨): [#43](https://github.com/visualkhh/Lv2_bee_Assignment/pull/43) · [#45](https://github.com/visualkhh/Lv2_bee_Assignment/pull/45) · [#48](https://github.com/visualkhh/Lv2_bee_Assignment/pull/48) · [#50](https://github.com/visualkhh/Lv2_bee_Assignment/pull/50) · [#59](https://github.com/visualkhh/Lv2_bee_Assignment/pull/59) · [#62](https://github.com/visualkhh/Lv2_bee_Assignment/pull/62) · [#63](https://github.com/visualkhh/Lv2_bee_Assignment/pull/63) · [#78](https://github.com/visualkhh/Lv2_bee_Assignment/pull/78)
  - 리뷰 내용: [#48](https://github.com/visualkhh/Lv2_bee_Assignment/pull/48)(coordinator 팔 skill)에 skill 이름을 참고해 이후 정책 이름도 통일하겠다고 남겼다. #45·#50·#59·#62·#63·#78 리뷰는 확인·승인 코멘트라 구체적 지적은 없다.
- 설명: 신호 읽기부터 복귀까지: 출발 구역에서 바코드(Code 128)와 신호등 12회 점멸을 읽어 색 순서와 R·Y·G 횟수를 기억하고(`BARCODE`·`READ`), 바코드가 오름/내림차순을 정한다. 시작 박스 6곳을 돌며 퍽을 확인해 적재함에 싣고 12개를 L1~L3 그룹에 한 번에 할당한다(`SURVEY`). 퍽마다 장소 앞으로 가서 주사위 칸에 놓고(`BUILD`), 새 관측으로 칸별 판정과 중복·누락을 재검증한 뒤(`VERIFY`) 출발 구역으로 돌아와 3초 정지한다(`RETURN`). 관측 오류·복귀 실패는 `ERROR`로 안전 정지하고 퍽 단위 실패는 기록하고 이어간다 ([architecture.md](architecture.md), `mission_solver.cpp`의 `next_phase` 기준). 맡지 않은 인지·계획 쪽: 신호등 해석·퍽 인지 [#45](https://github.com/visualkhh/Lv2_bee_Assignment/pull/45)와 Isaac 미션 시뮬(무작위 출제·바코드) [#62](https://github.com/visualkhh/Lv2_bee_Assignment/pull/62)를 리뷰했고, 코너 회전 뒤 위치 오차는 cuVSLAM 설정(`BEE_CUVSLAM_LEVEL`)을 다룬 [#225](https://github.com/visualkhh/Lv2_bee_Assignment/pull/225)에서 직접 다뤘다.

## 권형중
- 담당: 통합 (`plan`, `ros_system`, `bringup`, `tools/cli`, `faker`)
- GitHub: `JuneKunst` (`hjkwon9401@gmail.com`)
- 기록 기준: Git 작성자와 원격에 push된 커밋을 근거로 정리했다. PR merge 행위와 merge commit은 기여 내용에서 제외했다. 아래 PR은 관련 구현 맥락을 확인하는 링크이며, 병합 건수나 diff 크기를 성과 지표로 쓰지 않는다.

### 주요 작업

1. **미션 계획·실행 흐름 통합**
   - `mission_manager`·`mission_solver`에 신호·바코드 관측부터 전체 퍽 할당, SURVEY·BUILD·VERIFY·RETURN까지 상태 전이를 구현했다. coordinator가 계획을 실행하도록 `ExecutePlan` Action을 연결하고, 진단 모드·단계별 시간 기록·상자 이동 시험도 추가했다.
   - 근거: [주사위 미션 흐름 #49](https://github.com/visualkhh/Lv2_bee_Assignment/pull/49) · [ExecutePlan 연결 #195](https://github.com/visualkhh/Lv2_bee_Assignment/pull/195) · [실기 진단 모드 #202](https://github.com/visualkhh/Lv2_bee_Assignment/pull/202) · [상자 이동 시험 #209](https://github.com/visualkhh/Lv2_bee_Assignment/pull/209) · [그 자리 재검증 #257](https://github.com/visualkhh/Lv2_bee_Assignment/pull/257)

2. **SLAM·지도·경기장 좌표 통합**
   - 위치 추정과 지도 생성을 cuVSLAM·nvblox로 연결하고, 경기장 좌표와 현재 map 위치 간 보정, 벽·금지 구역 및 인지된 상자 중심 기준 접근을 정리했다. 11월 16일에는 박스 보정·벽 칸 유지 조건을 반영했다.
   - 근거: [초기 Nav2 설정 #2](https://github.com/visualkhh/Lv2_bee_Assignment/pull/2) · [cuVSLAM·nvblox 전환 #43](https://github.com/visualkhh/Lv2_bee_Assignment/pull/43) · [벽을 고려한 접근 #141](https://github.com/visualkhh/Lv2_bee_Assignment/pull/141) · [map 위치 보정 #239](https://github.com/visualkhh/Lv2_bee_Assignment/pull/239) · [금지 구역·보정 분리 #248](https://github.com/visualkhh/Lv2_bee_Assignment/pull/248) · [벽 정합 검증 기본 설정 #277](https://github.com/visualkhh/Lv2_bee_Assignment/pull/277) · [박스 보정/벽 유지](https://github.com/visualkhh/Lv2_bee_Assignment/commit/d596253b) · [인지 상자 중심 접근](https://github.com/visualkhh/Lv2_bee_Assignment/commit/d6011de2)

3. **Nav2 주행 안정성·경로 복구**
   - 진척 없는 목표를 감지하고, 상자·장소 방문 순서를 계산하며, 로봇 몸체를 반영한 충돌 감시·후진 복구와 경유 경로 녹화·재생을 연결했다.
   - 근거: [주행 진척 감시 #145](https://github.com/visualkhh/Lv2_bee_Assignment/pull/145) · [방문 순서 최적화 #146](https://github.com/visualkhh/Lv2_bee_Assignment/pull/146) · [Nav2 복구 개선 #234](https://github.com/visualkhh/Lv2_bee_Assignment/pull/234) · [가르친 경로 #256](https://github.com/visualkhh/Lv2_bee_Assignment/pull/256) · [경로 재생·몸체 충돌 감시 #273](https://github.com/visualkhh/Lv2_bee_Assignment/pull/273) · [현장 설정 #276](https://github.com/visualkhh/Lv2_bee_Assignment/pull/276) · [11월 16일 Nav2 현장 수정](https://github.com/visualkhh/Lv2_bee_Assignment/commit/ad76b113)

4. **현장 시험·운영 화면·진단**
   - rosbag과 단계별 이벤트를 남기는 `field-test.py`, TUI의 미션·인지·Nav2 상태 표시, 단계 실패를 계속 기록하는 DIAG 실행 및 상자로 이동하는 경로 녹화를 마련했다.
   - 근거: [단계별 실기 시험 도구 #173](https://github.com/visualkhh/Lv2_bee_Assignment/pull/173) · [진단 모드 #202](https://github.com/visualkhh/Lv2_bee_Assignment/pull/202) · [TUI 미션 상태 #249](https://github.com/visualkhh/Lv2_bee_Assignment/pull/249) · [경로 상태 설명 #250](https://github.com/visualkhh/Lv2_bee_Assignment/pull/250) · [경로 녹화 #256](https://github.com/visualkhh/Lv2_bee_Assignment/pull/256) · [실기 시작 안내 #281](https://github.com/visualkhh/Lv2_bee_Assignment/pull/281)

5. **Jetson 메모리·실행 부하 개선**
   - 저메모리 실행 구성을 만들고 보조 노드·cuVSLAM 등 Python 런타임을 C++·컴포넌트로 전환했다. Nav2 주행 중 인지 주기, EKF 주기, cuVSLAM 기록량을 줄이고 GPU 기반 점 판정·YOLO 실행도 연결했다.
   - 근거: [저메모리 통합 #217](https://github.com/visualkhh/Lv2_bee_Assignment/pull/217) · [노드별 부하 표시 #220](https://github.com/visualkhh/Lv2_bee_Assignment/pull/220) · [EKF 주기 조정 #242](https://github.com/visualkhh/Lv2_bee_Assignment/pull/242) · [주행 중 인지 조절 #244](https://github.com/visualkhh/Lv2_bee_Assignment/pull/244) · [cuVSLAM 메모리 조정 #247](https://github.com/visualkhh/Lv2_bee_Assignment/pull/247) · [주행 중 인지 2 Hz #278](https://github.com/visualkhh/Lv2_bee_Assignment/pull/278) · [C++ 보조 노드](https://github.com/visualkhh/Lv2_bee_Assignment/commit/3d946600) · [C++ cuVSLAM/GPU 입력](https://github.com/visualkhh/Lv2_bee_Assignment/commit/0ef31eb2) · [GPU 점 판정·YOLO CUDA](https://github.com/visualkhh/Lv2_bee_Assignment/commit/9fd88a1b)

### 확인된 결과와 한계

- 시험 기록 T-14에는 모의 주행에서 진척 없음 상태를 약 26초에 감지한 결과가 있다. T-25의 33.6 m → 26.6 m(-21%)는 벽을 고려해 계산한 경로 길이이며 실측 주행 거리가 아니다.
- T-07·T-08의 실기 기록은 `barcode_missing` 7회와 `approach_blocked` 11회이며, 해당 미션이 SURVEY 이후로 진행되지 못했다. 이는 진단 도구가 남긴 팀 시험 기록이지, 이 문서만으로 개인의 시험 수행자를 확정하지 않는다.
- 공식 1·2회차 결과와 전체 실기 미션 완주 기록은 저장소에 없다. 원본 로그·영상이 없는 PR 기반 시험 수치는 출처의 한계를 유지한다. 세부 근거: [시험 결과](../results/test-results.csv) · [프로젝트 현황](../README.md#5-시험-결과).

### 구현 이력: 작성·push 날짜별

아래는 GitHub에 작성자 `hjkwon9401@gmail.com`으로 기록된 구현 커밋을 날짜별로 묶은 작업 내용이다. 각 날짜에는 해당 날짜의 코드·설정·문서 변경이 원격 저장소에 push됐다. 같은 날의 연관 작업은 분야별로 묶었다. PR 링크는 구현 맥락과 변경을 확인할 수 있는 대표 자료다.

| 날짜 | 분야 | 실제 구현·정리한 내용 | 대표 근거 |
| --- | --- | --- | --- |
| 2026-10-16 | Nav2 설정 | 프로젝트 초기 Nav2 서버·컨트롤러 파라미터 파일을 작성해 주행 설정의 기준을 마련했다. | [PR #2](https://github.com/visualkhh/Lv2_bee_Assignment/pull/2) |
| 2026-10-27 | 위치 추정 | cuVSLAM odometry와 SLAM 설정을 시험할 수 있도록 관련 구성과 실행 기반을 손봤다. | [cuVSLAM·nvblox 통합 PR #43](https://github.com/visualkhh/Lv2_bee_Assignment/pull/43) |
| 2026-10-28 | SLAM 실행·관측 | cuVSLAM 설치·현장 실행 절차와 원격 RViz 확인 환경을 정리했다. | [PR #43](https://github.com/visualkhh/Lv2_bee_Assignment/pull/43) |
| 2026-10-29 | SLAM·지도 | RTAB-Map 구성을 cuVSLAM·nvblox 중심으로 전환하고 map TF 및 RViz 구성을 맞췄다. | [PR #43](https://github.com/visualkhh/Lv2_bee_Assignment/pull/43) |
| 2026-10-30 | 미션 실행 | 미션 skill 묶음과 coordinator 실행 시나리오를 구성해 작업 흐름을 연결했다. | [주사위 미션 흐름 PR #49](https://github.com/visualkhh/Lv2_bee_Assignment/pull/49) |
| 2026-11-02 | 미션·조작 통합 | 주사위 미션 상태 흐름과 stand-up·arm skill 실행을 통합했다. | [PR #49](https://github.com/visualkhh/Lv2_bee_Assignment/pull/49) |
| 2026-11-03 | 적재·팔 동작 | load/unload 동작과 팔 보정 정책을 연결하고, 블랙박스 현장 로그로 실행 실패를 추적했다. | [PR #49](https://github.com/visualkhh/Lv2_bee_Assignment/pull/49) |
| 2026-11-04 | 미션·skill·경로 | 미션 상태 흐름과 skill 실행을 연결하고 Nav2 A* 경로 설정을 적용했다. | [미션 흐름 PR #49](https://github.com/visualkhh/Lv2_bee_Assignment/pull/49) · [skill 학습 PR #53](https://github.com/visualkhh/Lv2_bee_Assignment/pull/53) · [Nav2 A* PR #58](https://github.com/visualkhh/Lv2_bee_Assignment/pull/58) |
| 2026-11-06 | Isaac 실행·미션 상태 | Isaac 시뮬 실행 구조와 mission_manager의 퍽 tag 추적을 정리했다. | [Isaac 실행·정리 PR #75](https://github.com/visualkhh/Lv2_bee_Assignment/pull/75) |
| 2026-11-09 | 정책 실행·현장 비교 | 학습 정책을 ONNX Runtime으로 실행하고 `.onnx`/`.policy` 선택 비교를 추가했다. | [정책 실행 PR #83](https://github.com/visualkhh/Lv2_bee_Assignment/pull/83) |
| 2026-11-10 | Isaac 시험·실행 환경 | Isaac 팔 시험 환경과 먼 퍽 도달성 분석을 정리하고 Jetson 현장 실행 구성을 손봤다. | [팔 시험·도달성 PR #139](https://github.com/visualkhh/Lv2_bee_Assignment/pull/139) |
| 2026-11-11 | 위치·주행 안전 | cuVSLAM map jump 보정, 벽을 고려한 접근 면, 진척 없음 감지를 구현했다. | [벽 고려 접근 PR #141](https://github.com/visualkhh/Lv2_bee_Assignment/pull/141) · [진척 감시 PR #145](https://github.com/visualkhh/Lv2_bee_Assignment/pull/145) · [map 보정 PR #147](https://github.com/visualkhh/Lv2_bee_Assignment/pull/147) |
| 2026-11-12 | 관측·미션 진단 | 바코드와 신호등을 미션 관측 단계에 연결하고, rosbag을 남기는 field-test 실행 도구를 추가했다. | [field-test PR #173](https://github.com/visualkhh/Lv2_bee_Assignment/pull/173) · [ExecutePlan PR #195](https://github.com/visualkhh/Lv2_bee_Assignment/pull/195) |
| 2026-11-13 | 진단·시험 도구 | 단계 실행 시간·실패 이유를 기록하는 DIAG 흐름, TUI 현장 상태, 상자 이동 시험을 구현했다. | [DIAG PR #202](https://github.com/visualkhh/Lv2_bee_Assignment/pull/202) · [상자 이동 PR #209](https://github.com/visualkhh/Lv2_bee_Assignment/pull/209) |
| 2026-11-14 | 자원·지도 관측 | 노드별 CPU·메모리 현황과 static 2D/ESDF 입력을 Nav2 주행에 연결했다. | [노드별 부하 표시 PR #220](https://github.com/visualkhh/Lv2_bee_Assignment/pull/220) · [static 2D 입력 PR #218](https://github.com/visualkhh/Lv2_bee_Assignment/pull/218) |
| 2026-11-15 | 경로·충돌 복구 | 사람이 기록한 경로의 저장·재생, 벽 보정, Nav2 복구와 로봇 몸체 충돌 확인을 보강했다. | [경로 녹화 PR #256](https://github.com/visualkhh/Lv2_bee_Assignment/pull/256) · [경로 재생·충돌 감시 PR #273](https://github.com/visualkhh/Lv2_bee_Assignment/pull/273) |
| 2026-11-16 | 현장 통합·성능 | arena 좌표·상자 접근 보정, Nav2 no-path 대응, 보조 Python 기능의 C++ 전환, cuVSLAM/GPU 입력 및 GPU perception 실행, 시작 안내를 반영했다. | [박스 보정 커밋](https://github.com/visualkhh/Lv2_bee_Assignment/commit/d596253b) · [상자 중심 접근 커밋](https://github.com/visualkhh/Lv2_bee_Assignment/commit/d6011de2) · [Nav2 현장 조정 커밋](https://github.com/visualkhh/Lv2_bee_Assignment/commit/ad76b113) · [C++ 런타임 커밋](https://github.com/visualkhh/Lv2_bee_Assignment/commit/3d946600) · [cuVSLAM/GPU 입력 커밋](https://github.com/visualkhh/Lv2_bee_Assignment/commit/0ef31eb2) · [GPU perception 커밋](https://github.com/visualkhh/Lv2_bee_Assignment/commit/9fd88a1b) · [시작 안내 PR #281](https://github.com/visualkhh/Lv2_bee_Assignment/pull/281) |

날짜는 커밋 작성일을 기준으로 했으며, 커밋이 확인되지 않는 날짜를 임의로 채우지 않았다. PR·커밋으로 확인되는 구현과 별도의 시험 수행자 기록은 구분한다. 실기 시험 결과는 위의 시험 기록과 한계 항목을 따른다.

### 리뷰·시스템 설명

- 리뷰 근거: [#3](https://github.com/visualkhh/Lv2_bee_Assignment/pull/3)에서 테스트 스크립트를 테스트 디렉터리에 분리하자는 의견을 남겼고, [#193](https://github.com/visualkhh/Lv2_bee_Assignment/pull/193)에서 merge 뒤 확인을 요청했다. 나머지 링크는 주로 승인 기록이며 구체적인 기술 피드백이 없으면 별도 성과로 표현하지 않는다.
- 전체 흐름: `mission_manager`가 바코드·신호를 바탕으로 퍽을 할당하고, coordinator와 Nav2·skill 노드가 조사·수집·배치를 수행한다. 새 관측으로 결과를 확인하고 출발 구역에 복귀한다. 공식 실기에서는 이 전체 흐름의 완주가 확인되지 않았다 ([시스템 설계](architecture.md)).

## 전승혜
- 담당: 인지 (`perception`)
- GitHub: `SeungHye-J` — 아래 후보는 `gh pr list --author SeungHye-J --state merged`와 `gh search prs --reviewed-by SeungHye-J`로 자동으로 뽑은 것이라 **본인이 확인하고 고쳐 쓴다.**
- 구현 — 병합 13건 중 **본인 커밋이 있는 12건**. 인지 담당 PR과 인지 밖 PR을 나눴다. ([#63](https://github.com/visualkhh/Lv2_bee_Assignment/pull/63)은 `gh pr list --author` 자동 목록에 빠져 있어 `gh pr view 63`으로 확인해 넣음 — 작성자·커밋 모두 SeungHye-J, develop에 병합)

  **① 인지 (perception) — 8건**

  | PR | 병합일 | 무엇을 (제목 기준) | 규모 |
  | --- | --- | --- | --- |
  | [#9](https://github.com/visualkhh/Lv2_bee_Assignment/pull/9) | 10-22 | 퍽 기억·퍽 판단 및 새 YOLO 모델 추가 (모델 v1) | +522/−29 |
  | [#45](https://github.com/visualkhh/Lv2_bee_Assignment/pull/45) | 11-02 | 신호등 12회 점멸 해석, 퍽 인지 개선, 상자 인지 (모델 v3) | +825/−178 |
  | [#63](https://github.com/visualkhh/Lv2_bee_Assignment/pull/63) | 11-04 | 모델 v4 · 기억은 통합으로 · 상자 인지 보강 · 주사위 사진 | +474/−20 |
  | [#95](https://github.com/visualkhh/Lv2_bee_Assignment/pull/95) | 11-09 | 퍽 자세를 윗면 높이로 판단 | +81/−12 |
  | [#144](https://github.com/visualkhh/Lv2_bee_Assignment/pull/144) | 11-11 | 퍽 모델 v5 · 긴 기억 삭제 · API.md 수정 | +1294/−581 |
  | [#164](https://github.com/visualkhh/Lv2_bee_Assignment/pull/164) | 11-12 | 인지 노드 C++ 전환 · 젯슨 CPU 줄이기 | +3212/−142 |
  | [#180](https://github.com/visualkhh/Lv2_bee_Assignment/pull/180) | 11-12 | 퍽+상자 YOLO v6 · 상자 도킹 정보 · 인지 CPU 옵션 | +305/−31 |
  | [#246](https://github.com/visualkhh/Lv2_bee_Assignment/pull/246) | 11-15 | YOLO TensorRT FP16 추론 추가 | +243/−23 |

  **② 인지 밖 (조작·주행·계획) — 4건**

  | PR | 병합일 | 무엇을 (제목 기준) | 규모 |
  | --- | --- | --- | --- |
  | [#152](https://github.com/visualkhh/Lv2_bee_Assignment/pull/152) | 11-12 | MoveIt octomap 센서 연결 (pole_front_cam 점구름) | +78/−7 |
  | [#192](https://github.com/visualkhh/Lv2_bee_Assignment/pull/192) | 11-13 | pick_far 먼 퍽 집기 (강화학습 정책 + skill + coordinator 연결) | +757/−13 |
  | [#243](https://github.com/visualkhh/Lv2_bee_Assignment/pull/243) | 11-15 | 충돌 감시 구역 — 문틈에서 감속·정지 줄임 | +12/−6 |
  | [#284](https://github.com/visualkhh/Lv2_bee_Assignment/pull/284) | 11-16 | 조사 때 상자 윗면 안의 태그 퍽을 모두 등록 (plan) | +38/−11 |

  - **제외**: [#193](https://github.com/visualkhh/Lv2_bee_Assignment/pull/193) — 커밋 28개 중 본인 1개(JuneKunst 14 · claude 13). PR만 열고 병합한 건이라 구현 목록에서 제외.
  - 변경 효과 (**PR 본문 수치** — 원본 로그는 저장소에 없음 · 시험 ID는 [results/test-results.csv](../results/test-results.csv)):
    - [#95](https://github.com/visualkhh/Lv2_bee_Assignment/pull/95): 퍽 자세 판단 — Isaac 22% → 100%, 실물 그리퍼캠 88% → 99% (T-06)
    - [#144](https://github.com/visualkhh/Lv2_bee_Assignment/pull/144): 퍽 모델 v5 — 퍽 727/767 (94.8%), 신호등 상태 579/582 (T-04)
    - [#164](https://github.com/visualkhh/Lv2_bee_Assignment/pull/164): 인지 노드 C++ 전환 — CPU 약 108% → 약 23% (T-19)
    - [#246](https://github.com/visualkhh/Lv2_bee_Assignment/pull/246): YOLO TensorRT FP16 — 추론 21/24 ms → 7/7 ms (T-21)
    - [#192](https://github.com/visualkhh/Lv2_bee_Assignment/pull/192): 먼 퍽 집기 — 45° 기울여 진입하면 도달 영역 448 → 533 cm² (분석은 [#139](https://github.com/visualkhh/Lv2_bee_Assignment/pull/139)·송정혁, skill 구현·실기 확인은 #192) (T-24)
    - 위 표의 나머지 행은 효과 수치를 확인하지 않았다.
  - **YOLO 모델 작업** (인지 ①의 #9·#45·#63·#144·#180에 들어간 모델 v1~v6 — 자세한 근거는 [perception/YOLO_HISTORY.md](../../lv2_module6/ros2_ws/src/perception/YOLO_HISTORY.md)):
    - 라벨링: 직접 라벨·확인 사진 약 900장·네모 약 3,000개(11-02까지 누적) + 이후 노트북 녹화 272장·상자 234장. 실물 경광등은 손 라벨 676장. 로봇 촬영분은 8,000장에서 흐림·중복을 걸러 1,256장만 썼다.
    - 검수 5번: 신호등 라벨 48개 고침(10-29) · 시험 라벨 15곳(11-02) · 자동 상자 라벨 9건(11-09) · 학습 라벨 재검수 후 4번 재학습(11-10) · 상자 라벨 오류 11/15 확인 후 83장 재검수(11-12).
    - 시험: 실물 90장 → 옛 실물 퍽 184·193개 → 11-02부터 **로봇 카메라 시험 세트**(최종 193장·퍽 767개) · 상자 39개 · 경광등 약 590장. 가중치가 남은 학습 57개, 채점 파일 98개.
    - 저장소 모델 6개(v1~v6). 비교 뒤 버린 모델: 합성만 쓴 bee_v1~v3(실물 0.0003~0.68), 합성 추가·큰 모델 v5·v6_s(개선 없음), 퍽+상자 v11~13(작은 퍽 손해), 팀원 데이터 v14(실물 하락), 누운 퍽 OBB 3개(각도 19~22/126) 등.
    - 이 수치는 모두 노트북에서 녹화·사진으로 채점한 값이다(개인 작업 폴더 기록, 저장소 밖).
  - **먼 퍽 집기 pick_far** ([#192](https://github.com/visualkhh/Lv2_bee_Assignment/pull/192), 조작 쪽 — 팀 분석 #139를 이어받아 강화학습·skill 구현):
    - Isaac Lab 재학습(11-11): 6144대 × 3000번(5시간 6분), 16384판 평가에서 학습 전 기본 동작 **78.0%** → 예전 정책 98.2% → 새 정책 **99.2%** (퍽 좌우 ±8 cm, 박스 거리 1~5 cm 구간별 99.0~99.7%).
    - 속도·박스 각도 추가 학습(11-12~13): 팔 속도 50~70%, 박스 각도 ±45° → `pick_far_by_6600` 속도 0.5에서 99.3%, 0.7에서 97.9% (4096판).
    - ROS skill(`PickFarSkill`, 경유점 계산 `far_plan`)과 coordinator 연결(`pick_unreachable`이면 `pick_far`): Isaac + ROS 스택 화면 시험 4판 중 4판 성공.
    - 한계: 실기(11-13)에서 정책을 켜면 손이 퍽 윗부분으로 가서 놓쳐 **정책은 꺼 둠**(`models/skills/pick_standing_far/meta.yaml` `use: false`). 시뮬 수치는 모의 결과다.
- 검증:
  - **직접 수행한 실물 시험 (2026-11-17)** — T-36 ~ T-44, 증빙 [results/perception_1117/](../results/perception_1117/README.md) (사진 22장 · 원본 rosbag·영상은 본인 노트북 보관, 319 MB). T-36~T-43은 수동 모드 인지 시험이며, T-44는 자동 모드에서 바코드 단계의 미등록 입력 거부·오류 종료·정지 명령까지 확인했다.
    - 신호 12회: 사이클 31번 중 30번 정답(G G Y Y R G R Y R R Y Y = R4·Y5·G3), 1번 한 칸 오독 (T-36)
    - 바코드 P2-01~04: 4/4 (모두 손 카메라 · 폴 카메라로는 못 읽음) (T-37)
    - 퍽 ID 0~11: 12/12 · 색·모양도 맞음 · 누운 퍽 3개 맞음 (T-38)
    - 장소 ID: 태그 12·13·14 3/3 판독 (14는 손 카메라 look) (T-39) · 거리별 인지 0.62~1.37 m (T-40) · 사람 최대 2.75 m · 장애물 최대 1.61 m (T-41)
    - 인지 노드 CPU 평균 42.6 % (코어 1개 = 100 %) · YOLO 5 ms · `/tracked_objects` 9.4 Hz (T-43)
  - **미등록 바코드 오류 처리(T-44)**: 미등록 문자열 거부 → `unknown_barcode` 오류 종료 → 종료 후 속도 명령 0 확인. [실물 성공 증빙](../results/perception_1117/unknown_barcode/README.md). 정상 P2-09 판독과 자동 모드의 미등록 문자열 처리 회차는 구분했다.
  - 그 전 기록 T-04 · T-06 · T-19 · T-21 · T-24는 **PR 본문 수치**(원본 로그 없음)다.
  - 한계·개선 (11-17 실물에서 확인):
    - 상자 **먼 끝에 서 있는 퍽을 누움으로 오판** 2/2 (T-42) — 손 카메라가 비스듬히 볼 때. 개선: 먼 끝 퍽은 윗면 높이 기준을 다시 보거나 더 붙어서 위에서 다시 보기.
    - **장소 상자는 태그를 놓친 프레임에서 `start_box`로 돌아감** (`perception_node.cpp` 741~790 · 1194~1205). 미션은 장소를 도면 좌표로 정해 동작 영향은 작음. 개선: 읽은 장소 태그를 그 상자 자리에 기억.
    - 멀리서(1.1~1.5 m) 상자 하나를 2~3개로 잡음 · 초록 테이프를 퍽으로 오검출.
    - 신호 확정이 한 사이클(`confirm: 1`)이라 오독 사이클이 그대로 확정 신호로 한 번 나감.
    - 시험 범위: 신호 순서 1가지 · 장소 태그 3/3 판독 · 거리는 줄자 기준 없이 이동량과만 비교.
  - **인지 담당 미션 진행 현황** (발제 「요구사항 평가표」 No.3·4·7과 기본/결합 채점 항목에서 인지가 맡는 부분 · 작성 기준 develop `48cc6b2b`, 2026-11-17)

    | 요구사항 | 내 담당 범위 | 구현 | 검증 | 코드 근거 | 시험 근거 | 남은 작업 |
    | --- | --- | --- | --- | --- | --- | --- |
    | 퍽 ID 0~11 판독 | 손 카메라 AprilTag → `details.tag` | 구현 | **검증 완료 (실물 12/12)** | `perception_node.cpp` 태그 · `tag_cuda` | T-38 | — |
    | 장소 ID 12~14 판독 | 장소 태그 → `location_box` | 구현 | **검증 완료 (실물 3/3)** | `perception_node.cpp` 741~790 | T-39 | 태그 놓침 시 `start_box` 전환은 별도 상자 인지 항목 |
    | 바코드 4종 | zxing-cpp → `/mission/qr` | 구현 | **검증 완료 (실물 4/4)** | `read_barcodes` | T-37 | 폴 카메라로는 못 읽음 |
    | 신호 순서·횟수 (12회) | `signal_decoder` → `/mission/signal` | 구현 | 부분 (31 사이클 중 1 오독 · 순서 1가지) | `signal_decoder.cpp` | T-36 | 다른 순서 시험, 확정 기준(`confirm`) 검토 |
    | 신호 종료 판단 (3초 소등) | 사이클 경계 | 구현 | **검증 완료 (실물 31 사이클 경계)** | `signal_decoder.cpp` | T-36 | — |
    | 신호 오류 상태 출력 (누락·불명) | `signal_status` error(count·range·unknown) | 구현 | 미확인 (가림 시험에서 오류 출력 미확인) | `signal_decoder.cpp` `check()` | 없음 | 신호 가림·누락 시험 |
    | 퍽 색·모양 | YOLO 클래스 → `color`·`shape` | 구현 | **검증 완료 (실물 12/12)** | `perception_core.cpp` | T-38 | — |
    | 퍽 자세 (서 있음/누움) | 윗면 높이 → `posture` | 구현 | 실패/개선 필요 (가까운 쪽 맞음 · 먼 끝 2/2 오판) | `posture_from_height` | T-38 · T-42 | 먼 끝 오판 |
    | 위치·거리 (깊이 → map) | `pose` · `details.distance` | 구현 | 부분 (이동량과 맞음, 줄자 기준 없음) | `perception_node.cpp` 1385 | T-40 | 줄자 비교 |
    | 상자 인지 (시작·장소, 면 거리) | `start_box`·`location_box` · `face_dist_m` | 구현 | 실패/개선 필요 (멀리서 중복 · 장소 상자 전환) | `perception_node.cpp` 741~790 | T-39 · T-40 | 중복·전환 |
    | 사람·장애물 | YOLO `person`·`obstacle` | 구현 | **검증 완료 (실물 검출)** | 모델 v6 클래스 | T-41 | 정답 개수 기준 시험 |
    | ROS 발행·연동 | `/tracked_objects` 10 Hz · `/mission/qr` · `/mission/signal` | 구현 | **검증 완료 (rosbag 9.4 Hz, 세 토픽 기록)** | `on_publish` | T-43 · rosbag | — |

    - 인지 담당 필수 요구사항: 총 **12**개 / 코드로 구현 확인 **12**개 / 증빙 기반 검증 완료 **7**개 / 부분·실패·재시험 필요 **4**개 / 미확인 **1**개
    - **검증 완료율 = 7 ÷ 12 × 100 ≈ 58.3 %**, 구현 완료율 = 12 ÷ 12 × 100 = 100 % (모든 검증은 실물 · 동일 가중치의 **임시 관리 지표**이며 공식 경기 점수가 아님 · 요구사항 나눔은 본인 기준)
- 리뷰 — 후보 PR (타인 PR, 모두 병합됨): [#47](https://github.com/visualkhh/Lv2_bee_Assignment/pull/47)
  - 리뷰 내용: `gh pr view 47` 기준 APPROVED(2026-11-02). `vlm_task_planner.cpp` 4곳의 `location` → `location_box`를 확인했고 PR 브랜치에 옛 이름이 남은 곳이 없음, API.md의 상자 설명이 `perception_node.py` 동작과 같음, MissionSignal 옛 칸은 perception이 실제로 채우지 않음을 남겼다. 그리고 `location_box`의 `yaw_period`는 늘 360이 아니라 태그 법선을 못 구하면 90이라고 정정을 요청했다. 현재 API.md의 TrackedObject 설명에 "360 태그 법선 · 못 구했으면 90 윗면 변"이 들어 있다.
- 설명: 신호 읽기부터 복귀까지: 출발 구역에서 바코드(Code 128)와 신호등 12회 점멸을 읽어 색 순서와 R·Y·G 횟수를 기억하고(`BARCODE`·`READ`), 바코드가 오름/내림차순을 정한다. 시작 박스 6곳을 돌며 퍽을 확인해 적재함에 싣고 12개를 L1~L3 그룹에 한 번에 할당한다(`SURVEY`). 퍽마다 장소 앞으로 가서 주사위 칸에 놓고(`BUILD`), 새 관측으로 칸별 판정과 중복·누락을 재검증한 뒤(`VERIFY`) 출발 구역으로 돌아와 3초 정지한다(`RETURN`). 관측 오류·복귀 실패는 `ERROR`로 안전 정지하고 퍽 단위 실패는 기록하고 이어간다 ([architecture.md](architecture.md), `mission_solver.cpp`의 `next_phase` 기준). 맡지 않은 계획 쪽: [#47](https://github.com/visualkhh/Lv2_bee_Assignment/pull/47)에서 plan 쪽 `vlm_task_planner.cpp`의 물체 이름 사용처를 직접 열어 확인했고, 인지 결과가 SURVEY에서 퍽·상자 후보로 쓰이는 흐름을 이해해 상자 윗면 태그 퍽 등록([#284](https://github.com/visualkhh/Lv2_bee_Assignment/pull/284))을 수정했다.
