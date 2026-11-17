# 제출 전 TODO 목록

각 항목을 끝내면 **그 파일의 `TODO:` 줄을 지우고 여기 체크박스도 체크**한다. 남은 것 찾기: `grep -rn "TODO:" project2`
📥 표시는 **붙여넣을 자리**다. 틀(빈 파일·안내)이 이미 만들어져 있으니 거기에 실제 내용을 넣고 그 파일의 `TODO`를 지운다. 출처: 발제문 "프로젝트 평가를 위한 필수 수행 사항" · "주요 산출물"

## ① 문제 패키지 → `project2/problem/`
- [ ] [problem/problem.md](problem/problem.md) — 확정 문제를 누구나 재현할 수 있게 작성
- [ ] [problem/signals.json](problem/signals.json) — 정답 신호(12회 점멸)와 바코드
- [ ] [problem/placement.csv](problem/placement.csv) — 퍽 12개 ID·색·모양·시작 박스
- [ ] 태그 `project2-problem-submit` (병합 후)

## ② 설계·시험 → `project2/docs/` · `project2/results/`
- [ ] [docs/architecture.md](docs/architecture.md) — 좌표계 · 상태도 · 단계별 완료 조건 · 안전 정지/오류 처리
- [ ] [results/test-results.csv](results/test-results.csv) — 기능별 시험 기록 (실물과 모의 구분)
- [ ] [results/signal.json](results/signal.json) — 실제 관측한 신호
- [ ] [results/allocation.json](results/allocation.json) — 계산한 L1~L3 할당
- [ ] 📥 [docs/issues.md](docs/issues.md) — GitHub Issue 목록(오류·조건·원인·담당·검토자·처리 현황)

## ③ 실행 코드·최종 경기 결과 → `main` + 태그 `project2-submit`
- [ ] [README.md](README.md) — 빈 칸(경기 초기화, 실제 버전, 시험 결과 요약, 실패 원인과 수정 전후)
- [ ] [results/scorecard.md](results/scorecard.md) — 1·2회차 100점 판정표
- [ ] 📥 [results/round1/video.md](results/round1/video.md) · 📥 [results/round1/log/](results/round1/log/) — 공식 1회차 영상·로그
- [ ] 📥 [results/round2/video.md](results/round2/video.md) · 📥 [results/round2/log/](results/round2/log/) — 공식 2회차 영상·로그
- [ ] [results/reproduction.md](results/reproduction.md) — 다른 팀원의 재현 시험 기록
- [ ] [docs/presentation.md](docs/presentation.md) — 발표 자료(개요 + 실제 슬라이드 링크)
- [ ] 비밀 정보 확인 (토큰·비밀번호·내부 주소가 문서·로그에 없는지)

## ④ 개인 기여 → `project2/docs/contributions.md`
- [ ] [docs/contributions.md](docs/contributions.md) — **팀원 전원**: 구현·검증·리뷰·설명 (링크 포함)

## ⑤ 선택 도전 (필수 완료 후 · 추가 배점 없음)
- [ ] 📥 [challenge/A/](../challenge/A/) 신호 인식 안정화 (30회) · 📥 [challenge/B/](../challenge/B/) 퍽 할당 비교 (12건) · 📥 [challenge/C/](../challenge/C/) 이동 경로 비교 (6회) — 하기로 한 것만

## 팀 노션 (노션에서 작성 · README 상단 노션 링크)
- [ ] 프로젝트 소개 · [ ] 개인별 역할 및 수행 내용 · [ ] 개인 작업 일지 · [ ] 팀 논의 및 의사결정 기록 · [ ] 프로젝트 산출물(문제 검증·설계·시험·경기 영상/로그·수정 전후·재현 기록)

## 마지막
- [ ] 가상 예시가 실제 파일에 섞이지 않았는지: `grep -rn "_example\|EX-0" project2 --exclude-dir=examples --exclude=TODO.md` — 결과가 없어야 함 ([results/examples/README.md](results/examples/README.md))
- [ ] `main`에 병합 · 저장소 Public 확인 · 태그 `project2-submit` — 만드는 법은 [README.md](README.md) 하단 "제출" 절

---

## 파일 안에 TODO를 못 적는 것 (CSV · 아직 없는 파일) — 만드는 법

### `results/test-results.csv` (헤더만 있음)
열: `시험 ID,날짜,시험 대상,장비,입력 조건,기대 결과,실제 결과,판정,시도 횟수,오차,영상·로그·커밋`
- **행 만들기:** 시험 한 번당 한 행. 판정은 `성공` · `실패` · `미확인` 중 하나. 미수행 항목을 성공으로 쓰지 않는다.
- **장비 열에 `실물` 또는 `모의(도커/faker)`를 반드시 적는다** (발제문 요구).
- **재료:** 자동 시험은 `test-all`(도커 컨테이너 안) 출력과 패키지별 gtest/pytest 결과, 실기 시험은 bee에서 본 로그·화면. 기능별로 최소 신호 인식 · 바코드 · 장소 접근/정지 · 퍽 ID · 장소 ID · 그룹 배정 · 주행 · 집기/놓기 · 재검증 · 복귀 · 안전 정지.
- **링크 열:** 영상·로그는 `results/round*/log/` 또는 `./run-cli bag start --label <이름>`으로 남긴 bag, 커밋은 `git rev-parse --short HEAD`.

### 📥 `docs/issues.md` — GitHub Issue 목록
- **만드는 법:** `gh issue list --repo visualkhh/Lv2_bee_Assignment --state all --limit 200 --json number,title,state,assignees,url`로 뽑아 표로 만든다. 열: `# · 제목 · 발견된 오류 · 발생 조건 · 원인 · 수정 담당자 · 검토자 · 처리 현황`. 원인·검토자는 이슈와 PR 본문을 읽고 적는다(자동으로 안 나온다).

### 📥 `results/round1/` · `results/round2/` — 공식 경기 1·2회차
각 폴더에 만들 것:
- `video.md`: 신호 인식부터 최종 결과까지 **전체 영상**의 접근 가능한 링크(영상이 크면 외부 링크). 접근 권한을 확인한다.
- `log/`: 그 회차의 mission_manager 기록 — 로봇의 `~/.ros/mission/<시작 시각>/` 폴더(`signal·allocation·build·verify·return(·error).json` + `photos/`)를 `scp -r`로 복사. 추가로 `./run-cli bag start --label round1`로 bag을 녹화했다면 그 위치.
- 판정은 [results/scorecard.md](results/scorecard.md)에 적는다. 두 회차를 섞지 않는다.

### 📥 `challenge/A|B|C/` — 선택 도전 (하기로 한 것만)
- **A 신호 인식 안정화:** `report.md`, `results.csv`(30회), 기존·개선 설정 2개, 원본 영상·로그
- **B 퍽 할당 비교:** `cases.json`(6건), `expected.json`, `results.csv`, 알고리즘 코드, `report.md`
- **C 이동 경로 개선:** 기존·개선 방문 순서 각 3회(총 6회) 결과와 `report.md` — `lv2_module6/tools/plan/route_order.py`와 `plan.yaml`의 `survey_order`가 이미 있어서 재료가 된다.
