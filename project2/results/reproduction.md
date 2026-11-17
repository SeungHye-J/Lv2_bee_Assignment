# 재현 시험 기록

> 사람이 새 환경에서 README만 보고 따라 한 재현 기록은 없다. 대신 GitHub Actions `ci` 워크플로가 매번 **새 clone에서 도커 테스트베드(test-all: 빌드 → 단위 시험 → faker)를 처음부터 실행**한다. 아래는 그 실행 기록이다 (실행자: GitHub Actions 자동 실행, 모의 환경). 실기(Jetson) 재현은 한 적이 없다.

| 날짜 | 실행자 | 장비 · 설정 · 소프트웨어 버전 | 커밋 | 실행 결과 |
| --- | --- | --- | --- | --- |
| 2026-11-16 | GitHub Actions (PR 검증) | 도커 testbed, test-all | `d0f12d12` (develop) | 성공 — [run 43830265489](https://github.com/visualkhh/Lv2_bee_Assignment/actions/runs/43830265489) |
| 2026-11-16 | GitHub Actions (push) | 도커 testbed, test-all | `3ffaf682` (develop) | 성공 — [run 43826093974](https://github.com/visualkhh/Lv2_bee_Assignment/actions/runs/43826093974) |
| 2026-11-16 | GitHub Actions (push) | 도커 testbed, test-all | `c9002372` (develop) | **실패** — [run 43836774523](https://github.com/visualkhh/Lv2_bee_Assignment/actions/runs/43836774523). `plan.yaml` 163행 키 누락으로 faker 단계 ParserError |
| 2026-11-17 | GitHub Actions (push · 예약 실행) | 도커 testbed, test-all | `0119580b` (main) | **실패** — [run 43991586191](https://github.com/visualkhh/Lv2_bee_Assignment/actions/runs/43991586191) · [43994595225](https://github.com/visualkhh/Lv2_bee_Assignment/actions/runs/43994595225). 같은 원인 |
