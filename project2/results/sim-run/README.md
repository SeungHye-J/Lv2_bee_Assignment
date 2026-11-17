# 모의 실행 기록 (도커 testbed · 공식 경기 아님)

2026-11-17 로컬 PC의 도커 테스트베드(`lv2_module6`, faker)에서 미션을 한 번 실제로 돌려 나온 파일이다. **공식 경기 기록이 아니다.** 이 폴더의 값은 로봇(모의)이 남긴 그대로이고 손대지 않았다.

| 항목 | 내용 |
| --- | --- |
| 실행 | `python3 isaac/mission/run_mission.py --clear-basket`를 `use_sim_time`만 끄고 컨테이너에서 실행 (신호·바코드를 읽었다고 치고 `/mission/signal_status`·`/mission/qr`로 **주입**, `start` 전송) |
| 주입한 입력 | 신호 `R G Y R G R Y G R G Y R` (R5·Y3·G4), 바코드 `P2-01` — 신호등을 인식한 결과가 아니다 |
| 환경 | `faker.launch.py` (`map:=saved`, `operation:=AUTO`), 퍽·상자 인식 대역 없음 (faker는 센서 대역만 흉내) |
| 커밋 | `0119580b`(main) + `plan.yaml` 163행 `start_box_retreat` 키 누락을 **로컬에서만** 고침 (CI 실패 원인과 같은 줄) |
| 소요 | 약 14.5분 (`phases.jsonl` MISSION 870.97 s) |

## 결과 (파일 그대로)
- `signal.json`: 주입한 신호가 그대로 기록됨 (order `YGR`).
- SURVEY 701.7 s 만에 `puck_count:0` — 시작 박스 6곳 모두 `nav_no_progress`(43~73 s) 뒤 `approach_failed`. 퍽 0개 → `allocation.json`은 `{"ok": false, "why": "puck_count"}`.
- `build.json`·`verify.json` 없음: 퍽이 없어 BUILD·VERIFY가 실행되지 않음.
- RETURN: 이동은 `nav_no_progress` 3회 뒤 `approach park_home`이 `approach_blocked`로 3회 실패했지만, 이미 출발 구역(734, 950 mm) 안이라 `return.json`이 `in_zone: true`로 기록되고 단계는 `DONE`.
- 즉 **DONE이지만 놓은 퍽 0/0, 실패 8건**이다. 성공이 아니다.

## 해석할 때 주의
- 박스 접근이 계속 실패하는 것은 faker에 퍽·상자 인식과 실제 주행 물리가 없기 때문일 수 있다. 같은 실패가 실기에서 난 것으로 읽으면 안 된다.
- 주입한 신호·바코드 때문에 READ·BARCODE 단계(각 약 1초)는 인식 성능과 무관하다.

## 파일
`signal.json` · `allocation.json` · `return.json` · `phases.jsonl` · `steps.jsonl`(34단계) · `photos/` · `run_mission.log`(실행 로그) · `faker.log`(faker 전체 로그, 488 KB — 1행의 PC 호스트명만 `<호스트명 삭제>`로 가림, 나머지 원본)
