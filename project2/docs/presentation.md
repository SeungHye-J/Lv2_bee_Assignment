# 발표 자료 (개요)

이 문서가 발표 자료다 (별도 슬라이드 파일은 만들지 않았다). 아래는 발표 내용의 요지이며, 수치는 [시험 결과](../results/test-results.csv)의 PR 본문 기록이다.

1. **문제와 목표** — 신호등 12회 점멸과 바코드를 읽고, 주사위 눈을 맞춰 퍽을 상자에 담고, 재확인 후 출발 구역으로 복귀한다. ([problem/problem.md](../problem/problem.md))
2. **시스템 흐름** — IDLE → GO_OBSERVE → BARCODE → READ → SURVEY → BUILD → VERIFY → RETURN → DONE/ERROR. ([architecture.md](architecture.md))
3. **주요 기능** — cuVSLAM+nvblox 위치·지도, Nav2(MPPI + RL 보정), YOLO 퍽·신호등 인식(TensorRT FP16), 팔 skill(pick·place·load·unload), 안전 정지(collision_monitor·e-stop).
4. **실제 수행 결과** — 시험 27건 중 성공 18·실패 8·부분 성공 1. 실기 미션은 SURVEY를 넘지 못했고 공식 회차 점수는 미확인. ([README §5](../README.md#5-시험-결과))
5. **실패 원인과 개선** — 주행 TF 지연·goal checker·맴돌기 수정, 젯슨 CPU 과부하(EKF 주기·인지 C++ 전환), 그리퍼 카메라 높이 보정. ([README §6](../README.md))
6. **한계** — 실물 BUILD·VERIFY·RETURN 완주 기록 없음, 원본 로그·영상 없음, 많은 결과가 모의·이미지 단위 수치.
