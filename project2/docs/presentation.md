# 발표 자료 (개요)

이 문서가 발표 자료다 (별도 슬라이드 파일은 만들지 않았다). 아래는 발표 내용의 요지이며, 근거는 [시험 결과](../results/test-results.csv)의 기존 PR 보고값과 11-17 실물 인지 시험 기록이다. 두 출처를 구분한다.

1. **문제와 목표** — 신호등 12회 점멸과 바코드를 읽고, 주사위 눈을 맞춰 퍽을 상자에 담고, 재확인 후 출발 구역으로 복귀한다. ([problem/problem.md](../problem/problem.md))
2. **시스템 흐름** — IDLE → GO_OBSERVE → BARCODE → READ → SURVEY → BUILD → VERIFY → RETURN → DONE/ERROR. ([architecture.md](architecture.md))
3. **주요 기능** — cuVSLAM+nvblox 위치·지도, Nav2(MPPI + RL 보정), YOLO 퍽·신호등 인식(TensorRT FP16), 팔 skill(pick·place·load·unload), 안전 정지(collision_monitor·e-stop).
4. **실제 수행 결과** — 시험 총 44건(T-01~T-44). 기존 T-01~T-35는 성공 20·실패 9·부분 성공 6이며, T-36~T-44는 실물 인지 시험이다. 정상 바코드 4/4, 퍽 ID 12/12, 장소 ID 3/3 판독 및 미등록 문자열 거부·오류 종료·정지 명령을 확인했다. 신호는 31사이클 중 30사이클 정답과 일치했다. 기존 통합 실기 기록에서는 SURVEY를 넘지 못했고 공식 회차 점수는 미확인. 실기 팔 단위로는 pick → load 618회 반복(11-14 pick 94 %), 누운 퍽 세우기가 가장 약함(15~47 %). ([README §5](../README.md#5-시험-결과), [arm-rl-tests](../results/arm-rl-tests/README.md))
5. **실패 원인과 개선** — 주행 TF 지연·goal checker·맴돌기 수정, 젯슨 CPU 과부하(EKF 주기·인지 C++ 전환), 그리퍼 카메라 높이 보정. ([README §6](../README.md))
6. **한계** — 실물 BUILD·VERIFY·RETURN 완주 기록 없음, 기존 PR 보고값 중 일부는 원본 로그·영상이 없고 모의·이미지 단위 수치다. 11-17 실물 인지 시험은 [사진·선별 로그](../results/perception_1117/README.md)를 저장소에 포함했고 원본 영상·rosbag은 노트북에 보관한다. 인지 단위 시험을 전체 미션 완주로 집계하지 않는다.
