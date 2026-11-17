# 시스템 설계

이미 있는 설계 문서는 복사하지 않고 연결한다. 이 파일에는 **발제문이 요구하는데 아직 없는 것**(상태도 · 단계별 완료 조건 · 안전 정지/오류 처리)만 쓴다.

## 이미 있는 문서
- ROS 2 노드·토픽·메시지, 전체 구조: [lv2_module6/README.md — 전체 아키텍처](../../lv2_module6/README.md#전체-아키텍처) · [documents/project/ros](../../documents/project/ros/README.md)
- 미션 한 번의 흐름: [lv2_module6/README.md — 미션 한 번의 흐름](../../lv2_module6/README.md#미션-한-번의-흐름) · [documents/project/ros/03-plan.md](../../documents/project/ros/03-plan.md)
- 수동 조작·비상정지: [lv2_module6/README.md — 수동 조작 · 비상정지](../../lv2_module6/README.md#수동-조작--비상정지)

## 좌표계
TODO: 좌표계 설명 — **만드는 법:** base_footprint(로봇 기준점 = 앞 바퀴축) · map(출발 자리가 원점) · arena(도면 mm) · 카메라 프레임의 관계를 표 하나로. 출발 자세 변환은 `plan.yaml`의 `start_pose_arena`와 `mission_solver.cpp`의 `arena_to_map`, 높이(퍽 바닥 70 mm · 윗면 130 mm)는 documents/mission/README.md 근거.

## 실행 순서
TODO: 실행 순서 요약(관측→계획→주행→조작→재검증→복귀) — **만드는 법:** 위 "미션 한 번의 흐름" 링크의 단계(GO_OBSERVE·BARCODE·READ·SURVEY·BUILD·VERIFY·RETURN)를 6줄 이내 번호로 요약한다. 실제 코드는 `lv2_module6/ros2_ws/src/plan/src/mission_manager.cpp`.

## 단계별 완료 조건 · 안전 정지 · 오류 처리 (상태도)
TODO: 상태도(mermaid) + 단계별 완료 조건 표 + 안전 정지/오류 처리 — **만드는 법:** 상태 전이는 `mission_solver.cpp`의 `next_phase`와 [documents/project/ros/03-plan.md](../../documents/project/ros/03-plan.md)의 단계표·`ERROR` 전이를 근거로 `stateDiagram-v2`로 그린다. 표 열은 `단계 | 완료 조건 | 실패 시 | 안전 정지`. 신호·바코드 오류(추정 금지 → 안전 정지)와 복귀 실패는 `ERROR`로 멈추고 비상정지는 `/cmd/e_stop`(바퀴 명령 차단)이라는 점을 포함한다.
