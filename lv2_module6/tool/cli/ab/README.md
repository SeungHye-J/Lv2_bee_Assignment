# tool/cli/ab — A/B 자동 주행·분석

노드별 A/B 동작을 `command.json`으로 정의하고, `report.py`가 rosbag2를 비교한다.
`run-cli`가 이 파일을 읽어 rosbag 기록과 Nav2 `/navigate_to_pose` 주행을 직접 수행한다.
사용자가 주행할 필요는 없다.

```text
tool/cli/ab/
├── common/
│   └── bagio.py                    # rosbag2 읽기 · 횡오차 · yaw
└── rl_nav_correction/
    ├── command.json                # A/B 시작 자세·route·반복·timeout
    └── report.py                   # bag → report.png
```

## command.json

| 키 | 뜻 |
|---|---|
| `frame_id` | goal 좌표계 (기본 `map`) |
| `start` | A/B 시작 자세 `{x, y, yaw_deg}`. `null`이면 현재 `map→base_footprint` 자세를 시작점으로 잡는다 |
| `route` | 주행할 goal 목록 `{x, y, yaw_deg}`. A/B 모두 같은 순서로 주행 |
| `repeats` | A/B 쌍을 몇 번 반복할지 (기본 1) |
| `goal_timeout_s` | goal 하나당 제한 시간 [s] |
| `settle_s` | goal 도착 뒤 다음 goal 전 대기 [s] |
| `auto_mode` | true면 시작 전에 조작 모드를 AUTO로 전환 |
| `move_to_start` | `start`가 있을 때 시작 전에 그 자세로 이동 |
| `return_to_start` | A 주행 뒤 B를 위해 시작 자세로 복귀 |

## 실행

```bash
cd tool
./run-cli ab rl_nav_correction guide --label test1
```

순서:

1. E-STOP OFF · AUTO 확인 (필요하면 AUTO로 전환)
2. `start`로 이동 (설정한 경우)
3. A 녹화 시작 (보정 ON) → `route` 주행 → 녹화 종료
4. 시작 자세로 복귀 → 보정 OFF
5. B 녹화 시작 → 같은 `route` 주행 → 녹화 종료
6. 보정 ON 복원 → 최근 A/B 리포트 생성

TUI는 **14) A/B 테스트 (노드별) → rl_nav_correction → 6) 자동 A/B**에서 같은 설정을 쓴다.

## report.png

`report.py`가 bag 두 개 이상을 읽어 4패널을 그린다.

| 패널 | 내용 |
|---|---|
| 선속도 | `/cmd_vel_nav` vs `/cmd_vel_corrected` vs `/odometry/filtered` |
| XY 궤적 | ON/OFF 궤적 + `/plan` |
| 경로 횡오차 | 진행 거리별 최단거리 [cm] |
| 적용 Δ | `/rl_nav_correction/debug`의 Δv·Δω |

요약: 횡오차 RMSE/최대 · 헤딩 MAE · 슬립 · Δv 평균/최대 · 최종 goal 오차 · 시간·거리 · 보정 ON 비율.
Δ는 `/rl_nav_correction/debug`에서 직접 읽어 토픽 정렬 오차를 피한다.

수동 녹화만 한 경우:

```bash
./run-cli ab rl_nav_correction report --last
./run-cli ab rl_nav_correction report ~/bags/ab/A ~/bags/ab/B --labels A B --out ~/ab.png
```
