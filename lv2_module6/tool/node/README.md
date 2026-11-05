# tool/node — 노드별 분석 도구

노드마다 폴더 하나. **기록·ON/OFF·리포트 진입점은 `run-cli ab` (메뉴 14) 하나**이고,
이 폴더에는 리포트가 쓰는 분석 스크립트만 둔다. 공통 bag 읽기·궤적 계산은 `common/bagio.py`.

```text
tool/node/
├── README.md
├── common/
│   └── bagio.py                    # rosbag2 읽기 · 경로 투영(횡오차) · yaw
└── rl_nav_correction/
    └── report.py                   # bag → report.png (횡오차·헤딩·슬립·Δ)
```

## rl_nav_correction

```bash
cd tool

# A — 보정 ON (기본)
./run-cli ab status
./run-cli ab record --label A
# 다른 터미널/RViz에서 주행 후
./run-cli ab stop

# B — 같은 출발점·같은 goal로 보정 OFF
./run-cli ab off
./run-cli ab record --label B
# 주행 후
./run-cli ab stop
./run-cli ab on

# 비교 리포트 (인자 없으면 최근 2개)
./run-cli ab report --last
```

- 기록은 bee의 `~/bags/ab/<시각>_<라벨>`에 쌓인다 (rosbag2 · mcap).
  bee가 `ROS_DOMAIN_ID=88` + `fastdds_local_only.xml`(SHM 전용)이라 PC에서 원격 기록할 수 없다.
- `run-cli ab report <A> <B>`로 bag 경로를 직접 줄 수도 있다.
- 리포트는 `report.png` 4패널:

| 패널 | 내용 |
|---|---|
| 선속도 | `/cmd_vel_nav` vs `/cmd_vel_corrected` vs `/odometry/filtered` |
| XY 궤적 | ON/OFF 궤적 + `/plan` |
| 경로 횡오차 | 진행 거리별 최단거리 [cm] |
| 적용 Δ | `/rl_nav_correction/debug`의 Δv·Δω |

요약(그림 아래): 횡오차 RMSE/최대 · 헤딩 MAE · 슬립(|ωimu−ωodom|) · Δv 평균/최대 ·
최종 goal 오차 · 시간·거리 · 보정 ON 비율.

> `report.py`는 `/rl_nav_correction/debug`의 Δ를 직접 읽어 토픽 개수 차이로 생기는
> 정렬 오차를 피한다. bag은 같은 `ROS_DOMAIN_ID`에서 기록해야 한다 (bee는 88).

## 노드 추가 시

1. `tool/node/<노드>/` 폴더
2. `report.py` — 그 노드의 기록을 읽어 지표를 내는 분석 스크립트
3. `run-cli`에 `ab`와 같은 서브커맨드/메뉴를 연결 (기록·토글은 `run-cli`가 담당)
4. 이 README 표에 한 줄
