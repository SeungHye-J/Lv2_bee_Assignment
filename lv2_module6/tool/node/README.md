# tool/node — 노드별 관리 도구

노드마다 폴더 하나. 그 노드의 **기록(rosbag2)** · **분석(report.png)** · 진단을 한 곳에 모은다.
공통 bag 읽기·궤적 계산은 `common/bagio.py`.

```text
tool/node/
├── README.md
├── common/
│   └── bagio.py                    # rosbag2 읽기 · 경로 투영(횡오차) · yaw
├── rl_nav_correction/              # RL 주행 보정 (Δ 더하기)
│   ├── run                         # ./run record … | ./run report …
│   ├── record.sh                   # ros2 bag record 프리셋
│   └── report.py                   # bag → report.png (횡오차·헤딩·슬립·Δ)
└── rl_nav_controller/              # Nav2 주행 컨트롤러 (정책 vs DWB)
    ├── run
    ├── record.sh
    └── report.py                   # 횡오차·평균속도·정책 행동
```

## rl_nav_correction

```bash
cd tool/node/rl_nav_correction

# 1) 주행 기록 (다른 터미널에서 주행)
./run record ~/bags/on

# 2) 보정 끄고 같은 주행
ros2 service call /rl_nav_correction/toggle std_srvs/srv/SetBool "{data: false}"
./run record ~/bags/off

# 3) 비교 리포트
./run report ~/bags/on ~/bags/off --labels ON OFF --out ~/rl_report.png
```

`report.png` 4패널:

| 패널 | 내용 |
|---|---|
| 선속도 | `/cmd_vel_nav` vs `/cmd_vel_corrected` vs `/odometry/filtered` |
| XY 궤적 | ON/OFF 궤적 + `/plan` |
| 경로 횡오차 | 진행 거리별 최단거리 [cm] |
| 적용 Δ | `/rl_nav_correction/debug`의 Δv·Δω |

요약(그림 아래): 횡오차 RMSE/최대 · 헤딩 MAE · 슬립(|ωimu−ωodom|) · Δv 평균/최대 · 시간·거리 · 보정 ON 비율.

> Lyrical의 `ros2 bag record`는 토픽을 `--topics`로 받는다 (record.sh에 반영됨).
> 기본 저장은 mcap — report.py가 metadata.yaml을 읽어 자동 처리.
> 로봇과 같은 `ROS_DOMAIN_ID`에서 실행해야 한다 (~/.bashrc 값).

## 노드 추가 시

1. `tool/node/<노드>/` 폴더
2. `record.sh` — 그 노드가 내는 토픽·디버그 토픽
3. `report.py` — 지표 정의(예: arm은 관절 추종오차·전류, controller는 정책 행동·횡오차)
4. `run` — `record|report` 서브커맨드
5. 이 README 표에 한 줄
