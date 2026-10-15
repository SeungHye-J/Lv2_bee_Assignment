# 대회 코스 — Twin Closed Maze

미로 4.464 × 6.364m, 벽 높이 0.39m. 가운데(y=0) 벽이 위·아래 두 미로로 완전히 나눈다 — 로봇 시작 칸(`spawn_xy_m`)이 있는 쪽만 오갈 수 있다.

| 파일 | 내용 |
| --- | --- |
| `twin_closed_maze/walls.json` | **우리가 쓰는 코스** — 기둥 83 + 벽판 80 상자 (세로 벽 포함), 크기 · 벽 높이 · 시작 위치. 시뮬레이터가 읽는다 |
| `twin_closed_maze/model.sdf` · `model.config` | 같은 상자로 만든 Gazebo 모델 (`model://twin_closed_maze`) |
| `worlds/twin_closed_maze.sdf` | Gazebo 월드 (원본 그대로 — 바닥 · 조명 · 위 모델 include) |
| `twin_closed_maze/original/` | 원본 그대로 (model.sdf · source.json · LICENSE · NOTICE) |
| `tools/build_walls.py` | 원본 메시(STL)에서 위 파일들을 다시 만든다 — 코스가 바뀔 때만 |

- 원본: [SeongminJaden/waffle-lyrical-slam](https://github.com/SeongminJaden/waffle-lyrical-slam/tree/7fc1ef486f3fc2513b06ca9bc76cd00f4df4fb03/src/waffle_slam_course) @ `7fc1ef4`, Apache-2.0 (`original/LICENSE`, `original/NOTICE`). 미로 형상의 출처는 `original/source.json`.
- **원본 `model.sdf`의 충돌 상자에는 세로 벽(바깥 테두리 · 안쪽 4개)이 빠져 있다** — 그대로 Gazebo에 쓰면 로봇이 세로 벽을 통과한다. 그래서 메시를 위에서 본 모양으로 기둥 사이 벽판을 판정해 다시 만들었다.
- 원본 메시(29MB STL)는 저장소에 넣지 않았다 — `build_walls.py`를 돌릴 때만 받는다 (`tools/.cache/`, gitignore).
