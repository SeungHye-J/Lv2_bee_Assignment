# 공식 2회차 — 로그 (여기에 붙여넣기)

> TODO: 로그 — 공식 2회차 로그는 남아 있지 않아 이 폴더에 복사한 파일이 없다. 아래 표는 로그가 있을 때 넣을 파일 목록이다. 내용을 새로 만들어 넣지 않는다.

## 넣을 파일

| 파일 | 어디서 | 만드는 법 |
| --- | --- | --- |
| `signal.json` · `allocation.json` · `build.json` · `verify.json` · `return.json` (실패했으면 `error.json`) | 로봇 `~/.ros/mission/<시작 시각>/` | 그 회차 폴더를 `scp -r`로 복사 (mission_manager가 회차마다 남김) |
| `photos/` | 위 같은 폴더 | 단계 사진 |
| bag (`.mcap` + `metadata.yaml`) 또는 외부 링크 | `./run-cli bag start --label round2` 로 녹화한 폴더 | 용량이 크면 이 폴더에는 링크만 두고 `bag.md`에 위치를 적는다 |
| `sys.csv` | bag과 같은 폴더 | CPU·메모리·GPU 부하 기록 (`--no-sys`를 안 줬다면 같이 생김) |

## 확인할 것
- 이 폴더의 `signal.json`·`allocation.json`은 [results/](../../)의 같은 이름 파일과 **같은 회차**여야 한다 (점수가 반영된 회차).
- 비밀번호·API 토큰·내부 주소가 로그에 없는지 확인한다.
- 실물인지 모의(도커)인지 구분해서 [판정표](../../scorecard.md)에 적는다.
