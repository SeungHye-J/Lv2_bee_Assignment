# 형식 예시 (⚠️ 가상 데이터 · 실제 기록 아님)

이 폴더의 파일은 **실제 로그가 아니다.** 로봇이 남기는 JSON과 시험 기록 CSV가 **어떤 모양인지** 보여 주는 가상 예시이고, 값(퍽 번호·색·횟수·시험 결과)은 모두 지어낸 것이다.

- **제출하는 실제 파일에 복사해 쓰지 않는다.** 실제 [signal.json](../signal.json) · [allocation.json](../allocation.json) · [test-results.csv](../test-results.csv)는 실제 기록에서 만든다.
- 구조는 `mission_manager.cpp`가 쓰는 필드를 따랐다 (`signal.json`: barcode · ascending · sequence · raw · cycles · counts · order / `allocation.json`: pucks · ok · why · partial · empty · locations). `cell`은 3×3 격자 번호(0~8, 가운데 4)이고 눈마다 쓰는 칸(3 = 0·4·8, 4 = 0·2·6·8, 5 = 0·2·4·6·8)은 `documents/project/ros/03-plan.md`의 `dice_cells`와 같게 맞췄다. 그 밖의 값은 모두 가상이다.
- 이 폴더는 형식을 보여 주는 용도라 제출 전 지워도 되고, 남기면 아래 확인 명령에 걸리지 않게 둔다.

## 제출 전 확인 (가상 데이터가 실제 파일에 섞이지 않았는지)
```bash
grep -rn "_example\|EX-0" project2 --exclude-dir=examples --exclude=REMAINING.md   # 아무것도 안 나와야 한다
```

| 예시 파일 | 대응하는 실제 파일 |
| --- | --- |
| [signal.example.json](signal.example.json) | [../signal.json](../signal.json) |
| [allocation.example.json](allocation.example.json) | [../allocation.json](../allocation.json) |
| [placement.example.csv](placement.example.csv) | [../../problem/placement.csv](../../problem/placement.csv) |
| [scorecard.example.md](scorecard.example.md) | [../scorecard.md](../scorecard.md) |
| [test-results.example.csv](test-results.example.csv) | [../test-results.csv](../test-results.csv) |
