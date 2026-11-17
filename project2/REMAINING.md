# 제출 전 남은 항목

git·이슈·노션·CI 기록으로 채울 수 있는 것은 채웠다. 아래는 그런 기록이 존재하지 않아 비워 둔 것이다 (값을 지어 넣지 않았다).

- [ ] `problem/` — 현재 저장소에는 팀 모의 문제(`signals.json`, `placement.csv`, `answer.json`)와 자체 검사(`validate_mock_problem.py`, `validation.md`)가 있다. **공식 경기 문제**의 확정 값·다른 팀 검토·발제의 `validate_problem.py` 검증은 아직 없다. 모의 검증 결과를 공식 문제 검증으로 표시하지 않는다.
- [x] `results/` — 공식 1·2회차는 기록이 없어 **모의 결과(sim-run)로 대체**했다 (공식 점수·영상은 없음)
- [ ] `docs/contributions.md` — 권형중 섹션은 본인 작성·push 커밋 기준으로 작업 주제별 정리했다. 다른 팀원의 확인과 각자 직접 수행한 시험·리뷰·설명 근거 확인은 남아 있다.
- [ ] 제출 — `docs/report` PR 병합, 저장소 Public 확인, 태그 `project2-problem-submit` · `project2-submit`, 제출 폼
- [ ] 제출 전 확인: `grep -rn "_example\|EX-0" project2 --exclude-dir=examples --exclude=REMAINING.md` 결과가 비어 있어야 한다
