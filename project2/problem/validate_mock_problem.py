#!/usr/bin/env python3
"""우리 팀 모의 문제 검증 — signals.json · placement.csv · answer.json (표준 라이브러리만).
발제의 validate_problem.py가 아니라 mission.pdf §04 규칙을 직접 옮긴 검사다.
  규칙: 퍽 12개 한 번씩 · R=빨강만 · G=초록만 · Y=한 모양 통일(색 무관) · 각 그룹 크기 = 신호 횟수 · 횟수 합 12
        바코드 P2-01/03 오름차순 · P2-02/04 내림차순 (동률은 R→Y→G) · L1→L2→L3 순서
"""
import csv, itertools, json, sys
from pathlib import Path

D = Path(__file__).parent
sig = json.loads((D / 'signals.json').read_text(encoding='utf-8'))
ans = json.loads((D / 'answer.json').read_text(encoding='utf-8'))
pucks = {r['퍽 ID']: r for r in csv.DictReader(open(D / 'placement.csv', encoding='utf-8'))}
err = []
cnt = {c: sig['sequence'].count(c) for c in ('R', 'Y', 'G')}
if len(sig['sequence']) != 12: err.append('신호가 12개가 아님')
if cnt != sig['counts']: err.append(f'counts 불일치 {cnt} != {sig["counts"]}')
if not all(1 <= v <= 6 for v in cnt.values()): err.append('횟수가 1~6 밖')
if len(pucks) != 12: err.append('퍽이 12개가 아님')
asc = sig['barcode'] in ('P2-01', 'P2-03')
order = sorted('RYG', key=lambda g: (cnt[g] if asc else -cnt[g], 'RYG'.index(g)))
if [ans['L1']['group'], ans['L2']['group'], ans['L3']['group']] != order: err.append(f'L1~L3 그룹 순서가 {order}와 다름')
used = []
for L in ('L1', 'L2', 'L3'):
    g, ids = ans[L]['group'], ans[L]['pucks']
    used += ids
    if len(ids) != cnt[g]: err.append(f'{L}: 퍽 수 {len(ids)} != {g} 횟수 {cnt[g]}')
    for i in ids:
        if g == 'R' and pucks[i]['색상'] != 'red': err.append(f'{L}: {i}는 빨강이 아님')
        if g == 'G' and pucks[i]['색상'] != 'green': err.append(f'{L}: {i}는 초록이 아님')
    if g == 'Y' and len({pucks[i]['모양'] for i in ids}) != 1: err.append(f'{L}: Y 퍽 모양이 섞임')
if sorted(used) != sorted(pucks): err.append('중복 또는 누락 있음')

def solutions():  # 유효한 전체 할당 수 (그룹 크기·색·모양 규칙만, 장소 배정은 order로 고정)
    ids = sorted(pucks); n = 0
    for Y in itertools.combinations(ids, cnt['Y']):
        if len({pucks[i]['모양'] for i in Y}) != 1: continue
        rest = [i for i in ids if i not in Y]
        for R in itertools.combinations(rest, cnt['R']):
            G = [i for i in rest if i not in R]
            if all(pucks[i]['색상'] == 'red' for i in R) and all(pucks[i]['색상'] == 'green' for i in G): n += 1
    return n

print(f'신호 {" ".join(sig["sequence"])} · 횟수 {cnt} · 바코드 {sig["barcode"]} ({"오름" if asc else "내림"}차순) → 장소 순서 {order}')
print(f'퍽 {len(pucks)}개 · 빨강 {sum(p["색상"]=="red" for p in pucks.values())} · 초록 {sum(p["색상"]=="green" for p in pucks.values())}')
print(f'유효한 전체 할당 수: {solutions()}')
print('결과:', 'PASS' if not err else 'FAIL')
for e in err: print(' -', e)
sys.exit(1 if err else 0)
