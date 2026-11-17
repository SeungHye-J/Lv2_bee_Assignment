#!/usr/bin/env python3
"""Summarise arm-skill test logs (isaac/loops/logs, kept out of git) into
summary.csv and figures/*.png for project2/results/arm-rl-tests.

usage: python3 make_figures.py [LOGS_DIR]
LOGS_DIR defaults to lv2_module6/isaac/loops/logs of this repo.
A run is "sim" when its rosbag starts on the Isaac clock (< 1e15 ns), else real.
RL evaluation numbers come from the training eval logs (1024 episodes each),
copied below because those logs are not in the repo.
"""
import csv, glob, json, os, re, sys, collections as C, statistics as S
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager as fm

HERE = os.path.dirname(os.path.abspath(__file__))
LOGS = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, '../../../lv2_module6/isaac/loops/logs')
FIG = os.path.join(HERE, 'figures')
os.makedirs(FIG, exist_ok=True)

for f in fm.findSystemFonts():
    if 'NotoSansCJK-Regular' in f:
        fm.fontManager.addfont(f)
        plt.rcParams['font.family'] = fm.FontProperties(fname=f).get_name()
        break
INK, INK2, GRID, SURF = '#1f1f1e', '#6b6a63', '#e5e4df', '#fcfcfb'
BLUE, ORANGE, AQUA, GRAY = '#2a78d6', '#eb6834', '#1baf7a', '#b4b2a9'
plt.rcParams.update({'figure.facecolor': SURF, 'axes.facecolor': SURF, 'axes.edgecolor': GRID,
                     'axes.labelcolor': INK2, 'xtick.color': INK2, 'ytick.color': INK2, 'text.color': INK,
                     'axes.spines.top': False, 'axes.spines.right': False, 'font.size': 10})


def rows(pattern):
    out = []
    for f in sorted(glob.glob(os.path.join(LOGS, pattern))):
        try:
            out += [dict(x, _f=os.path.basename(f)) for x in csv.DictReader(open(f))]
        except Exception:
            pass
    return out


def is_sim(path):
    for m in glob.glob(path + '/**/metadata.yaml', recursive=True)[:1]:
        n = int(re.search(r'nanoseconds_since_epoch: (\d+)', open(m).read()).group(1))
        return n < 1e15
    return None


def day(name):
    return re.search(r'2026(\d{4})_', name).group(1)


def md(d):
    return f'{d[:2]}-{d[2:]}'


def finish(ax, name, title):
    ax.set_title(title, loc='left', fontsize=11, color=INK, pad=10)
    ax.grid(axis='y', color=GRID, lw=0.8)
    ax.set_axisbelow(True)
    plt.tight_layout()
    plt.savefig(os.path.join(FIG, name), dpi=150)
    plt.close()


def pct(a, b):
    return 100.0 * a / b if b else float('nan')


summary = []

# ---- 1. real pick -> load loop (pick_loop.py) ----
pl = [x for x in rows('pick_loop_2026*.csv')
      if not x.get('sim_puck') and not is_sim(os.path.join(LOGS, x['_f'][:-4] + '_bags'))]
by = C.defaultdict(list)
for x in pl:
    by[day(x['_f'])].append(x)
days = sorted(by)
pick_rate, load_rate, seat_rate = [], [], []
for d in days:
    r = by[d]
    tried = [x for x in r if x.get('pick') in ('ok', 'fail')]
    ok = [x for x in tried if x['pick'] == 'ok']
    ld = [x for x in r if x.get('load') in ('ok', 'fail')]
    ldok = [x for x in ld if x['load'] == 'ok']
    seat = [x for x in ldok if x.get('load_seated') in ('y', 'n')]
    seat_y = [x for x in seat if x['load_seated'] == 'y']
    cyc = [float(x['seconds']) for x in r if x.get('pick') == 'ok' and x.get('load') == 'ok' and x.get('seconds')]
    hand = [(float(x['hand_rel_x_mm']) ** 2 + float(x['hand_rel_y_mm']) ** 2) ** .5 for x in r if x.get('hand_rel_x_mm')]
    fails = C.Counter(x['reason'] for x in tried if x['pick'] == 'fail')
    pick_rate.append(pct(len(ok), len(tried)))
    load_rate.append(pct(len(ldok), len(ld)))
    seat_rate.append(pct(len(seat_y), len(seat)))
    summary.append(dict(test='pick_loop', equipment='real', day=md(d), files=len({x['_f'] for x in r}),
                        attempts=len(tried), success=len(ok), rate_pct=round(pct(len(ok), len(tried)), 1),
                        load_attempts=len(ld), load_ok=len(ldok), seated_checked=len(seat), seated_yes=len(seat_y),
                        cycle_median_s=round(S.median(cyc), 1) if cyc else '',
                        hand_cam_median_mm=round(S.median(hand), 1) if hand else '',
                        top_fail='; '.join(f'{k} {v}' for k, v in fails.most_common(3))))

fig, ax = plt.subplots(figsize=(7.2, 3.6))
w = 0.26
xs = range(len(days))
for i, (vals, col, lab) in enumerate([(pick_rate, BLUE, 'pick 성공'), (load_rate, ORANGE, 'load 동작 성공'),
                                      (seat_rate, AQUA, '칸에 바로 섬 (눈으로 확인한 것만)')]):
    b = ax.bar([x + (i - 1) * (w + 0.02) for x in xs], vals, w, color=col, label=lab)
    for rect, v in zip(b, vals):
        if v == v:
            ax.text(rect.get_x() + w / 2, v + 1.5, f'{v:.0f}', ha='center', fontsize=8, color=INK2)
ax.set_xticks(list(xs), [f'{md(d)}\n(n={s["attempts"]})' for d, s in zip(days, [s for s in summary if s['test'] == 'pick_loop'])])
ax.set_ylim(0, 112)
ax.set_ylabel('%')
ax.legend(frameon=False, fontsize=8, loc='lower left', ncol=3, bbox_to_anchor=(0, 1.02))
finish(ax, 'pick_load_real_by_day.png', '')

# ---- 2. real pick policy off vs on (11-08, ruler measured) ----
fig, axs = plt.subplots(1, 2, figsize=(7.2, 3.0))
for ax, (t, off, on, sd_off, sd_on) in zip(axs, [('손 안 어긋남 |평균| (mm)', 6.0, 2.2, 7.0, 2.9),
                                                  ('손 카메라 퍽−손끝 거리 (mm)', 9.2, 2.3, 2.5, 0.8)]):
    b = ax.bar(['정책 off', '정책 on'], [off, on], 0.5, color=[GRAY, BLUE], yerr=[sd_off, sd_on],
               error_kw=dict(ecolor=INK2, lw=1, capsize=3))
    for rect, v in zip(b, [off, on]):
        ax.text(rect.get_x() + rect.get_width() / 2, v / 2, f'{v}', ha='center', color='white', fontsize=10)
    ax.set_title(t, loc='left', fontsize=10, color=INK)
    ax.grid(axis='y', color=GRID, lw=0.8)
    ax.set_axisbelow(True)
plt.tight_layout()
plt.savefig(os.path.join(FIG, 'pick_policy_off_on_1108.png'), dpi=150)
plt.close()

# ---- 3. real stand (stand_loop: ruler y/n · stand_auto: camera verdict) ----
st = rows('stand_loop_2026*.csv')
sa = rows('stand_auto_2026*.csv')
labels, stood, n_obs = [], [], []
for d in sorted({day(x['_f']) for x in st}):
    r = [x for x in st if day(x['_f']) == d and x['result'] != 'SKIPPED' and x['reason'] != 'nav_unavailable']
    obs = [x for x in r if x['standing'] in ('y', 'n') or x['result'] == 'FAILED']
    y = [x for x in obs if x['standing'] == 'y']
    if not obs:
        continue
    labels.append(f'{md(d)}\nstand_loop'); stood.append(pct(len(y), len(obs))); n_obs.append(len(obs))
    summary.append(dict(test='stand_loop', equipment='real', day=md(d), attempts=len(obs), success=len(y),
                        rate_pct=round(pct(len(y), len(obs)), 1),
                        top_fail='; '.join(f'{k} {v}' for k, v in C.Counter(x['reason'] or 'stood_no' for x in obs if x['standing'] != 'y').most_common(3))))
for d in sorted({day(x['_f']) for x in sa}):
    r = [x for x in sa if day(x['_f']) == d and x['stand'] in ('ok', 'fail')]
    y = [x for x in r if x['cam_verdict'] == 'standing']
    labels.append(f'{md(d)}\nstand_auto'); stood.append(pct(len(y), len(r))); n_obs.append(len(r))
    summary.append(dict(test='stand_auto', equipment='real', day=md(d), attempts=len(r), success=len(y),
                        rate_pct=round(pct(len(y), len(r)), 1),
                        top_fail='; '.join(f'{k} {v}' for k, v in C.Counter((x['stand_reason'] or 'cam_' + x['cam_verdict']) for x in r if x['cam_verdict'] != 'standing').most_common(3))))
fig, ax = plt.subplots(figsize=(7.2, 3.2))
b = ax.bar(range(len(labels)), stood, 0.5, color=BLUE)
for rect, v, n in zip(b, stood, n_obs):
    ax.text(rect.get_x() + 0.25, v + 1.5, f'{v:.0f} %\n(n={n})', ha='center', fontsize=8, color=INK2)
ax.set_xticks(range(len(labels)), labels, fontsize=8)
ax.set_ylim(0, 100)
ax.set_ylabel('세워진 비율 %')
finish(ax, 'stand_real_by_day.png', '')

# ---- 4. real place (place_loop) ----
pc_place = rows('place_loop_2026*.csv')
for d in sorted({day(x['_f']) for x in pc_place}):
    r = [x for x in pc_place if day(x['_f']) == d and x['place'] in ('ok', 'fail')]
    ok = [x for x in r if x['place'] == 'ok']
    summary.append(dict(test='place_loop', equipment='real', day=md(d), attempts=len(r), success=len(ok),
                        rate_pct=round(pct(len(ok), len(r)), 1),
                        top_fail='; '.join(f'{k} {v}' for k, v in C.Counter(x['place_reason'] for x in r if x['place'] != 'ok').most_common(3))))

# ---- 5. real coordinator box clearing (pick_coord) ----
pc = rows('pick_coord_2026*.csv')
cats = [('puck', '선 퍽'), ('puck_lying', '누운 퍽')]
order = ['ok', 'stand_still_lying', 'stand_failed', 'grasp_empty', 'arm_jam', 'slot_full', 'other']
names = {'ok': '집어 실음', 'stand_still_lying': '세웠는데 여전히 누움', 'stand_failed': 'stand 실패',
         'grasp_empty': '빈손', 'arm_jam': '팔 막힘(arm_jam)', 'slot_full': '칸 가득', 'other': '기타'}
cols = {'ok': BLUE, 'stand_still_lying': ORANGE, 'stand_failed': '#eda100', 'grasp_empty': '#e87ba4',
        'arm_jam': '#4a3aa7', 'slot_full': GRAY, 'other': '#d8d6cf'}
counts = {}
for k, _ in cats:
    c = C.Counter()
    for x in pc:
        if x['class'] != k:
            continue
        rsn = 'ok' if x['ok'] == 'True' else x['reason'].split(' ')[0]
        c[rsn if rsn in order else 'other'] += 1
    counts[k] = c
    n = sum(c.values())
    summary.append(dict(test=f'pick_coord ({k})', equipment='real', day='11-14~15', files=len({x['_f'] for x in pc}),
                        attempts=n, success=c['ok'], rate_pct=round(pct(c['ok'], n), 1),
                        top_fail='; '.join(f'{a} {b}' for a, b in c.most_common() if a != 'ok')))
fig, ax = plt.subplots(figsize=(7.2, 2.6))
for yi, (k, lab) in enumerate(cats):
    left, n = 0, sum(counts[k].values())
    for o in order:
        v = counts[k][o]
        if not v:
            continue
        ax.barh(yi, v, left=left, color=cols[o], edgecolor=SURF, lw=2, label=names[o] if yi == 0 or k == 'puck_lying' else None)
        if v >= 5:
            ax.text(left + v / 2, yi, str(v), ha='center', va='center', fontsize=8, color='white')
        left += v
    ax.text(left + 1, yi, f'{pct(counts[k]["ok"], n):.0f} % 성공 (n={n})', va='center', fontsize=8, color=INK2)
ax.set_yticks([0, 1], [c[1] for c in cats])
ax.invert_yaxis()
ax.set_xlim(0, 100)
ax.set_xlabel('퍽 수')
h, l = ax.get_legend_handles_labels()
seen = dict(zip(l, h))
ax.legend(seen.values(), seen.keys(), frameon=False, fontsize=7, ncol=4, loc='lower left', bbox_to_anchor=(0, 1.02))
ax.grid(axis='x', color=GRID, lw=0.8)
ax.set_axisbelow(True)
plt.tight_layout()
plt.savefig(os.path.join(FIG, 'pick_coord_real.png'), dpi=150)
plt.close()

# ---- 6. Isaac: sim pick_loop (11-09) and approach docking ----
sim_pl = [x for x in rows('pick_loop_2026*.csv')
          if x.get('sim_puck') or is_sim(os.path.join(LOGS, x['_f'][:-4] + '_bags'))]
for d in sorted({day(x['_f']) for x in sim_pl}):
    r = [x for x in sim_pl if day(x['_f']) == d and x.get('pick') in ('ok', 'fail')]
    ok = [x for x in r if x['pick'] == 'ok']
    summary.append(dict(test='pick_loop', equipment='sim', day=md(d), attempts=len(r), success=len(ok),
                        rate_pct=round(pct(len(ok), len(r)), 1),
                        top_fail='; '.join(f'{k} {v}' for k, v in C.Counter(x['reason'] for x in r if x['pick'] != 'ok').most_common(3))))
ap = C.defaultdict(C.Counter)
for f in glob.glob(os.path.join(LOGS, 'approach_1*/result.json')):
    if is_sim(os.path.join(os.path.dirname(f), 'bag')) is False:
        continue
    try:
        j = json.load(open(f))
    except Exception:
        continue
    bad = [s for s in j.get('steps', []) if not s['ok']]
    ap[j.get('scene', '?')]['ok' if j.get('ok') else (bad[0]['reason'] if bad else 'interrupted')] += 1
scene_lab = {'scene_approach': '단순 장면', 'scene_approach_matte': '무광 바닥 장면', 'scene_sim': '미션 장면'}
aorder = ['ok', 'nav_no_progress', 'approach_blocked', 'approach_box_unseen', 'nav_failed', 'other']
anames = {'ok': '도킹 성공', 'nav_no_progress': '주행 진척 없음', 'approach_blocked': '접근 막힘',
          'approach_box_unseen': '상자 안 보임', 'nav_failed': '주행 실패', 'other': '기타'}
acol = {'ok': BLUE, 'nav_no_progress': ORANGE, 'approach_blocked': '#eda100', 'approach_box_unseen': '#e87ba4',
        'nav_failed': '#4a3aa7', 'other': GRAY}
fig, ax = plt.subplots(figsize=(7.2, 2.9))
scenes = [s for s in scene_lab if s in ap]
for yi, s in enumerate(scenes):
    c = C.Counter()
    for k, v in ap[s].items():
        c[k if k in aorder else 'other'] += v
    n, left = sum(c.values()), 0
    for o in aorder:
        if c[o]:
            ax.barh(yi, c[o], left=left, color=acol[o], edgecolor=SURF, lw=2, label=anames[o])
            if c[o] >= 6:
                ax.text(left + c[o] / 2, yi, str(c[o]), ha='center', va='center', fontsize=8, color='white')
            left += c[o]
    ax.text(left + 1, yi, f'{pct(c["ok"], n):.0f} % (n={n})', va='center', fontsize=8, color=INK2)
    summary.append(dict(test=f'approach ({s})', equipment='sim', day='11-12~16', attempts=n, success=c['ok'],
                        rate_pct=round(pct(c['ok'], n), 1),
                        top_fail='; '.join(f'{a} {b}' for a, b in c.most_common() if a != 'ok')))
ax.set_yticks(range(len(scenes)), [scene_lab[s] for s in scenes])
ax.invert_yaxis()
ax.set_xlim(0, 175)
ax.set_xlabel('시도 수')
h, l = ax.get_legend_handles_labels()
seen = dict(zip(l, h))
ax.legend(seen.values(), seen.keys(), frameon=False, fontsize=7, ncol=6, loc='lower left', bbox_to_anchor=(0, 1.02))
ax.grid(axis='x', color=GRID, lw=0.8)
ax.set_axisbelow(True)
plt.tight_layout()
plt.savefig(os.path.join(FIG, 'approach_isaac.png'), dpi=150)
plt.close()

# ---- 7. RL eval in Isaac Lab (baseline Δ = 0 vs policy, 1024 episodes each) ----
RL = [  # (group, condition, baseline %, policy %)
    ('stand C (11-08)', '학습 조건', 99.6, 94.2), ('stand C (11-08)', '인식 오차 켬', 81.7, 89.2),
    ('stand_grasp v1 (11-09)', '학습 조건', 61.6, 83.4), ('stand_grasp v1 (11-09)', '넓은 오차', 35.8, 84.4),
    ('load v01→v04 (11-11)', 'v01 방식 A', 94.0, 92.3), ('load v01→v04 (11-11)', 'v04 (기준 미기록)', None, 99.9),
    ('pick 5단계 (11-08)', '섞음', 49.8, 97.0), ('pick 5단계 (11-08)', '오차 판 (기준 미기록)', None, 90.7),
]
groups = list(dict.fromkeys(g for g, *_ in RL))
fig, axs = plt.subplots(1, len(groups), figsize=(9.6, 3.2), sharey=True)
for ax, g in zip(axs, groups):
    items = [r for r in RL if r[0] == g]
    for i, (_, cond, base, pol) in enumerate(items):
        if base is not None:
            ax.bar(i - 0.19, base, 0.36, color=GRAY, label='기준 동작 (Δ = 0)')
            ax.text(i - 0.19, base + 1.5, f'{base:.0f}', ha='center', fontsize=7, color=INK2)
        ax.bar(i + 0.19, pol, 0.36, color=BLUE, label='학습 정책')
        ax.text(i + 0.19, pol + 1.5, f'{pol:.0f}', ha='center', fontsize=7, color=INK2)
    ax.set_xticks(range(len(items)), [r[1] for r in items], fontsize=8)
    ax.set_title(g, fontsize=9, color=INK, loc='left')
    ax.set_ylim(0, 110)
    ax.grid(axis='y', color=GRID, lw=0.8)
    ax.set_axisbelow(True)
axs[0].set_ylabel('성공률 %')
h, l = axs[0].get_legend_handles_labels()
seen = dict(zip(l, h))
fig.legend(seen.values(), seen.keys(), frameon=False, fontsize=8, ncol=2, loc='upper right')
plt.tight_layout(rect=(0, 0, 1, 0.93))
plt.savefig(os.path.join(FIG, 'rl_isaac_eval.png'), dpi=150)
plt.close()

keys = ['test', 'equipment', 'day', 'files', 'attempts', 'success', 'rate_pct', 'load_attempts', 'load_ok',
        'seated_checked', 'seated_yes', 'cycle_median_s', 'hand_cam_median_mm', 'top_fail']
with open(os.path.join(HERE, 'summary.csv'), 'w', newline='') as f:
    w = csv.DictWriter(f, keys)
    w.writeheader()
    w.writerows(summary)
for s in summary:
    print(s)
