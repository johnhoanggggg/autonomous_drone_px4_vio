"""Summarise loop_sim runs: loop_out/<flight>.<shift>.<variant>."""
import glob, re, sys
import numpy as np
names = {'A': 'as flown', 'B': 'carrot', 'C': 'carrot+hold', 'D': 'carrot+hold+pull', 'E': 'hold only'}
runs = {}
for f in glob.glob('loop_out/*.*.?'):
    d, v = f.split('/')[-1].split('.')[0], f[-1]
    s = open(f).read()
    if not s:
        continue
    g = re.search(r'goal (\S+) s', s)
    x = dict(reach=float(g.group(1)) if g else np.nan,
             lat=float(re.search(r'lat p50 \S+ p95 (\S+),', s).group(1)),
             vert=float(re.search(r'vert p95 (\S+) cm', s).group(1)),
             cmin=float(re.search(r'clear min (\S+),', s).group(1)),
             below=float(re.search(r'<0.30 (\S+) s', s).group(1)),
             holds=float(re.search(r'holds (\S+) s', s).group(1)),
             dips=int(re.search(r'dips (\d+)', s).group(1)),
             length=float(re.search(r'length (\S+) m', s).group(1)),
             wob=float(re.search(r'wobble (\S+) ', s).group(1)))
    runs.setdefault(v, {}).setdefault(d, []).append(x)
flights = sorted(runs['A'])
print('per flight: reach s (runs reached) | holds s | wobble')
for d in flights:
    print('%-3s' % d, '  '.join('%s %5.1f(%d) h%4.1f w%.3f' % (v, np.nanmean([r['reach'] for r in runs[v][d]]), sum(~np.isnan([r['reach'] for r in runs[v][d]])), np.mean([r['holds'] for r in runs[v][d]]), np.mean([r['wob'] for r in runs[v][d]])) for v in names))
print()
for v, n in names.items():
    all_ = [r for d in flights for r in runs[v][d]]
    m = lambda k: np.nanmean([r[k] for r in all_])
    print('%-17s reached %2d/%d  time %.1f s  length %.2f m  holds %.2f s  dips %.2f  wobble %.3f  lat95 %.1f  vert95 %.1f  clear<0.30 %.2f s  worst clear %.2f' % (
        n, sum(~np.isnan([r['reach'] for r in all_])), len(all_), m('reach'), m('length'), m('holds'), m('dips'), m('wob'), m('lat'), m('vert'), m('below'), min(r['cmin'] for r in all_)))
