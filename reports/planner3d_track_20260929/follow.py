"""How well the drone and command follow the path they were given.

For each FOLLOWING tick, compare the drone (body_map) and the command (command_map)
with the path that was active `delay` seconds earlier (so the path's own start is not
the current drone position). Also count large route re-shapes between consecutive
paths (max distance of the new path's remaining route from the old one).
"""
import pickle
import sys

import numpy as np


def cross(route, p):
    best = None
    for a, b in zip(route[:-1], route[1:]):
        d = b - a
        L2 = float(d @ d)
        t = 0.0 if L2 < 1e-12 else float(np.clip((p - a) @ d / L2, 0, 1))
        c = a + t * d
        e = p - c
        dh = d[:2] / max(np.linalg.norm(d[:2]), 1e-9)
        lat = float(dh[0] * e[1] - dh[1] * e[0])
        n = float(np.linalg.norm(e))
        if best is None or n < best[0]:
            best = (n, lat, float(e[2]))
    return best


def run(pkl, delay=1.0):
    D = pickle.load(open(pkl, 'rb'))
    st, rp = D['/planner3d/flight/status'], [(t, np.array(p)) for t, p in D['/planner3d/reference_path'] if len(p) >= 2]
    rows = []
    for t, v in st:
        if v.get('state') != 'FOLLOWING' or not v.get('command_map'):
            continue
        old = [p for tp, p in rp if tp <= t - delay]
        if not old:
            continue
        P = old[-1]
        rows.append((t, *cross(P, np.array(v['body_map'])), *cross(P, np.array(v['command_map']))))
    R = np.array(rows)
    shapes = []
    for (t0, a), (t1, b) in zip(rp[:-1], rp[1:]):
        if len(b) < 3:
            continue
        dev = max(cross(a, q)[0] for q in b[2:])
        shapes.append((t1, dev))
    return D, R, np.array(shapes)


if __name__ == '__main__':
    q = lambda x: 'p50 %4.1f p95 %4.1f max %4.1f' % (100 * np.percentile(np.abs(x), 50), 100 * np.percentile(np.abs(x), 95), 100 * np.max(np.abs(x)))
    for pkl in sys.argv[1:]:
        D, R, S = run(pkl)
        print('== %s (%d FOLLOWING ticks) vs the path active 1 s earlier, cm' % (pkl, len(R)))
        print('  drone   total %s | lateral %s | vertical %s (mean %+.1f)' % (q(R[:, 1]), q(R[:, 2]), q(R[:, 3]), 100 * R[:, 3].mean()))
        print('  command total %s | lateral %s | vertical %s (mean %+.1f)' % (q(R[:, 4]), q(R[:, 5]), q(R[:, 6]), 100 * R[:, 6].mean()))
        big = S[S[:, 1] > 0.15]
        print('  route re-shapes >15 cm: %d of %d path updates at t = %s' % (len(big), len(S), ', '.join('%.1f(%.0fcm)' % (t, 100 * d) for t, d in big[:12])))
