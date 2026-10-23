"""Long corridor flight (global 20260930T044126Z) as a column-width paper figure.

Same content as the earlier path report (final occupancy grid, clearance band, first
planned route, flown track, holds, takeoff, goal, landing, door), but rotated so the
corridor runs left to right and fills a wide panel. Axes are replaced by a 1 m scale bar
and arrows showing the map x and y directions.

Inputs: door_044126Z.pkl (grids, paths, body, t_arm; times from bag start) and
states_044126Z.pkl ([(t, state)] from /planner/flight/status, same time base).
Usage: python plot_corridor_rotated.py <door.pkl> <states.pkl> <out_stem> [landing_s_after_arm]
"""
import pickle
import sys

import numpy as np
from scipy.ndimage import distance_transform_edt
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import matplotlib.patches as mpatches  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402

pkl, spkl, stem = sys.argv[1], sys.argv[2], sys.argv[3]
LAND_AFTER_ARM = float(sys.argv[4]) if len(sys.argv) > 4 else 86.0
DOOR = np.array([5.5, -9.4])          # doorway centre in map frame (m)
d = pickle.load(open(pkl, 'rb'))
states = pickle.load(open(spkl, 'rb'))
t_arm = d['t_arm']

SURF, INK = '#fcfcfb', '#0b0b0b'
FREE, UNKNOWN, OCC, BAND = (0.957, 0.953, 0.937), (0.82, 0.82, 0.80), (0.12, 0.16, 0.22), (0.97, 0.84, 0.66)
ROUTE, FLOWN, HOLD, CLEAR = '#eb6834', '#1a9e55', '#d4890f', 0.25

t_land = t_arm + LAND_AFTER_ARM
_, G, res, ox, oy = [g for g in d['grids'] if g[0] <= t_land][-1]
body = d['body'][d['body'][:, 0] <= t_land]
jump = np.where(np.linalg.norm(np.diff(body[:, 1:3], axis=0), axis=1) > 0.5)[0]
if len(jump):                                           # VIO reset at touchdown snaps the pose back
    body = body[:jump[0] + 1]
    t_land = body[-1, 0]
route = [p for p in d['paths'] if len(p[1]) > 1][0][1]  # first planned route
goal = route[-1]

# holds = adapter HOLD / COMMAND_HOLD intervals while flying
holds = []
for (t, s), nxt in zip(states, states[1:] + [(t_land, 'END')]):
    if s in ('HOLD', 'COMMAND_HOLD') and t < t_land - 1.0:
        if holds and t - holds[-1][1] < 0.5:            # COMMAND_HOLD then HOLD = one hold
            holds[-1] = (holds[-1][0], nxt[0])
        else:
            holds.append((t, nxt[0]))
hold_pts = [body[np.argmin(abs(body[:, 0] - (a + b) / 2)), 1:3] for a, b in holds]

# rotation that lays the corridor horizontally, takeoff on the left
pts = np.vstack([body[:, 1:3], route])
c = pts.mean(axis=0)
w, v = np.linalg.eigh(np.cov((pts - c).T))
ax_dir = v[:, np.argmax(w)]
if np.dot(goal - body[0, 1:3], ax_dir) < 0:
    ax_dir = -ax_dir
theta = -np.arctan2(ax_dir[1], ax_dir[0])
R = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])


def rot(p):
    return (np.atleast_2d(p) - c) @ R.T


occ = G >= 65
img = np.empty(G.shape + (3,))
img[:] = FREE
img[G < 0] = UNKNOWN
dist = distance_transform_edt(~occ) * res
img[(dist <= CLEAR) & ~occ & (G >= 0)] = BAND
img[occ] = OCC

plt.rcParams.update({'font.size': 9, 'font.family': ['DejaVu Sans']})
r, b = rot(route), rot(body[:, 1:3])
lo, hi = np.vstack([r, b]).min(axis=0) - [0.6, 0.9], np.vstack([r, b]).max(axis=0) + [0.6, 0.9]

# resample the grid onto the rotated frame (nearest cell); outside the map = unmapped
step = res / 2
us, vs = np.arange(lo[0], hi[0], step), np.arange(lo[1], hi[1], step)
U, V = np.meshgrid(us, vs)
XY = np.stack([U, V], -1) @ R + c                       # inverse rotation back to the map frame
ci = np.floor((XY[..., 0] - ox) / res).astype(int)
ri = np.floor((XY[..., 1] - oy) / res).astype(int)
ok = (ci >= 0) & (ci < G.shape[1]) & (ri >= 0) & (ri < G.shape[0])
rimg = np.empty(U.shape + (3,))
rimg[:] = UNKNOWN
rimg[ok] = img[ri[ok], ci[ok]]

fig, ax = plt.subplots(figsize=(7.0, 3.3), facecolor=SURF)
ax.imshow(rimg, origin='lower', extent=(lo[0], lo[0] + len(us) * step, lo[1], lo[1] + len(vs) * step),
          interpolation='nearest')
ax.plot(r[:, 0], r[:, 1], color=ROUTE, lw=1.8, ls=(0, (4, 2)))
ax.plot(b[:, 0], b[:, 1], color=FLOWN, lw=2.2)
for k in range(6, int(LAND_AFTER_ARM), 10):          # time ticks every 10 s after arming
    i = np.argmin(abs(body[:, 0] - (t_arm + k)))
    p = b[i]
    ax.plot(*p, 'o', color=INK, ms=2.5)
    ax.annotate(f'{k} s', p, xytext=(0, 6), textcoords='offset points', ha='center', fontsize=7.5, color=INK)
for hp in hold_pts:
    ax.plot(*rot(hp)[0], 'o', mfc='none', mec=HOLD, mew=1.8, ms=7)
ax.plot(*b[0], '^', color=FLOWN, mec=INK, mew=0.8, ms=8)
ax.plot(*rot(goal)[0], 'o', mfc='white', mec=INK, mew=1.5, ms=8)
ax.plot(*b[-1], 'v', color='#d2402a', mec=INK, mew=0.8, ms=8)
dp = rot(DOOR)[0]
ax.annotate('door', dp, xytext=(0, -22), textcoords='offset points', ha='center', fontsize=8.5,
            arrowprops=dict(arrowstyle='->', color=INK, lw=0.9),
            bbox=dict(boxstyle='round,pad=0.2', fc='white', ec=INK, lw=0.6))

ax.set_xlim(lo[0], hi[0])
ax.set_ylim(lo[1], hi[1])
ax.set_aspect('equal')
ax.set_xticks([]); ax.set_yticks([])
for sp in ax.spines.values():
    sp.set_color('#c9c8c2')

# legend, scale bar and map axis directions sit in the empty unmapped area, bottom left
h = [Line2D([], [], color=ROUTE, lw=1.8, ls=(0, (4, 2))), Line2D([], [], color=FLOWN, lw=2.2),
     Line2D([], [], marker='o', ls='', mfc='none', mec=HOLD, mew=1.8, ms=7),
     Line2D([], [], marker='^', ls='', color=FLOWN, mec=INK, ms=8),
     Line2D([], [], marker='o', ls='', mfc='white', mec=INK, mew=1.5, ms=8),
     Line2D([], [], marker='v', ls='', color='#d2402a', mec=INK, ms=8),
     mpatches.Patch(color=OCC), mpatches.Patch(color=BAND), mpatches.Patch(fc=UNKNOWN, ec='#7a7a76')]
lab = ['planned route', 'flown track', 'hold', 'takeoff', 'goal', 'landing', 'obstacle',
       f'{CLEAR:.2f} m clearance', 'unmapped']
leg = ax.legend(h, lab, loc='lower left', ncol=3, frameon=False, fontsize=7.5, handlelength=1.5,
                columnspacing=0.9, labelspacing=0.45, borderaxespad=0.4)

fig.tight_layout()
fig.canvas.draw()
bb = leg.get_window_extent().transformed(ax.transData.inverted())   # legend box in data units
x0, y0 = bb.x0 + 0.15, bb.y1 + 0.35
ax.plot([x0, x0 + 1.0], [y0, y0], color=INK, lw=2)
ax.text(x0 + 0.5, y0 + 0.1, '1 m', ha='center', va='bottom', fontsize=8)
o = np.array([x0 + 1.8, y0 + 0.05])
for vec, txt in ((np.array([1.0, 0.0]), 'x'), (np.array([0.0, 1.0]), 'y')):
    e = R @ vec * 0.55
    ax.annotate('', o + e, o, arrowprops=dict(arrowstyle='->', color='#52514e', lw=1.0))
    ax.text(*(o + e * 1.3), txt, ha='center', va='center', fontsize=8, color='#52514e')
for ext in ('png', 'pdf'):
    fig.savefig(f'{stem}.{ext}', dpi=250, facecolor=SURF, bbox_inches='tight', pad_inches=0.03)
print('wrote', stem, 'rotation %.1f deg, holds %s' % (np.degrees(theta), [(round(a - t_arm), round(b - t_arm)) for a, b in holds]))
