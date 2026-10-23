"""Top-view map of a 2D (global planner) flight for the paper, legend inside the plot.

Final occupancy grid with a 0.25 m clearance band, first planned route, flown track with
time ticks every 10 s after arming, holds, takeoff, goal, landing and an optional door
label. The legend and a 1 m scale bar are drawn
inside the plot (on the unmapped area), so the map can use the whole figure. The map can
be rotated so a long route fills a wide panel.

Input pickle (extract_global_path.py): grids, paths, body (t, x, y), t_arm, states.
Usage: python plot_path_map.py <in.pkl> <out_stem> [--angle auto|DEG] [--door X,Y]
       [--door-text-offset DX,DY] [--land S_AFTER_ARM] [--states states.pkl]
       [--legend-loc 'lower left'] [--legend-ncol 3] [--figsize W,H] [--pad PX,PY]
"""
import argparse
import pickle

import numpy as np
from scipy.ndimage import distance_transform_edt
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import matplotlib.patches as mpatches  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument('pkl')
ap.add_argument('stem')
ap.add_argument('--angle', default='auto', help="'auto' lays the route horizontally; a number in degrees")
ap.add_argument('--door', default=None, help='door centre X,Y in the map frame (m)')
ap.add_argument('--door-text-offset', default='0,-22', help='door label offset in points')
ap.add_argument('--land', type=float, default=None, help='landing time, s after arming')
ap.add_argument('--states', default=None, help='separate states pickle (older extractions)')
ap.add_argument('--legend-loc', default='lower left')
ap.add_argument('--legend-ncol', type=int, default=3)
ap.add_argument('--scale-anchor', default=None, help='X,Y in axes fraction for the left end of the scale bar')
ap.add_argument('--legend-anchor', default=None, help='X,Y in axes fraction for the legend corner given by --legend-loc')
ap.add_argument('--figsize', default='auto', help="W,H in inches, or 'auto' (7 in wide, map aspect)")
ap.add_argument('--pad', default='0.6,0.9', help='map margin around route and track (m): PX,PY or X0,X1,Y0,Y1')
a = ap.parse_args()

d = pickle.load(open(a.pkl, 'rb'))
states = pickle.load(open(a.states, 'rb')) if a.states else d['states']
t_arm = d['t_arm']

SURF, INK = '#fcfcfb', '#0b0b0b'
FREE, UNKNOWN, OCC, BAND = (0.957, 0.953, 0.937), (0.82, 0.82, 0.80), (0.12, 0.16, 0.22), (0.97, 0.84, 0.66)
ROUTE, FLOWN, HOLD, CLEAR = '#eb6834', '#1a9e55', '#d4890f', 0.25

body = d['body']
t_land = t_arm + a.land if a.land is not None else body[-1, 0]
body = body[body[:, 0] <= t_land]
jump = np.where(np.linalg.norm(np.diff(body[:, 1:3], axis=0), axis=1) > 0.5)[0]
if len(jump):                                           # VIO reset at touchdown snaps the pose back
    body = body[:jump[0] + 1]
t_land = body[-1, 0]
_, G, res, ox, oy = [g for g in d['grids'] if g[0] <= t_land + 2.0][-1]
route = [p for p in d['paths'] if len(p[1]) > 1][0][1]  # first planned route
goal = route[-1]

holds = []                                              # adapter HOLD / COMMAND_HOLD while flying
for (t, s), nxt in zip(states, states[1:] + [(t_land, 'END')]):
    if s in ('HOLD', 'COMMAND_HOLD') and t_arm < t < t_land - 1.0:
        if holds and t - holds[-1][1] < 0.5:            # COMMAND_HOLD then HOLD = one hold
            holds[-1] = (holds[-1][0], nxt[0])
        else:
            holds.append((t, nxt[0]))
hold_pts = [body[np.argmin(abs(body[:, 0] - (h0 + h1) / 2)), 1:3] for h0, h1 in holds]

pts = np.vstack([body[:, 1:3], route])
c = pts.mean(axis=0)
if a.angle == 'auto':                                   # principal axis horizontal, takeoff on the left
    w, v = np.linalg.eigh(np.cov((pts - c).T))
    ax_dir = v[:, np.argmax(w)]
    if np.dot(goal - body[0, 1:3], ax_dir) < 0:
        ax_dir = -ax_dir
    theta = -np.arctan2(ax_dir[1], ax_dir[0])
else:
    theta = np.radians(float(a.angle))
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
pad = [float(v) for v in a.pad.split(',')]
if len(pad) == 2:
    pad = [pad[0], pad[0], pad[1], pad[1]]
lo = np.vstack([r, b]).min(axis=0) - [pad[0], pad[2]]
hi = np.vstack([r, b]).max(axis=0) + [pad[1], pad[3]]

step = res / 2                                          # resample grid onto the plot frame (nearest cell)
us, vs = np.arange(lo[0], hi[0], step), np.arange(lo[1], hi[1], step)
U, V = np.meshgrid(us, vs)
XY = np.stack([U, V], -1) @ R + c
ci = np.floor((XY[..., 0] - ox) / res).astype(int)
ri = np.floor((XY[..., 1] - oy) / res).astype(int)
ok = (ci >= 0) & (ci < G.shape[1]) & (ri >= 0) & (ri < G.shape[0])
rimg = np.empty(U.shape + (3,))
rimg[:] = UNKNOWN
rimg[ok] = img[ri[ok], ci[ok]]

if a.figsize == 'auto':
    figsize = (7.0, 7.0 * (hi[1] - lo[1]) / (hi[0] - lo[0]))
else:
    figsize = tuple(float(v) for v in a.figsize.split(','))
fig, ax = plt.subplots(figsize=figsize, facecolor=SURF)
ax.imshow(rimg, origin='lower', extent=(lo[0], lo[0] + len(us) * step, lo[1], lo[1] + len(vs) * step),
          interpolation='nearest')
ax.plot(r[:, 0], r[:, 1], color=ROUTE, lw=1.8, ls=(0, (4, 2)))
ax.plot(b[:, 0], b[:, 1], color=FLOWN, lw=2.2)
for k in range(6, int(t_land - t_arm), 10):            # time ticks every 10 s after arming
    p = b[np.argmin(abs(body[:, 0] - (t_arm + k)))]
    ax.plot(*p, 'o', color=INK, ms=2.5)
    ax.annotate(f'{k} s', p, xytext=(0, 6), textcoords='offset points', ha='center', fontsize=10, color=INK)
for hp in hold_pts:
    ax.plot(*rot(hp)[0], 'o', mfc='none', mec=HOLD, mew=1.8, ms=7)
ax.plot(*b[0], '^', color=FLOWN, mec=INK, mew=0.8, ms=8)
ax.plot(*rot(goal)[0], 'o', mfc='white', mec=INK, mew=1.5, ms=8)
ax.plot(*b[-1], 'v', color='#d2402a', mec=INK, mew=0.8, ms=8)
if a.door:
    dp = rot([float(v) for v in a.door.split(',')])[0]
    ax.annotate('door', dp, xytext=tuple(float(v) for v in a.door_text_offset.split(',')),
                textcoords='offset points', ha='center', fontsize=11,
                arrowprops=dict(arrowstyle='->', color=INK, lw=0.9),
                bbox=dict(boxstyle='round,pad=0.2', fc='white', ec=INK, lw=0.6))

ax.set_xlim(lo[0], hi[0])
ax.set_ylim(lo[1], hi[1])
ax.set_aspect('equal')
ax.set_xticks([]); ax.set_yticks([])
for sp in ax.spines.values():
    sp.set_color('#c9c8c2')

h = [Line2D([], [], color=ROUTE, lw=1.8, ls=(0, (4, 2))), Line2D([], [], color=FLOWN, lw=2.2),
     Line2D([], [], marker='o', ls='', mfc='none', mec=HOLD, mew=1.8, ms=7),
     Line2D([], [], marker='^', ls='', color=FLOWN, mec=INK, ms=8),
     Line2D([], [], marker='o', ls='', mfc='white', mec=INK, mew=1.5, ms=8),
     Line2D([], [], marker='v', ls='', color='#d2402a', mec=INK, ms=8),
     mpatches.Patch(color=OCC), mpatches.Patch(color=BAND), mpatches.Patch(fc=UNKNOWN, ec='#7a7a76')]
lab = ['planned route', 'flown track', 'hold', 'takeoff', 'goal', 'landing', 'obstacle',
       f'{CLEAR:.2f} m clearance', 'unmapped']
leg = ax.legend(h, lab, loc=a.legend_loc, ncol=a.legend_ncol, frameon=True, fontsize=12,
                facecolor='white', edgecolor='#9a9a96', framealpha=1.0, borderpad=0.4, fancybox=False,
                handlelength=1.5, columnspacing=0.9, labelspacing=0.35, borderaxespad=0.4,
                bbox_to_anchor=tuple(float(v) for v in a.legend_anchor.split(',')) if a.legend_anchor else None)

fig.tight_layout()
fig.canvas.draw()
bb = leg.get_window_extent().transformed(ax.transData.inverted())   # legend box in data units
x0 = bb.x0 + 0.15 if 'left' in a.legend_loc else bb.x1 - 2.6
if a.scale_anchor:
    fx, fy = (float(v) for v in a.scale_anchor.split(','))
    x0, y0 = lo[0] + fx * (hi[0] - lo[0]), lo[1] + fy * (hi[1] - lo[1])
else:
    y0 = bb.y1 + 0.35 if 'lower' in a.legend_loc else bb.y0 - 0.75  # scale bar on the open side
ax.plot([x0, x0 + 1.0], [y0, y0], color=INK, lw=2)
ax.text(x0 + 0.5, y0 + 0.1, '1 m', ha='center', va='bottom', fontsize=11)
for ext in ('png', 'pdf'):
    fig.savefig(f'{a.stem}.{ext}', dpi=250, facecolor=SURF, bbox_inches='tight', pad_inches=0.03)
print('wrote', a.stem, 'rotation %.1f deg, holds %s, flight %.0f s'
      % (np.degrees(theta), [(round(h0 - t_arm), round(h1 - t_arm)) for h0, h1 in holds], t_land - t_arm))
