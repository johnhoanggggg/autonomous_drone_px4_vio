"""Two-panel report figure: the 044126Z doorway open in the map, then closed by map noise.

Input pickle (from the bag): grids [(t, G, res, ox, oy)], paths [(t, Nx2)], body [t, x, y].
Usage: python3 plot_door_closing.py <pkl> <out_stem> <t_open> <t_closed>
"""
import pickle
import sys

import numpy as np
from scipy.ndimage import distance_transform_edt
sys.path.append('/home/john/autonomous_drone_px4_vio/.venv-mavlink/lib/python3.12/site-packages')
import matplotlib  # noqa: E402
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import matplotlib.patches as mpatches  # noqa: E402

pkl, stem, t_open, t_closed = sys.argv[1], sys.argv[2], float(sys.argv[3]), float(sys.argv[4])
d = pickle.load(open(pkl, 'rb'))
SURF, INK, INK2 = '#fcfcfb', '#0b0b0b', '#52514e'
FREE, UNKNOWN, OCC, BAND = (0.957, 0.953, 0.937), (0.82, 0.82, 0.80), (0.12, 0.16, 0.22), (0.97, 0.84, 0.66)
ROUTE, FLOWN, CLEAR = '#eb6834', '#1a9e55', 0.25
X0, X1, Y0, Y1 = 4.3, 7.5, -10.4, -7.6
plt.rcParams.update({'font.size': 15, 'font.family': ['DejaVu Sans']})


def rgb(G, res):
    occ = G >= 65
    out = np.empty(G.shape + (3,))
    out[:] = FREE
    out[G < 0] = UNKNOWN
    dist = distance_transform_edt(~occ) * res
    out[(dist <= CLEAR) & ~occ & (G >= 0)] = BAND
    out[occ] = OCC
    return out


fig, axs = plt.subplots(1, 2, figsize=(12, 6.6), facecolor=SURF)
for ax, tt, lab in zip(axs, (t_open, t_closed), ('(a)', '(b)')):
    t, G, res, ox, oy = [g for g in d['grids'] if g[0] <= tt][-1]
    ax.imshow(rgb(G, res), origin='lower', extent=(ox, ox + G.shape[1] * res, oy, oy + G.shape[0] * res),
              interpolation='nearest')
    p = [q for q in d['paths'] if q[0] <= tt]
    if p and len(p[-1][1]) > 1:
        ax.plot(p[-1][1][:, 0], p[-1][1][:, 1], color=ROUTE, lw=2.6, ls=(0, (4, 2)), label='planned route')
    b = d['body'][d['body'][:, 0] <= tt]
    ax.plot(b[:, 1], b[:, 2], color=FLOWN, lw=3.2, label='flown track')
    ax.plot(b[-1, 1], b[-1, 2], 'o', color=INK, ms=9, label='drone')
    ax.set_xlim(X0, X1); ax.set_ylim(Y0, Y1); ax.set_aspect('equal')
    ax.set_title(f'{lab}  t = {t - d["t_arm"]:.0f} s after arming', loc='left', fontsize=16, color=INK)
    ax.set_xlabel('x (m)', color=INK2)
    ax.tick_params(colors=INK2, labelsize=13)
    for sp in ax.spines.values():
        sp.set_color('#c9c8c2')
axs[0].set_ylabel('y (m)', color=INK2)
h, l = axs[0].get_legend_handles_labels()
h += [mpatches.Patch(color=OCC), mpatches.Patch(color=BAND), mpatches.Patch(color=UNKNOWN, ec='#9a9a96')]
l += ['obstacle', '%.2f m clearance' % CLEAR, 'unmapped']
fig.legend(h, l, loc='lower center', bbox_to_anchor=(0.5, 0.02), ncol=6, fontsize=13, frameon=False, handlelength=1.6, columnspacing=1.2)
fig.tight_layout(rect=(0, 0.06, 1, 1))
for ext in ('png', 'pdf'):
    fig.savefig(f'{stem}.{ext}', dpi=200, facecolor=SURF, bbox_inches='tight', pad_inches=0.1)
print('wrote', stem + '.png/.pdf')
