"""Isometric cube render of a planner3d flight: OctoMap, flown track, first planned route, goal.

Variant of render_octomap_iso.py for pickles from extract_planner3d_iso.py.

Every occupied voxel is drawn as a cube; only faces not shared with another
occupied voxel are emitted, shaded by face direction and coloured by height.
Input pickle: vox (Nx3 voxel centres, m), res, body ([t, x, y, z] map frame), states.
Usage: python render_planner3d_iso.py <pkl> <out.png> <azim_deg> [dpi] [title] [options]
Options for close-ups:
  --box=x0:x1:y0:y1:z0:z1  keep only voxels (and track/route points) inside this box
  --elev=DEG               camera elevation (default isometric, 35.3)
  --persp=F                perspective projection with focal length F (default orthographic)
  --track-box=x0:x1:y0:y1:z0:z1  clip only the track/route to this box (default --box), e.g. to stop the line
                           before it passes behind an obstacle it would otherwise be drawn over
  --gap=x:y0:y1:z          draw a width bar across a gap at x, from y0 to y1, at height z
  --cage-x=X               draw the 0.5 m cage ring around the drone where the track first reaches map x = X
  --report                 larger text for a paper figure; drops the first planned route
  --legend-loc=LOC         matplotlib legend location (default upper left)
  --legend-at=X:Y          anchor the legend at this figure-fraction point
  --legend-size=PT         legend font size (default 16 with --report)
"""
import math
import pickle
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from mpl_toolkits.mplot3d.art3d import Poly3DCollection  # noqa: E402

_opts = dict(a[2:].split('=', 1) for a in sys.argv[1:] if a.startswith('--') and '=' in a)
_args = [a for a in sys.argv[1:] if not a.startswith('--')]
pkl, out, azim = _args[0], _args[1], float(_args[2])
dpi = int(_args[3]) if len(_args) > 3 else 150
title = _args[4] if len(_args) > 4 else ''
BOX = [float(x) for x in _opts['box'].split(':')] if 'box' in _opts else None
ELEV = float(_opts.get('elev', math.degrees(math.atan(1 / math.sqrt(2)))))
GAP = [float(x) for x in _opts['gap'].split(':')] if 'gap' in _opts else None
REPORT = '--report' in sys.argv
CAGE_X = float(_opts['cage-x']) if 'cage-x' in _opts else None
LEG_LOC = _opts.get('legend-loc', 'upper left')                                   # e.g. --legend-loc=upper right
LEG_AT = tuple(float(x) for x in _opts['legend-at'].split(':')) if 'legend-at' in _opts else None  # figure fraction
TBOX = [float(x) for x in _opts['track-box'].split(':')] if 'track-box' in _opts else BOX
d = pickle.load(open(pkl, 'rb'))
v, res, body = d['vox'], float(d['res']), d['body']


def inside(p, box=None):
    box = box or BOX
    if box is None:
        return np.ones(len(p), bool)
    return ((p[:, 0] >= box[0]) & (p[:, 0] <= box[1]) & (p[:, 1] >= box[2]) & (p[:, 1] <= box[3])
            & (p[:, 2] >= box[4]) & (p[:, 2] <= box[5]))


def clipped(p):
    """Points outside the track box become NaN so the line breaks there instead of leaving the axes."""
    p = np.array(p, float)
    if TBOX is not None and len(p) > 1:   # resample to 1 cm so sparse routes clip at the box, not at a vertex
        seg = np.r_[0, np.cumsum(np.linalg.norm(np.diff(p, axis=0), axis=1))]
        s_new = np.arange(0, seg[-1], 0.01)
        p = np.column_stack([np.interp(s_new, seg, p[:, k]) for k in range(3)])
    p[~inside(p, TBOX)] = np.nan
    return p


v = v[inside(v)]
SURF, INK, INK2 = '#fcfcfb', '#0b0b0b', '#52514e'

# integer voxel grid
lo = v.min(axis=0)
idx = np.rint((v - lo) / res).astype(np.int64)
dims = idx.max(axis=0) + 3
occ = np.zeros(dims, bool)
occ[idx[:, 0] + 1, idx[:, 1] + 1, idx[:, 2] + 1] = True

# unit-cube faces: (neighbour offset, 4 corner offsets, shade)
FACES = [
    ((0, 0, 1), [(0, 0, 1), (1, 0, 1), (1, 1, 1), (0, 1, 1)], 1.00),
    ((0, 0, -1), [(0, 0, 0), (0, 1, 0), (1, 1, 0), (1, 0, 0)], 0.45),
    ((1, 0, 0), [(1, 0, 0), (1, 1, 0), (1, 1, 1), (1, 0, 1)], 0.80),
    ((-1, 0, 0), [(0, 0, 0), (0, 0, 1), (0, 1, 1), (0, 1, 0)], 0.80),
    ((0, 1, 0), [(0, 1, 0), (0, 1, 1), (1, 1, 1), (1, 1, 0)], 0.64),
    ((0, -1, 0), [(0, 0, 0), (1, 0, 0), (1, 0, 1), (0, 0, 1)], 0.64),
]
# Height colour: one sequential blue ramp, light (low) -> dark (high).
ramp = matplotlib.colors.LinearSegmentedColormap.from_list(
    'h', ['#dbe8f8', '#8fb4e6', '#2a78d6', '#1c4f8f', '#13335c'])
z = v[:, 2]
zlo, zhi = np.percentile(z, 1), np.percentile(z, 99)
base = ramp(np.clip((z - zlo) / max(zhi - zlo, 1e-6), 0, 1))[:, :3]

polys, cols = [], []
corner0 = lo - res / 2.0
for (dx, dy, dz), corners, shade in FACES:
    nb = occ[idx[:, 0] + 1 + dx, idx[:, 1] + 1 + dy, idx[:, 2] + 1 + dz]
    sel = np.nonzero(~nb)[0]
    cor = np.array(corners, float) * res
    origin = corner0 + idx[sel] * res
    polys.append(origin[:, None, :] + cor[None, :, :])
    cols.append(np.clip(base[sel] * shade, 0, 1))
polys = np.concatenate(polys)
cols = np.concatenate(cols)

fig = plt.figure(figsize=(12, 9), facecolor=SURF)
ax = fig.add_axes([0, 0.02, 1, 0.9], projection='3d', facecolor=SURF)
ax.set_proj_type('persp', focal_length=float(_opts['persp'])) if 'persp' in _opts else ax.set_proj_type('ortho')
ax.computed_zorder = False
pc = Poly3DCollection(polys, facecolors=cols, edgecolors=np.clip(cols * 0.75, 0, 1), linewidths=0.15)
ax.add_collection3d(pc)
# Flown track drawn last and on top so it stays visible through the map.
trk = clipped(body[:, 1:4])
ax.plot(trk[:, 0], trk[:, 1], trk[:, 2], color='#eb6834', lw=3.0, zorder=10, label='flown track')
route = clipped(next(r for _, r in d['routes'] if len(r)))   # skip empty (no-route) messages
if not REPORT:
    ax.plot(route[:, 0], route[:, 1], route[:, 2], color=INK2, lw=1.6, ls=(0, (4, 3)), zorder=9,
            label='first planned route')
if CAGE_X is not None:
    # first track point that reaches x = CAGE_X, whichever way the drone travels in x
    k = int(np.argmax(body[:, 1] >= CAGE_X)) if body[0, 1] < CAGE_X else int(np.argmax(body[:, 1] <= CAGE_X))
    cx, cy, cz = body[k, 1:4]
    a = np.linspace(0, 2 * np.pi, 120)
    ax.plot(cx + 0.25 * np.cos(a), cy + 0.25 * np.sin(a), np.full_like(a, cz), color=INK, lw=2.0, zorder=12,
            label='drone + 0.5 m cage')
    ax.scatter(cx, cy, cz, s=30, c=INK, depthshade=False, zorder=12)
last_route = [r for _, r in d['routes'] if len(r)][-1]
# goal: the clicked point's xy; flights with a launch-arg goal have no click, so use the last route's end
gx, gy = d['goal'][:2] if d['goal'] is not None else last_route[-1, :2]
gz = last_route[-1, 2]
if inside(np.array([[gx, gy, gz]]))[0]:
    if not REPORT:
        ax.plot([gx, gx], [gy, gy], [d['ground_z'], gz], color=INK, lw=0.8, ls=':', zorder=9)
    ax.scatter(gx, gy, gz, s=320, marker='*', c=INK, edgecolors=SURF, linewidths=1.2, zorder=12,
               depthshade=False, label='goal')
if inside(body[:1, 1:4])[0]:
    ax.scatter(*body[0, 1:4], s=70, c=SURF, edgecolors='#eb6834', linewidths=2, zorder=11, depthshade=False,
               label='takeoff')
if GAP:
    gx0, gy0, gy1, gzz = GAP
    ax.plot([gx0, gx0], [gy0, gy1], [gzz, gzz], color=INK, lw=1.2, zorder=13, marker='|', markersize=10)
    ax.text(gx0, (gy0 + gy1) / 2 if REPORT else gy0 + 0.3 * (gy1 - gy0), gzz + 0.06, f'{gy1 - gy0:.2f} m', color=INK, fontsize=16 if REPORT else 10,
            ha='center',
            va='bottom', zorder=14)
if BOX:   # axes cover both boxes so a longer clipped track stays inside them
    lo3 = np.minimum(BOX[0::2], TBOX[0::2])
    hi = np.maximum(BOX[1::2], TBOX[1::2])
else:
    hi = v.max(axis=0) + res
    lo3 = v.min(axis=0) - res
ax.set_xlim(lo3[0], hi[0]); ax.set_ylim(lo3[1], hi[1]); ax.set_zlim(lo3[2], hi[2])
ax.set_box_aspect(hi - lo3)
ax.view_init(elev=ELEV, azim=azim)
for axis in (ax.xaxis, ax.yaxis, ax.zaxis):
    axis.pane.set_visible(False)
    axis.line.set_color('#c9c8c2')
    axis._axinfo['grid']['color'] = '#e4e3df'
    axis._axinfo['grid']['linewidth'] = 0.5
FS = 13 if REPORT else 8
ax.tick_params(colors=INK2, labelsize=FS - 1)
ax.set_xlabel('x (m)', color=INK2, fontsize=FS, labelpad=8 if REPORT else 4)
ax.set_ylabel('y (m)', color=INK2, fontsize=FS, labelpad=8 if REPORT else 4)
ax.set_zlabel('z (m)', color=INK2, fontsize=FS, labelpad=6 if REPORT else 4)
if REPORT:
    for axis in (ax.xaxis, ax.yaxis, ax.zaxis):
        axis.set_major_locator(matplotlib.ticker.MultipleLocator(1.0 if max(hi - lo3) > 3.0 else 0.5))
ax.legend(loc=LEG_LOC, fontsize=float(_opts.get('legend-size', 16 if REPORT else 9)), markerscale=1.4, frameon=True, facecolor='white', edgecolor='#9a9a96',
          framealpha=1.0, fancybox=False, bbox_to_anchor=LEG_AT, bbox_transform=fig.transFigure if LEG_AT else None)
if title:
    fig.text(0.02, 0.965, title, ha='left', va='top', fontsize=13, fontweight='bold', color=INK)
    fig.text(0.02, 0.93, f'OctoMap at end of flight: {len(v):,} occupied voxels, {res * 100:.0f} cm cubes, '
             f'coloured by height (light = low, dark = high). Isometric orthographic view.',
             ha='left', va='top', fontsize=9, color=INK2)
fig.savefig(out, dpi=dpi, facecolor=SURF, bbox_inches='tight', pad_inches=0.15)
print('wrote', out, f'({len(polys):,} faces)')
