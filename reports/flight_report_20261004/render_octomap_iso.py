"""Isometric cube render of a whole OctoMap with the flown track.

Every occupied voxel is drawn as a cube; only faces not shared with another
occupied voxel are emitted, shaded by face direction and coloured by height.
Input pickle: vox (Nx3 voxel centres, m), res, body ([t, x, y, z] map frame), states.
Usage: python render_octomap_iso.py <pkl> <out.png> <azim_deg> [dpi] [title]
"""
import math
import pickle
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from mpl_toolkits.mplot3d.art3d import Poly3DCollection  # noqa: E402

pkl, out, azim = sys.argv[1], sys.argv[2], float(sys.argv[3])
dpi = int(sys.argv[4]) if len(sys.argv) > 4 else 150
title = sys.argv[5] if len(sys.argv) > 5 else ''
d = pickle.load(open(pkl, 'rb'))
v, res, body = d['vox'], float(d['res']), d['body']
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
ax.set_proj_type('ortho')
pc = Poly3DCollection(polys, facecolors=cols, edgecolors=np.clip(cols * 0.75, 0, 1), linewidths=0.15)
ax.add_collection3d(pc)
# Flown track drawn last and on top so it stays visible through the map.
ax.plot(body[:, 1], body[:, 2], body[:, 3], color='#eb6834', lw=3.0, zorder=10, label='flown track (body pose)')
ax.scatter(*body[0, 1:4], s=70, c=SURF, edgecolors='#eb6834', linewidths=2, zorder=11, depthshade=False, label='takeoff')
ax.scatter(*body[-1, 1:4], s=70, c='#eb6834', edgecolors=SURF, linewidths=1.5, marker='s', zorder=11,
           depthshade=False, label='end of track')
hi = v.max(axis=0) + res
lo3 = v.min(axis=0) - res
ax.set_xlim(lo3[0], hi[0]); ax.set_ylim(lo3[1], hi[1]); ax.set_zlim(lo3[2], hi[2])
ax.set_box_aspect(hi - lo3)
ax.view_init(elev=math.degrees(math.atan(1 / math.sqrt(2))), azim=azim)
for axis in (ax.xaxis, ax.yaxis, ax.zaxis):
    axis.pane.set_visible(False)
    axis.line.set_color('#c9c8c2')
    axis._axinfo['grid']['color'] = '#e4e3df'
    axis._axinfo['grid']['linewidth'] = 0.5
ax.tick_params(colors=INK2, labelsize=7)
ax.set_xlabel('x (m)', color=INK2, fontsize=8); ax.set_ylabel('y (m)', color=INK2, fontsize=8)
ax.set_zlabel('z (m)', color=INK2, fontsize=8)
ax.legend(loc='upper left', fontsize=9, frameon=False)
if title:
    fig.text(0.02, 0.965, title, ha='left', va='top', fontsize=13, fontweight='bold', color=INK)
    fig.text(0.02, 0.93, f'OctoMap at end of flight: {len(v):,} occupied voxels, {res * 100:.0f} cm cubes, '
             f'coloured by height (light = low, dark = high). Isometric orthographic view.',
             ha='left', va='top', fontsize=9, color=INK2)
fig.savefig(out, dpi=dpi, facecolor=SURF)
print('wrote', out, f'({len(polys):,} faces)')
