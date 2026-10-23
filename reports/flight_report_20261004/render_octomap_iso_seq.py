"""Isometric cube renders of an OctoMap at chosen moments of a flight.

For each requested time: the latest OctoMap at or before it (cubes, exposed faces
only, shaded by face direction, coloured by height), the track flown up to then,
and the drone with its 0.5 m cage ring. All frames share the final map's extents,
colour scale and camera so they can be compared side by side.
Input pickle: maps [(t, bytes, res)], body [[t, x, y, z]], arm (bag s).
Optional --focus=<t0>:<t1>:<radius_m>: voxels within radius (horizontal) of the track
flown between bag times t0 and t1 stay solid; the rest of the map is drawn faded.
--report (with --compact): panel shows only its label as a large heading and larger axis text, for
make_iso_row_report.py to join into a captioned-in-the-report row.
Usage: python render_octomap_iso_seq.py <pkl> <azim> <out_stem> [--focus=...] <t>:<label> [...]
"""
import math
import pickle
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from mpl_toolkits.mplot3d.art3d import Poly3DCollection  # noqa: E402
sys.path.insert(0, '/home/john/autonomous_drone_px4_vio/scripts')
from octomap_binary import decode_binary, expand  # noqa: E402

pkl, azim, stem = sys.argv[1], float(sys.argv[2]), sys.argv[3]
focus = next((a.split('=', 1)[1] for a in sys.argv[4:] if a.startswith('--focus=')), None)
moments = [(float(a.split(':', 1)[0]), a.split(':', 1)[1]) for a in sys.argv[4:] if not a.startswith('--')]
d = pickle.load(open(pkl, 'rb'))
maps, body, arm = d['maps'], d['body'], d['arm']
name = d.get('name', '')
SURF, INK, INK2, TRACK = '#fcfcfb', '#0b0b0b', '#52514e', '#eb6834'
CAGE_R = 0.25


def map_at(t):
    row = max((m for m in maps if m[0] <= t), key=lambda m: m[0])
    c, s = decode_binary(row[1], row[2])
    return expand(c, s, row[2]), float(row[2]), row[0]


final, res, _ = map_at(maps[-1][0])
LO = np.minimum(final.min(axis=0), body[:, 1:4].min(axis=0)) - res
HI = np.maximum(final.max(axis=0), body[:, 1:4].max(axis=0)) + res
ZLO, ZHI = np.percentile(final[:, 2], 1), np.percentile(final[:, 2], 99)
ramp = matplotlib.colors.LinearSegmentedColormap.from_list(
    'h', ['#dbe8f8', '#8fb4e6', '#2a78d6', '#1c4f8f', '#13335c'])
FACES = [
    ((0, 0, 1), [(0, 0, 1), (1, 0, 1), (1, 1, 1), (0, 1, 1)], 1.00),
    ((0, 0, -1), [(0, 0, 0), (0, 1, 0), (1, 1, 0), (1, 0, 0)], 0.45),
    ((1, 0, 0), [(1, 0, 0), (1, 1, 0), (1, 1, 1), (1, 0, 1)], 0.80),
    ((-1, 0, 0), [(0, 0, 0), (0, 0, 1), (0, 1, 1), (0, 1, 0)], 0.80),
    ((0, 1, 0), [(0, 1, 0), (0, 1, 1), (1, 1, 1), (1, 1, 0)], 0.64),
    ((0, -1, 0), [(0, 0, 0), (1, 0, 0), (1, 0, 1), (0, 0, 1)], 0.64),
]


FOCUS_PTS, FOCUS_R = None, 0.0
COMPACT = '--compact' in sys.argv
if focus:
    f0, f1, FOCUS_R = (float(x) for x in focus.split(':'))
    FOCUS_PTS = body[(body[:, 0] >= f0) & (body[:, 0] <= f1), 1:3]
    # Crop the scene to the focus region (plus margin) so the obstacle fills the frame;
    # the whole flown track is always kept inside.
    crop = float(next((a.split('=', 1)[1] for a in sys.argv[4:] if a.startswith('--crop=')), 'nan'))
    if np.isfinite(crop):
        LO[:2] = np.minimum(FOCUS_PTS.min(axis=0) - crop, body[:, 1:3].min(axis=0) - 0.2)
        HI[:2] = np.maximum(FOCUS_PTS.max(axis=0) + crop, body[:, 1:3].max(axis=0) + 0.2)
        inside = final[np.all((final[:, :2] >= LO[:2]) & (final[:, :2] <= HI[:2]), axis=1)]
        if '--no-context' in sys.argv:
            dmin = np.full(len(final), np.inf)
            for q in FOCUS_PTS:
                dmin = np.minimum(dmin, np.hypot(final[:, 0] - q[0], final[:, 1] - q[1]))
            inside = final[dmin <= FOCUS_R]
        pts = np.vstack([inside, body[:, 1:4]])
        LO = pts.min(axis=0) - res
        HI = pts.max(axis=0) + res
        if '--box=obstacle' in sys.argv:
            # Box hugs the obstacle alone; the track and drone are drawn unclipped,
            # so the approach and exit show just outside it.
            LO = inside.min(axis=0) - res
            HI = inside.max(axis=0) + res
        else:
            # keep the whole 0.5 m cage ring inside the box at every track point
            LO[:2] = np.minimum(LO[:2], body[:, 1:3].min(axis=0) - CAGE_R - res)
            HI[:2] = np.maximum(HI[:2], body[:, 1:3].max(axis=0) + CAGE_R + res)
CONTEXT_RGBA = (0.72, 0.72, 0.70, 0.08)


def in_focus(v):
    if FOCUS_PTS is None:
        return np.ones(len(v), bool)
    dmin = np.full(len(v), np.inf)
    for p in FOCUS_PTS:
        dmin = np.minimum(dmin, np.hypot(v[:, 0] - p[0], v[:, 1] - p[1]))
    return dmin <= FOCUS_R


def cube_faces(v):
    lo = v.min(axis=0)
    idx = np.rint((v - lo) / res).astype(np.int64)
    occ = np.zeros(idx.max(axis=0) + 3, bool)
    occ[idx[:, 0] + 1, idx[:, 1] + 1, idx[:, 2] + 1] = True
    base = ramp(np.clip((v[:, 2] - ZLO) / max(ZHI - ZLO, 1e-6), 0, 1))[:, :3]
    hot = in_focus(v)
    polys, cols = [], []
    for (dx, dy, dz), corners, shade in FACES:
        sel = np.nonzero(~occ[idx[:, 0] + 1 + dx, idx[:, 1] + 1 + dy, idx[:, 2] + 1 + dz])[0]
        origin = (lo - res / 2.0) + idx[sel] * res
        polys.append(origin[:, None, :] + (np.array(corners, float) * res)[None, :, :])
        rgba = np.empty((len(sel), 4))
        rgba[:, :3] = np.clip(base[sel] * shade, 0, 1)
        rgba[:, 3] = 1.0
        rgba[~hot[sel]] = CONTEXT_RGBA
        cols.append(rgba)
    return np.concatenate(polys), np.concatenate(cols)


for k, (t, label) in enumerate(moments, start=1):
    v, _, tmap = map_at(t)
    v = v[np.all((v >= LO) & (v <= HI), axis=1)]
    if '--no-context' in sys.argv:
        v = v[in_focus(v)]
    polys, cols = cube_faces(v)
    fig = plt.figure(figsize=(9, 9) if COMPACT else (12, 9), facecolor=SURF)
    ax = fig.add_axes([0.14, 0.14, 0.72, 0.72] if COMPACT else [0, 0.02, 1, 0.88], projection='3d', facecolor=SURF)
    ax.set_proj_type('ortho')
    edges = cols.copy()
    edges[:, :3] *= 0.75
    edges[cols[:, 3] < 1, 3] = 0.0  # no outlines on the faded context
    ax.add_collection3d(Poly3DCollection(polys, facecolors=cols, edgecolors=edges, linewidths=0.15))
    if FOCUS_PTS is not None:
        ax.plot([], [], 's', ms=9, color='#2a78d6', label=f'obstacle around the gap (within {FOCUS_R:.1f} m)')
        ax.plot([], [], 's', ms=9, color=CONTEXT_RGBA[:3], alpha=0.5, label='rest of the map (faded)')
    tr = body[body[:, 0] <= t]
    p = tr[-1, 1:4]
    ax.plot(tr[:, 1], tr[:, 2], tr[:, 3], color=TRACK, lw=3.0, zorder=10, label='flown track so far', clip_on=False)
    ax.scatter(*tr[0, 1:4], s=70, c=SURF, edgecolors=TRACK, linewidths=2, zorder=11, depthshade=False,
               label='takeoff', clip_on=False)
    ang = np.linspace(0, 2 * np.pi, 60)
    ax.plot(p[0] + CAGE_R * np.cos(ang), p[1] + CAGE_R * np.sin(ang), np.full_like(ang, p[2]),
            color=INK, lw=1.8, zorder=12, label='drone + 0.5 m cage', clip_on=False)
    ax.scatter(*p, s=40, c=INK, zorder=13, depthshade=False, clip_on=False)
    ax.set_xlim(LO[0], HI[0]); ax.set_ylim(LO[1], HI[1]); ax.set_zlim(LO[2], HI[2])
    ax.set_box_aspect(HI - LO, zoom=float(next((a.split('=', 1)[1] for a in sys.argv if a.startswith('--zoom=')), 1.45)))
    ax.view_init(elev=math.degrees(math.atan(1 / math.sqrt(2))), azim=azim)
    for axis in (ax.xaxis, ax.yaxis, ax.zaxis):
        axis.pane.set_visible(False)
        axis.line.set_color('#c9c8c2')
        axis._axinfo['grid']['color'] = '#e4e3df'
        axis._axinfo['grid']['linewidth'] = 0.5
    ax.tick_params(colors=INK2, labelsize=7)
    ax.set_xlabel('x (m)', color=INK2, fontsize=8); ax.set_ylabel('y (m)', color=INK2, fontsize=8)
    ax.set_zlabel('z (m)', color=INK2, fontsize=8)
    if COMPACT:
        if '--report' in sys.argv:
            # No axis numbers: unreadable at a third of a column; the 0.5 m cage ring gives the scale.
            fig.text(0.5, 0.965, label, ha='center', va='top', fontsize=34, color=INK)
            ax.set_xticklabels([]); ax.set_yticklabels([]); ax.set_zticklabels([])
            ax.set_xlabel(''); ax.set_ylabel(''); ax.set_zlabel('')
            ax.tick_params(length=0)
        else:
            fig.text(0.03, 0.975, f'{k}  {label}', ha='left', va='top', fontsize=20, fontweight='bold', color=INK)
            fig.text(0.03, 0.93, f't = {t - arm:.1f} s after arming', ha='left', va='top', fontsize=14, color=INK2)
            ax.tick_params(labelsize=10)
            for lab in (ax.xaxis.label, ax.yaxis.label, ax.zaxis.label):
                lab.set_fontsize(11)
        out = f'{stem}_{k}.png'
        # Fixed canvas (no tight bbox) so every panel has identical pixel geometry and
        # make_iso_composite.py can crop all of them with one rectangle.
        fig.savefig(out, dpi=int(d.get('dpi', 200)), facecolor=SURF)
        plt.close(fig)
        print('wrote', out, f'map at {tmap:.1f} s, {len(v):,} voxels')
        continue
    ax.legend(loc='upper left', bbox_to_anchor=(0.02, 0.98), fontsize=9, frameon=False)
    fig.text(0.02, 0.975, f'{k}/{len(moments)}  {label}', ha='left', va='top', fontsize=14,
             fontweight='bold', color=INK)
    fig.text(0.02, 0.937, f'{name}  ·  t = {t - arm:.1f} s after arming  ·  OctoMap as it was then: '
             f'{len(v):,} occupied voxels, {res * 100:.0f} cm cubes, coloured by height (light = low, dark = high)',
             ha='left', va='top', fontsize=9, color=INK2)
    out = f'{stem}_{k}.png'
    fig.savefig(out, dpi=int(d.get('dpi', 200)), facecolor=SURF, bbox_inches='tight', pad_inches=0.15)
    plt.close(fig)
    print('wrote', out, f'map at {tmap:.1f} s, {len(v):,} voxels, {len(polys):,} faces')
