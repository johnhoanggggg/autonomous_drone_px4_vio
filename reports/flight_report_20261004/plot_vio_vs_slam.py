"""Illustrate VIO pose vs SLAM pose on a real flight.

Input pickle (built from the bag): vio / slam / corr = [(t, [x,y,z], yaw_deg)], arm, dis,
final grid (int8, map frame), origin, res. SLAM = correction x VIO was verified exactly on
this bag before plotting. Usage: python plot_vio_vs_slam.py <pkl> <out.png>
"""
import pickle
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch  # noqa: E402

d = pickle.load(open(sys.argv[1], 'rb'))
out = sys.argv[2]
SURF, INK, INK2, MUTED, GRID = '#fcfcfb', '#0b0b0b', '#52514e', '#8a8984', '#e4e3df'
SLAM_C, VIO_C, CORR_C = '#2a78d6', '#eb6834', '#8a8984'
plt.rcParams.update({'font.size': 9, 'axes.edgecolor': GRID, 'axes.labelcolor': INK2,
                     'xtick.color': INK2, 'ytick.color': INK2, 'axes.titlelocation': 'left',
                     'axes.titleweight': 'bold', 'axes.titlesize': 10, 'axes.titlecolor': INK,
                     'axes.spines.top': False, 'axes.spines.right': False})

arm, dis = d['arm'], d['dis']


def arr(key):
    t = np.array([s[0] for s in d[key]])
    xyz = np.array([s[1] for s in d[key]])
    yaw = np.array([s[2] for s in d[key]])
    m = (t >= arm) & (t <= dis)
    return t[m] - arm, xyz[m], yaw[m]


tv, v, _ = arr('vio')
ts, s, _ = arr('slam')
tc, c, cyaw = arr('corr')

fig = plt.figure(figsize=(14, 9.2), facecolor=SURF)
gs = fig.add_gridspec(3, 2, height_ratios=[0.9, 1.6, 1.6], width_ratios=[1.55, 1.0],
                      left=0.04, right=0.98, top=0.93, bottom=0.07, hspace=0.42, wspace=0.12)
fig.suptitle('VIO pose vs SLAM pose — flight 080337Z', x=0.04, ha='left', fontsize=14,
             fontweight='bold', color=INK)

# ---------------------------------------------------------------- (a) schematic
ax = fig.add_subplot(gs[0, :])
ax.set_xlim(0, 100)
ax.set_ylim(0, 22)
ax.axis('off')
ax.set_title('(a) How the two poses are produced', pad=4)


def box(x, y, w, h, text, fc, ec):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle='round,pad=0.4,rounding_size=1.2',
                                fc=fc, ec=ec, lw=1.3))
    ax.text(x + w / 2, y + h / 2, text, ha='center', va='center', fontsize=8.6, color=INK)


def arrow(x0, y0, x1, y1, text='', color=INK2, dy=1.2):
    ax.add_patch(FancyArrowPatch((x0, y0), (x1, y1), arrowstyle='-|>', mutation_scale=11,
                                 lw=1.3, color=color))
    if text:
        ax.text((x0 + x1) / 2, (y0 + y1) / 2 + dy, text, ha='center', va='bottom', fontsize=8,
                color=color)


box(0.5, 7, 11, 8, 'OAK-D Lite\nstereo images\n+ IMU', '#eeeeea', MUTED)
box(19, 12.5, 18, 8, 'Visual-inertial odometry (VIO)\nframe-to-frame motion of\ntracked features + IMU',
    '#fde6dc', VIO_C)
box(19, 0.5, 18, 8, 'RTAB-Map SLAM\nkeyframe map, place\nrecognition, loop closures',
    '#dbe8f8', SLAM_C)
box(46, 0.5, 17, 8, 'map correction C\n(map ← odom)\nupdated at each closure', '#eeeeea', MUTED)
box(72, 12.5, 15, 8, 'PX4 EKF2\nflight control', '#fde6dc', VIO_C)
box(72, 0.5, 26, 8, 'SLAM pose = C × VIO pose\nplanner, follower, map, goals',
    '#dbe8f8', SLAM_C)
arrow(11.9, 12.5, 18.6, 16.5)
arrow(11.9, 9.5, 18.6, 4.5)
arrow(37.4, 16.5, 71.6, 16.5, 'VIO pose  (odom frame): smooth, ~14 Hz, drifts', VIO_C)
arrow(28, 12.1, 28, 8.9, '', VIO_C)
arrow(37.4, 4.5, 45.6, 4.5)
arrow(63.4, 4.5, 71.6, 4.5)

# ---------------------------------------------------------------- (b) map view
ax = fig.add_subplot(gs[1:, 0])
grid = d['grid']
res = d['res']
ox, oy = d['origin']
img = np.full(grid.shape + (3,), (0.82, 0.82, 0.80))
img[grid == 0] = (0.957, 0.953, 0.937)
img[grid > 50] = (0.12, 0.16, 0.22)
h, w = grid.shape
ax.imshow(img, origin='lower', extent=(ox, ox + w * res, oy, oy + h * res), interpolation='nearest')
ax.plot(s[:, 0], s[:, 1], color=SLAM_C, lw=2.2, label='SLAM pose (map frame) — lines up with the map')
ax.plot(v[:, 0], v[:, 1], color=VIO_C, lw=2.0, ls='--', label='VIO pose (odom frame) — same flight, drifted frame')
for k in np.linspace(0, len(ts) - 1, 9).astype(int):
    j = np.argmin(abs(tv - ts[k]))
    ax.annotate('', xy=(s[k, 0], s[k, 1]), xytext=(v[j, 0], v[j, 1]),
                arrowprops=dict(arrowstyle='-|>', color=CORR_C, lw=0.9, mutation_scale=9))
for pts, col in ((s, SLAM_C), (v, VIO_C)):
    ax.plot(*pts[0, :2], 'o', ms=8, mfc=SURF, mec=col, mew=2)
    ax.plot(*pts[-1, :2], 's', ms=7, mfc=col, mec=SURF, mew=1.5)
allx = np.concatenate([s[:, 0], v[:, 0]])
ally = np.concatenate([s[:, 1], v[:, 1]])
pad = 0.6
ax.set_xlim(allx.min() - pad, allx.max() + pad)
ax.set_ylim(ally.min() - pad, ally.max() + pad)
ax.set_aspect('equal')
ax.set_xlabel('x (m)')
ax.set_ylabel('y (m)')
ax.set_title('(b) The same flight in both frames, drawn on the SLAM map')
ax.plot([], [], color=CORR_C, lw=0.9, label='grey arrows: the correction C at that moment')
ax.plot([], [], 'o', ms=7, mfc=SURF, mec=INK2, mew=1.5, ls='', label='takeoff')
ax.plot([], [], 's', ms=6, mfc=INK2, mec=SURF, ls='', label='landing')
ax.legend(loc='lower left', fontsize=8, frameon=True, facecolor=SURF, edgecolor=GRID)

# ---------------------------------------------------------------- (c) correction over time
mag = np.hypot(c[:, 0], c[:, 1])
ax1 = fig.add_subplot(gs[1, 1])
ax1.plot(tc, mag, color=INK2, lw=1.6, drawstyle='steps-post')
ax1.set_ylabel('m')
ax1.set_title('(c) Size of the correction C (horizontal)')
ax1.grid(True, color=GRID, lw=0.6)
ax2 = fig.add_subplot(gs[2, 1], sharex=ax1)
ax2.plot(tc, cyaw, color=INK2, lw=1.6, drawstyle='steps-post')
ax2.set_ylabel('deg')
ax2.set_xlabel('time since arming (s)')
ax2.set_title('(d) Heading part of the correction')
ax2.grid(True, color=GRID, lw=0.6)
for a in (ax1, ax2):
    a.set_facecolor(SURF)
ax1.text(0.98, 0.04, 'each step = a loop closure re-aligning VIO to the map',
         transform=ax1.transAxes, ha='right', va='bottom', fontsize=8, color=INK2,
         bbox=dict(facecolor=SURF, edgecolor='none', pad=1.5))
fig.savefig(out, dpi=150, facecolor=SURF)
print('wrote', out, f'| correction {mag.min():.2f}-{mag.max():.2f} m, yaw {cyaw.min():.1f}..{cyaw.max():.1f} deg, '
      f'{int((np.abs(np.diff(mag)) > 0.02).sum())} steps > 2 cm')
