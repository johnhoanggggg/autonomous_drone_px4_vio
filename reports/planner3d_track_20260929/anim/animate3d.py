"""Clean 3D replay of a planner3d flight (like the Foxglove 3D panel), as an MP4 (H.264, QuickTime-compatible) or a GIF, by the output extension.

Scene per frame: the OctoMap as it was at that moment (voxels coloured by
height, floor band faint), the planner's current route, the command point, the
drone as a small quad with its cage ring, heading tick and floor shadow, a
fading trail, the goal, and a status readout. The camera sits behind and above
the drone and turns with its heading (yaw smoothed over ~0.4 s, map clipped to a
disc so the rotating view has no box corners). --fixed-cam keeps the old view:
fixed behind the overall direction of travel, following the drone in xy.
Default look follows the iwrzwr/visual-archive design language (kagan.in): black
page, off-white ink, muted grey, 0.5 px hairlines at 12% white, light lowercase
sans, greyscale map (brighter = higher) and a single red accent for the route.
--light restores the earlier light look.
--drafting --camera-feat: 1440x1080, square-cropped left camera with per-frame Shi-Tomasi corners (detect_features.py,
NOT the tracker that flew) as hollow orange handles inside a construction frame, chase view on the right.
--drafting: light-grey drafting-sheet look (from a drone-racing teaser the user liked): grey paper, halftone floor
dots, greyscale voxel blocks with dark edges, one orange-red accent drawn as design-tool construction frames
(square handles, diagonals, dimension ticks) around the drone, goal and title block, black uppercase grotesk type.
--camera-bg lays the scene over the left camera feed (/oak/left/image_rect/compressed, dimmed, between
the footer rule and the state lines). It is a layered composite, not a projection: the 3D scene does not line
up with the image (render_ar.py does the aligned version). Use fps 10 to match the camera.
--view=F zooms out by F (default 1): F x wider area and map crop, voxel markers shrink to match.
--at=t1,t2 writes stills <out>_<t>.png at those bag times instead of the video.
Usage (ROS sourced, for bag reading): python3 animate3d.py <bag_dir> <out.mp4|out.gif> [fps] [--fixed-cam] [--light|--drafting [--camera-feat]] [--camera-bg] [--view=F]
MP4 uses the ffmpeg bundled with imageio-ffmpeg in .venv-mavlink (no system ffmpeg needed).
"""
import json
import math
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

sys.path.append('/home/john/autonomous_drone_px4_vio/.venv-mavlink/lib/python3.12/site-packages')
sys.path.insert(0, '/home/john/autonomous_drone_px4_vio/scripts')
import matplotlib  # noqa: E402
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.animation import FuncAnimation, PillowWriter  # noqa: E402
from octomap_binary import decode_binary, expand  # noqa: E402

FIXED_CAM = '--fixed-cam' in sys.argv
LIGHT = '--light' in sys.argv
DRAFT = '--drafting' in sys.argv
CAM_FEAT = '--camera-feat' in sys.argv
if CAM_FEAT and not DRAFT:
    sys.exit('--camera-feat is only laid out for --drafting')
CAM_TOPIC = '/oak/left/image_rect/compressed'
CAMERA_BG = '--camera-bg' in sys.argv
_opts = dict(a[2:].split('=', 1) for a in sys.argv[1:] if a.startswith('--') and '=' in a)
VIEW = float(_opts.get('view', 1.0))
STILLS = [float(x) for x in _opts['at'].split(',')] if 'at' in _opts else None
_args = [a for a in sys.argv[1:] if not a.startswith('--')]
bag, out = _args[0], _args[1]
FPS = float(_args[2]) if len(_args) > 2 else 5.0
if DRAFT:
    SURF, INK, INK2, MUTED = '#d8d8d6', '#0b0b0b', '#2e2e2e', '#5e5e5c'
    ROUTE, CMD, HOLD, GOAL = '#ff4b1f', '#0b0b0b', '#ff4b1f', '#0b0b0b'
    T = dict(grid=('#a9a9a7', 0.5, 0.8), ramp=['#3a3a3a', '#8c8c8c', '#f2f2f0'], floor=('#bdbdbb', 0.8),
             shadow='#9c9c9a', map_key=('map, lighter = higher', '#8c8c8c'))
    plt.rcParams['font.family'] = ['Nimbus Sans', 'Liberation Sans', 'DejaVu Sans']
elif LIGHT:
    SURF, INK, INK2, MUTED = '#fcfcfb', '#0b0b0b', '#52514e', '#8a8984'
    ROUTE, CMD, HOLD, GOAL = '#eb6834', '#2a78d6', '#d98a00', '#0b0b0b'
    T = dict(grid=('#e3e2dc', 0.7, 1.0), ramp=['#c9d6e8', '#5b7aa6', '#1f3350'], floor=('#dcdbd4', 0.45),
             shadow='#cfcec7', map_key=('map (darker = higher)', '#5b7aa6'))
else:
    SURF, INK, INK2, MUTED = '#000000', '#f7f7f4', '#b6b6b6', '#777671'
    ROUTE, CMD, HOLD, GOAL = '#ff0000', '#339cff', '#ff963e', '#f7f7f4'
    T = dict(grid=('#ffffff', 0.5, 0.12), ramp=['#525252', '#b6b6b6', '#f7f7f4'], floor=('#303030', 0.7),
             shadow='#303030', map_key=('map, brighter = higher', '#b6b6b6'))
    plt.rcParams['font.family'] = ['Liberation Sans', 'DejaVu Sans']
HAIR = dict(color='#ffffff', alpha=0.12, lw=0.5)
CAGE_R, ARM = 0.25, 0.17

# ---------------------------------------------------------------- read the bag
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
types = {t.name: t.type for t in r.get_all_topics_and_types()}
want = ['/rtabmap/octomap', '/planner3d/flight/status', '/planner3d/reference_path', '/rtabmap/vio_pose',
        '/waypoint/clicked'] + ([CAM_TOPIC] if CAMERA_BG or CAM_FEAT else [])
r.set_filter(rosbag2_py.StorageFilter(topics=[w for w in want if w in types]))
t0 = r.get_metadata().starting_time.nanoseconds
maps, status, routes, vio, goals, cams = [], [], [], [], [], []
while r.has_next():
    tp, d, ns = r.read_next()
    t = (ns - t0) / 1e9
    m = deserialize_message(d, get_message(types[tp]))
    if tp == '/rtabmap/octomap':
        maps.append((t, bytes(m.data), m.resolution))
    elif tp == '/planner3d/flight/status':
        try:
            status.append((t, json.loads(m.data)))
        except ValueError:
            pass
    elif tp == '/planner3d/reference_path':
        routes.append((t, np.array([[p.pose.position.x, p.pose.position.y, p.pose.position.z] for p in m.poses])))
    elif tp == '/rtabmap/vio_pose':
        q = m.pose.orientation if hasattr(m.pose, 'orientation') else m.pose.pose.orientation
        vio.append((t, (q.x, q.y, q.z, q.w)))
    elif tp == '/waypoint/clicked':
        goals.append((t, np.array([m.point.x, m.point.y, m.point.z])))
    elif tp == '/oak/left/image_rect/compressed':
        cams.append((t, bytes(m.data)))
cams.sort(key=lambda c: c[0])


def latest(rows, t):
    best = None
    for row in rows:
        if row[0] > t:
            break
        best = row
    return best


def qmat(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


t_start = next(t for t, v in status if v.get('state') == 'CLIMB') - 1.0
t_stop = max(t for t, v in status if v.get('body_map')) + 0.2
body = np.array([[t, *v['body_map']] for t, v in status if v.get('body_map')])
# POSE_STALE status messages carry no body_map / map_from_odom; draw the last known pose through them
# (falling back to body[0] snapped the drone to the takeoff point for a frame).
posed = [(t, v) for t, v in status if v.get('body_map')]
ground_z = next((v['ground_map_z'] for t, v in status if v.get('ground_map_z') is not None), body[0, 3])
frames = np.arange(t_start, t_stop, 1.0 / FPS)

# scene box around the flown corridor
xs, ys = body[:, 1], body[:, 2]
X0, X1 = xs.min() - 0.8 * VIEW, xs.max() + 0.9 * VIEW
Y0, Y1 = ys.min() - 1.1 * VIEW, ys.max() + 1.1 * VIEW
# The view grows with the flight: a 2 m climb must stay in frame.
Z0, Z1 = ground_z - 0.05, max(ground_z + 1.1, body[:, 3].max() + 0.5)
ZSPAN = Z1 - Z0
# Chase from behind the overall direction of travel (was fixed to +x).
travel = body[-1, 1:3] - body[0, 1:3]
heading = math.degrees(math.atan2(travel[1], travel[0])) if np.hypot(*travel) > 0.3 else 0.0
AZIM = heading + 198.0
UNIT = np.array([math.cos(math.radians(heading)), math.sin(math.radians(heading))])



def body_rot(t):
    """Drone orientation in the map: VIO (odom) attitude rotated by map_from_odom."""
    qv = latest(vio, t)
    b = latest(posed, t)
    mfo = b[1].get('map_from_odom') if b else None
    Rb = qmat(qv[1]) if qv else np.eye(3)
    if mfo:
        Rb = qmat(mfo[3:]) @ Rb
    return Rb


# Camera yaw per frame: the drone's heading, unwrapped and Gaussian-smoothed
# (centred, so the view eases into turns without lagging them).
cam_yaw = np.unwrap([math.atan2(R[1, 0], R[0, 0]) for R in (body_rot(t) for t in frames)])
_sig = max(1.0, 0.4 * FPS)
_n = int(3 * _sig)
_k = np.exp(-0.5 * (np.arange(-_n, _n + 1) / _sig) ** 2)
cam_yaw = np.convolve(np.pad(cam_yaw, _n, mode='edge'), _k / _k.sum(), mode='valid')

_map_cache = {}


def voxels_at(t):
    row = latest(maps, t)
    if row is None:
        return np.zeros((0, 3))
    key = row[0]
    if key not in _map_cache:
        c, s = decode_binary(row[1], row[2])
        v = expand(c, s, row[2])
        v = v[(v[:, 0] > X0) & (v[:, 0] < X1) & (v[:, 1] > Y0) & (v[:, 1] < Y1) & (v[:, 2] > Z0 - 0.3) & (v[:, 2] < Z1)]
        _map_cache.clear()
        _map_cache[key] = v
    return _map_cache[key]


# ---------------------------------------------------------------- figure
fig = plt.figure(figsize=(12, 9) if CAM_FEAT else (10, 7), facecolor=SURF)
AX_BOX = [0.47, 0.06, 0.53, 0.90] if CAM_FEAT else [0, 0, 1, 1]
ax = fig.add_axes(AX_BOX, projection='3d', facecolor=SURF)
# Explicit layers instead of matplotlib's per-artist depth sort: that sort only reorders collections (scatters),
# each by its single nearest point, and lines keep zorder 2, so the whole voxel cloud always painted over the drone.
# The map is split at the drone's depth instead (Z_* below): voxels behind it draw first, voxels in front after.
ax.computed_zorder = False
Z_FLOOR, Z_SHADOW, Z_FAR, Z_GOAL, Z_ROUTE, Z_TRAIL, Z_DRONE, Z_NEAR = 0.5, 1.5, 2, 3, 5, 6, 7, 8
if CAM_FEAT:
    import io
    import os
    import subprocess
    import tempfile
    from PIL import Image
    # cv2 needs the system numpy (PYTHONNOUSERSITE), so detect in a child process, as render_global.py does
    with tempfile.TemporaryDirectory() as tmp:
        npz = os.path.join(tmp, 'features.npz')
        subprocess.run([sys.executable, os.path.join(os.path.dirname(os.path.abspath(__file__)), 'detect_features.py'),
                        bag, npz, CAM_TOPIC], check=True, env=dict(os.environ, PYTHONNOUSERSITE='1'))
        d = np.load(npz)
        feats = list(zip(d['t'], np.split(d['xy'], np.cumsum(d['n'])[:-1])))
    CAM_W = 0.44   # square camera panel, left, under the title block
    CAM_BOX = [0.03, 0.115, CAM_W, CAM_W * fig.get_figwidth() / fig.get_figheight()]
    camax = fig.add_axes(CAM_BOX, zorder=2)
if CAMERA_BG:
    import io
    from PIL import Image
    BG_BOX = [0, 0.075, 1, 0.75]   # footer hairline up to just below the state/readout lines
    bgax = fig.add_axes(BG_BOX, zorder=0)
    bgax.set_axis_off()
    ax.set_zorder(1)
    _bg_aspect = fig.get_figwidth() * BG_BOX[2] / (fig.get_figheight() * BG_BOX[3])
    bgim = [None]


def camera_frame(t):
    """Latest camera image at t, centre-cropped to the background box's aspect, as greyscale."""
    row = latest(cams, t) or cams[0]
    g = np.asarray(Image.open(io.BytesIO(row[1])).convert('L'))
    h, w = g.shape
    if w / h > _bg_aspect:
        cw = int(round(h * _bg_aspect))
        g = g[:, (w - cw) // 2:(w - cw) // 2 + cw]
    else:
        ch = int(round(w / _bg_aspect))
        g = g[(h - ch) // 2:(h - ch) // 2 + ch]
    return g
cmap = matplotlib.colors.LinearSegmentedColormap.from_list('ink', T['ramp'])
state_label = {'CLIMB': 'climbing', 'WAITING_FOR_GOAL': 'waiting for route', 'ROUTE_START': 'route start',
               'FOLLOWING': 'following route', 'HOLD': 'HOLD', 'YAW_ALIGN': 'turning to heading',
               'GOAL_REACHED': 'goal reached', 'SAFE_APPROACH_REACHED': 'approach reached', 'LAND': 'landing'}
cam = [body[0, 1:3].astype(float)]


# ---------------------------------------------------------------- drafting overlay (2D, figure coordinates)
if DRAFT:
    from mpl_toolkits.mplot3d import proj3d
    ov = fig.add_axes([0, 0, 1, 1], zorder=4)
    ov.set_axis_off()
    ov.patch.set_alpha(0)
    ACC = dict(color=ROUTE, lw=0.6)
    FIG_AR = fig.get_figwidth() / fig.get_figheight()


def to_fig(P):
    """3D map points -> figure fractions, through the 3D axes' current projection."""
    P = np.atleast_2d(P)
    ax.apply_aspect()   # the 3D axes settle their 2D box at draw time; do it now so the overlay lines up
    x2, y2, _ = proj3d.proj_transform(P[:, 0], P[:, 1], P[:, 2], ax.get_proj())
    return fig.transFigure.inverted().transform(ax.transData.transform(np.c_[x2, y2]))


def construction_frame(x0, y0, x1, y1, p=1.0, diag=True, ticks=True, guides=True, diag_alpha=0.55):
    """Design-tool selection frame: hairline box, square handles, corner diagonals, guide overshoot, ruler ticks.
    p in [0, 1] draws it in (edges grow from the top-left corner, handles land at the end)."""
    w, h = (x1 - x0) * p, (y1 - y0) * p
    xa, ya = x0 + w, y1 - h
    ov.plot([x0, xa, xa, x0, x0], [y1, y1, ya, ya, y1], **ACC)
    if p < 1:
        return
    if guides:   # guide lines run past the corners like snapped rulers
        e = 0.018
        for yy in (y0, y1):
            ov.plot([x0 - e, x1 + e], [yy, yy], color=ROUTE, lw=0.4, alpha=0.6)
        for xx in (x0, x1):
            ov.plot([xx, xx], [y0 - e * FIG_AR, y1 + e * FIG_AR], color=ROUTE, lw=0.4, alpha=0.6)
    if diag:
        ov.plot([x0, x1], [y0, y1], color=ROUTE, lw=0.35, alpha=diag_alpha)
        ov.plot([x0, x1], [y1, y0], color=ROUTE, lw=0.35, alpha=diag_alpha)
    hx = [x0, (x0 + x1) / 2, x1]
    hy = [y0, (y0 + y1) / 2, y1]
    pts = [(a, b) for a in hx for b in hy if not (a == hx[1] and b == hy[1])]
    ov.scatter(*zip(*pts), marker='s', s=12, facecolors=SURF, edgecolors=ROUTE, linewidths=0.7, zorder=5)
    if ticks:   # ruler ticks just outside the top edge
        for xx in np.linspace(x0, x1, 9)[1:-1]:
            ov.plot([xx, xx], [y1 + 0.006, y1 + 0.013], color=ROUTE, lw=0.4)


def draft_overlay(t, i, pos, ring, lab, holding, spd, goal):
    ov.cla()
    ov.set_axis_off()
    ov.set_xlim(0, 1)
    ov.set_ylim(0, 1)
    p = min(max((t - t_start) / 1.2, 0.0), 1.0)
    p = 1 - (1 - p) ** 3   # ease out
    stamp = bag.rstrip('/').split('_')[-1].upper()
    dy = lambda pt: pt / 72 / fig.get_figheight()   # points -> figure fraction, vertical
    rnd = fig.canvas.get_renderer()

    def typed(s):   # titles type themselves in over the first second, like the teaser's title card
        return s[:int(round(len(s) * p))]
    # title block, laid out like the teaser's title card: two small lines, a big display line, a small line
    X0_, X1_, Y1_ = 0.03, 0.45 if CAM_FEAT else 0.50, 0.965
    Y0_ = Y1_ - dy(118)
    construction_frame(X0_ - 0.012, Y0_, X1_ + 0.012, Y1_, p=p, diag_alpha=0.3)
    ov.text(X0_, Y1_ - dy(22), typed('AUTONOMOUS INDOOR FLIGHT'), color=INK, fontsize=13.5)
    ov.text(X0_, Y1_ - dy(43), typed('PLANNER3D\u2014' + stamp), color=INK, fontsize=13.5)
    ov.text(X1_, Y1_ - dy(43), typed('\u00a9   [2026]'), color=INK, fontsize=13.5, ha='right')
    ov.text(X1_, Y1_ - dy(22), typed('T+%05.1f S' % (t - t_start)), color=INK, fontsize=13.5, ha='right')
    big = ov.text(X0_ - 0.003, Y1_ - dy(76), typed(lab.upper()), color=HOLD if holding else INK, fontsize=29, fontweight='bold')
    bw = big.get_window_extent(renderer=rnd).width / fig.bbox.width
    if bw > X1_ - X0_:   # long hold reasons shrink to fit the block
        big.set_fontsize(29 * (X1_ - X0_) / bw)
    ov.text(X0_, Y0_ + dy(13), typed('HEIGHT %.2f M   SPEED %.2f M/S' % (pos[2] - ground_z, spd)), color=INK, fontsize=13.5)
    ov.text(X1_, Y0_ + dy(12), typed('VIO + OCTOMAP\nPI 5 ONBOARD'), color=INK, fontsize=6.5, ha='right', va='bottom',
            linespacing=1.15)
    # drone frame from the projected cage
    pts = to_fig(ring)
    bx0, by0 = pts.min(0) - 0.012
    bx1, by1 = pts.max(0) + 0.012
    if AX_BOX[0] < bx0 and bx1 < 1 and AX_BOX[1] + 0.02 < by0 and by1 < 1 and not (bx0 < X1_ + 0.03 and by1 > Y0_ - 0.03):
        construction_frame(bx0, by0, bx1, by1, ticks=False)
        ov.text(bx0, by1 + 0.022, 'UAV / H %.2f M' % (pos[2] - ground_z), color=ROUTE, fontsize=7)
        yd = by0 - 0.02   # dimension line: cage diameter
        ov.plot([bx0, bx1], [yd, yd], color=ROUTE, lw=0.5)
        for xx in (bx0, bx1):
            ov.plot([xx, xx], [yd - dy(3), yd + dy(3)], color=ROUTE, lw=0.5)
        ov.text((bx0 + bx1) / 2, yd - dy(10), '\u00d8 %d CM' % round(CAGE_R * 200), color=ROUTE, fontsize=6, ha='center')
    if goal is not None:
        gp = goal[1].copy(); gp[2] = max(gp[2], ground_z + 0.3)
        gx, gy = to_fig(gp)[0]
        d = 0.03 * (7 / 9 if CAM_FEAT else 1)
        hx_, hy_ = d, d * fig.get_figwidth() / fig.get_figheight()
        near_uav = np.hypot(*(gp[:2] - pos[:2])) < 0.4   # at the goal the drone's own frame says enough
        if AX_BOX[0] + 0.05 < gx < 0.95 and AX_BOX[1] + 0.03 < gy < Y0_ - 0.05 and not near_uav:
            construction_frame(gx - hx_, gy - hy_, gx + hx_, gy + hy_, diag=False, ticks=False, guides=False)
            ov.text(gx + hx_ + 0.008, gy + hy_ - 0.012, 'GOAL\n%.1f M' % np.linalg.norm(gp[:2] - pos[:2]),
                    color=ROUTE, fontsize=6.5, va='top', linespacing=1.2)
    if CAM_FEAT:
        camera_panel(t)
    # footer
    yr, yl = dy(38), dy(19)
    ov.plot([0, 1], [yr, yr], color=INK, lw=0.5, alpha=0.35)
    items = [('ROUTE', ROUTE, '-'), ('COMMAND', CMD, 's'), ('UAV, 50 CM CAGE', INK, '-'), ('TRAIL, 6 S', INK2, '-'),
             (T['map_key'][0].upper(), T['map_key'][1], 's'), ('GOAL', ROUTE, 'o')]
    if CAM_FEAT:
        items.append(('IMAGE CORNER', ROUTE, 'h'))
    x = 0.03
    for txt, c, mk in items:
        if mk == '-':
            ov.plot([x, x + 0.018], [yl, yl], color=c, lw=2)
        elif mk == 'o':
            ov.scatter([x + 0.009], [yl], marker='o', s=30, facecolors='none', edgecolors=c, linewidths=1)
        elif mk == 'h':
            ov.scatter([x + 0.009], [yl], marker='s', s=22, facecolors=SURF, edgecolors=c, linewidths=0.8)
        else:
            ov.scatter([x + 0.009], [yl], marker='s', s=26, color=c)
        tx = ov.text(x + 0.026, yl - dy(4), txt, color=INK, fontsize=8.5)
        x += 0.026 + tx.get_window_extent(renderer=rnd).width / fig.bbox.width + 0.04


def camera_panel(t):
    """Left camera, greyscale, centre-cropped square; corners as hollow orange handles inside a construction frame."""
    camax.cla()
    camax.set_axis_off()
    row = latest(cams, t) or cams[0]
    g = np.asarray(Image.open(io.BytesIO(row[1])).convert('L'))
    g = g[8:-8]   # rectification leaves black rows at the top and bottom
    h, w = g.shape
    x0 = (w - h) / 2
    camax.imshow(g, cmap='gray', vmin=-25, vmax=280, interpolation='bilinear', aspect='auto')   # lifted toward the paper grey
    camax.set_xlim(x0 - 0.5, w - x0 - 0.5); camax.set_ylim(h - 0.5, -0.5)
    f = latest(feats, t)
    xy = np.zeros((0, 2)) if f is None else f[1] - [0, 8]
    xy = xy[(xy[:, 0] >= x0) & (xy[:, 0] < w - x0) & (xy[:, 1] >= 0) & (xy[:, 1] < h)]
    if len(xy):
        camax.scatter(xy[:, 0], xy[:, 1], marker='s', s=20, facecolors='none', edgecolors=ROUTE, linewidths=0.7)
    # panel frame + label, then a frame around where the corners are
    to = lambda P: fig.transFigure.inverted().transform(camax.transData.transform(np.atleast_2d(P)))
    (px0, py0), (px1, py1) = to([[x0 - 0.5, h - 0.5], [w - x0 - 0.5, -0.5]])
    construction_frame(px0, py0, px1, py1, diag=False, ticks=True)
    ov.text(px0, py1 + 0.03, 'CAM / LEFT %dx%d' % (h, h), color=ROUTE, fontsize=7)
    ov.text(px1, py1 + 0.03, '%d CORNERS' % len(xy), color=ROUTE, fontsize=7, ha='right')
    ov.text(px0, py0 - 0.03, 'SHI-TOMASI, PER FRAME\nNOT THE ONBOARD TRACKER', color=INK, fontsize=6, va='top', linespacing=1.15)
    if len(xy) >= 4:
        lo, hi = np.percentile(xy, 5, axis=0), np.percentile(xy, 95, axis=0)
        (fx0, fy1), (fx1, fy0) = to([lo, hi])
        construction_frame(fx0, fy0, fx1, fy1, ticks=False, guides=False, diag_alpha=0.45)


def draw(i):
    t = frames[i]
    ax.cla()
    if CAMERA_BG:
        g = camera_frame(t)
        if bgim[0] is None:   # dimmed to 55% so the scene reads on top
            bgim[0] = bgax.imshow(g, cmap='gray', vmin=0, vmax=255 / 0.55, aspect='auto', interpolation='bilinear')
        else:
            bgim[0].set_data(g)
    ax.set_axis_off()
    ax.set_facecolor((0, 0, 0, 0) if CAMERA_BG else SURF)
    # smooth camera follow in xy, leading along the direction of travel
    b = latest(status, t)[1] if latest(status, t) else {}
    bp = latest(posed, t)
    pos = np.array(bp[1]['body_map'] if bp else body[0, 1:4])
    cam[0] = cam[0] + 0.25 * (pos[:2] - cam[0])
    half = (1.05 + 0.25 * max(ZSPAN - 1.1, 0.0)) * VIEW
    if FIXED_CAM:
        azim, unit = AZIM, UNIT
    else:
        azim = math.degrees(cam_yaw[i]) + 180.0
        unit = np.array([math.cos(cam_yaw[i]), math.sin(cam_yaw[i])])
    cx, cy = cam[0] + 0.55 * unit
    cx = (X0 + X1) / 2 if X1 - X0 < 2 * half else min(max(cx, X0 + half), X1 - half)
    cy = (Y0 + Y1) / 2 if Y1 - Y0 < 2 * half else min(max(cy, Y0 + half), Y1 - half)
    ax.set_xlim(cx - half, cx + half); ax.set_ylim(cy - half, cy + half); ax.set_zlim(Z0, Z1)
    ax.set_box_aspect((2 * half, 2 * half, ZSPAN), zoom=1.55 if ZSPAN <= 1.2 else 1.25)
    ax.set_proj_type('persp', focal_length=0.55)
    ax.view_init(elev=20, azim=azim)
    EYE = np.array([math.cos(math.radians(20)) * math.cos(math.radians(azim)),
                    math.cos(math.radians(20)) * math.sin(math.radians(azim)), math.sin(math.radians(20))])
    xl, yl = ax.get_xlim(), ax.get_ylim()
    if DRAFT:   # halftone dot field on the floor, like the teaser's printed-dot texture
        gx, gy = np.meshgrid(np.arange(math.floor(xl[0] * 10) / 10, xl[1], 0.1), np.arange(math.floor(yl[0] * 10) / 10, yl[1], 0.1))
        keep = np.hypot(gx - cx, gy - cy) < half if not FIXED_CAM else np.ones_like(gx, bool)
        ax.scatter(gx[keep], gy[keep], np.full(keep.sum(), Z0), color='#8e8e8c', s=0.6, linewidths=0, depthshade=False, zorder=0)
    # floor grid (clipped to the view disc when the camera turns)
    GSTEP = 1.0 if DRAFT else 0.5
    for gx in np.arange(math.floor(xl[0] / GSTEP) * GSTEP, xl[1] + 1e-6, GSTEP):
        h = yl if FIXED_CAM else (cy - math.sqrt(max(half ** 2 - (gx - cx) ** 2, 0)),
                                  cy + math.sqrt(max(half ** 2 - (gx - cx) ** 2, 0)))
        ax.plot([gx, gx], h, [Z0, Z0], color=T['grid'][0], lw=T['grid'][1], alpha=T['grid'][2], zorder=0)
    for gy in np.arange(math.floor(yl[0] / GSTEP) * GSTEP, yl[1] + 1e-6, GSTEP):
        h = xl if FIXED_CAM else (cx - math.sqrt(max(half ** 2 - (gy - cy) ** 2, 0)),
                                  cx + math.sqrt(max(half ** 2 - (gy - cy) ** 2, 0)))
        ax.plot(h, [gy, gy], [Z0, Z0], color=T['grid'][0], lw=T['grid'][1], alpha=T['grid'][2], zorder=0)
    # map
    v = voxels_at(t)
    v = v[(v[:, 0] > xl[0]) & (v[:, 0] < xl[1]) & (v[:, 1] > yl[0]) & (v[:, 1] < yl[1])]
    if not FIXED_CAM:
        v = v[np.hypot(v[:, 0] - cx, v[:, 1] - cy) < half]
    floor = v[:, 2] < ground_z + 0.08
    if (~floor).any():
        vv = v[~floor]
        depth = (vv - pos) @ EYE   # > 0: between the drone and the camera
        o = np.argsort(depth)      # far to near, so nearer blocks paint over farther ones
        vv, depth = vv[o], depth[o]
        rgba = cmap(np.clip((vv[:, 2] - ground_z) / max(ZSPAN - 0.1, 1.0), 0, 1))
        dist = np.linalg.norm(vv - pos, axis=1)
        if DRAFT:   # solid grey blocks, dark-edged, like the teaser's pixelated map
            rgba[:, 3] = np.interp(dist, [0.5 * VIEW, 2.4 * VIEW], [0.95, 0.35])
            kw = dict(marker='s', s=52 / VIEW ** 2, linewidths=0.25, edgecolors=(0.1, 0.1, 0.1, 0.35), depthshade=False)
        else:
            rgba[:, 3] = np.interp(dist, [0.5 * VIEW, 2.2 * VIEW], [0.7, 0.14])   # near walls strong, background recedes
            kw = dict(marker='s', s=34 / VIEW ** 2, linewidths=0, depthshade=False)
        for sel, z in ((depth <= CAGE_R, Z_FAR), (depth > CAGE_R, Z_NEAR)):
            if sel.any():
                ax.scatter(vv[sel, 0], vv[sel, 1], vv[sel, 2], c=rgba[sel], zorder=z, **kw)
    if floor.any():
        ax.scatter(v[floor, 0], v[floor, 1], np.full(floor.sum(), Z0), color=T['floor'][0], marker='s', s=20 / VIEW ** 2, alpha=T['floor'][1],
                   linewidths=0, depthshade=False, zorder=Z_FLOOR)
    # goal
    g = latest(goals, t)
    if g is not None:
        gp = g[1].copy(); gp[2] = max(gp[2], ground_z + 0.3)
        ax.plot([gp[0], gp[0]], [gp[1], gp[1]], [Z0, gp[2]], color=MUTED, lw=0.8, ls=':', zorder=Z_GOAL)
        if DRAFT:   # crosshair target; its construction frame is drawn in the overlay
            ax.scatter([gp[0]], [gp[1]], [gp[2]], marker='+', s=220, color=GOAL, linewidths=1.2, depthshade=False, zorder=Z_GOAL)
            ax.scatter([gp[0]], [gp[1]], [gp[2]], marker='o', s=90, facecolors='none', edgecolors=ROUTE, linewidths=1.2, depthshade=False, zorder=Z_GOAL)
        else:
            ax.scatter([gp[0]], [gp[1]], [gp[2]], marker='*', s=160, color=GOAL, depthshade=False, zorder=Z_GOAL)
    # route
    rt = latest(routes, t)
    if rt is not None and len(rt[1]) >= 2:
        R = rt[1]
        if DRAFT:   # solid accent line with hollow square handles every ~0.4 m, like vector-editor anchors
            R = R.copy()
            if not FIXED_CAM:   # clip to the view disc (the map is clipped there too)
                R[np.hypot(R[:, 0] - cx, R[:, 1] - cy) > half] = np.nan
            ax.plot(R[:, 0], R[:, 1], R[:, 2], color=ROUTE, lw=2.2, zorder=Z_ROUTE)
            seg = np.r_[0, np.cumsum(np.nan_to_num(np.linalg.norm(np.diff(R, axis=0), axis=1)))]
            idx = np.unique(np.searchsorted(seg, np.arange(0, seg[-1] + 1e-6, 0.4)).clip(0, len(R) - 1))
            ax.scatter(R[idx, 0], R[idx, 1], R[idx, 2], marker='s', s=14, facecolors=SURF, edgecolors=ROUTE,
                       linewidths=0.9, depthshade=False, zorder=Z_ROUTE + 0.1)
        else:
            ax.plot(R[:, 0], R[:, 1], R[:, 2], color=ROUTE, lw=3, ls=(0, (4, 2)), zorder=Z_ROUTE)
    # trail (last 6 s, fading)
    tr = body[(body[:, 0] <= t) & (body[:, 0] >= t - 6.0)]
    if len(tr) > 1:
        n = len(tr)
        for k in range(1, n):
            a = 0.1 + 0.8 * k / n
            ax.plot(tr[k - 1:k + 1, 1], tr[k - 1:k + 1, 2], tr[k - 1:k + 1, 3], color=INK2, lw=2.4, alpha=a, zorder=Z_TRAIL)
    # command point
    cmd = b.get('command_map')
    state = b.get('state', '')
    holding = state in ('HOLD', 'YAW_ALIGN')
    if cmd and not holding:
        ax.plot([pos[0], cmd[0]], [pos[1], cmd[1]], [pos[2], cmd[2]], color=CMD, lw=1, alpha=0.7, zorder=Z_TRAIL)
        if DRAFT:
            ax.scatter([cmd[0]], [cmd[1]], [cmd[2]], marker='s', s=26, color=CMD, depthshade=False, zorder=Z_TRAIL)
        else:
            ax.scatter([cmd[0]], [cmd[1]], [cmd[2]], color=CMD, s=36, depthshade=False, zorder=Z_TRAIL)
    # drone: orientation from VIO (odom) rotated into the map
    Rb = body_rot(t)
    col = INK if DRAFT else (HOLD if holding else INK)
    th = np.linspace(0, 2 * math.pi, 40)
    ring = np.stack([CAGE_R * np.cos(th), CAGE_R * np.sin(th), np.zeros_like(th)], 1) @ Rb.T + pos
    ax.plot(ring[:, 0], ring[:, 1], ring[:, 2], color=col, lw=1.2, alpha=0.55, zorder=Z_DRONE)
    for ang in (45, 135, 225, 315):
        a = math.radians(ang)
        tip = np.array([ARM * math.cos(a), ARM * math.sin(a), 0])
        arm = np.stack([np.zeros(3), tip]) @ Rb.T + pos
        ax.plot(arm[:, 0], arm[:, 1], arm[:, 2], color=col, lw=2.6, zorder=Z_DRONE)
        rot = (np.stack([0.06 * np.cos(th), 0.06 * np.sin(th), np.zeros_like(th)], 1) + tip) @ Rb.T + pos
        ax.plot(rot[:, 0], rot[:, 1], rot[:, 2], color=col, lw=1.2, zorder=Z_DRONE)
    nose = np.stack([np.zeros(3), np.array([CAGE_R + 0.06, 0, 0])]) @ Rb.T + pos
    ax.plot(nose[:, 0], nose[:, 1], nose[:, 2], color=ROUTE if DRAFT else (CMD if not holding else HOLD), lw=2.4, zorder=Z_DRONE)
    # shadow
    sh = np.stack([CAGE_R * np.cos(th) + pos[0], CAGE_R * np.sin(th) + pos[1], np.full_like(th, Z0)], 1)
    ax.plot(sh[:, 0], sh[:, 1], sh[:, 2], color=T['shadow'], lw=4, alpha=0.6, zorder=Z_SHADOW)
    ax.plot([pos[0], pos[0]], [pos[1], pos[1]], [Z0, pos[2]], color=T['shadow'], lw=0.8, ls=':', zorder=Z_SHADOW)
    # readout
    prev = latest(posed, t - 0.5)
    spd = 0.0
    if prev:
        spd = float(np.linalg.norm(pos - np.array(prev[1]['body_map']))) / 0.5
    lab = state_label.get(state, state.lower().replace('_', ' '))
    if state == 'HOLD' and b.get('fault'):
        lab += ' - ' + b['fault'].lower().replace('_', ' ')
    fig.texts.clear()
    fig.patches.clear()
    fig.lines.clear()
    if LIGHT:
        fig.patches.append(matplotlib.patches.FancyBboxPatch((0.015, 0.845), 0.47, 0.14, boxstyle='round,pad=0.005,rounding_size=0.01',
                           transform=fig.transFigure, facecolor=SURF, edgecolor='#e3e2dc', alpha=0.92, zorder=1))
        fig.text(0.03, 0.95, bag.rstrip('/').split('_')[-1], color=INK2, fontsize=10, family='monospace')
        fig.text(0.03, 0.905, lab, color=HOLD if holding else INK, fontsize=15, fontweight='bold')
        fig.text(0.03, 0.865, 't %5.1f s    height %.2f m    speed %.2f m/s' % (t, pos[2] - ground_z, spd), color=INK2,
                 fontsize=10.5, family='monospace')
        for x, txt, c in [(0.03, 'route', ROUTE), (0.11, 'command', CMD), (0.22, 'drone + 50 cm cage', INK),
                          (0.42, 'trail (6 s)', INK2), (0.55, *T['map_key']),
                          (0.77, 'goal', GOAL)]:
            fig.text(x, 0.03, txt, color=c, fontsize=10, fontweight='bold')
        return []
    if DRAFT:
        draft_overlay(t, i, pos, ring, lab, holding, spd, g)
        return []
    # archive look: muted/ink title, hairline rules, lowercase light text, colour only in the swatches
    stamp = bag.rstrip('/').split('_')[-1].lower()
    t1 = fig.text(0.03, 0.935, 'planner3d/', color=MUTED, fontsize=15)
    w = t1.get_window_extent(renderer=fig.canvas.get_renderer()).width / fig.bbox.width
    fig.text(0.03 + w, 0.935, stamp, color=INK, fontsize=15)
    fig.text(0.97, 0.935, '%5.1f s' % t, color=MUTED, fontsize=15, ha='right')
    fig.add_artist(matplotlib.lines.Line2D([0, 1], [0.915, 0.915], transform=fig.transFigure, **HAIR))
    fig.text(0.03, 0.875, lab, color=HOLD if holding else INK, fontsize=11.5)
    fig.text(0.03, 0.845, 'height %.2f m    speed %.2f m/s' % (pos[2] - ground_z, spd), color=MUTED, fontsize=11.5)
    fig.add_artist(matplotlib.lines.Line2D([0, 1], [0.075, 0.075], transform=fig.transFigure, **HAIR))
    for x, txt, c in [(0.03, 'route', ROUTE), (0.13, 'command', CMD), (0.255, 'drone, 50 cm cage', INK),
                      (0.445, 'trail, 6 s', INK2), (0.57, *T['map_key']), (0.79, 'goal', GOAL)]:
        fig.add_artist(matplotlib.lines.Line2D([x, x + 0.018], [0.036, 0.036], transform=fig.transFigure, color=c, lw=2))
        fig.text(x + 0.026, 0.028, txt, color=MUTED, fontsize=11)
    return []


if STILLS:
    for st_ in STILLS:
        i = int(np.argmin(np.abs(frames - st_)))
        for _ in range(15):   # let the smoothed camera settle on this frame
            draw(i)
        fig.savefig('%s_%.1f.png' % (out.rsplit('.', 1)[0], frames[i]), dpi=120, facecolor=SURF)
    sys.exit(0)
anim = FuncAnimation(fig, draw, frames=len(frames), interval=1000 / FPS, blit=False)
if out.endswith('.mp4'):
    import imageio_ffmpeg
    from matplotlib.animation import FFMpegWriter
    matplotlib.rcParams['animation.ffmpeg_path'] = imageio_ffmpeg.get_ffmpeg_exe()
    writer = FFMpegWriter(fps=FPS, codec='libx264', extra_args=[
        '-pix_fmt', 'yuv420p', '-crf', '20', '-preset', 'slow', '-movflags', '+faststart'])
    anim.save(out, writer=writer, dpi=120, savefig_kwargs={'facecolor': SURF})   # 1200x840
else:
    anim.save(out, writer=PillowWriter(fps=FPS), dpi=80, savefig_kwargs={'facecolor': SURF})
print('wrote', out, '%d frames at %.0f fps, %.1f s' % (len(frames), FPS, len(frames) / FPS))
