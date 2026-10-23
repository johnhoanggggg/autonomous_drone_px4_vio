"""Path render + MP4 replay of a 2D global-planner flight (offboard_global_* bags).

Writes <out_dir>/<name>_path.png (top view: final grid with 0.25 m clearance band,
first planned route, command sent, flown track with time ticks, takeoff, goal,
landing) and <out_dir>/<name>_flight.mp4 (left: whole map with the track so far;
right: chase view of the grid as it was at that moment, current route, carrot,
drone + cage, 6 s trail and a status readout).
The drone track ends at the first VIO jump (RTAB-Map reset sentinel after landing).
MP4 default (as animate3d.py): archive look (black page, greyscale grid, red route) and a chase view that
turns with the drone's heading (smoothed ~0.4 s, clipped to a disc) -> <name>_flight_behind.mp4.
--fixed-cam keeps the north-up chase view (-> <name>_flight.mp4), --light the earlier light look.
--camera-left (archive look) puts the recorded camera feed (/rtabmap/image/compressed) on the left in place
of the overview map -> <name>_flight_behind_camera.mp4 (1680x840).
--no-text drops the header, readout and legend: camera and chase disc side by side, 4:3 (1440x1080).
--features=x|o (implies --camera-left) marks Shi-Tomasi corners on the camera (detect_features.py, per-frame
detection, not the tracker that flew) as 1 px red crosses (12 px arms) or circles (24 px radius) -> <name>_flight_behind_camera_feat_<x|o>.mp4.
--feat-stroke=N draws the feature markers with an N px stroke (default 1) -> ..._feat_<x|o>_wN.mp4.
--square-cam crops the camera to a centred square (400x400 of 640x400) -> ..._camera_sq...mp4.
The path PNG is always the light look; --mp4-only skips it.
--report (implies --png-only) writes <name>_path_report.png + .pdf for a paper: no title, large text, legend under
the plot with the map colours, short labels; --legend-side puts the legend in one column on the right (for tall maps). --label=x,y,text (repeatable) marks a map feature, e.g. a door.
Usage (ROS sourced): python3 render_global.py <bag_dir> <out_dir> <name> [fps] [--png-only] [--mp4-only]
                     [--fixed-cam] [--light] [--camera-left] [--no-text] [--features=x|o]
                     [--square-cam] [--feat-stroke=N]
"""
import io
import math
import os
import subprocess
import sys
import tempfile

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.ndimage import distance_transform_edt

sys.path.append('/home/john/autonomous_drone_px4_vio/.venv-mavlink/lib/python3.12/site-packages')
import matplotlib  # noqa: E402
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import matplotlib.lines  # noqa: E402,F401
import matplotlib.transforms  # noqa: E402,F401
from matplotlib.animation import FuncAnimation  # noqa: E402
from PIL import Image  # noqa: E402

args = [a for a in sys.argv[1:] if not a.startswith('--')]
bag, out_dir, name = args[0], args[1].rstrip('/'), args[2]
FPS = float(args[3]) if len(args) > 3 else 5.0
REPORT = '--report' in sys.argv
PNG_ONLY = '--png-only' in sys.argv or REPORT
LEGEND_SIDE = '--legend-side' in sys.argv
LABELS = [a.split('=', 1)[1].split(',', 2) for a in sys.argv if a.startswith('--label=')]
MP4_ONLY = '--mp4-only' in sys.argv
FIXED_CAM = '--fixed-cam' in sys.argv
LIGHT = '--light' in sys.argv
FEAT = next((a.split('=', 1)[1] for a in sys.argv if a.startswith('--features=')), None)
CAMERA = '--camera-left' in sys.argv or FEAT is not None
NO_TEXT = '--no-text' in sys.argv
SQUARE = '--square-cam' in sys.argv
STROKE = float(next((a.split('=', 1)[1] for a in sys.argv if a.startswith('--feat-stroke=')), 1))
IMAGE = '/rtabmap/image/compressed'
SURF, INK, INK2, MUTED = '#fcfcfb', '#0b0b0b', '#52514e', '#8a8984'
ROUTE, CMD, HOLD, GOAL, FLOWN = '#eb6834', '#2a78d6', '#d98a00', '#0b0b0b', '#1a9e55'
FREE, UNKNOWN, OCC, BAND = (0.957, 0.953, 0.937), (0.82, 0.82, 0.80), (0.12, 0.16, 0.22), (0.97, 0.84, 0.66)
CAGE_R, ARM, CLEAR = 0.25, 0.17, 0.25

# ---------------------------------------------------------------- read the bag
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
types = {t.name: t.type for t in r.get_all_topics_and_types()}
want = ['/rtabmap/grid', '/rtabmap/body_pose', '/planner/path', '/planner/follower/carrot', '/planner/effective_goal',
        '/waypoint/clicked', '/planner/flight/status', '/planner/follower/status', '/fmu/out/vehicle_local_position_v1',
        '/fmu/out/vehicle_land_detected', '/fmu/out/vehicle_status_v1', '/fmu/in/vehicle_command']
if CAMERA:
    if IMAGE not in types:
        sys.exit(f'{bag} has no {IMAGE}; fly with record_camera:=true')
    want.append(IMAGE)
r.set_filter(rosbag2_py.StorageFilter(topics=[w for w in want if w in types]))
t0 = r.get_metadata().starting_time.nanoseconds
grids, pose, routes, carrot, goals, status, zned, landed, armed, fstatus, landcmd = [], [], [], [], [], [], [], [], [], [], []
images = []
while r.has_next():
    tp, d, ns = r.read_next()
    t = (ns - t0) / 1e9
    m = deserialize_message(d, get_message(types[tp]))
    if tp == '/rtabmap/grid':
        i = m.info
        grids.append((t, np.array(m.data, dtype=np.int8).reshape(i.height, i.width), i.resolution,
                      i.origin.position.x, i.origin.position.y))
    elif tp == '/rtabmap/body_pose':
        p, q = m.pose.position, m.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        pose.append((t, p.x, p.y, p.z, yaw))
    elif tp == '/planner/path':
        if m.poses:
            routes.append((t, np.array([[s.pose.position.x, s.pose.position.y] for s in m.poses])))
    elif tp == '/planner/follower/carrot':
        carrot.append((t, m.pose.position.x, m.pose.position.y))
    elif tp in ('/planner/effective_goal', '/waypoint/clicked'):
        goals.append((t, m.point.x, m.point.y, tp))
    elif tp == '/planner/flight/status':
        status.append((t, m.data))
    elif tp == '/planner/follower/status':
        fstatus.append((t, m.data))
    elif tp == '/fmu/out/vehicle_local_position_v1':
        zned.append((t, m.z))
    elif tp == '/fmu/out/vehicle_land_detected':
        landed.append((t, m.landed))
    elif tp == '/fmu/out/vehicle_status_v1':
        armed.append((t, m.arming_state == 2))
    elif tp == IMAGE:
        images.append((t, bytes(m.data)))
    elif tp == '/fmu/in/vehicle_command' and m.command == 21:   # NAV_LAND
        landcmd.append(t)

pose = np.array(pose)
t_arm = next(t for t, a in armed if a)
t_disarm = next((t for t, a in armed if not a and t > t_arm), pose[-1, 0])
t_liftoff = next((t for t, l in landed if not l and t > t_arm - 1), t_arm)
t_touch = next((t for t, l in landed if l and t > t_liftoff), t_disarm)
t_land = next((t for t in landcmd if t > t_liftoff), t_touch)
# Cut the track at the first VIO jump (> 0.4 m between samples) after arming.
step = np.r_[0, np.hypot(np.diff(pose[:, 1]), np.diff(pose[:, 2]))]
jump = np.where((step > 0.4) & (pose[:, 0] > t_arm))[0]
t_end = min(t_disarm + 0.5, pose[jump[0] - 1, 0] if len(jump) else 1e9)
t_start = t_arm - 1.0
fl = pose[(pose[:, 0] >= t_start) & (pose[:, 0] <= t_end)]
zt = np.array(zned)
z_ground = np.median(zt[(zt[:, 0] > t_arm - 3) & (zt[:, 0] < t_arm), 1]) if len(zt) else 0.0


def latest(rows, t):
    best = None
    for row in rows:
        if row[0] > t:
            break
        best = row
    return best


def state_at(t):
    s = latest(status, t)
    return (s[1].split()[0] if s and s[1] else ''), (s[1] if s else '')


def height_at(t):
    if not len(zt):
        return float('nan')
    k = np.searchsorted(zt[:, 0], t)
    return -(zt[min(k, len(zt) - 1), 1] - z_ground)


goal_rows = [g for g in goals if g[3] == '/waypoint/clicked'] or goals
goal_xy = np.array(goal_rows[0][1:3]) if goal_rows else None
eff_goal = [g for g in goals if g[3] == '/planner/effective_goal' and g[0] <= t_end]

# ---------------------------------------------------------------- grid images
_img_cache = {}


def grid_rgb(row):
    key = row[0]
    if key in _img_cache:
        return _img_cache[key]
    _, g, res, ox, oy = row
    occ = g >= 65
    rgb = np.empty(g.shape + (3,))
    rgb[:] = FREE
    rgb[g < 0] = UNKNOWN
    dist = distance_transform_edt(~occ) * res
    rgb[(dist <= CLEAR) & ~occ & (g >= 0)] = BAND
    rgb[occ] = OCC
    ext = (ox, ox + g.shape[1] * res, oy, oy + g.shape[0] * res)
    _img_cache.clear()
    _img_cache[key] = (rgb, ext)
    return rgb, ext


final_grid = latest(grids, t_end)
xs, ys = fl[:, 1], fl[:, 2]
pts = [fl[:, 1:3]] + ([goal_xy[None]] if goal_xy is not None else [])
allp = np.vstack(pts)
X0, X1 = allp[:, 0].min() - 1.2, allp[:, 0].max() + 1.2
Y0, Y1 = allp[:, 1].min() - 1.2, allp[:, 1].max() + 1.2
first_route = next((rt for rt in routes if rt[0] >= t_arm - 5), routes[0] if routes else None)
cmd = np.array([c for c in carrot if t_liftoff <= c[0] <= t_end]) if carrot else np.zeros((0, 3))
holds = [(t, s) for t, s in status if t_start <= t <= t_end and s.split()[0] in ('HOLD', 'COMMAND_HOLD')]
reached = min([t for t, s in status + fstatus if s.startswith('GOAL_REACHED') and t <= t_end] or [None],
              key=lambda v: 1e9 if v is None else v)
length = float(np.sum(np.hypot(np.diff(xs), np.diff(ys))))
dur = t_touch - t_liftoff


def legend_style(ax):
    leg = ax.legend(loc='best', fontsize=8.5, framealpha=0.92, facecolor=SURF, edgecolor='#e3e2dc')
    for txt in leg.get_texts():
        txt.set_color(INK2)


# ---------------------------------------------------------------- report path figure
if REPORT:
    import matplotlib.patches as mpatches
    from matplotlib import patheffects
    plt.rcParams.update({'font.size': 15, 'font.family': ['DejaVu Sans']})
    aspect = (Y1 - Y0) / (X1 - X0)
    fig, ax = plt.subplots(figsize=(8 + 3.2, 8 * aspect) if LEGEND_SIDE else (8, 8 * aspect + 1.9), facecolor=SURF)
    ax.set_facecolor(UNKNOWN)
    rgb, ext = grid_rgb(final_grid)
    ax.imshow(rgb, origin='lower', extent=ext, interpolation='nearest')
    if first_route is not None:
        ax.plot(first_route[1][:, 0], first_route[1][:, 1], color=ROUTE, lw=2.2, ls=(0, (4, 2)), label='planned route')
    ax.plot(xs, ys, color=FLOWN, lw=3.2, label='flown track')
    for tt in np.arange(math.ceil(t_liftoff / 10) * 10, t_end, 10):
        k = np.searchsorted(fl[:, 0], tt)
        ax.plot(fl[k, 1], fl[k, 2], 'o', color=INK, ms=4.5)
        ax.annotate('%.0f s' % (tt - t_arm), (fl[k, 1], fl[k, 2]), xytext=(6, 6), textcoords='offset points',
                    fontsize=13, color=INK, path_effects=[patheffects.withStroke(linewidth=3.5, foreground='white')])
    if holds:
        hp = np.array([fl[min(np.searchsorted(fl[:, 0], t), len(fl) - 1), 1:3] for t, _ in holds])
        ax.plot(hp[:, 0], hp[:, 1], 'o', mfc='none', mec=HOLD, ms=10, mew=2, label='hold')
    ax.plot(xs[0], ys[0], '^', color=FLOWN, mec=INK, ms=14, label='takeoff')
    if goal_xy is not None:
        ax.plot(*goal_xy, 'o', mfc='white', mec=INK, ms=15, mew=2, label='goal')
    kt = min(np.searchsorted(fl[:, 0], t_touch), len(fl) - 1)
    ax.plot(fl[kt, 1], fl[kt, 2], 'v', color='#d6402b', mec=INK, ms=14, label='landing')
    for lx, ly, txt in LABELS:
        ax.annotate(txt, (float(lx), float(ly)), xytext=(0, -48), textcoords='offset points', ha='center', va='top',
                    fontsize=15, color=INK, bbox=dict(boxstyle='round,pad=0.25', fc='white', ec=INK, lw=1),
                    arrowprops=dict(arrowstyle='-|>', color=INK, lw=1.5), zorder=20)
    ax.set_xlim(X0, X1); ax.set_ylim(Y0, Y1); ax.set_aspect('equal')
    ax.set_xlabel('x (m)', color=INK2); ax.set_ylabel('y (m)', color=INK2)
    ax.tick_params(colors=INK2, labelsize=14)
    for sp in ax.spines.values():
        sp.set_color('#c9c8c2')
    h, l = ax.get_legend_handles_labels()
    h += [mpatches.Patch(color=OCC), mpatches.Patch(color=BAND), mpatches.Patch(color=UNKNOWN, ec='#9a9a96')]
    l += ['obstacle', '%.2f m clearance' % CLEAR, 'unmapped']
    if LEGEND_SIDE:
        leg = ax.legend(h, l, loc='center left', bbox_to_anchor=(1.02, 0.5), ncol=1, fontsize=15, frameon=False,
                        handlelength=1.8, labelspacing=0.9)
    else:
        leg = ax.legend(h, l, loc='upper center', bbox_to_anchor=(0.5, -0.11), ncol=3, fontsize=14, frameon=False,
                        handlelength=1.8, columnspacing=1.4)
    for txt in leg.get_texts():
        txt.set_color(INK)
    fig.tight_layout()
    for ext_ in ('png', 'pdf'):
        fig.savefig('%s/%s_path_report.%s' % (out_dir, name, ext_), dpi=200, facecolor=SURF, bbox_inches='tight',
                    pad_inches=0.1)
    print('wrote %s/%s_path_report.png/.pdf' % (out_dir, name))
    plt.close(fig)
    sys.exit(0)

# ---------------------------------------------------------------- path PNG
fig, ax = plt.subplots(figsize=(10, 10 * (Y1 - Y0) / (X1 - X0) + 0.6) if (Y1 - Y0) / (X1 - X0) < 1.4 else (8, 11),
                       facecolor=SURF)
ax.set_facecolor(UNKNOWN)
rgb, ext = grid_rgb(final_grid)
ax.imshow(rgb, origin='lower', extent=ext, interpolation='nearest')
if first_route is not None:
    ax.plot(first_route[1][:, 0], first_route[1][:, 1], color=ROUTE, lw=1.6, ls=(0, (4, 2)), label='planned route (first)')
if len(cmd):
    ax.plot(cmd[:, 1], cmd[:, 2], color=CMD, lw=1.0, alpha=0.75, label='command (carrot)')
ax.plot(xs, ys, color=FLOWN, lw=2.4, label='flown (body pose)')
for tt in np.arange(math.ceil(t_liftoff / 10) * 10, t_end, 10):
    k = np.searchsorted(fl[:, 0], tt)
    ax.plot(fl[k, 1], fl[k, 2], 'o', color=INK, ms=3)
    ax.annotate('%.0fs' % (tt - t_arm), (fl[k, 1], fl[k, 2]), xytext=(4, 4), textcoords='offset points', fontsize=8,
                color=INK)
if holds:
    hp = np.array([fl[min(np.searchsorted(fl[:, 0], t), len(fl) - 1), 1:3] for t, _ in holds])
    ax.plot(hp[:, 0], hp[:, 1], 'o', mfc='none', mec=HOLD, ms=7, mew=1.4, label='hold / command hold')
ax.plot(xs[0], ys[0], '^', color=FLOWN, mec=INK, ms=11, label='takeoff')
if goal_xy is not None:
    ax.plot(*goal_xy, 'o', mfc='white', mec=INK, ms=12, mew=1.5, label='goal (clicked)')
if eff_goal and np.hypot(eff_goal[-1][1] - goal_xy[0], eff_goal[-1][2] - goal_xy[1]) > 0.15:
    ax.plot(eff_goal[-1][1], eff_goal[-1][2], 'D', mfc='white', mec=ROUTE, ms=8, mew=1.5, label='safe approach goal')
kt = np.searchsorted(fl[:, 0], t_touch)
ax.plot(fl[min(kt, len(fl) - 1), 1], fl[min(kt, len(fl) - 1), 2], 'v', color='#d6402b', mec=INK, ms=11, label='landing')
ax.set_xlim(X0, X1); ax.set_ylim(Y0, Y1); ax.set_aspect('equal')
ax.set_xlabel('map x (m)', color=INK2); ax.set_ylabel('map y (m)', color=INK2)
ax.tick_params(colors=INK2)
for sp in ax.spines.values():
    sp.set_color('#e3e2dc')
legend_style(ax)
hh = np.array([height_at(t) for t in fl[:, 0]])
cruise = (fl[:, 0] > t_liftoff + 4) & (fl[:, 0] < t_land)
ax.set_title('Global flight %s  |  %s  |  %.1f m flown in %.0f s  |  cruise height %.2f-%.2f m\n'
             'height = PX4 z vs takeoff (the VIO frame tilts, so not height above the floor)\n'
             'shaded = %.2f m clearance band, grey = unknown' %
             (name.split('_')[-1], 'goal reached at %.0f s' % (reached - t_arm) if reached else 'goal NOT reached',
              length, dur, np.nanmin(hh[cruise]) if cruise.any() else 0, np.nanmax(hh[cruise]) if cruise.any() else 0,
              CLEAR), color=INK, fontsize=10)
fig.tight_layout()
if not MP4_ONLY:
    fig.savefig('%s/%s_path.png' % (out_dir, name), dpi=120, facecolor=SURF)
    print('wrote %s/%s_path.png' % (out_dir, name))
plt.close(fig)
if PNG_ONLY:
    sys.exit(0)

# ---------------------------------------------------------------- MP4
frames = np.arange(t_start, t_end, 1.0 / FPS)
HALF = 1.6
label = {'ROUTE': 'following route', 'HOLD': 'HOLD', 'COMMAND_HOLD': 'COMMAND HOLD', 'DEGRADED': 'degraded (crawl)',
         'YAW_ALIGN': 'turning to heading', 'GOAL_REACHED': 'goal reached', 'LAND': 'landing'}
if not LIGHT:
    # archive look (iwrzwr/visual-archive, as animate3d.py): black page, greyscale grid, red route only accent
    SURF, INK, INK2, MUTED = '#000000', '#f7f7f4', '#b6b6b6', '#777671'
    ROUTE, CMD, HOLD, GOAL, FLOWN = '#ff0000', '#339cff', '#ff963e', '#f7f7f4', '#b6b6b6'
    FREE, UNKNOWN, OCC, BAND = (0.09, 0.09, 0.09), (0.0, 0.0, 0.0), (0.86, 0.86, 0.84), (0.25, 0.25, 0.24)
    plt.rcParams['font.family'] = ['Liberation Sans', 'DejaVu Sans']
    _img_cache.clear()
HAIR = dict(color='#ffffff', alpha=0.12, lw=0.5)


def pose_at(t):
    k = min(max(np.searchsorted(fl[:, 0], t) - 1, 0), len(fl) - 1)
    return fl[k]


# Camera yaw per frame: the drone's heading, unwrapped and Gaussian-smoothed (centred, ~0.4 s),
# so the rotating chase view eases into turns without lagging them.
cam_yaw = np.unwrap([pose_at(t)[4] for t in frames])
_sig = max(1.0, 0.4 * FPS)
_n = int(3 * _sig)
_k = np.exp(-0.5 * (np.arange(-_n, _n + 1) / _sig) ** 2)
cam_yaw = np.convolve(np.pad(cam_yaw, _n, mode='edge'), _k / _k.sum(), mode='valid')

if FEAT:
    # cv2 needs the system numpy, this script the ~/.local one (scipy), so detect in a child process
    with tempfile.TemporaryDirectory() as tmp:
        npz = os.path.join(tmp, 'features.npz')
        subprocess.run([sys.executable, os.path.join(os.path.dirname(os.path.abspath(__file__)), 'detect_features.py'),
                        bag, npz], check=True, env=dict(os.environ, PYTHONNOUSERSITE='1'))
        d = np.load(npz)
        feats = list(zip(d['t'], np.split(d['xy'], np.cumsum(d['n'])[:-1])))
    PX = 72 / 120   # one output pixel in points (saved at dpi 120)
    FEAT_ARM = 24 if FEAT == 'o' else 12   # circle radius / cross half-size in output px (crosses as render_tracking.py)

fig = plt.figure(figsize=(14 if CAMERA else 10, 7), facecolor=SURF)
XS = 10 / 14 if CAMERA else 1.0   # figure-x scale for the header/legend positions
if CAMERA and NO_TEXT:
    # 4:3 landscape, 1440x1080 px: camera (16:10) and chase disc at the same height across the width
    # (30 px margins, 40 px gap), centred with black bands above and below
    FW, FH, M, GAP = 1440, 1080, 30, 40
    D = round((FW - 2 * M - GAP) / (1 + (1.0 if SQUARE else 1.6)))
    WC, Y = FW - 2 * M - GAP - D, (FH - D) // 2
    fig.set_size_inches(FW / 120, FH / 120)
    axo = fig.add_axes([M / FW, Y / FH, WC / FW, D / FH], facecolor=SURF)
    axc = fig.add_axes([(M + WC + GAP) / FW, Y / FH, D / FW, D / FH], facecolor=SURF)
elif CAMERA:
    axo = fig.add_axes([0.02, 0.10, 0.48, 0.70], facecolor=SURF)   # camera
    axc = fig.add_axes([0.52, 0.085, 0.46, 0.82], facecolor=SURF)  # chase
elif LIGHT:
    axo = fig.add_axes([0.015, 0.08, 0.36, 0.70], facecolor=SURF)   # overview
    axc = fig.add_axes([0.39, 0.06, 0.60, 0.92], facecolor=SURF)    # chase
else:
    axo = fig.add_axes([0.03, 0.10, 0.33, 0.68], facecolor=SURF)
    axc = fig.add_axes([0.38, 0.085, 0.60, 0.82], facecolor=SURF)
cam = [fl[0, 1:3].copy()]


def draw(i):
    t = frames[i]
    p = pose_at(t)
    pos, yaw = p[1:3], p[4]
    st, full = state_at(t)
    if t < t_liftoff:
        st, full = 'ARMED', ''
    elif t >= t_touch:
        st = 'LANDED'
    elif t >= t_land:
        st, full = 'LAND', ''
    holding = st in ('HOLD', 'COMMAND_HOLD', 'YAW_ALIGN')
    g = latest(grids, t) or grids[0]
    rgb, ext = grid_rgb(g)
    rt = latest(routes, t)
    # chase camera: smooth xy follow; the rotating view leads along the heading and puts it up the screen
    cam[0] = cam[0] + 0.3 * (pos - cam[0])
    if FIXED_CAM:
        ctr, rot = cam[0], 0.0
    else:
        unit = np.array([math.cos(cam_yaw[i]), math.sin(cam_yaw[i])])
        ctr, rot = cam[0] + 0.45 * unit, math.pi / 2 - cam_yaw[i]
    cx, cy = ctr
    if CAMERA:
        axo.cla(); axo.set_axis_off()
        im = latest(images, t)
        if im is not None:
            frame = np.asarray(Image.open(io.BytesIO(im[1])).convert('RGB'))
            axo.imshow(frame, interpolation='bilinear')
            if FEAT:
                f = latest(feats, t)
                if f is not None and len(f[1]):
                    # marker size in points: arm in output px -> camera px -> points; stroke exactly 1 output px
                    size = 2 * FEAT_ARM * PX
                    if FEAT == 'x':
                        axo.scatter(f[1][:, 0], f[1][:, 1], s=size ** 2, marker='x', c=ROUTE, linewidths=STROKE * PX)
                    else:
                        axo.scatter(f[1][:, 0], f[1][:, 1], s=size ** 2, marker='o', facecolors='none',
                                    edgecolors=ROUTE, linewidths=STROKE * PX)
            h, w = frame.shape[:2]
            x0 = (w - h) / 2 if SQUARE else 0.0   # centred square crop
            axo.set_xlim(x0 - 0.5, w - x0 - 0.5); axo.set_ylim(h - 0.5, -0.5)
            axo.add_patch(plt.Rectangle((0, 0), 1, 1, transform=axo.transAxes, fill=False, **HAIR))
    # overview
    if not CAMERA:
        axo.cla(); axo.set_axis_off()
        axo.imshow(rgb, origin='lower', extent=ext, interpolation='nearest')
        if rt is not None:
            axo.plot(rt[1][:, 0], rt[1][:, 1], color=ROUTE, lw=1.0, ls=(0, (3, 2)))
        done = fl[fl[:, 0] <= t]
        axo.plot(done[:, 1], done[:, 2], color=FLOWN, lw=1.4)
        axo.plot(*pos, 'o', color=HOLD if holding else INK, ms=5)
        if goal_xy is not None:
            axo.plot(*goal_xy, 'o', mfc=SURF, mec=INK, ms=7)
        if FIXED_CAM:
            axo.add_patch(plt.Rectangle((cx - HALF, cy - HALF), 2 * HALF, 2 * HALF, fill=False, ec=CMD, lw=1.0))
        else:
            axo.add_patch(plt.Circle((cx, cy), HALF, fill=False, ec=CMD, lw=0.8))
        axo.set_xlim(X0, X1); axo.set_ylim(Y0, Y1); axo.set_aspect('equal')
    # chase: everything drawn in camera coordinates (map -> centred on ctr, rotated by rot)
    tf = matplotlib.transforms.Affine2D().translate(-cx, -cy).rotate(rot) + axc.transData
    axc.cla(); axc.set_facecolor(UNKNOWN if LIGHT else SURF)
    im = axc.imshow(rgb, origin='lower', extent=ext, interpolation='nearest', transform=tf)
    if not FIXED_CAM:
        disc = plt.Circle((0, 0), HALF, transform=axc.transData)
        im.set_clip_path(disc)
    gl = dict(color='#000000', lw=0.4, alpha=0.08) if LIGHT else HAIR
    R = HALF * 1.5
    for gx in np.arange(math.floor(cx - R), cx + R + 1e-6, 0.5):
        h = math.sqrt(max(R ** 2 - (gx - cx) ** 2, 0))
        ln, = axc.plot([gx, gx], [cy - h, cy + h], transform=tf, **gl)
        if not FIXED_CAM:
            ln.set_clip_path(disc)
    for gy in np.arange(math.floor(cy - R), cy + R + 1e-6, 0.5):
        h = math.sqrt(max(R ** 2 - (gy - cy) ** 2, 0))
        ln, = axc.plot([cx - h, cx + h], [gy, gy], transform=tf, **gl)
        if not FIXED_CAM:
            ln.set_clip_path(disc)
    if not FIXED_CAM:
        axc.add_patch(plt.Circle((0, 0), HALF, fill=False, gid='rim', **HAIR))
    if rt is not None:
        axc.plot(rt[1][:, 0], rt[1][:, 1], color=ROUTE, lw=3, ls=(0, (4, 2)), transform=tf)
    if goal_xy is not None:
        axc.plot(*goal_xy, '*', color=GOAL, ms=16, transform=tf)
    eg = latest([e for e in goals if e[3] == '/planner/effective_goal'], t)
    if eg is not None and goal_xy is not None and np.hypot(eg[1] - goal_xy[0], eg[2] - goal_xy[1]) > 0.15:
        axc.plot(eg[1], eg[2], 'D', mfc=SURF, mec=ROUTE, ms=9, mew=1.6, transform=tf)
    tr = fl[(fl[:, 0] <= t) & (fl[:, 0] >= t - 6.0)]
    for k in range(1, len(tr)):
        axc.plot(tr[k - 1:k + 1, 1], tr[k - 1:k + 1, 2], color=INK2, lw=2.4, alpha=0.1 + 0.8 * k / len(tr), transform=tf)
    c = latest(carrot, t)
    if c is not None and t_liftoff <= t < t_touch and t - c[0] < 1.0 and not holding:
        axc.plot([pos[0], c[1]], [pos[1], c[2]], color=CMD, lw=1, alpha=0.7, transform=tf)
        axc.plot(c[1], c[2], 'o', color=CMD, ms=7, transform=tf)
    col = HOLD if holding else INK
    th = np.linspace(0, 2 * math.pi, 48)
    axc.plot(pos[0] + CAGE_R * np.cos(th), pos[1] + CAGE_R * np.sin(th), color=col, lw=1.3, alpha=0.6, transform=tf)
    for ang in (45, 135, 225, 315):
        a = yaw + math.radians(ang)
        tip = pos + ARM * np.array([math.cos(a), math.sin(a)])
        axc.plot([pos[0], tip[0]], [pos[1], tip[1]], color=col, lw=2.6, transform=tf)
        axc.plot(tip[0] + 0.06 * np.cos(th), tip[1] + 0.06 * np.sin(th), color=col, lw=1.2, transform=tf)
    axc.plot([pos[0], pos[0] + (CAGE_R + 0.08) * math.cos(yaw)], [pos[1], pos[1] + (CAGE_R + 0.08) * math.sin(yaw)],
             color=HOLD if holding else CMD, lw=2.6, transform=tf)
    if not FIXED_CAM:
        for a in axc.lines + axc.patches:   # keep route, trail, cage etc. inside the disc
            if a.get_clip_path() is None and a.get_gid() != 'rim':
                a.set_clip_path(disc)
    axc.set_xlim(-HALF, HALF); axc.set_ylim(-HALF * (1 if not FIXED_CAM else 0.92), HALF * (1 if not FIXED_CAM else 0.92))
    axc.set_aspect('equal')
    axc.set_xticks([]); axc.set_yticks([])
    for sp in axc.spines.values():
        sp.set_color('#e3e2dc')
        sp.set_visible(LIGHT or FIXED_CAM)
    # readout
    prev = pose_at(t - 0.5)
    spd = float(np.hypot(*(pos - prev[1:3]))) / 0.5
    lab = label.get(st, st.lower().replace('_', ' '))
    if st in ('HOLD', 'COMMAND_HOLD') and full:
        lab += ' - ' + ' '.join(full.split()[1:5]).rstrip(';:,')
    fig.texts.clear()
    fig.lines.clear()
    if NO_TEXT:
        return []
    if LIGHT:
        fig.text(0.02, 0.95, name.split('_')[-1] + '  global planner', color=INK2, fontsize=10, family='monospace')
        fig.text(0.02, 0.905, lab[:42], color=HOLD if holding else INK, fontsize=14, fontweight='bold')
        fig.text(0.02, 0.865, 't %5.1f s   z %.2f m   speed %.2f m/s' % (t - t_arm, height_at(t), spd), color=INK2,
                 fontsize=10, family='monospace')
        fig.text(0.02, 0.83, 'z = PX4 height vs takeoff   box on map = chase view', color=MUTED, fontsize=9)
        for x, txt, cc in [(0.02, 'route', ROUTE), (0.08, 'command', CMD), (0.17, 'drone + 50 cm cage', INK),
                           (0.355, 'trail (6 s)', INK2), (0.46, 'flown', FLOWN), (0.525, 'clearance band', '#c98a3a'),
                           (0.675, 'wall', '#1f2937'), (0.73, 'unknown', MUTED), (0.815, 'goal', GOAL)]:
            fig.text(x, 0.02, txt, color=cc, fontsize=10, fontweight='bold')
        return []
    # archive look: muted/ink title, hairline rules, lowercase light text, colour only in the swatches
    t1 = fig.text(0.03 * XS, 0.935, 'global/', color=MUTED, fontsize=15)
    w = t1.get_window_extent(renderer=fig.canvas.get_renderer()).width / fig.bbox.width
    fig.text(0.03 * XS + w, 0.935, name.split('_')[-1].lower(), color=INK, fontsize=15)
    fig.text(1 - 0.03 * XS, 0.935, '%5.1f s' % (t - t_arm), color=MUTED, fontsize=15, ha='right')
    fig.add_artist(matplotlib.lines.Line2D([0, 1], [0.915, 0.915], transform=fig.transFigure, **HAIR))
    fig.text(0.03 * XS, 0.875, lab.lower()[:42], color=HOLD if holding else INK, fontsize=11.5)
    fig.text(0.03 * XS, 0.845, 'height %.2f m    speed %.2f m/s' % (height_at(t), spd), color=MUTED, fontsize=11.5)
    fig.text(0.03 * XS, 0.815, 'height = px4 z vs takeoff' + ('' if CAMERA else '    circle = chase view'),
             color=MUTED, fontsize=9)
    fig.add_artist(matplotlib.lines.Line2D([0, 1], [0.075, 0.075], transform=fig.transFigure, **HAIR))
    for x, txt, cc in [(0.03, 'route', ROUTE), (0.12, 'command', CMD), (0.235, 'drone, 50 cm cage', INK),
                       (0.415, 'trail / flown', INK2), (0.545, 'wall', OCC), (0.625, 'clearance band', BAND),
                       (0.79, 'goal', GOAL)]:
        x *= XS
        fig.add_artist(matplotlib.lines.Line2D([x, x + 0.018 * XS], [0.036, 0.036], transform=fig.transFigure, color=cc,
                                               lw=2))
        fig.text(x + 0.026 * XS, 0.028, txt, color=MUTED, fontsize=11)
    return []


anim = FuncAnimation(fig, draw, frames=len(frames), interval=1000 / FPS, blit=False)
import imageio_ffmpeg  # noqa: E402
from matplotlib.animation import FFMpegWriter  # noqa: E402
matplotlib.rcParams['animation.ffmpeg_path'] = imageio_ffmpeg.get_ffmpeg_exe()
writer = FFMpegWriter(fps=FPS, codec='libx264', extra_args=[
    '-pix_fmt', 'yuv420p', '-crf', '20', '-preset', 'slow', '-movflags', '+faststart'])
out = '%s/%s_flight%s%s%s.mp4' % (out_dir, name, '' if FIXED_CAM else '_behind', '_camera' + ('_sq' if SQUARE else '') if CAMERA else '',
                                  ('_feat_' + FEAT + ('_w%g' % STROKE if STROKE != 1 else '')) if FEAT else '')
anim.save(out, writer=writer, dpi=120, savefig_kwargs={'facecolor': SURF})   # 1200x840 (1680x840 with camera)
print('wrote', out, '%d frames at %.0f fps, %.1f s' % (len(frames), FPS, len(frames) / FPS))
