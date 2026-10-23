"""AR view of a ROS-backend planner3d flight: the OctoMap, the planner's route, the command point and
the goal projected into the left camera image, in the iwrzwr/visual-archive look (black page,
off-white ink, muted grey, hairlines, lowercase sans; red route, blue command, orange hold).

Camera pose per image: /planner3d/pose_snapshot 'pose' (the oak frame in world, map correction
applied) nearest the image stamp, times the /tf_static chain oak -> oak_left_camera_optical_frame.
Intrinsics: the bag has no camera_info, so they come from the OAK's saved device calibration (stereo
pair calibrated at 640x480, f 456 px; the 640x400 stream is a 40-row centre crop, so cy - 40).
Override with --cx/--cy/--f if the overlay sits off.
Map voxels behind nearer voxels are culled with a coarse depth buffer; route samples hidden behind
the map are drawn faint. The drone itself is the camera, so it cannot appear here.
Times are bag time since the bag start, the same clock as animate3d.py and render_tracking.py.
Usage: PYTHONNOUSERSITE=1 python3 render_ar.py <bag_dir> <out.mp4> [--cx=316.4] [--cy=194.3] [--f=456.2]
       --at=t1,t2,... writes stills <out>_<t>.png at those bag times instead of the video (for checking alignment)
"""
import bisect
import json
import subprocess
import sys

import cv2
import numpy as np
import rosbag2_py
from PIL import Image, ImageDraw, ImageFont
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

sys.path.append('/home/john/autonomous_drone_px4_vio/.venv-mavlink/lib/python3.12/site-packages')
sys.path.insert(0, '/home/john/autonomous_drone_px4_vio/scripts')
import imageio_ffmpeg  # noqa: E402
from octomap_binary import decode_binary, expand  # noqa: E402

opts = dict(a[2:].split('=', 1) for a in sys.argv[1:] if a.startswith('--'))
bag, out = [a for a in sys.argv[1:] if not a.startswith('--')][:2]
FX = FY = float(opts.get('f', 456.2))
CX, CY = float(opts.get('cx', 316.4)), float(opts.get('cy', 194.3))
CAM_FRAME = 'oak_left_camera_optical_frame'

PAGE, INK, MUTED, HAIR = (0, 0, 0), (244, 247, 247), (113, 118, 119), (31, 31, 31)
RED, CMD, HOLD, FLOOR = (0, 0, 255), (255, 156, 51), (62, 150, 255), (150, 150, 150)
FONT = '/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf'
F_TITLE, F_TEXT, F_SMALL = ImageFont.truetype(FONT, 22), ImageFont.truetype(FONT, 16), ImageFont.truetype(FONT, 14)
W, H, PAD = 1200, 890, 40
IMG_Y, IMG_W, IMG_H = 104, W - 2 * PAD, 700
STATE = {'CLIMB': 'climbing', 'WAITING_FOR_GOAL': 'waiting for route', 'ROUTE_START': 'route start',
         'FOLLOWING': 'following route', 'HOLD': 'hold', 'YAW_ALIGN': 'turning to heading',
         'GOAL_REACHED': 'goal reached', 'SAFE_APPROACH_REACHED': 'approach reached', 'LAND': 'landing'}


def qmat(x, y, z, w):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def tmat(t, q):
    T = np.eye(4)
    T[:3, :3] = qmat(*q)
    T[:3, 3] = t
    return T


# ---------------------------------------------------------------- read the bag
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
types = {t.name: t.type for t in r.get_all_topics_and_types()}
want = ['/oak/left/image_rect/compressed', '/planner3d/flight/status', '/planner3d/pose_snapshot',
        '/planner3d/reference_path', '/rtabmap/octomap', '/waypoint/clicked', '/tf_static']
r.set_filter(rosbag2_py.StorageFilter(topics=[w for w in want if w in types]))
t0 = r.get_metadata().starting_time.nanoseconds
images, status, snaps, routes, maps, goals, tf = [], [], [], [], [], [], {}
while r.has_next():
    tp, d, ns = r.read_next()
    t = (ns - t0) / 1e9
    m = deserialize_message(d, get_message(types[tp]))
    if tp == '/oak/left/image_rect/compressed':
        images.append((t, m.header.stamp.sec * 10**9 + m.header.stamp.nanosec, bytes(m.data)))
    elif tp in ('/planner3d/flight/status', '/planner3d/pose_snapshot'):
        try:
            v = json.loads(m.data)
        except ValueError:
            continue
        if tp.endswith('status'):
            status.append((t, v))
        elif v.get('pose'):
            snaps.append((v['stamp_ns'], v['pose']))
    elif tp == '/planner3d/reference_path':
        routes.append((t, np.array([[p.pose.position.x, p.pose.position.y, p.pose.position.z] for p in m.poses])))
    elif tp == '/rtabmap/octomap':
        maps.append((t, bytes(m.data), m.resolution))
    elif tp == '/waypoint/clicked':
        goals.append((t, np.array([m.point.x, m.point.y, m.point.z])))
    else:
        for x in m.transforms:
            tr, q = x.transform.translation, x.transform.rotation
            tf[x.child_frame_id] = (x.header.frame_id, tmat((tr.x, tr.y, tr.z), (q.x, q.y, q.z, q.w)))
images.sort(key=lambda f: f[0])
snaps.sort(key=lambda s: s[0])
snap_ns = [s[0] for s in snaps]
fps = (len(images) - 1) / (images[-1][0] - images[0][0])

# oak -> left optical, from the static TF chain
T_oak_cam, f = np.eye(4), CAM_FRAME
while f != 'oak':
    parent, T = tf[f]
    T_oak_cam = T @ T_oak_cam
    f = parent
ground_z = next((v['ground_map_z'] for _, v in status if v.get('ground_map_z') is not None), None)
if ground_z is None:
    ground_z = min(s[1][2] for s in snaps) - 0.05


def latest(rows, t):
    i = bisect.bisect_right([row[0] for row in rows], t) - 1
    return rows[i] if i >= 0 else None


_map = {}


def voxels_at(t):
    row = latest(maps, t)
    if row is None:
        return np.zeros((0, 3))
    if row[0] not in _map:
        c, s = decode_binary(row[1], row[2])
        _map.clear()
        _map[row[0]] = expand(c, s, row[2])
    return _map[row[0]]


def project(Pw, T_cw):
    """World points -> (u, v, depth) in the 640x400 image."""
    Pc = (Pw - T_cw[:3, 3]) @ T_cw[:3, :3]
    z = Pc[:, 2]
    with np.errstate(divide='ignore', invalid='ignore'):
        u = FX * Pc[:, 0] / z + CX
        v = FY * Pc[:, 1] / z + CY
    return u, v, z


# ---------------------------------------------------------------- render
STILLS = [float(x) for x in opts['at'].split(',')] if 'at' in opts else None
if STILLS:
    _ti = [f[0] for f in images]
    images = [images[int(np.argmin(np.abs(np.array(_ti) - s)))] for s in STILLS]
ff = None if STILLS else subprocess.Popen([imageio_ffmpeg.get_ffmpeg_exe(), '-v', 'error', '-y', '-f', 'rawvideo', '-pix_fmt', 'bgr24',
                       '-s', '%dx%d' % (W, H), '-r', '%.4f' % fps, '-i', '-', '-c:v', 'libx264', '-pix_fmt', 'yuv420p',
                       '-crf', '20', '-preset', 'slow', '-movflags', '+faststart', out], stdin=subprocess.PIPE)
stamp = bag.rstrip('/').split('_')[-1].lower()
legend = [('map', 'dot', INK), ('floor', 'small', FLOOR), ('route', 'line', RED), ('route, hidden', 'faint', RED),
          ('command', 'dot', CMD), ('goal', 'ring', INK)]
max_dt = []
for t, stamp_ns, data in images:
    img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_GRAYSCALE)
    h, w = img.shape
    sx, sy = IMG_W / w, IMG_H / h
    can = np.zeros((H, W, 3), np.uint8)
    base = cv2.resize(img, (IMG_W, IMG_H), interpolation=cv2.INTER_LINEAR)
    can[IMG_Y:IMG_Y + IMG_H, PAD:PAD + IMG_W] = cv2.cvtColor((base * 0.78).astype(np.uint8), cv2.COLOR_GRAY2BGR)
    st = latest(status, t)
    b = st[1] if st else {}
    k = bisect.bisect_left(snap_ns, stamp_ns)
    k = min((j for j in (k - 1, k) if 0 <= j < len(snaps)), key=lambda j: abs(snap_ns[j] - stamp_ns))
    dt = abs(snap_ns[k] - stamp_ns) / 1e9
    max_dt.append(dt)
    pose_ok = dt < 0.15
    view = can[IMG_Y:IMG_Y + IMG_H, PAD:PAD + IMG_W]
    goal_px = None
    if pose_ok:
        p = snaps[k][1]
        T_cw = tmat(p[:3], p[3:]) @ T_oak_cam
        # map: depth-buffer cull on a 6 px grid, then dots sized by distance
        V = voxels_at(t)
        u, v, z = project(V, T_cw)
        m = (z > 0.15) & (z < 6.0) & (u >= 0) & (u < w) & (v >= 0) & (v < h)
        V, u, v, z = V[m], u[m], v[m], z[m]
        cell = (v // 6).astype(int) * (w // 6 + 1) + (u // 6).astype(int)
        zmin = np.full((h // 6 + 1) * (w // 6 + 1), np.inf)
        np.minimum.at(zmin, cell, z)
        keep = z <= zmin[cell] + 0.25
        floor = V[:, 2] < ground_z + 0.08
        for near, far, alpha in ((3.0, 6.0, 0.25), (1.5, 3.0, 0.45), (0.0, 1.5, 0.7)):
            layer = view.copy()
            sel = np.flatnonzero(keep & (z >= near) & (z < far))
            for i in sel[np.argsort(-z[sel])]:
                c = (int(u[i] * sx), int(v[i] * sy))
                rad = int(np.clip(0.05 * FX / z[i] * sx * 0.1, 1, 3))
                cv2.circle(layer, c, rad, FLOOR if floor[i] else INK, -1, cv2.LINE_AA)
            cv2.addWeighted(layer, alpha, view, 1 - alpha, 0, dst=view)
        # route: dense samples, hidden where the map is nearer
        rt = latest(routes, t)
        if rt is not None and len(rt[1]) >= 2:
            R = rt[1]
            seg = [R[j] + (R[j + 1] - R[j]) * s for j in range(len(R) - 1)
                   for s in np.linspace(0, 1, max(2, int(np.linalg.norm(R[j + 1] - R[j]) / 0.02)), endpoint=False)]
            seg = np.array(seg + [R[-1]])
            ru, rv, rz = project(seg, T_cw)
            ok = rz > 0.15
            inside = ok & (ru >= -w) & (ru < 2 * w) & (rv >= -h) & (rv < 2 * h)
            ci = np.clip((rv // 6).astype(int), 0, h // 6) * (w // 6 + 1) + np.clip((ru // 6).astype(int), 0, w // 6)
            hidden = inside & (ru >= 0) & (ru < w) & (rv >= 0) & (rv < h) & (rz > zmin[np.where(inside, ci, 0)] + 0.15)
            faint = view.copy()
            pairs = [((int(ru[j] * sx), int(rv[j] * sy)), (int(ru[j + 1] * sx), int(rv[j + 1] * sy)), hidden[j] or hidden[j + 1])
                     for j in range(len(seg) - 1) if inside[j] and inside[j + 1]]
            for a, c, hid in pairs:          # dark edge first, so joins stay clean
                if not hid:
                    for img_ in (view, faint):
                        cv2.line(img_, a, c, PAGE, 5, cv2.LINE_AA)
            for a, c, hid in pairs:
                for img_ in ((faint,) if hid else (view, faint)):
                    cv2.line(img_, a, c, RED, 2, cv2.LINE_AA)
            cv2.addWeighted(faint, 0.35, view, 0.65, 0, dst=view)
        # command point
        cmd = b.get('command_map')
        if cmd and b.get('state') not in ('HOLD', 'YAW_ALIGN'):
            cu, cv_, cz = project(np.array([cmd]), T_cw)
            if cz[0] > 0.15 and 0 <= cu[0] < w and 0 <= cv_[0] < h:
                c = (int(cu[0] * sx), int(cv_[0] * sy))
                cv2.circle(view, c, 6, PAGE, -1, cv2.LINE_AA)
                cv2.circle(view, c, 5, CMD, -1, cv2.LINE_AA)
                cv2.circle(view, c, 11, CMD, 1, cv2.LINE_AA)
        # goal
        g = latest(goals, t)
        if g is not None:
            gp = g[1].copy()
            gp[2] = max(gp[2], ground_z + 0.3)
            gu, gv, gz = project(np.array([gp]), T_cw)
            if gz[0] > 0.15 and 0 <= gu[0] < w and 0 <= gv[0] < h:
                goal_px = (int(gu[0] * sx), int(gv[0] * sy))
                rad = int(np.clip(0.15 * FX / gz[0] * sx, 6, 40))
                cv2.circle(view, goal_px, rad, PAGE, 3, cv2.LINE_AA)
                cv2.circle(view, goal_px, rad, INK, 1, cv2.LINE_AA)
                cv2.drawMarker(view, goal_px, INK, cv2.MARKER_CROSS, 8, 1, cv2.LINE_AA)
    cv2.rectangle(can, (PAD - 1, IMG_Y - 1), (PAD + IMG_W, IMG_Y + IMG_H), HAIR, 1)
    cv2.line(can, (0, 64), (W, 64), HAIR, 1)
    cv2.line(can, (0, 824), (W, 824), HAIR, 1)
    # legend markers, right-aligned; labels go on in the text pass
    lx, marks = W - PAD, []
    for label, kind, col in reversed(legend):
        lx -= F_SMALL.getlength(label)
        marks.append((lx, label))
        x = int(lx) - 14
        if kind == 'dot':
            cv2.circle(can, (x, 852), 4, col, -1, cv2.LINE_AA)
        elif kind == 'small':
            cv2.circle(can, (x, 852), 2, col, -1, cv2.LINE_AA)
        elif kind == 'ring':
            cv2.circle(can, (x, 852), 6, col, 1, cv2.LINE_AA)
        else:
            col = col if kind == 'line' else tuple(int(c * 0.4) for c in col)
            cv2.line(can, (x - 10, 852), (x + 6, 852), col, 2, cv2.LINE_AA)
            x -= 4
        lx = x - 28
    # text
    pil = Image.fromarray(can[:, :, ::-1])
    dr = ImageDraw.Draw(pil)
    rgb = lambda c: c[::-1]  # noqa: E731
    dr.text((PAD, 24), 'planner3d ar/', font=F_TITLE, fill=rgb(MUTED))
    dr.text((PAD + F_TITLE.getlength('planner3d ar/'), 24), stamp, font=F_TITLE, fill=rgb(INK))
    ts = '%5.1f s' % t
    dr.text((W - PAD - F_TITLE.getlength(ts), 24), ts, font=F_TITLE, fill=rgb(MUTED))
    state, fault = b.get('state', ''), b.get('fault') or ''
    lab = STATE.get(state, state.lower().replace('_', ' ')) or 'on the ground'
    if state == 'HOLD' and fault:
        lab += ' - ' + fault.lower().replace('_', ' ')
    holding = state in ('HOLD', 'YAW_ALIGN')
    dr.text((PAD, 76), lab, font=F_TEXT, fill=rgb(HOLD if holding else INK))
    if b.get('body_map'):
        right = 'height %.2f m' % (b['body_map'][2] - ground_z)
    else:
        right = 'pose stale'
    if not pose_ok:
        right = 'no camera pose'
    dr.text((W - PAD - F_TEXT.getlength(right), 76), right, font=F_TEXT, fill=rgb(MUTED))
    if goal_px:
        dr.text((PAD + goal_px[0] + 14, IMG_Y + goal_px[1] - 22), 'goal', font=F_SMALL, fill=rgb(INK))
    for x, label in marks:
        dr.text((x, 843), label, font=F_SMALL, fill=rgb(MUTED))
    if STILLS:
        pil.save('%s_%.1f.png' % (out.rsplit('.', 1)[0], t))
    else:
        ff.stdin.write(np.asarray(pil)[:, :, ::-1].tobytes())
if ff:
    ff.stdin.close()
    ff.wait()
md = np.array(max_dt)
print('wrote', out, '%d frames at %.2f fps; image-pose match median %.1f ms, %d frames without pose'
      % (len(images), fps, np.median(md) * 1e3, int((md >= 0.15).sum())), 'cx %.1f cy %.1f f %.1f' % (CX, CY, FX))
