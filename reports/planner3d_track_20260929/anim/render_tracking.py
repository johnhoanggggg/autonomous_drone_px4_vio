"""Simulated VIO front-end on the left camera of a ROS-backend 3D flight, as an MP4 in the
iwrzwr/visual-archive look (black page, off-white ink, muted grey, hairlines, lowercase sans,
one red accent). This is an OpenCV re-implementation of a typical VIO tracker, not the
RTAB-Map/Basalt tracker that flew:
  Shi-Tomasi corners (max 200, 14 px apart) -> pyramidal Lucas-Kanade -> forward-backward
  check (< 1 px) -> RANSAC fundamental matrix (1 px) rejects outliers -> top up with new corners.
Per frame: the camera image (slightly dimmed), every current feature (tracked or newly detected)
as a red cross, the planner3d state, counts, and a whole-flight sparkline of tracked features.
No optical-flow trails are drawn.
Times are bag time since the bag start, the same clock as animate3d.py.
Usage (system cv2 needs system numpy): PYTHONNOUSERSITE=1 python3 render_tracking.py <bag_dir> <out.mp4>
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
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

sys.path.append('/home/john/autonomous_drone_px4_vio/.venv-mavlink/lib/python3.12/site-packages')
import imageio_ffmpeg  # noqa: E402

bag, out = sys.argv[1], sys.argv[2]
MAX_FEAT, MIN_DIST, FB_PX, RANSAC_PX, TRAIL = 200, 14, 1.0, 1.0, 12
ARM = 12  # feature cross half-size in output px (stroke 1 px)
LK = dict(winSize=(21, 21), maxLevel=3, criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))

# palette (BGR) from the archive CSS: --page #000, --ink #f7f7f4, --muted #777671, hairline 12% white
PAGE, INK, MUTED, HAIR = (0, 0, 0), (244, 247, 247), (113, 118, 119), (31, 31, 31)
RED, HOLD = (0, 0, 255), (62, 150, 255)
FONT = '/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf'
F_TITLE, F_TEXT, F_SMALL = ImageFont.truetype(FONT, 22), ImageFont.truetype(FONT, 16), ImageFont.truetype(FONT, 14)
W, H, PAD = 1200, 890, 40
IMG_Y, IMG_W, IMG_H = 104, W - 2 * PAD, 700
STATE = {'CLIMB': 'climbing', 'WAITING_FOR_GOAL': 'waiting for route', 'ROUTE_START': 'route start',
         'FOLLOWING': 'following route', 'HOLD': 'hold', 'YAW_ALIGN': 'turning to heading',
         'GOAL_REACHED': 'goal reached', 'SAFE_APPROACH_REACHED': 'approach reached', 'LAND': 'landing'}

# ---------------------------------------------------------------- read the bag
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
r.set_filter(rosbag2_py.StorageFilter(topics=['/oak/left/image_rect/compressed', '/planner3d/flight/status']))
t0 = r.get_metadata().starting_time.nanoseconds
frames, st_t, st = [], [], []
while r.has_next():
    tp, d, ns = r.read_next()
    t = (ns - t0) / 1e9
    if tp.endswith('compressed'):
        frames.append((t, bytes(deserialize_message(d, CompressedImage).data)))
    else:
        try:
            s = json.loads(deserialize_message(d, String).data)
        except ValueError:
            continue
        st_t.append(t)
        st.append((s.get('state', ''), s.get('fault') or ''))
frames.sort(key=lambda f: f[0])
fps = (len(frames) - 1) / (frames[-1][0] - frames[0][0])

# ---------------------------------------------------------------- track (pass 1)
tracks = {}          # id -> list of (x, y) in image pixels
pts, ids = np.zeros((0, 2), np.float32), []
next_id = 0
prev = None
per_frame = []
for t, data in frames:
    img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_GRAYSCALE)
    h, w = img.shape
    rejected, flow = [], []
    if prev is not None and len(pts):
        p1, ok, _ = cv2.calcOpticalFlowPyrLK(prev, img, pts, None, **LK)
        p0r, ok2, _ = cv2.calcOpticalFlowPyrLK(img, prev, p1, None, **LK)
        good = (ok.ravel() == 1) & (ok2.ravel() == 1) & (np.linalg.norm(p0r - pts, axis=1) < FB_PX)
        good &= (p1[:, 0] >= 0) & (p1[:, 0] < w) & (p1[:, 1] >= 0) & (p1[:, 1] < h)
        inl = good.copy()
        if good.sum() >= 8:
            _, m = cv2.findFundamentalMat(pts[good], p1[good], cv2.FM_RANSAC, RANSAC_PX, 0.99)
            if m is not None:
                inl[np.flatnonzero(good)] = m.ravel() == 1
        rejected = [(pts[k].copy(), p1[k].copy()) for k in np.flatnonzero(good & ~inl)]
        keep = np.flatnonzero(inl)
        for k in np.flatnonzero(~inl):
            tracks.pop(ids[k], None)
        ids = [ids[k] for k in keep]
        pts = p1[keep]
        for k, i in enumerate(ids):
            tracks[i].append(tuple(pts[k]))
            del tracks[i][:-TRAIL]
        flow = [np.hypot(tracks[i][-1][0] - tracks[i][-2][0], tracks[i][-1][1] - tracks[i][-2][1]) for i in ids]
    new = []
    if len(pts) < MAX_FEAT:
        mask = np.full(img.shape, 255, np.uint8)
        for x, y in pts:
            cv2.circle(mask, (int(x), int(y)), MIN_DIST, 0, -1)
        c = cv2.goodFeaturesToTrack(img, MAX_FEAT - len(pts), 0.01, MIN_DIST, mask=mask, blockSize=7)
        if c is not None:
            c = c.reshape(-1, 2).astype(np.float32)
            for x, y in c:
                tracks[next_id] = [(x, y)]
                ids.append(next_id)
                next_id += 1
            pts = np.vstack([pts, c])
            new = [tuple(p) for p in c]
    per_frame.append(dict(
        t=t,
        tracks=[(list(tracks[i]), len(tracks[i])) for i in ids if len(tracks[i]) > 1],
        new=new, rejected=rejected,
        n=sum(1 for i in ids if len(tracks[i]) > 1),
        flow=float(np.median(flow)) if len(flow) else 0.0))
    prev = img
counts = np.array([f['n'] for f in per_frame], float)
times = np.array([f['t'] for f in per_frame])

# ---------------------------------------------------------------- render (pass 2)
sx, sy = IMG_W / w, IMG_H / h
S = lambda p: (int(round(PAD + p[0] * sx)), int(round(IMG_Y + p[1] * sy)))  # noqa: E731


def text_w(s, font):
    return font.getlength(s)


SP_X0, SP_X1, SP_Y0, SP_Y1 = PAD + 150, 700, 836, 866
legend = [('feature', 'cross', RED)]
stamp = bag.rstrip('/').split('_')[-1].lower()
ff = subprocess.Popen([imageio_ffmpeg.get_ffmpeg_exe(), '-v', 'error', '-y', '-f', 'rawvideo', '-pix_fmt', 'bgr24',
                       '-s', '%dx%d' % (W, H), '-r', '%.4f' % fps, '-i', '-', '-c:v', 'libx264', '-pix_fmt', 'yuv420p',
                       '-crf', '20', '-preset', 'slow', '-movflags', '+faststart', out], stdin=subprocess.PIPE)
for (t, data), f in zip(frames, per_frame):
    img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_GRAYSCALE)
    can = np.zeros((H, W, 3), np.uint8)
    base = cv2.resize(img, (IMG_W, IMG_H), interpolation=cv2.INTER_LINEAR)
    can[IMG_Y:IMG_Y + IMG_H, PAD:PAD + IMG_W] = cv2.cvtColor((base * 0.78).astype(np.uint8), cv2.COLOR_GRAY2BGR)
    cv2.rectangle(can, (PAD - 1, IMG_Y - 1), (PAD + IMG_W, IMG_Y + IMG_H), HAIR, 1)
    # features: red crosses at every current feature, tracked or new
    for p in [hist[-1] for hist, _ in f['tracks']] + f['new']:
        x, y = S(p)
        cv2.line(can, (x - ARM, y - ARM), (x + ARM, y + ARM), RED, 1, cv2.LINE_AA)
        cv2.line(can, (x - ARM, y + ARM), (x + ARM, y - ARM), RED, 1, cv2.LINE_AA)
    # hairlines + sparkline
    cv2.line(can, (0, 64), (W, 64), HAIR, 1)
    cv2.line(can, (0, 824), (W, 824), HAIR, 1)
    cmax = max(MAX_FEAT, counts.max())
    xs = SP_X0 + (times - times[0]) / (times[-1] - times[0]) * (SP_X1 - SP_X0)
    ys = SP_Y1 - counts / cmax * (SP_Y1 - SP_Y0)
    cv2.polylines(can, [np.stack([xs, ys], 1).round().astype(np.int32)], False, MUTED, 1, cv2.LINE_AA)
    k = bisect.bisect_right(times, t) - 1
    cv2.line(can, (int(xs[k]), SP_Y0 - 4), (int(xs[k]), SP_Y1 + 4), INK, 1)
    cv2.circle(can, (int(xs[k]), int(ys[k])), 3, INK, -1, cv2.LINE_AA)
    # legend markers (drawn right-aligned; labels go on in the text pass)
    lx = W - PAD
    marks = []
    for label, kind, col in reversed(legend):
        lx -= text_w(label, F_SMALL)
        marks.append((lx, label))
        lx -= 14
        x = int(lx)
        if kind == 'dot':
            cv2.circle(can, (x, 852), 3, col, -1, cv2.LINE_AA)
        elif kind == 'ring':
            cv2.circle(can, (x, 852), 5, col, 1, cv2.LINE_AA)
        elif kind == 'cross':
            x -= ARM - 4
            cv2.line(can, (x - ARM, 852 - ARM), (x + ARM, 852 + ARM), col, 1, cv2.LINE_AA)
            cv2.line(can, (x - ARM, 852 + ARM), (x + ARM, 852 - ARM), col, 1, cv2.LINE_AA)
        lx -= 28
    # text
    pil = Image.fromarray(can[:, :, ::-1])
    dr = ImageDraw.Draw(pil)
    rgb = lambda c: c[::-1]  # noqa: E731
    dr.text((PAD, 24), 'tracking/', font=F_TITLE, fill=rgb(MUTED))
    dr.text((PAD + text_w('tracking/', F_TITLE), 24), stamp, font=F_TITLE, fill=rgb(INK))
    ts = '%5.1f s' % t
    dr.text((W - PAD - text_w(ts, F_TITLE), 24), ts, font=F_TITLE, fill=rgb(MUTED))
    i = bisect.bisect_right(st_t, t) - 1
    state, fault = st[i] if i >= 0 else ('', '')
    lab = STATE.get(state, state.lower().replace('_', ' ')) or 'on the ground'
    if state == 'HOLD' and fault:
        lab += ' - ' + fault.lower().replace('_', ' ')
    holding = state in ('HOLD', 'YAW_ALIGN')
    dr.text((PAD, 76), lab, font=F_TEXT, fill=rgb(HOLD if holding else INK))
    stats = '%d tracked    %d new    %d rejected' % (f['n'], len(f['new']), len(f['rejected']))
    dr.text((W - PAD - text_w(stats, F_TEXT), 76), stats, font=F_TEXT, fill=rgb(MUTED))
    dr.text((PAD, 843), 'tracked features', font=F_SMALL, fill=rgb(MUTED))
    for x, label in marks:
        dr.text((x, 843), label, font=F_SMALL, fill=rgb(MUTED))
    ff.stdin.write(np.asarray(pil)[:, :, ::-1].tobytes())
ff.stdin.close()
ff.wait()
print('wrote', out, '%d frames at %.2f fps, %.1f s; median tracked %d, %d tracks total'
      % (len(frames), fps, frames[-1][0] - frames[0][0], np.median(counts), next_id))
