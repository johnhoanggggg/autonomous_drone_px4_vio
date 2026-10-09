#!/usr/bin/env python3
import sys, json, numpy as np
from scipy.spatial.transform import Rotation as R

def stats(path, warm=5.0):
    d = np.load(path + '.npy'); meta = json.load(open(path + '.meta.json'))
    if len(d) < 10: return dict(name=path, n=len(d))
    sentinel = np.all(np.abs(d[:, 2:5]) < 1e-9, axis=1)
    d = d[(d[:, 0] > warm) & ~sentinel]
    t, p = d[:, 0], d[:, 2:5]
    rot = R.from_quat(d[:, 5:9])
    ypr = rot.as_euler('ZYX', degrees=True)
    step = np.linalg.norm(np.diff(p, axis=0), axis=1)
    exc = np.linalg.norm(p - p[0], axis=1)
    dur = t[-1] - t[0]
    dyaw = (rot[-1] * rot[0].inv()).as_rotvec(degrees=True)
    late = t > t[0] + 25
    pl, tl = p[late], t[late]
    late_rate = 1000 * np.linalg.norm(pl[-1] - pl[0]) / (tl[-1] - tl[0]) * 60 if late.sum() > 10 else float('nan')
    late_yaw = (rot[-1] * rot[np.argmax(late)].inv()).as_rotvec(degrees=True)[2] / (tl[-1] - tl[0]) * 60 if late.sum() > 10 else float('nan')
    lat = d[:, 1] * 1000
    return dict(lat_p50=float(np.median(lat)), lat_p95=float(np.percentile(lat, 95)), late_mm_min=late_rate, late_yaw_deg_min=float(late_yaw), name=path.split('/')[-1], hz=(len(t) - 1) / dur, dur=dur, sentinel=int(sentinel.sum()),
                drift_mm=1000 * exc[-1], drift_mm_per_min=1000 * exc[-1] / dur * 60, max_exc_mm=1000 * exc.max(),
                step_rms_mm=1000 * np.sqrt((step ** 2).mean()), step_max_mm=1000 * step.max(),
                rot_drift_deg=float(np.linalg.norm(dyaw)), yaw_drift_deg=float(dyaw[2]),
                roll=float(np.median(ypr[:, 2])), pitch=float(np.median(ypr[:, 1])), cpu=meta['cpu_pct'],
                gap_max_ms=1000 * np.diff(t).max())

if __name__ == '__main__':
    cols = ['name', 'hz', 'sentinel', 'drift_mm', 'late_mm_min', 'late_yaw_deg_min', 'max_exc_mm', 'step_rms_mm', 'step_max_mm',
            'lat_p50', 'lat_p95', 'roll', 'cpu', 'gap_max_ms']
    print(' '.join(f'{c[:12]:>12}' for c in cols))
    for pth in sys.argv[1:]:
        s = stats(pth)
        print(' '.join(f'{s.get(c, float("nan")):>12.3f}' if not isinstance(s.get(c), str) else f'{s[c][:12]:>12}' for c in cols))
