#!/usr/bin/env python3
"""Build reports/stop_start_20260831.html from the 2026-08-31T064x flight bags.

Kept next to its output so the figures can be regenerated when the bags are
re-analysed. Run from the repo root with the ROS 2 environment sourced.
"""
import base64, glob, json, os, re, sys
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LOGS = os.path.join(ROOT, "ros2_ws", "flight_logs")
FLIGHTS = ["offboard_global_20260831T064152Z",
           "offboard_global_20260831T064236Z",
           "offboard_global_20260831T064828Z"]
OLD = ["offboard_global_20260831T033557Z", "offboard_global_20260831T033736Z",
       "offboard_global_20260830T132311Z"]
TOPICS = {"/planner/flight/status", "/planner/follower/path_generation"}


def series(bag):
    mcap = glob.glob(os.path.join(LOGS, bag, "*.mcap"))[0]
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=mcap, storage_id="mcap"),
           rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    d = {k: [] for k in TOPICS}
    while r.has_next():
        t, data, ts = r.read_next()
        if t in TOPICS:
            try: d[t].append((ts, deserialize_message(data, get_message(types[t]))))
            except Exception: pass
    for k in d: d[k].sort(key=lambda x: x[0])
    t0 = min(v[0][0] for v in d.values() if v)
    T = lambda ts: (ts - t0) / 1e9
    spd = [(T(ts), float(m.group(1)))
           for ts, msg in d["/planner/flight/status"]
           for m in [re.search(r"command_speed=([\d.]+)", msg.data)] if m]
    gen = [(T(ts), msg.data) for ts, msg in d["/planner/follower/path_generation"]]
    gt = np.array([t for t, _ in gen]); gv = np.array([v for _, v in gen])
    rebuilds = gt[1:][np.diff(gv) != 0] if len(gt) > 1 else np.array([])
    return np.array([t for t, _ in spd]), np.array([v for _, v in spd]), rebuilds


BINS = [(0.0, 0.2), (0.2, 0.4), (0.4, 0.6), (0.6, 0.9), (0.9, 1.5), (1.5, np.inf)]
LABELS = ["0–0.2", "0.2–0.4", "0.4–0.6", "0.6–0.9", "0.9–1.5", "1.5 +"]


def profile(times, values, rebuilds):
    since = np.full(len(times), np.inf)
    for i, when in enumerate(times):
        earlier = rebuilds[rebuilds <= when]
        if len(earlier): since[i] = when - earlier[-1]
    out = []
    for lo, hi in BINS:
        w = (since >= lo) & (since < hi)
        out.append(float(values[w].mean()) if w.sum() >= 3 else None)
    return out


def rate(times, rebuilds):
    span = times[-1] - times[0]
    return len(rebuilds) / span if span > 0 else 0.0


data = {b: series(b) for b in FLIGHTS}
profiles = {b: profile(*data[b]) for b in FLIGHTS}
rates = {b: rate(data[b][0], data[b][2]) for b in FLIGHTS}
old_rates = {}
for b in OLD:
    t, v, g = series(b)
    old_rates[b] = rate(t, g)

# ---- chart 1: commanded speed vs time since rebuild -------------------------
W, H, L, R, TP, BM = 880, 320, 58, 20, 26, 62
pw, ph = W - L - R, H - TP - BM
ymax = 0.21
COLS = ["var(--ink)", "var(--peachdeep)", "var(--corr)"]
NAMES = ["06:41:52", "06:42:36", "06:48:28"]
g, bars = [], []
for y in (0.05, 0.10, 0.15, 0.20):
    yy = TP + ph * (1 - y / ymax)
    g.append(f'<line x1="{L}" y1="{yy:.1f}" x2="{W-R}" y2="{yy:.1f}" stroke="var(--rule)" stroke-width="1"/>')
    g.append(f'<text x="{L-8}" y="{yy+3.5:.1f}" text-anchor="end" class="ax">{y:.2f}</text>')
slot = pw / len(BINS)
for bi in range(len(BINS)):
    x0 = L + bi * slot
    g.append(f'<text x="{x0+slot/2:.1f}" y="{H-38}" text-anchor="middle" class="ax">{LABELS[bi]}</text>')
    for fi, b in enumerate(FLIGHTS):
        v = profiles[b][bi]
        if v is None: continue
        bw = slot * 0.22
        bx = x0 + slot * 0.16 + fi * bw
        by = TP + ph * (1 - v / ymax)
        bars.append(f'<rect x="{bx:.1f}" y="{by:.1f}" width="{bw-2:.1f}" '
                    f'height="{TP+ph-by:.1f}" fill="{COLS[fi]}" opacity="0.88"/>')
ceil_y = TP + ph * (1 - 0.20 / ymax)
legend = "".join(
    f'<rect x="{L+i*150}" y="{H-20}" width="10" height="10" fill="{COLS[i]}"/>'
    f'<text x="{L+i*150+16}" y="{H-11}" class="ax">{NAMES[i]}</text>'
    for i in range(3))
chart1 = f'''<svg viewBox="0 0 {W} {H}" role="img" aria-label="Commanded speed against time since the last path rebuild">
<g>{''.join(g)}</g><g>{''.join(bars)}</g>
<line x1="{L}" y1="{ceil_y:.1f}" x2="{W-R}" y2="{ceil_y:.1f}" stroke="var(--ok)" stroke-width="1.5" stroke-dasharray="5 3"/>
<text x="{W-R}" y="{ceil_y-6:.1f}" text-anchor="end" class="thr" style="fill:var(--ok)">command_speed ceiling 0.20 m/s</text>
<text x="{L-8}" y="{TP-10}" text-anchor="end" class="ax">m/s</text>
<text x="{L+pw/2:.1f}" y="{H-52}" text-anchor="middle" class="ax">seconds since the last path rebuild</text>
{legend}</svg>'''

# ---- chart 2: rebuild rate before/after -------------------------------------
W2, H2, L2 = 880, 190, 150
pw2 = W2 - L2 - 130
rows = [("30 Aug 13:23", old_rates[OLD[2]], "0.12", False),
        ("31 Aug 03:35", old_rates[OLD[0]], "0.12", False),
        ("31 Aug 03:37", old_rates[OLD[1]], "0.12", False),
        ("31 Aug 06:41", rates[FLIGHTS[0]], "0.04", True),
        ("31 Aug 06:42", rates[FLIGHTS[1]], "0.04", True),
        ("31 Aug 06:48", rates[FLIGHTS[2]], "0.04", True)]
sc = lambda v: pw2 * v / 1.35
b2 = []
for i, (lab, v, retain, new) in enumerate(rows):
    y = 24 + i * 25
    col = "var(--crit)" if new else "var(--mute)"
    b2.append(f'<text x="{L2-12}" y="{y+11}" text-anchor="end" class="lbl">{lab}</text>')
    b2.append(f'<rect x="{L2}" y="{y}" width="{sc(v):.1f}" height="16" rx="2" fill="{col}" opacity="0.85"/>')
    b2.append(f'<text x="{L2+sc(v)+8:.1f}" y="{y+12}" class="val">{v:.2f}/s</text>')
    b2.append(f'<text x="{W2-40}" y="{y+12}" class="ax">retain {retain}</text>')
one = L2 + sc(1/0.67)
chart2 = f'''<svg viewBox="0 0 {W2} {H2}" role="img" aria-label="Path rebuild rate before and after the retain-tolerance change">
<line x1="{one:.1f}" y1="16" x2="{one:.1f}" y2="{24+6*25}" stroke="var(--ok)" stroke-width="1.5" stroke-dasharray="4 3"/>
<text x="{one+6:.1f}" y="14" class="thr" style="fill:var(--ok)">1.49/s = one per ramp time</text>
{''.join(b2)}</svg>'''

def img(p):
    return "data:image/png;base64," + base64.b64encode(open(p, "rb").read()).decode()

assets = {
    "CHART1": chart1, "CHART2": chart2,
    "IMG_OK": img(os.path.join(LOGS, FLIGHTS[1], "render", "overview.png")),
    "IMG_STUCK": img(os.path.join(LOGS, FLIGHTS[2], "render", "overview.png")),
}
tpl = open(os.path.join(ROOT, "reports", "stop_start_template.html")).read()
for k, v in assets.items():
    tpl = tpl.replace("{{" + k + "}}", v)
left = re.findall(r"\{\{(\w+)\}\}", tpl)
assert not left, left
out = os.path.join(ROOT, "reports", "stop_start_20260831.html")
open(out, "w").write(tpl)
print("wrote", out, len(tpl), "bytes")
print("rebuild rates:", {k: round(v, 2) for k, v in rates.items()},
      "old:", {k: round(v, 2) for k, v in old_rates.items()})
