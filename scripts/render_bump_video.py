#!/usr/bin/env python3
"""Bump-test phone video with the ULog signals drawn in sync, rendered at 1920x1080.

Usage:
    .venv-mavlink/bin/python3 scripts/render_bump_video.py <video.mov> <log.ulg> <offset_s> <out.mp4>
        [--layout=overlay|side] [--title=TEXT] [--sub=TEXT] [--crop-y=PX]

offset_s = log time since arm at video t=0 (find it by matching push frames to the
pushes plot_bump_test.py lists). The iPhone HDR (HLG) video is tone-mapped to SDR here.
overlay: video fills the frame (scaled to 1920 wide, then cropped to 1080 from
--crop-y) with a translucent strip of 4 plots along the bottom.
side: video fitted into the left 1200 px, the 4 plots stacked on the right.

Panels: push force along the push axis; displacement from setpoint (along push,
sideways, height); tilt command vs actual along the push axis (+ = leaning back
toward the setpoint); motor commands. The push axis is the mean impulse direction
of the pushes inside the clip.
"""
import re
import subprocess
import sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch
from pyulog import ULog

ROOT = Path(__file__).resolve().parent.parent
FFMPEG = str(next((ROOT / ".venv-mavlink/lib").glob(
    "python3*/site-packages/imageio_ffmpeg/binaries/ffmpeg-linux-*")))
W, H, FPS = 1920, 1080, 30

pos = [a for a in sys.argv[1:] if not a.startswith("--")]
opt = dict(a[2:].split("=", 1) for a in sys.argv[1:] if a.startswith("--"))
VIDEO, ULG, OFFSET, OUT = pos[0], pos[1], float(pos[2]), pos[3]
LAYOUT = opt.get("layout", "overlay")
TITLE = opt.get("title", "Bump test")
SUB = opt.get("sub", "")
CROP_Y = opt.get("crop-y")

src = (ROOT / "scripts/plot_bump_test.py").read_text()
ns = {}
argv, sys.argv = sys.argv, ["plot_bump_test.py", "unused"]
exec(src.split("\nruns = [")[0], ns)
sys.argv = argv
MASS = ns["MASS"]

# dark-surface categorical steps (dataviz reference palette)
BLUE, ORANGE, AQUA, YELLOW = "#3987e5", "#d95926", "#199e70", "#c98500"
INK, INK2, MUTED = "#f0efec", "#c3c2b7", "#8f8e86"
SURF = "#1a1a19"


def probe_duration(path):
    err = subprocess.run([FFMPEG, "-hide_banner", "-i", path], capture_output=True, text=True).stderr
    hms = err.split("Duration: ")[1].split(",")[0].split(":")
    return float(hms[0]) * 3600 + float(hms[1]) * 60 + float(hms[2])


def probe_size(path):
    err = subprocess.run([FFMPEG, "-hide_banner", "-i", path], capture_output=True, text=True).stderr
    line = next(l for l in err.splitlines() if "Video:" in l)
    m = re.search(r"\s(\d{2,5})x(\d{2,5})[\s,]", line)
    return int(m.group(1)), int(m.group(2))


DUR = probe_duration(VIDEO)
NF = int(np.floor(DUR * FPS))

# ------------------------------------------------------------------ signals
r = ns["load"](ULG)
u = ULog(ULG, ["vehicle_status", "vehicle_attitude", "vehicle_attitude_setpoint", "actuator_motors"])
d = {x.name: x.data for x in u.data_list if x.multi_id == 0}
vs = d["vehicle_status"]
t_arm = vs["timestamp"][vs["arming_state"] == 2][0]
t_end = r["t_end"]

in_clip = [row for row in r["rows"] if OFFSET <= row["t"] <= OFFSET + DUR]
J = np.sum([row["Jvec"] for row in in_clip], axis=0)
ax_u = J / np.linalg.norm(J)                       # push axis, NED (north, east)
ax_v = np.array([-ax_u[1], ax_u[0]])               # sideways, 90 deg right of push

tg = np.arange(0, DUR, 0.01)                       # video time grid
tl = tg + OFFSET                                   # since arm
live = tl <= t_end


def on_grid(t, y):
    out = np.interp(tl, t, y, left=np.nan, right=np.nan)
    out[~live] = np.nan
    return out


force = on_grid(r["ta"], r["F"][0] * ax_u[0] + r["F"][1] * ax_u[1])
d_along = on_grid(r["tl"], r["D"][0] * ax_u[0] + r["D"][1] * ax_u[1])
d_side = on_grid(r["tl"], r["D"][0] * ax_v[0] + r["D"][1] * ax_v[1])
d_up = on_grid(r["tl"], r["D"][2])


def tilt_back(ts, q):
    """Lean toward -push axis in degrees (thrust direction = body -z in NED)."""
    w, x, y, z = q
    zb = np.vstack([2 * (x * z + w * y), 2 * (y * z - w * x)])     # body z axis, N/E part
    thrust_h = -zb
    s = -(thrust_h[0] * ax_u[0] + thrust_h[1] * ax_u[1])
    return on_grid((ts - t_arm) / 1e6, np.degrees(np.arcsin(np.clip(s, -1, 1))))


att, sp = d["vehicle_attitude"], d["vehicle_attitude_setpoint"]
tilt_act = tilt_back(att["timestamp"], [att[f"q[{i}]"] for i in range(4)])
tilt_cmd = tilt_back(sp["timestamp"], [sp[f"q_d[{i}]"] for i in range(4)])
mo = d["actuator_motors"]
k = np.ones(5) / 5                                  # 25 ms box, takes the PWM-rate fuzz off
motors = [on_grid((mo["timestamp"] - t_arm) / 1e6, np.convolve(mo[f"control[{i}]"], k, "same"))
          for i in range(4)]

pushes = [dict(t=row["t"] - OFFSET, dv=row["J"] / MASS, tpk=row["t"] + row["tmax"] - OFFSET,
               pk=row["dmax"]) for row in in_clip]
print(f"push axis {np.degrees(np.arctan2(ax_u[1], ax_u[0])) % 360:.0f} deg NED; pushes in clip:",
      ", ".join(f"{p['t']:.2f}s dv={p['dv']:.2f}" for p in pushes))
for row in in_clip:
    print(f"   dir {row['dirn']:.0f} deg, |J| {row['J']:.2f}")


def lim(*ys, sym=False, pad=0.12, floor=1.0):
    a = np.concatenate([y[np.isfinite(y)] for y in ys])
    lo, hi = a.min(), a.max()
    if sym:
        m = max(abs(lo), abs(hi), floor) * (1 + pad)
        return -m, m
    span = max(hi - lo, floor)
    return lo - pad * span, hi + pad * span


# ------------------------------------------------------------------ figure
plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 13, "text.color": INK,
                     "axes.labelcolor": INK2, "xtick.color": MUTED, "ytick.color": MUTED,
                     "axes.edgecolor": "none"})
fig = plt.figure(figsize=(W / 100, H / 100), dpi=100)
fig.patch.set_alpha(0)

if LAYOUT == "overlay":
    x0, x1, y0, y1 = 20 / W, 1 - 20 / W, 20 / H, 330 / H
    fig.add_artist(FancyBboxPatch((x0, y0), x1 - x0, y1 - y0, boxstyle="round,pad=0,rounding_size=0.008",
                                  transform=fig.transFigure, fc=(0.06, 0.06, 0.055, 0.74), ec="none",
                                  zorder=-10))
    gs = fig.add_gridspec(1, 4, left=x0 + 52 / W, right=x1 - 18 / W, bottom=y0 + 42 / H,
                          top=y1 - 74 / H, wspace=0.24)
    axes = [fig.add_subplot(gs[0, i]) for i in range(4)]
    # title tag top-left
    fig.add_artist(FancyBboxPatch((20 / W, 1 - 108 / H), 820 / W, 86 / H,
                                  boxstyle="round,pad=0,rounding_size=0.008", transform=fig.transFigure,
                                  fc=(0.06, 0.06, 0.055, 0.74), ec="none", zorder=-10))
    fig.text(40 / W, 1 - 52 / H, TITLE, fontsize=22, weight="bold", color=INK, va="baseline")
    fig.text(40 / W, 1 - 86 / H, SUB, fontsize=14, color=INK2, va="baseline")
    title_kw = dict(loc="left", fontsize=14, color=INK, pad=28)
else:
    fig.add_artist(plt.Rectangle((1200 / W, 0), 720 / W, 1, transform=fig.transFigure, fc=SURF, ec="none",
                                 zorder=-10))
    gs = fig.add_gridspec(4, 1, left=1270 / W, right=1 - 36 / W, bottom=64 / H, top=1 - 84 / H, hspace=0.62)
    axes = [fig.add_subplot(gs[i, 0]) for i in range(4)]
    vw, vh = probe_size(VIDEO)
    band = (H - 1200 * vh / vw) / 2
    ty = 1 - max(band - 48, 36) / H if band > 90 else 1 - 50 / H
    fig.text(40 / W, ty, TITLE, fontsize=24, weight="bold", color=INK, va="baseline")
    fig.text(40 / W, ty - 34 / H, SUB, fontsize=15, color=INK2, va="baseline")
    title_kw = dict(loc="left", fontsize=15, color=INK, pad=28)

a_f, a_d, a_t, a_m = axes
for a in axes:
    a.set_facecolor("none")
    a.set_xlim(0, DUR)
    a.grid(True, color="white", alpha=0.10, lw=0.8)
    a.axhline(0, color="white", alpha=0.28, lw=1)
    a.tick_params(length=0, labelsize=11)
    for p in pushes:
        a.axvline(p["t"], color="white", alpha=0.12, lw=1, ls=(0, (2, 3)))
(a_m if LAYOUT == "side" else a_f).set_xlabel("video time (s)", fontsize=10.5, color=MUTED, labelpad=1,
                                               loc="left" if LAYOUT == "overlay" else "center")

a_f.set_title("Push force (N)", **title_kw)
a_d.set_title("Displacement from setpoint (cm)", **title_kw)
a_t.set_title("Tilt back toward setpoint (deg)", **title_kw)
a_m.set_title("Motor commands (0 to 1)", **title_kw)
a_f.set_ylim(*lim(force, pad=0.18))
a_d.set_ylim(*lim(d_along, d_side, d_up, np.zeros(1), pad=0.16, floor=5))
a_t.set_ylim(*lim(tilt_act, tilt_cmd, np.zeros(1), pad=0.1, floor=5))
mlo, mhi = lim(*motors, pad=0.08, floor=0.2)
a_m.set_ylim(max(0, mlo), min(1, mhi))

# animated artists
ln = lambda a, c, lw=2.0, **kw: a.plot([], [], color=c, lw=lw, animated=True, solid_capstyle="round", **kw)[0]
series = [
    (a_f, force, ln(a_f, ORANGE)),
    (a_d, d_side, ln(a_d, MUTED, 1.6)),
    (a_d, d_up, ln(a_d, AQUA)),
    (a_d, d_along, ln(a_d, BLUE, 2.4)),
    (a_t, tilt_cmd, ln(a_t, INK2, 1.6, ls=(0, (4, 2)))),
    (a_t, tilt_act, ln(a_t, BLUE, 2.2)),
] + [(a_m, m, ln(a_m, c, 1.4)) for m, c in zip(motors, (BLUE, ORANGE, AQUA, YELLOW))]
dots = [(a, y, a.plot([], [], "o", ms=7, color=l.get_color(), mec=SURF, mew=1.5, animated=True)[0])
        for a, y, l in series if a is not a_m]
cursors = [a.axvline(0, color="white", alpha=0.45, lw=1, animated=True) for a in axes]

# legends (static); identity never by colour alone
leg_kw = dict(loc="lower left", bbox_to_anchor=(-0.01, 0.99), fontsize=10.5, frameon=False, handlelength=1.4,
              labelcolor=INK2, borderaxespad=0.0, borderpad=0.0, handletextpad=0.5, columnspacing=0.9)
proxy = lambda c, lw=2.0, **kw: plt.Line2D([], [], color=c, lw=lw, **kw)
a_d.legend([proxy(BLUE, 2.4), proxy(MUTED, 1.6), proxy(AQUA)], ["along push", "sideways", "height"],
           ncol=3, **leg_kw)
a_t.legend([proxy(INK2, 1.6, ls=(0, (4, 2))), proxy(BLUE, 2.2)], ["command", "actual"], ncol=2, **leg_kw)
a_m.legend([proxy(c) for c in (BLUE, ORANGE, AQUA, YELLOW)], ["M1", "M2", "M3", "M4"], ncol=4, **leg_kw)
a_f.text(-0.01, 1.0, "labels: push size as Δv (m/s)", transform=a_f.transAxes, fontsize=10.5, color=INK2,
         va="bottom", ha="left")

# per-push labels that appear as the clip reaches them
labels = []
fy = a_f.get_ylim()[1]
dy = np.diff(a_d.get_ylim())[0]
for i, p in enumerate(pushes):
    close = i > 0 and p["t"] - pushes[i - 1]["t"] < 1.3 and i % 2 == 1
    labels.append((p["t"], a_f.text(p["t"] + 0.08, fy * (0.80 if close else 0.97), f"{p['dv']:.1f}",
                                      fontsize=11, color=INK, va="top", animated=True)))
    # peak of the plotted along-push line before the next push
    t_next = pushes[i + 1]["t"] if i + 1 < len(pushes) else p["t"] + 2.5
    w = (tg >= p["t"]) & (tg < min(t_next, p["t"] + 2.5)) & np.isfinite(d_along)
    if w.any():
        j = np.argmax(np.where(w, d_along, -np.inf))
        if d_along[j] > 3:
            labels.append((tg[j], a_d.text(tg[j], d_along[j] + 0.03 * dy, f"{d_along[j]:.0f}", fontsize=11,
                                            color=INK, ha="center", va="bottom", animated=True)))
fig.canvas.draw()
bg = fig.canvas.copy_from_bbox(fig.bbox)

# ------------------------------------------------------------------ ffmpeg
TM = ("zscale=t=linear:npl=203,format=gbrpf32le,zscale=p=bt709,tonemap=hable:desat=0,"
      "zscale=t=bt709:m=bt709:r=tv,format=yuv420p")
if LAYOUT == "overlay":
    vw, vh = probe_size(VIDEO)
    sh = int(round(vh * W / vw / 2)) * 2
    cy = int(CROP_Y) if CROP_Y is not None else (sh - H) // 2
    geo = f"scale={W}:{sh},crop={W}:{H}:0:{cy}" if sh > H else f"scale={W}:{H}"
else:
    geo = f"scale=1200:-2,pad={W}:{H}:0:(oh-ih)/2:color=0x1a1a19"
fc = (f"[0:v]fps={FPS},{TM},{geo},format=rgba[v];[v][1:v]overlay=0:0:format=auto,format=yuv420p[o]")
cmd = [FFMPEG, "-v", "error", "-y", "-i", VIDEO,
       "-f", "rawvideo", "-pix_fmt", "rgba", "-s", f"{W}x{H}", "-r", str(FPS), "-i", "-",
       "-filter_complex", fc, "-map", "[o]", "-map", "0:a:0?", "-c:a", "aac", "-b:a", "160k",
       "-c:v", "libx264", "-preset", "medium", "-crf", "18", "-pix_fmt", "yuv420p",
       "-color_primaries", "bt709", "-color_trc", "bt709", "-colorspace", "bt709",
       "-movflags", "+faststart", "-frames:v", str(NF), "-shortest", OUT]
proc = subprocess.Popen(cmd, stdin=subprocess.PIPE)
for i in range(NF):
    t = i / FPS
    n = int(np.searchsorted(tg, t, side="right"))
    fig.canvas.restore_region(bg)
    for a, y, l in series:
        l.set_data(tg[:n], y[:n])
        a.draw_artist(l)
    for a, y, m in dots:
        j = n - 1
        ok = j >= 0 and np.isfinite(y[j])
        m.set_data([tg[j]] if ok else [], [y[j]] if ok else [])
        a.draw_artist(m)
    for c in cursors:
        c.set_xdata([t, t])
        c.axes.draw_artist(c)
    for t0, txt in labels:
        if t >= t0:
            txt.axes.draw_artist(txt)
    fig.canvas.blit(fig.bbox)
    proc.stdin.write(bytes(fig.canvas.buffer_rgba()))
    if i % 60 == 0:
        print(f"\r{i}/{NF}", end="", flush=True)
proc.stdin.close()
proc.wait()
print(f"\nwrote {OUT}")
