#!/usr/bin/env python3
"""Bump test: how hard and how fast the drone tilts back toward its setpoint, 3S vs 4S.

Usage: .venv-mavlink/bin/python3 scripts/plot_bump_tilt_3s_vs_4s.py <out_dir>

Same pushes as plot_bump_3s_vs_4s.py (its CONFIGS and exclusions). For each push the
tilt is projected on that push's own impulse direction: thrust direction = body -z in
NED, and "tilt back" is the lean against the push, so restoring tilt is positive.
Command = vehicle_attitude_setpoint.q_d, actual = vehicle_attitude.q.
Lag = shift of the actual tilt that best matches the command over the first 1.2 s.
"""
import sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from pyulog import ULog

ROOT = Path(__file__).resolve().parent.parent
LOGS = ROOT / "ros2_ws/flight_logs"
OUT = Path(sys.argv[1])
OUT.mkdir(parents=True, exist_ok=True)

src = (ROOT / "scripts/plot_bump_3s_vs_4s.py").read_text()
ns = {"__file__": str(ROOT / "scripts/plot_bump_3s_vs_4s.py")}
argv, sys.argv = sys.argv, ["plot_bump_3s_vs_4s.py", str(OUT)]
exec(src.split("\nT_WIN")[0], ns)      # MASS, CONFIGS; its own ns holds load()
sys.argv = argv
CONFIGS, MASS = ns["CONFIGS"], ns["MASS"]
T_WIN, FS = 2.5, 200.0
TILT_MAX = 45.0                        # MPC_TILTMAX_AIR in every log
plt.rcParams.update({"font.size": 10, "axes.grid": True, "grid.alpha": 0.3,
                     "axes.spines.top": False, "axes.spines.right": False})


def tilt_vec(q):
    """Horizontal (N, E) part of the thrust direction, body -z in NED."""
    w, x, y, z = q
    return -np.vstack([2 * (x * z + w * y), 2 * (y * z - w * x)])


def pushes(cfg):
    out = []
    for f, sess, mk in cfg["logs"]:
        r = ns["ns"]["load"](str(LOGS / f))
        u = ULog(str(LOGS / f), ["vehicle_status", "vehicle_attitude", "vehicle_attitude_setpoint"])
        d = {x.name: x.data for x in u.data_list if x.multi_id == 0}
        vs = d["vehicle_status"]
        t0 = vs["timestamp"][vs["arming_state"] == 2][0]
        att, sp = d["vehicle_attitude"], d["vehicle_attitude_setpoint"]
        ta, ha = (att["timestamp"] - t0) / 1e6, tilt_vec([att[f"q[{i}]"] for i in range(4)])
        ts, hs = (sp["timestamp"] - t0) / 1e6, tilt_vec([sp[f"q_d[{i}]"] for i in range(4)])
        tag = "_".join(Path(f).stem.split("_")[-3:])
        starts = [row["t"] for row in r["rows"]]
        for i, row in enumerate(r["rows"]):
            if row["cut"] or (tag, i + 1) in cfg["drop"]:
                continue
            s = row["t"]
            nxt = [x for x in starts if x > s]
            t_end = min(s + T_WIN, nxt[0] if nxt else np.inf)
            uax = row["Jvec"] / np.linalg.norm(row["Jvec"])
            tg = np.arange(-0.3, t_end - s, 1 / FS)
            back = lambda t, h: np.degrees(np.arcsin(np.clip(
                -(np.interp(tg + s, t, h[0]) * uax[0] + np.interp(tg + s, t, h[1]) * uax[1]), -1, 1)))
            act, cmd = back(ta, ha), back(ts, hs)
            dv = row["J"] / MASS
            w = (tg >= 0) & (tg <= 1.0)
            ka, kc = np.argmax(np.where(w, act, -np.inf)), np.argmax(np.where(w, cmd, -np.inf))
            # lag: shift of actual that best fits the command over the first 1.2 s
            wl = (tg >= 0) & (tg <= 1.2)
            lags = np.arange(0, 0.40, 1 / FS)
            err = [np.mean((cmd[wl] - np.interp(tg[wl] + L, tg, act)) ** 2) for L in lags]
            out.append(dict(tag=f"{tag} #{i+1}", sess=sess, mk=mk, dv=dv, t=tg, act=act, cmd=cmd,
                            pk_act=act[ka], t_act=tg[ka], pk_cmd=cmd[kc], t_cmd=tg[kc],
                            lag=lags[int(np.argmin(err))], sat=bool(cmd[w].max() >= TILT_MAX - 1)))
    return out


data = {c["key"]: pushes(c) for c in CONFIGS}
DVMAX = max(x["dv"] for p in data.values() for x in p) * 1.05

fig, (a1, a2) = plt.subplots(1, 2, figsize=(13, 5.2))
tg = np.arange(-0.3, T_WIN, 1 / FS)
for cfg in CONFIGS:
    p = data[cfg["key"]]; c = cfg["color"]; n = len(p)
    A, C = [], []
    for x in p:
        ya = np.interp(tg, x["t"], x["act"] / x["dv"], right=np.nan)
        yc = np.interp(tg, x["t"], x["cmd"] / x["dv"], right=np.nan)
        A.append(ya); C.append(yc)
    A, C = np.array(A), np.array(C)
    ok = np.sum(np.isfinite(A), axis=0) >= 3
    a1.plot(tg, np.where(ok, np.nanmedian(C, axis=0), np.nan), color=c, lw=1.6, ls=(0, (4, 2)),
            label=f"{cfg['name']} command, median")
    a1.plot(tg, np.where(ok, np.nanmedian(A, axis=0), np.nan), color=c, lw=2.6,
            label=f"{cfg['name']} actual, median (n={n})")

    for key, mk_fill in (("pk_act", True), ("pk_cmd", False)):
        a2.plot([x["dv"] for x in p], [x[key] for x in p], "o", ms=7 if mk_fill else 6,
                color=c if mk_fill else "white", mec=c, mew=1.5 if mk_fill else 1.3,
                label=f"{cfg['name']} {'actual' if mk_fill else 'command'}")
    dv = np.array([x["dv"] for x in p]); pk = np.array([x["pk_act"] for x in p])
    g = float(np.sum(dv * pk) / np.sum(dv ** 2))
    xx = np.linspace(0, DVMAX, 10)
    a2.plot(xx, g * xx, "--", color=c, lw=1)
    xl = min(DVMAX * 0.98, (TILT_MAX - 6) / g)
    a2.text(xl + 0.04, g * xl - 1.0, f"{g:.0f} deg per m/s", color="0.2", ha="left", va="top", fontsize=9)


a1.axhline(0, color="0.4", lw=0.8)
a1.set_xlim(-0.3, T_WIN); a1.set_ylim(-6, 25)
a1.set_xlabel("time since push [s]"); a1.set_ylabel("tilt back toward setpoint per Δv [deg per m/s]")
a1.set_title("(a) tilt response, normalised by push size (median)")
a1.legend(fontsize=8.5, loc="upper right")
a2.set_xlim(0, DVMAX); a2.set_ylim(0, 50)
a2.set_xlabel("velocity change from push, Δv [m/s]"); a2.set_ylabel("peak tilt back [deg]")
a2.set_title("(b) peak tilt back vs push size (filled = actual, open = command)")
a2.legend(fontsize=8.5, loc="lower right")
fig.suptitle("Bump test tilt response: old 3S config (09-14, ROLL/PITCH_P 4.0) vs "
             "new 4S config (10-05 + 10-08, ROLL/PITCH_P 6.5)", fontsize=12)
fig.tight_layout()
fig.savefig(OUT / "bump_tilt_3s_vs_4s.png", dpi=150)

for cfg in CONFIGS:
    p = data[cfg["key"]]
    dv = np.array([x["dv"] for x in p])
    print(f"{cfg['name']}: n={len(p)}  peak act/dv median {np.median([x['pk_act'] / x['dv'] for x in p]):.1f} deg/(m/s)"
          f"  peak cmd/dv {np.median([x['pk_cmd'] / x['dv'] for x in p]):.1f}"
          f"  lag median {np.median([x['lag'] for x in p]) * 1000:.0f} ms ({min(x['lag'] for x in p) * 1000:.0f}-{max(x['lag'] for x in p) * 1000:.0f})"
          f"  t_peak act {np.median([x['t_act'] for x in p]):.2f} s  t_peak cmd {np.median([x['t_cmd'] for x in p]):.2f} s"
          f"  cmd at limit {sum(x['sat'] for x in p)}/{len(p)}")
    for x in p:
        print(f"   {x['tag']:14s} dv={x['dv']:.2f} act={x['pk_act']:.1f}@{x['t_act']:.2f} cmd={x['pk_cmd']:.1f}@{x['t_cmd']:.2f} lag={x['lag']*1000:.0f}ms")
