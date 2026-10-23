#!/usr/bin/env python3
"""Bump test, old 3S config (2026-09-14) vs new 4S config (2026-10-05 + 2026-10-08).

Usage: .venv-mavlink/bin/python3 scripts/plot_bump_3s_vs_4s.py <out_dir>
Also writes the data as CSV under <out_dir>/csv/ (see the README.txt there).

Reuses load() from plot_bump_test.py. Push size is delta-v (integral of body
horizontal specific force), which needs no vehicle mass, so the lighter 3S
build compares directly. Left out: pushes cut short by AUTO.LAND, and pushes
with an EKF XY reset inside their response (19_59_21 #6, 10-08 21_28_31 #4).
"""
import csv
import sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parent.parent
LOGS = ROOT / "ros2_ws/flight_logs"
OUT = Path(sys.argv[1])
OUT.mkdir(parents=True, exist_ok=True)

src = (ROOT / "scripts/plot_bump_test.py").read_text()
ns = {}
argv, sys.argv = sys.argv, ["plot_bump_test.py", "unused"]
exec(src.split("\nruns = [")[0], ns)
sys.argv = argv
MASS = ns["MASS"]          # load() scales by this; divided back out for delta-v

CONFIGS = [
    dict(key="3s", name="old 3S config", date="2026-09-14", color="#eb6834", cmap="Oranges",
         hover="1.0 m hover",
         gains="ROLL/PITCH_P 4.0, rate I 0.1, XY_VEL_P 1.8, XY_VEL_I 0.4",
         logs=[(f"bump_candidates_20260914/px4_{s}.ulg", "09-14, 1.0 m", "o")
               for s in ("19_53_34", "19_55_29", "19_57_15", "19_58_24", "19_59_21")],
         drop={("19_59_21", 6)}),
    dict(key="4s", name="new 4S config", date="2026-10-05 and 10-08", color="#2a78d6", cmap="Blues",
         hover="0.3 m and 0.8 m hover",
         gains="ROLL/PITCH_P 6.5, rate I 0.2, XY_VEL_P 2.6, XY_VEL_I 0.9",
         logs=[("px4_bump_19_46_26.ulg", "10-05, 0.3 m", "o"), ("px4_bump_19_47_20.ulg", "10-05, 0.3 m", "o"),
               ("sd_4s_20261012/px4_20261008_21_28_31.ulg", "10-08, 0.8 m", "s")],
         drop={("21_28_31", 4)}),
]
T_WIN = 2.5
plt.rcParams.update({"font.size": 10, "axes.grid": True, "grid.alpha": 0.3,
                     "axes.spines.top": False, "axes.spines.right": False})


RUNS = []      # (cfg, log, session, load() result) for the CSV export


def pushes(cfg):
    """Every detected push, with `excl` set to why it is left out of the plots (or "")."""
    out = []
    for f, sess, mk in cfg["logs"]:
        r = ns["load"](str(LOGS / f))
        RUNS.append((cfg, f, sess, r))
        tag = "_".join(Path(f).stem.split("_")[-3:])
        starts = [row["t"] for row in r["rows"]]
        for i, row in enumerate(r["rows"]):
            excl = ("cut short by AUTO.LAND" if row["cut"] else
                    "EKF XY reset during response" if (tag, i + 1) in cfg["drop"] else "")
            s = row["t"]
            nxt = [x for x in starts if x > s]
            t_end = min(s + T_WIN, nxt[0] if nxt else np.inf)
            w = (r["tl"] >= s - 0.2) & (r["tl"] < t_end)
            out.append(dict(tag=f"{tag} #{i+1}", log=Path(f).name, n=i + 1, cfg=cfg["key"], sess=sess, mk=mk,
                            excl=excl, t0=s, dirn=row["dirn"],
                            dv=row["J"] / MASS, J=row["J"], fpk=row["peak"],
                            peak=row["dmax"],
                            t_peak=row["tmax"], rec=row["rec"], over=row["over"],
                            yr=row["yr"], t=r["tl"][w] - s, d=r["dh"][w],
                            dn=r["D"][0][w], de=r["D"][1][w], dz=r["D"][2][w]))
    return out


def scatter(ax, p, ykey, color, prefix="", xkey="dv"):
    for sess in dict.fromkeys(x["sess"] for x in p):
        q = [x for x in p if x["sess"] == sess]
        ax.plot([x[xkey] for x in q], [x[ykey] for x in q], q[0]["mk"], color=color, ms=8,
                mec="white", mew=1.5, label=f"{prefix}{sess} (n={len(q)})")


def fit(p):
    dv = np.array([x["dv"] for x in p]); pk = np.array([x["peak"] for x in p])
    return float(np.sum(dv * pk) / np.sum(dv ** 2))     # cm per m/s, through origin


ALL = {c["key"]: pushes(c) for c in CONFIGS}
data = {k: [x for x in v if not x["excl"]] for k, v in ALL.items()}
DVMAX = max(x["dv"] for p in data.values() for x in p) * 1.05
PKMAX = max(x["peak"] for p in data.values() for x in p) * 1.08

# ------------------------------------------------ one figure per config, shared axes
for cfg in CONFIGS:
    p = data[cfg["key"]]; g = fit(p)
    cm = matplotlib.colors.ListedColormap(plt.get_cmap(cfg["cmap"])(np.linspace(0.35, 0.95, 256)))
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(13, 5))
    for x in sorted(p, key=lambda x: x["dv"]):
        a1.plot(x["t"], x["d"], lw=1.4, color=cm(x["dv"] / DVMAX))
    a1.axhline(5, color="0.4", ls="--", lw=0.8)
    a1.text(-0.18, 6, "5 cm", ha="left", fontsize=8, color="0.3")
    a1.set_xlim(-0.2, T_WIN); a1.set_ylim(0, PKMAX)
    a1.set_xlabel("time since push [s]"); a1.set_ylabel("horizontal displacement [cm]")
    a1.set_title("(a) response to each push (darker = bigger push)")
    sm = plt.cm.ScalarMappable(cmap=cm, norm=plt.Normalize(0, DVMAX))
    fig.colorbar(sm, ax=a1, pad=0.01).set_label("Δv [m/s]")
    scatter(a2, p, "peak", cfg["color"])
    xx = np.linspace(0, DVMAX, 10)
    a2.plot(xx, g * xx, "--", color="0.35", lw=1, label=f"fit: {g/100:.2f} s × Δv")
    a2.set_xlim(0, DVMAX); a2.set_ylim(0, PKMAX)
    a2.set_xlabel("velocity change from push, Δv [m/s]"); a2.set_ylabel("peak displacement [cm]")
    a2.set_title("(b) peak displacement vs push size"); a2.legend(fontsize=9, loc="upper left")
    fig.suptitle(f"Bump test, {cfg['name']} ({cfg['date']}, {cfg['hover']})\n{cfg['gains']}",
                 fontsize=11)
    fig.tight_layout()
    fig.savefig(OUT / f"bump_{cfg['key']}.png", dpi=150)
    plt.close(fig)

# ------------------------------------------------ comparison
fig, ax = plt.subplots(2, 2, figsize=(13, 9.5))
(a1, a2), (a3, a4) = ax
xx = np.linspace(0, DVMAX, 10)
for cfg in CONFIGS:
    p = data[cfg["key"]]; g = fit(p); c = cfg["color"]
    lab = f"{cfg['name']} (n={len(p)})"
    scatter(a1, p, "peak", c, cfg["name"][4:] + " ")
    a1.plot(xx, g * xx, "--", color=c, lw=1.2)
    a1.text(DVMAX * 0.98, g * DVMAX * 0.98, f"{g/100:.2f} s × Δv", color="0.2", ha="right",
            va="bottom" if cfg["key"] == "3s" else "top", fontsize=9)
    scatter(a2, p, "t_peak", c, cfg["name"][4:] + " ")
    # displacement per unit push, so different-size pushes overlay
    tg = np.linspace(-0.2, T_WIN, 271)
    norm = []
    for x in p:
        y = np.interp(tg, x["t"], x["d"] / x["dv"], left=np.nan, right=np.nan)
        y[(tg < x["t"][0]) | (tg > x["t"][-1])] = np.nan
        norm.append(y)
        a3.plot(x["t"], x["d"] / x["dv"], color=c, lw=0.6, alpha=0.3)
    norm = np.array(norm)
    ok = np.sum(np.isfinite(norm), axis=0) >= 3
    med = np.where(ok, np.nanmedian(norm, axis=0), np.nan)
    a3.plot(tg, med, color=c, lw=2.5, label=f"{lab} median")
    scatter(a4, p, "over", c, cfg["name"][4:] + " ")
a1.set_xlim(0, DVMAX); a1.set_ylim(0, PKMAX)
a1.set_xlabel("Δv [m/s]"); a1.set_ylabel("peak displacement [cm]")
a1.set_title("(a) how far it gets pushed"); a1.legend(fontsize=9, loc="upper left")
a2.set_xlim(0, DVMAX); a2.set_ylim(0, 1.2)
a2.set_xlabel("Δv [m/s]"); a2.set_ylabel("time to peak displacement [s]")
a2.set_title("(b) how fast it stops"); a2.legend(fontsize=9, loc="lower right")
a3.set_xlim(-0.2, T_WIN); a3.set_ylim(0, 60)
a3.set_xlabel("time since push [s]"); a3.set_ylabel("displacement per Δv [cm per m/s]")
a3.set_title("(c) response normalised by push size (thin = each push)"); a3.legend(fontsize=9)
a4.set_xlim(0, DVMAX); a4.set_ylim(bottom=0)
a4.set_xlabel("Δv [m/s]"); a4.set_ylabel("overshoot past setpoint [cm]")
a4.set_title("(d) overshoot on the way back (before next push)"); a4.legend(fontsize=9, loc="upper left")
fig.suptitle("Bump test: old 3S config (09-14) vs new 4S config (10-05 + 10-08)", fontsize=12)
fig.tight_layout()
fig.savefig(OUT / "bump_3s_vs_4s.png", dpi=150)
plt.close(fig)

# ------------------------------------------------ force / impulse vs max displacement (1.8 kg)
fig, (a1, a2) = plt.subplots(1, 2, figsize=(13, 5.2))
JMAX = DVMAX * MASS
FMAX = max(x["fpk"] for p in data.values() for x in p) * 1.05
for cfg in CONFIGS:
    p = data[cfg["key"]]; c = cfg["color"]; g = fit(p) / MASS     # cm per N*s
    scatter(a1, p, "peak", c, cfg["name"][4:] + " ", xkey="J")
    xx = np.linspace(0, JMAX, 10)
    a1.plot(xx, g * xx, "--", color=c, lw=1.2)
    a1.text(JMAX * 0.98, g * JMAX * 0.98, f"{g:.0f} cm per N·s", color="0.2", ha="right",
            va="bottom" if cfg["key"] == "3s" else "top", fontsize=9)
    scatter(a2, p, "peak", c, cfg["name"][4:] + " ", xkey="fpk")
a1.set_xlim(0, JMAX); a1.set_ylim(0, PKMAX)
a1.set_xlabel("push impulse [N·s]  (= 1.8 kg × Δv)"); a1.set_ylabel("max displacement [cm]")
a1.set_title("(a) impulse vs max displacement"); a1.legend(fontsize=9, loc="upper left")
a2.set_xlim(0, FMAX); a2.set_ylim(0, PKMAX)
a2.set_xlabel("peak push force [N]  (IMU low-passed at 30 Hz)"); a2.set_ylabel("max displacement [cm]")
a2.set_title("(b) peak force vs max displacement"); a2.legend(fontsize=9, loc="upper left")
fig.suptitle("Push size vs max displacement, 1.8 kg: old 3S config vs new 4S config", fontsize=12)
fig.tight_layout()
fig.savefig(OUT / "bump_force_vs_displacement.png", dpi=150)
plt.close(fig)

for cfg in CONFIGS:
    p = data[cfg["key"]]
    rec = [x["rec"] for x in p if x["rec"] is not None]
    print(f"{cfg['name']}: n={len(p)} fit={fit(p)/100:.3f} s  dv {min(x['dv'] for x in p):.2f}-"
          f"{max(x['dv'] for x in p):.2f}  t_peak {np.median([x['t_peak'] for x in p]):.2f} "
          f"({min(x['t_peak'] for x in p):.2f}-{max(x['t_peak'] for x in p):.2f})  "
          f"rec<5cm {len(rec)}/{len(p)} median {np.median(rec):.2f}  "
          f"overshoot median {np.median([x['over'] for x in p]):.1f} max {max(x['over'] for x in p):.1f}")
    for x in p:
        print(f"   {x['tag']:14s} {x['sess']:13s} dv={x['dv']:.2f} J={x['J']:.2f} F={x['fpk']:.0f} peak={x['peak']:.1f} tpk={x['t_peak']:.2f} over={x['over']:.1f}")

# ------------------------------------------------ CSV export
CSV = OUT / "csv"
(CSV / "timeseries").mkdir(parents=True, exist_ok=True)
HOVER = {"09-14, 1.0 m": 1.0, "10-05, 0.3 m": 0.3, "10-08, 0.8 m": 0.8}
f2 = lambda v, n=3: "" if v is None or not np.isfinite(v) else f"{v:.{n}f}"

with open(CSV / "pushes.csv", "w", newline="") as fh:
    w = csv.writer(fh)
    w.writerow(["push_id", "config", "date", "log", "push_no", "hover_height_m", "included", "excluded_reason",
                "t_push_since_arm_s", "delta_v_m_s", "impulse_N_s", "peak_force_N", "push_dir_deg_ned",
                "max_displacement_cm", "time_to_max_s", "back_within_5cm_s", "overshoot_cm",
                "peak_yaw_rate_deg_s"])
    for cfg in CONFIGS:
        for x in ALL[cfg["key"]]:
            w.writerow([f"{x['cfg']}_{Path(x['log']).stem}_{x['n']}", cfg["key"].upper(), "2026-" + x["sess"][:5],
                        x["log"], x["n"], HOVER[x["sess"]], int(not x["excl"]), x["excl"],
                        f2(x["t0"]), f2(x["dv"]), f2(x["J"]), f2(x["fpk"], 1), f2(x["dirn"], 0),
                        f2(x["peak"], 1), f2(x["t_peak"]), f2(x["rec"]), f2(x["over"], 1), f2(x["yr"], 0)])

with open(CSV / "responses.csv", "w", newline="") as fh:
    w = csv.writer(fh)
    w.writerow(["push_id", "config", "included", "t_since_push_s", "disp_north_cm", "disp_east_cm",
                "disp_horizontal_cm", "height_error_cm"])
    for cfg in CONFIGS:
        for x in ALL[cfg["key"]]:
            pid = f"{x['cfg']}_{Path(x['log']).stem}_{x['n']}"
            for k in range(len(x["t"])):
                if np.isfinite(x["d"][k]):
                    w.writerow([pid, cfg["key"].upper(), int(not x["excl"]), f2(x["t"][k]), f2(x["dn"][k], 2),
                                f2(x["de"][k], 2), f2(x["d"][k], 2), f2(x["dz"][k], 2)])

for cfg, f, sess, r in RUNS:
    stem = Path(f).stem
    # IMU-rate force and the lower-rate position / yaw rate resampled onto it
    t = r["ta"]
    cols = [np.interp(t, r["tl"], r["D"][i], left=np.nan, right=np.nan) for i in range(3)]
    yr = np.interp(t, r["tw"], r["yr"])
    with open(CSV / "timeseries" / f"{cfg['key']}_{stem}.csv", "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["t_since_arm_s", "force_north_N", "force_east_N", "disp_north_cm", "disp_east_cm",
                    "height_error_cm", "yaw_rate_deg_s", "after_auto_land"])
        for k in range(len(t)):
            w.writerow([f2(t[k], 4), f2(r["F"][0][k], 2), f2(r["F"][1][k], 2), f2(cols[0][k], 2),
                        f2(cols[1][k], 2), f2(cols[2][k], 2), f2(yr[k], 1),
                        int(r["t_land"] is not None and t[k] >= r["t_land"])])
