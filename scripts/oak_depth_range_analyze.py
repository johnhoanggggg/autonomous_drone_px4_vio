#!/usr/bin/env python3
"""Analyse OAK-D Lite depth range sessions from oak_depth_range_record.py.

    .venv-mavlink/bin/python scripts/oak_depth_range_analyze.py SESSION [SESSION ...]

One session gives its accuracy-vs-distance curves; several (e.g. `current` and
`extended`) are overlaid for an A/B. Per stop, over a ROI that covers the
centre of the target:
  bias      median depth minus measured distance. Constant = tape-measure
            origin offset; growing with distance^2 = calibration/disparity bias.
  temporal  per-pixel frame-to-frame std (median over the ROI)
  spatial   RMS about a fitted plane (flatness; removes target tilt)
  fill      fraction of ROI pixels with any depth
  false     fraction of returned pixels more than max(10 cm, 10%) off - wrong
            matches, e.g. inside the minimum distance
Stereo error grows as Z^2 * sigma_d / (f*B); sigma_d (the matcher's real
precision, in pixels) is fitted from the temporal noise of the flat stops.

The effective range is the band of flat stops where fill >= --min-fill,
false <= --max-false and |bias|, temporal and spatial are all <= --max-rel-error
of the distance. Writes summary.csv and range_test.png to --out (default
<first session>/analysis).
"""
import argparse
import csv
import glob
import json
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from oak_depth_range_common import roi_box, stop_stats  # noqa: E402

SERIES = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100"]
INK, INK2, MUTED, GRID, AXIS, SURF = "#0b0b0b", "#52514e", "#898781", "#e1e0d9", "#c3c2b7", "#fcfcfb"
NOISE_FLOOR_CM = 0.01
FIELDS = ["session", "stop", "tag", "distance_m", "frames", "roi_px", "median_m", "bias_m",
          "temporal_m", "spatial_m", "fill", "outlier_frac", "passes"]


def parse_args():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("sessions", nargs="+")
    ap.add_argument("--out", help="output directory (default <first session>/analysis)")
    ap.add_argument("--truth-offset", type=float, default=0.0,
                    help="metres added to every typed distance, e.g. front face -> optical centre")
    ap.add_argument("--max-rel-error", type=float, default=0.05)
    ap.add_argument("--min-fill", type=float, default=0.8)
    ap.add_argument("--max-false", type=float, default=0.05)
    return ap.parse_args()


def load_session(path, truth_offset):
    with open(os.path.join(path, "session.json")) as f:
        meta = json.load(f)
    rows = []
    for npz in sorted(glob.glob(os.path.join(path, "stop_*.npz"))):
        data = np.load(npz)
        truth = float(data["distance_m"]) + truth_offset
        depth = data["depth"]
        roi = roi_box(meta["fx"], truth, meta["board_size_m"], meta["roi_fraction"],
                      depth.shape[2], depth.shape[1])
        s = stop_stats(depth, roi, meta["fx"], meta["fy"], meta["cx"], meta["cy"], truth)
        rows.append({"session": meta["label"], "stop": os.path.basename(npz)[:-4],
                     "tag": str(data["tag"]), "distance_m": truth, **s})
    return meta, rows


def passes(row, args):
    z = row["distance_m"]
    limit = args.max_rel_error * z
    checks = (row["fill"] >= args.min_fill, row["outlier_frac"] <= args.max_false,
              abs(row["bias_m"]) <= limit, row["temporal_m"] <= limit, row["spatial_m"] <= limit)
    return all(bool(c) and c == c for c in checks)  # NaN never passes


def effective_band(flat):
    """Longest run of consecutive passing distances, as (near, far) or None."""
    best, run = None, []
    for row in sorted(flat, key=lambda r: r["distance_m"]):
        if row["passes"]:
            run.append(row["distance_m"])
            if best is None or len(run) > len(best):
                best = list(run)
        else:
            run = []
    return (best[0], best[-1]) if best else None


def fit_sigma_d(flat, f_b):
    """sigma_d from temporal = Z^2 * sigma_d / (f*B): median of per-stop estimates.

    Skips stops dominated by false matches (inside the minimum distance) and
    sub-millimetre noise, where the 1 mm depth unit quantises the measurement.
    """
    est = [r["temporal_m"] * f_b / r["distance_m"] ** 2 for r in flat
           if r["fill"] >= 0.5 and r["outlier_frac"] <= 0.2
           and np.isfinite(r["temporal_m"]) and r["temporal_m"] >= 0.002]
    return float(np.median(est)) if len(est) >= 2 else float("nan")


def style():
    plt.rcParams.update({
        "figure.facecolor": SURF, "axes.facecolor": SURF, "savefig.facecolor": SURF,
        "axes.edgecolor": AXIS, "axes.labelcolor": INK2, "xtick.color": MUTED, "ytick.color": MUTED,
        "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.6,
        "axes.spines.top": False, "axes.spines.right": False, "font.size": 9,
        "axes.titlesize": 10, "axes.titleweight": "bold", "axes.titlecolor": INK,
        "axes.titlelocation": "left", "legend.frameon": False, "lines.linewidth": 2,
        "lines.markersize": 5,
    })


def plot(results, args, path):
    style()
    fig, axs = plt.subplots(2, 3, figsize=(14, 8), constrained_layout=True)
    fig.suptitle("OAK-D Lite depth vs distance (flat textured target)", x=0.01, ha="left",
                 fontsize=12, fontweight="bold", color=INK)
    panels = [
        (axs[0, 0], "bias_m", 100, "Bias: median depth − measured (cm)", False),
        (axs[0, 1], "temporal_m", 100, "Frame-to-frame noise (cm, log)", True),
        (axs[0, 2], "spatial_m", 100, "Flatness noise about fitted plane (cm, log)", True),
        (axs[1, 0], "fill", 100, "Fill: ROI pixels with depth (%)", False),
        (axs[1, 1], "outlier_frac", 100, "False matches (% of returned pixels)", False),
        (axs[1, 2], "rel", 100, "Worst of |bias|, noise as % of distance", False),
    ]
    zmax = max(r["distance_m"] for _, rows, _ in results for r in rows)
    zline = np.linspace(0.1, zmax * 1.05, 200)
    # Linear panels are scaled to the believable stops; near-zone garbage is
    # drawn as a triangle on the edge with its value, not allowed to squash them.
    sane = [r for _, rows, _ in results for r in rows
            if r["tag"] == "flat" and r["outlier_frac"] <= 0.2]
    bias_cm = max([abs(r["bias_m"]) * 100 for r in sane if np.isfinite(r["bias_m"])] + [2.0]) * 1.3
    limits = {"bias_m": (-bias_cm, bias_cm), "rel": (0.0, max(25.0, args.max_rel_error * 400))}
    for idx, (meta, rows, sigma_d) in enumerate(results):
        color = SERIES[idx % len(SERIES)]
        flat = sorted((r for r in rows if r["tag"] == "flat"), key=lambda r: r["distance_m"])
        if not flat:
            continue
        z = np.array([r["distance_m"] for r in flat])
        label = (f"{meta['label']} (σd {sigma_d:.2f} px)" if np.isfinite(sigma_d) else meta["label"])
        for ax, key, scale, _, log in panels:
            if key == "rel":
                y = np.array([max(abs(r["bias_m"]), r["temporal_m"], r["spatial_m"]) / r["distance_m"]
                              for r in flat])
            else:
                y = np.array([r[key] for r in flat])
            y = y * scale
            if log:
                y = np.maximum(y, NOISE_FLOOR_CM)  # depth comes in 1 mm steps
            if key in limits:
                lo, hi = limits[key]
                off = np.isfinite(y) & ((y > hi) | (y < lo))
                for zi, yi in zip(z[off], y[off]):
                    edge = hi if yi > hi else lo
                    ax.plot(zi, edge, "^" if yi > hi else "v", color=color, ms=7, clip_on=False)
                    ax.annotate(f"{yi:.0f}", (zi, edge), xytext=(5, -3 if yi > hi else 3),
                                textcoords="offset points", fontsize=7, color=INK2,
                                va="top" if yi > hi else "bottom")
                y = np.where(off, np.nan, y)
            ax.plot(z, y, "-o", color=color, label=label, mec=SURF, mew=1.2)
        f_b = meta["fx"] * meta["baseline_m"]
        if np.isfinite(sigma_d):
            axs[0, 1].plot(zline, zline ** 2 * sigma_d / f_b * 100, ls="--", lw=1, color=color, alpha=0.7)
        if idx == 0:
            for sd, txt in ((0.125, "theory, 1/8 px"), (1.0, "theory, 1 px")):
                axs[0, 1].plot(zline, zline ** 2 * sd / f_b * 100, ls=":", lw=1, color=MUTED)
                axs[0, 1].annotate(txt, (zline[-1], zline[-1] ** 2 * sd / f_b * 100), fontsize=7,
                                   color=INK2, ha="right", va="bottom")
        z_min = f_b / (190.0 if meta["extended_disparity"] else 95.0)
        for ax, *_ in panels:
            ax.axvline(z_min, color=color, lw=1, ls="--", alpha=0.6)
        axs[1, 0].annotate(f"{meta['label']} min ≈ {z_min:.2f} m", (z_min, 0.02 + 0.06 * idx),
                           xycoords=("data", "axes fraction"), xytext=(3, 0), textcoords="offset points",
                           fontsize=7, color=INK2)
    for ax, key, _, title, log in panels:
        ax.set_title(title)
        ax.set_xlabel("measured distance (m)")
        if log:
            ax.set_yscale("log")
    axs[0, 0].axhline(0, color=AXIS, lw=1)
    axs[1, 0].set_ylim(-3, 103)
    axs[1, 0].axhline(args.min_fill * 100, color=INK2, lw=1, ls=":")
    axs[1, 1].axhline(args.max_false * 100, color=INK2, lw=1, ls=":")
    axs[1, 2].axhline(args.max_rel_error * 100, color=INK2, lw=1, ls=":")
    for ax, key, *_ in panels:
        if key in limits:
            ax.set_ylim(*limits[key])
    axs[0, 1].set_ylim(bottom=NOISE_FLOOR_CM)
    axs[0, 2].set_ylim(bottom=NOISE_FLOOR_CM)
    if len(results) > 1:
        axs[0, 0].legend(fontsize=8)
    else:
        axs[0, 1].legend(fontsize=8)
    fig.savefig(path, dpi=130)
    plt.close(fig)


def fmt(v, scale=1.0, spec=".1f"):
    return "    -" if not np.isfinite(v) else format(v * scale, spec)


def main():
    args = parse_args()
    out = args.out or os.path.join(args.sessions[0], "analysis")
    os.makedirs(out, exist_ok=True)
    results, all_rows = [], []
    for session in args.sessions:
        meta, rows = load_session(session, args.truth_offset)
        for r in rows:
            r["passes"] = passes(r, args)
        flat = [r for r in rows if r["tag"] == "flat"]
        f_b = meta["fx"] * meta["baseline_m"]
        sigma_d = fit_sigma_d(flat, f_b)
        results.append((meta, rows, sigma_d))
        all_rows += rows

        print(f"\n== {meta['label']}  ({session})")
        print(f"   fx {meta['fx']:.1f} px, baseline {meta['baseline_m'] * 100:.2f} cm, f*B {f_b:.2f} px*m, "
              f"extended={meta['extended_disparity']} subpixel={meta['subpixel']} median={meta['median']}")
        print("   dist m  tag        bias cm  noise cm  flat cm   fill  false  pass")
        for r in sorted(rows, key=lambda r: (r["tag"] != "flat", r["tag"], r["distance_m"])):
            print(f"   {r['distance_m']:6.2f}  {r['tag']:<9}  {fmt(r['bias_m'], 100, '+7.1f')}  "
                  f"{fmt(r['temporal_m'], 100, '8.2f')}  {fmt(r['spatial_m'], 100, '7.2f')}  "
                  f"{fmt(r['fill'], 100, '4.0f')}%  {fmt(r['outlier_frac'], 100, '4.0f')}%  "
                  f"{'yes' if r['passes'] else 'no'}")
        band = effective_band(flat)
        print(f"   fitted matcher precision sigma_d = {fmt(sigma_d, 1, '.3f')} px")
        if band:
            print(f"   effective range (flat target, {args.max_rel_error:.0%} error, "
                  f"{args.min_fill:.0%} fill): {band[0]:.2f} - {band[1]:.2f} m")
        else:
            print("   no flat stop passes all thresholds")

    with open(os.path.join(out, "summary.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=FIELDS, extrasaction="ignore")
        w.writeheader()
        w.writerows(all_rows)
    if any(r["tag"] == "flat" for r in all_rows):
        plot(results, args, os.path.join(out, "range_test.png"))
    print(f"\nwrote {out}/summary.csv" + (" and range_test.png" if any(r['tag'] == 'flat' for r in all_rows) else ""))


if __name__ == "__main__":
    main()
