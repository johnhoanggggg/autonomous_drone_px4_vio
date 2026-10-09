#!/usr/bin/env python3
"""Record OAK-D Lite stereo depth at measured target distances (depth range test).

Uses the same StereoDepth settings as basalt_rtabmap_slam_ros2 (CAM_B/CAM_C at
640x480 -> 640x400, subpixel, LR check threshold 10, no extended disparity,
distortion correction, depth aligned to CAM_B) unless a flag overrides one, so
the numbers describe the depth the drone actually flies on.

The OAK takes one host connection: STOP THE FLIGHT STACK FIRST.
Run with the system python3 (it has depthai; .venv-mavlink does not):

    python3 scripts/oak_depth_range_record.py --label current
    python3 scripts/oak_depth_range_record.py --label extended --extended-disparity

At each prompt, set the target up square to the camera, measure from the
camera's front face, and type:
    1.5          distance in metres; tag defaults to "flat" (textured board)
    2 wall       distance plus a tag: wall, angle45, angle70, dowel, dim, ...
    p            3 s live readout of the image centre, for aiming
    q            finish

Each stop is saved as stop_NNN.npz (all depth frames, mm) and stop_NNN.png
(rectified left with the ROI box | depth, near = bright, invalid = black), and
gets a one-line accuracy summary so a bad stop can be redone on the spot.
Analyse with:
    .venv-mavlink/bin/python scripts/oak_depth_range_analyze.py <session dir>
"""
import argparse
import datetime
import json
import os
import sys
import time

import depthai as dai
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from oak_depth_range_common import preview_image, roi_box, stop_stats, write_png_gray  # noqa: E402

MEDIAN = {
    "off": dai.StereoDepthConfig.MedianFilter.MEDIAN_OFF,
    "3": dai.StereoDepthConfig.MedianFilter.KERNEL_3x3,
    "5": dai.StereoDepthConfig.MedianFilter.KERNEL_5x5,
    "7": dai.StereoDepthConfig.MedianFilter.KERNEL_7x7,
}


def parse_args():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--label", default="current", help="session name, e.g. current / extended")
    ap.add_argument("--out-root", default=os.path.expanduser("~/oak_range_tests"))
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=400)
    ap.add_argument("--fps", type=float, default=20.0)
    ap.add_argument("--seconds", type=float, default=5.0, help="recording length per stop")
    ap.add_argument("--extended-disparity", action="store_true")
    ap.add_argument("--no-subpixel", action="store_true")
    ap.add_argument("--lr-threshold", type=int, default=10)
    ap.add_argument("--median", choices=["default", *MEDIAN], default="default",
                    help="'default' leaves DepthAI's default, as the flight pipeline does")
    ap.add_argument("--board-size", type=float, default=0.6,
                    help="target width in metres; sizes the ROI at each distance")
    ap.add_argument("--roi-fraction", type=float, default=0.5,
                    help="ROI side as a fraction of the target's apparent width")
    return ap.parse_args()


def build_pipeline(args):
    p = dai.Pipeline()
    calib = p.getDefaultDevice().readCalibration()
    left = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B, sensorResolution=(640, 480),
                                           sensorFps=args.fps)
    right = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C, sensorResolution=(640, 480),
                                            sensorFps=args.fps)
    stereo = p.create(dai.node.StereoDepth)
    stereo.setExtendedDisparity(args.extended_disparity)
    stereo.setLeftRightCheck(True)
    stereo.setSubpixel(not args.no_subpixel)
    stereo.setRectifyEdgeFillColor(0)
    stereo.enableDistortionCorrection(True)
    stereo.initialConfig.setLeftRightCheckThreshold(args.lr_threshold)
    stereo.setDepthAlign(dai.CameraBoardSocket.CAM_B)
    if args.median != "default":
        stereo.initialConfig.setMedianFilter(MEDIAN[args.median])
    left.requestOutput((args.width, args.height)).link(stereo.left)
    right.requestOutput((args.width, args.height)).link(stereo.right)
    depth_q = stereo.depth.createOutputQueue(maxSize=8, blocking=False)
    rect_q = stereo.rectifiedLeft.createOutputQueue(maxSize=1, blocking=False)
    return p, calib, stereo, depth_q, rect_q


def config_readback(stereo):
    """What the stereo engine was actually told, for the session record."""
    cfg = stereo.initialConfig
    out = {}
    for name in ("getMedianFilter", "getConfidenceThreshold", "getLeftRightCheckThreshold",
                 "getMaxDisparity", "getSubpixelFractionalBits", "getDepthUnit",
                 "getExtendedDisparity", "getSubpixel", "getLeftRightCheck"):
        try:
            out[name[3:]] = str(getattr(cfg, name)())
        except Exception as exc:  # readback is informational only
            out[name[3:]] = f"unavailable ({exc.__class__.__name__})"
    return out


def drain(queue):
    while queue.tryGet() is not None:
        pass


def live_readout(depth_q, seconds=3.0):
    drain(depth_q)
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        frame = depth_q.get().getFrame()
        h, w = frame.shape
        patch = frame[h // 2 - 20:h // 2 + 20, w // 2 - 20:w // 2 + 20]
        good = patch[patch > 0]
        med = f"{np.median(good) / 1000:.3f} m" if good.size else "no depth"
        print(f"  centre 40x40 px: {med}, fill {good.size / patch.size:.0%}", flush=True)
        time.sleep(0.5)
        drain(depth_q)


def record_stop(depth_q, rect_q, seconds):
    drain(depth_q)
    frames, stamps = [], []
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        msg = depth_q.get()
        frames.append(msg.getFrame().copy())
        stamps.append(msg.getTimestamp().total_seconds())
    rect_msg = rect_q.get()
    return np.stack(frames), np.array(stamps), rect_msg.getFrame().copy()


def main():
    args = parse_args()
    stamp = datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    session = os.path.join(args.out_root, f"{stamp}_{args.label}")

    try:
        p, calib, stereo, depth_q, rect_q = build_pipeline(args)
    except RuntimeError as exc:
        sys.exit(f"could not open the OAK ({exc}).\nIs the flight stack still running? "
                 "The camera takes one host connection.")

    intr = calib.getCameraIntrinsics(dai.CameraBoardSocket.CAM_B, args.width, args.height)
    fx, fy, cx, cy = intr[0][0], intr[1][1], intr[0][2], intr[1][2]
    baseline_m = abs(calib.getBaselineDistance(dai.CameraBoardSocket.CAM_C,
                                               dai.CameraBoardSocket.CAM_B)) / 100.0
    os.makedirs(session)
    meta = {
        "label": args.label,
        "started_utc": stamp,
        "depthai_version": dai.__version__,
        "device_id": p.getDefaultDevice().getDeviceId(),
        "width": args.width, "height": args.height, "fps": args.fps, "seconds": args.seconds,
        "extended_disparity": args.extended_disparity, "subpixel": not args.no_subpixel,
        "lr_threshold": args.lr_threshold, "median": args.median,
        "board_size_m": args.board_size, "roi_fraction": args.roi_fraction,
        "fx": fx, "fy": fy, "cx": cx, "cy": cy, "baseline_m": baseline_m,
        "stereo_config": config_readback(stereo),
    }
    with open(os.path.join(session, "session.json"), "w") as f:
        json.dump(meta, f, indent=2)

    p.start()
    print(f"session {session}")
    print(f"fx={fx:.1f} px, baseline={baseline_m * 100:.2f} cm -> f*B={fx * baseline_m:.2f} px*m; "
          f"extended={args.extended_disparity} subpixel={not args.no_subpixel} median={args.median}")
    print("warming up 3 s ...", flush=True)
    time.sleep(3.0)

    index = 0
    try:
        while p.isRunning():
            line = input("\ndistance m [tag] | p | q > ").strip()
            if not line:
                continue
            if line.lower() == "q":
                break
            if line.lower() == "p":
                live_readout(depth_q)
                continue
            parts = line.split()
            try:
                distance = float(parts[0])
            except ValueError:
                print("  expected a distance in metres, p, or q")
                continue
            if not 0.05 <= distance <= 30.0:
                print("  distance out of range (0.05-30 m)")
                continue
            tag = parts[1] if len(parts) > 1 else "flat"

            print(f"  recording {args.seconds:.0f} s at {distance:.3f} m [{tag}] - keep still", flush=True)
            depth, stamps, rect = record_stop(depth_q, rect_q, args.seconds)
            name = f"stop_{index:03d}"
            np.savez_compressed(os.path.join(session, name + ".npz"), depth=depth, t=stamps, rect=rect,
                                distance_m=distance, tag=tag)
            roi = roi_box(fx, distance, args.board_size, args.roi_fraction, args.width, args.height)
            write_png_gray(os.path.join(session, name + ".png"),
                           preview_image(rect, depth[-1], roi, distance))
            s = stop_stats(depth, roi, fx, fy, cx, cy, distance)
            rate = (len(stamps) - 1) / (stamps[-1] - stamps[0]) if len(stamps) > 1 else float("nan")
            print(f"  {name}: {s['frames']} frames ({rate:.1f} fps), ROI {s['roi_px']} px | "
                  f"median {s['median_m']:.3f} m, bias {s['bias_m'] * 100:+.1f} cm, "
                  f"noise {s['temporal_m'] * 100:.2f} cm, fill {s['fill']:.0%}, "
                  f"false {s['outlier_frac']:.0%}")
            if s["fill"] < 0.5 or abs(s["bias_m"]) > 0.2 * distance:
                print(f"  check {name}.png: is the target centred and inside the ROI box?")
            index += 1
    except (KeyboardInterrupt, EOFError):
        print()
    print(f"{index} stops in {session}")
    if index:
        print(f"analyse: .venv-mavlink/bin/python scripts/oak_depth_range_analyze.py {session}")


if __name__ == "__main__":
    main()
