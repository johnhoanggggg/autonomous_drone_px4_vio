"""Shared numpy-only helpers for the OAK-D Lite depth range test.

Imported by oak_depth_range_record.py (system python3, has depthai) and
oak_depth_range_analyze.py (.venv-mavlink, has matplotlib), so it must depend
on numpy and the standard library only.
"""
import struct
import zlib

import numpy as np

# A ROI pixel further than this from the measured distance is a false match,
# not noise: max(OUTLIER_MIN_M, OUTLIER_REL * distance).
OUTLIER_MIN_M = 0.10
OUTLIER_REL = 0.10
# A pixel needs a valid depth in this fraction of frames to count toward the
# temporal (frame-to-frame) noise.
TEMPORAL_MIN_VALID = 0.8
SPATIAL_MAX_FRAMES = 20


def roi_box(fx, distance_m, board_m, fraction, width, height, center=None, min_px=8):
    """Square ROI (u0, u1, v0, v1) covering `fraction` of the target's width.

    The target spans fx * board_m / distance_m pixels, so the ROI shrinks with
    distance and stays on the board as long as the board is centred.
    """
    cu, cv = center if center is not None else (width / 2.0, height / 2.0)
    side = fraction * fx * board_m / max(distance_m, 1e-3)
    side = int(round(min(max(side, min_px), 0.8 * min(width, height))))
    u0 = int(round(cu - side / 2.0))
    v0 = int(round(cv - side / 2.0))
    u0 = min(max(u0, 0), width - side)
    v0 = min(max(v0, 0), height - side)
    return u0, u0 + side, v0, v0 + side


def _robust_plane_rms(points):
    """RMS depth residual about z = a*x + b*y + c, after one 3-sigma (MAD) rejection.

    Fitted in depth, not as a total-least-squares plane: stereo noise lies along
    depth, and far away it exceeds the target's width, where an SVD plane would
    pick a sideways normal and report the target's width as the "noise".
    """
    for _ in range(2):
        design = np.column_stack([points[:, 0], points[:, 1], np.ones(len(points))])
        coef, *_ = np.linalg.lstsq(design, points[:, 2], rcond=None)
        residual = points[:, 2] - design @ coef
        mad = np.median(np.abs(residual - np.median(residual)))
        keep = np.abs(residual) <= max(3.0 * 1.4826 * mad, 1e-4)
        if keep.sum() < 50:
            # Mostly false matches (MAD collapses onto them): rejection would
            # leave nothing, so report the unrejected spread.
            return float(np.sqrt(np.mean(residual ** 2)))
        if keep.all():
            break
        points = points[keep]
    return float(np.sqrt(np.mean(residual[keep] ** 2)))


def stop_stats(depth_mm, roi, fx, fy, cx, cy, truth_m):
    """Accuracy, noise and fill of one recorded stop over its ROI.

    depth_mm: (frames, H, W) uint16, 0 = invalid (DepthAI convention).
    Returns metres unless the key says otherwise.
    """
    u0, u1, v0, v1 = roi
    z = depth_mm[:, v0:v1, u0:u1].astype(np.float64) / 1000.0
    frames = z.shape[0]
    valid = z > 0.0
    samples = z[valid]
    out = {
        "frames": frames,
        "roi_px": u1 - u0,
        "fill": float(valid.mean()) if valid.size else float("nan"),
        "median_m": float("nan"),
        "bias_m": float("nan"),
        "outlier_frac": float("nan"),
        "temporal_m": float("nan"),
        "spatial_m": float("nan"),
    }
    if samples.size == 0:
        return out
    out["median_m"] = float(np.median(samples))
    out["bias_m"] = out["median_m"] - truth_m
    tolerance = max(OUTLIER_MIN_M, OUTLIER_REL * truth_m)
    out["outlier_frac"] = float(np.mean(np.abs(samples - truth_m) > tolerance))

    count = valid.sum(axis=0)
    steady = count >= max(3, TEMPORAL_MIN_VALID * frames)
    if steady.any():
        stack = np.where(valid, z, np.nan)[:, steady]
        out["temporal_m"] = float(np.median(np.nanstd(stack, axis=0)))

    vv, uu = np.mgrid[v0:v1, u0:u1]
    rms = []
    for i in np.unique(np.linspace(0, frames - 1, min(frames, SPATIAL_MAX_FRAMES)).astype(int)):
        m = valid[i]
        if m.sum() < 50:
            continue
        zi = z[i][m]
        pts = np.column_stack([(uu[m] - cx) * zi / fx, (vv[m] - cy) * zi / fy, zi])
        rms.append(_robust_plane_rms(pts))
    if rms:
        out["spatial_m"] = float(np.median(rms))
    return out


def write_png_gray(path, image):
    """Write an 8-bit grayscale PNG with zlib only (cv2 is broken on this Pi)."""
    image = np.ascontiguousarray(image, dtype=np.uint8)
    height, width = image.shape
    raw = b"".join(b"\x00" + image[row].tobytes() for row in range(height))

    def chunk(tag, data):
        body = tag + data
        return struct.pack(">I", len(data)) + body + struct.pack(">I", zlib.crc32(body) & 0xFFFFFFFF)

    with open(path, "wb") as f:
        f.write(b"\x89PNG\r\n\x1a\n")
        f.write(chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 0, 0, 0, 0)))
        f.write(chunk(b"IDAT", zlib.compress(raw, 6)))
        f.write(chunk(b"IEND", b""))


def preview_image(rect, depth_mm, roi, truth_m):
    """Side by side: rectified left with the ROI boxed | depth scaled to truth +-50%."""
    u0, u1, v0, v1 = roi
    left = np.array(rect, dtype=np.uint8, copy=True)
    z = depth_mm.astype(np.float64) / 1000.0
    lo, hi = 0.5 * truth_m, 1.5 * truth_m
    scaled = np.clip((hi - z) / (hi - lo), 0.0, 1.0) * 235 + 20  # near = bright
    depth = np.where(z > 0, scaled, 0).astype(np.uint8)            # invalid = black
    for img in (left, depth):
        for value, grow in ((0, 1), (255, 0)):
            a0, a1 = max(u0 - grow, 0), min(u1 + grow, img.shape[1]) - 1
            b0, b1 = max(v0 - grow, 0), min(v1 + grow, img.shape[0]) - 1
            img[b0, a0:a1 + 1] = value
            img[b1, a0:a1 + 1] = value
            img[b0:b1 + 1, a0] = value
            img[b0:b1 + 1, a1] = value
    return np.hstack([left, depth])
