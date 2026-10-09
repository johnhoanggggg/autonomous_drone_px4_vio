import struct
import zlib

import numpy as np
import pytest

from oak_depth_range_common import preview_image, roi_box, stop_stats, write_png_gray

FX, FY, CX, CY = 440.0, 440.0, 320.0, 200.0


def synthetic_stop(z, sigma_m, frames=40, bias=0.0, tilt=0.0, dropout=0.0, outliers=0.0, seed=3):
    """A flat target at z (optionally tilted about the vertical axis), as uint16 mm."""
    rng = np.random.default_rng(seed)
    v, u = np.mgrid[0:400, 0:640]
    x = (u - CX) / FX
    plane = (z + bias) / (1.0 - np.tan(tilt) * x)          # depth of a plane yawed by `tilt`
    depth = plane[None] + rng.normal(0.0, sigma_m, (frames, 400, 640))
    if outliers:
        wrong = rng.random(depth.shape) < outliers
        depth[wrong] = z * 2.5
    if dropout:
        depth[rng.random(depth.shape) < dropout] = 0.0
    return np.round(depth * 1000).clip(0, 65535).astype(np.uint16)


def test_roi_shrinks_with_distance_and_stays_on_image():
    near = roi_box(FX, 0.3, 0.6, 0.5, 640, 400)
    mid = roi_box(FX, 2.0, 0.6, 0.5, 640, 400)
    far = roi_box(FX, 20.0, 0.6, 0.5, 640, 400)
    assert near[1] - near[0] == 320                   # clamped to 0.8 * height
    assert mid[1] - mid[0] == 66                      # 0.5 * 440 * 0.6 / 2
    assert far[1] - far[0] == 8                       # min_px
    for u0, u1, v0, v1 in (near, mid, far):
        assert 0 <= u0 < u1 <= 640 and 0 <= v0 < v1 <= 400


def test_recovers_bias_noise_and_full_fill():
    depth = synthetic_stop(2.0, sigma_m=0.02, bias=0.05)
    s = stop_stats(depth, roi_box(FX, 2.0, 0.6, 0.5, 640, 400), FX, FY, CX, CY, 2.0)
    assert s["bias_m"] == pytest.approx(0.05, abs=0.003)
    assert s["temporal_m"] == pytest.approx(0.02, rel=0.15)
    assert s["spatial_m"] == pytest.approx(0.02, rel=0.15)
    assert s["fill"] == 1.0
    assert s["outlier_frac"] == 0.0


def test_spatial_noise_ignores_target_tilt():
    depth = synthetic_stop(1.5, sigma_m=0.005, tilt=np.radians(30))
    s = stop_stats(depth, roi_box(FX, 1.5, 0.6, 0.5, 640, 400), FX, FY, CX, CY, 1.5)
    assert s["spatial_m"] == pytest.approx(0.005, rel=0.2)


def test_dropout_lowers_fill_and_false_matches_are_counted():
    depth = synthetic_stop(1.0, sigma_m=0.005, dropout=0.3, outliers=0.1)
    s = stop_stats(depth, roi_box(FX, 1.0, 0.6, 0.5, 640, 400), FX, FY, CX, CY, 1.0)
    assert s["fill"] == pytest.approx(0.7, abs=0.02)
    assert s["outlier_frac"] == pytest.approx(0.1, abs=0.02)
    assert s["median_m"] == pytest.approx(1.0, abs=0.01)


def test_no_depth_at_all_reports_nan_not_a_crash():
    depth = np.zeros((5, 400, 640), dtype=np.uint16)
    s = stop_stats(depth, roi_box(FX, 0.2, 0.6, 0.5, 640, 400), FX, FY, CX, CY, 0.2)
    assert s["fill"] == 0.0
    assert np.isnan(s["bias_m"]) and np.isnan(s["temporal_m"]) and np.isnan(s["spatial_m"])


def test_png_round_trips(tmp_path):
    rect = (np.arange(400 * 640) % 251).reshape(400, 640).astype(np.uint8)
    depth = synthetic_stop(1.0, 0.01, frames=1)[0]
    img = preview_image(rect, depth, roi_box(FX, 1.0, 0.6, 0.5, 640, 400), 1.0)
    path = tmp_path / "p.png"
    write_png_gray(path, img)
    blob = path.read_bytes()
    assert blob[:8] == b"\x89PNG\r\n\x1a\n"
    width, height = struct.unpack(">II", blob[16:24])
    assert (width, height) == (1280, 400)
    idat_len = struct.unpack(">I", blob[33:37])[0]
    raw = zlib.decompress(blob[41:41 + idat_len])
    rows = np.frombuffer(raw, dtype=np.uint8).reshape(400, 1281)[:, 1:]
    assert np.array_equal(rows, img)
