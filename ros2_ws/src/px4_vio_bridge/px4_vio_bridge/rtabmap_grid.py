"""Strict conversion helpers for DepthAI RTAB-Map occupancy-grid images."""

import numpy as np
from nav_msgs.msg import OccupancyGrid


def decode_grid_image(values):
    """Invert DepthAI's RTAB-Map display image to ROS ``-1/0..100`` values.

    ``RTABMapSLAM.cpp`` calls RTAB-Map's ``convertMap2Image8U(map)`` and then
    flips the result vertically before putting it in ``MapData.map``. Its
    exact display encoding is 0=occupied, 89=unknown, 178=free and
    200=footprint-free, with intermediate probability values on either side
    of 89. This function reverses both the grayscale mapping and that flip.
    """
    grid = np.asarray(values)
    if grid.ndim == 3 and grid.shape[2] == 1:
        grid = grid[:, :, 0]
    if grid.ndim != 2:
        raise ValueError(f"occupancy grid must be 2-D, got shape {grid.shape}")
    if grid.size == 0:
        raise ValueError("occupancy grid is empty")
    if not np.issubdtype(grid.dtype, np.integer):
        raise ValueError("occupancy-grid image must contain integer grayscale values")
    gray = np.asarray(grid, dtype=np.int16)
    invalid = (gray < 0) | ((gray > 178) & (gray != 200))
    if invalid.any():
        bad = np.unique(gray[invalid])[:8].tolist()
        raise ValueError(f"unsupported RTAB-Map grid grayscale values: {bad}")
    occupancy = np.empty_like(gray, dtype=np.int16)
    occupancy[gray == 89] = -1
    occupancy[(gray == 178) | (gray == 200)] = 0
    lower = gray < 89
    occupancy[lower] = np.rint(100.0 - gray[lower] / 1.78).astype(np.int16)
    upper = (gray > 89) & (gray < 178)
    occupancy[upper] = np.rint((178.0 - gray[upper]) / 1.78).astype(np.int16)
    # DepthAI flipped RTAB-Map's map for image display. ROS OccupancyGrid uses
    # the unflipped row order with minX/minY as the lower map bounds.
    return np.flipud(np.clip(occupancy, -1, 100)).astype(np.int8, copy=False)


def make_occupancy_grid(values, *, min_x, min_y, resolution, stamp, frame_id="world"):
    """Convert one DepthAI map image into ``nav_msgs/OccupancyGrid``.

    RTAB-Map supplies the lower map bounds as ``minX``/``minY``. The live
    hardware validation must still confirm that the ImgFrame row direction
    matches ROS's increasing-Y row convention.
    """
    if resolution <= 0.0:
        raise ValueError("occupancy-grid resolution must be positive")
    grid = decode_grid_image(values)
    msg = OccupancyGrid()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.info.map_load_time = stamp
    msg.info.resolution = float(resolution)
    msg.info.width = int(grid.shape[1])
    msg.info.height = int(grid.shape[0])
    msg.info.origin.position.x = float(min_x)
    msg.info.origin.position.y = float(min_y)
    msg.info.origin.orientation.w = 1.0
    msg.data = grid.reshape(-1).astype(np.int8).tolist()
    return msg
