"""Pure-Python decoder for binary OctoMap messages (e.g. /rtabmap/octomap).

Useful for quick numpy analysis of recorded maps without the C++ toolchain.
It approximates voxels by their centres; for the adapter's exact swept-sphere
verdict use the `octomap_path_audit` binary (see audit_flight_3d_bag.py).

Binary encoding: two bits per child, least-significant pair first.
  0 = unknown, 1 = free leaf, 2 = occupied leaf, 3 = inner node (recurse).
"""
import sys

import numpy as np

TREE_DEPTH = 16
KEY_CENTER = 1 << (TREE_DEPTH - 1)


def decode_binary(data, resolution):
    """Return (centres Nx3, sizes N) of occupied leaves, in metres."""
    data = bytes(data)
    pos = 0
    occupied = []

    def read_node(lo_x, lo_y, lo_z, depth):
        nonlocal pos
        first, second = data[pos], data[pos + 1]
        pos += 2
        half = (1 << TREE_DEPTH) >> (depth + 1)
        kinds = [(first >> (2 * i)) & 3 for i in range(4)] + \
                [(second >> (2 * i)) & 3 for i in range(4)]
        children = [(lo_x + (half if i & 1 else 0), lo_y + (half if i & 2 else 0),
                     lo_z + (half if i & 4 else 0)) for i in range(8)]
        for kind, child in zip(kinds, children):
            if kind == 2:
                occupied.append((*child, half))
        for kind, child in zip(kinds, children):
            if kind == 3:
                read_node(*child, depth + 1)

    sys.setrecursionlimit(max(sys.getrecursionlimit(), 10000))
    read_node(0, 0, 0, 0)
    if not occupied:
        return np.zeros((0, 3)), np.zeros(0)
    a = np.array(occupied, dtype=np.float64)
    centres = (a[:, :3] + a[:, 3:4] / 2 - KEY_CENTER) * resolution
    return centres, a[:, 3] * resolution


def expand(centres, sizes, resolution):
    """Split pruned (larger) leaves into resolution-sized voxel centres."""
    out = []
    for centre, size in zip(centres, sizes):
        n = int(round(size / resolution))
        if n <= 1:
            out.append(centre)
            continue
        g = (np.arange(n) - (n - 1) / 2) * resolution
        x, y, z = np.meshgrid(g, g, g, indexing='ij')
        out.extend(np.stack([x.ravel(), y.ravel(), z.ravel()], 1) + centre)
    return np.array(out).reshape(-1, 3)


def segment_distance(points, a, b):
    """Distance from each of points (Nx3) to the segment a-b."""
    a = np.asarray(a, dtype=np.float64)
    b = np.asarray(b, dtype=np.float64)
    d = b - a
    length_sq = d @ d
    t = np.zeros(len(points)) if length_sq < 1e-12 else \
        np.clip(((points - a) @ d) / length_sq, 0.0, 1.0)
    return np.linalg.norm(points - (a + t[:, None] * d), axis=1)
