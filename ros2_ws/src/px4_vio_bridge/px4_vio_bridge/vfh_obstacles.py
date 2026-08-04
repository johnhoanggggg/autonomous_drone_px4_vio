"""Turn the RTAB-Map obstacle cloud into body-frame samples for VFH.

Input is `/rtabmap/obstacle_cloud`: a `sensor_msgs/PointCloud2` in the ENU
`world` frame that the SLAM node already ground-segments for us. The DepthAI
bridge publishes the latest obstacle cloud rather than a persistent map, so
this module keeps a short-lived, voxelised world-frame memory. An obstacle does
not disappear merely because the vehicle yawed until the camera lost sight of
it. Memory expires to avoid preserving stereo noise or moved objects forever.
It is only
published when the stack runs with `slam_publish_clouds:=true` — without that
argument this module sees nothing at all, which is the first thing to check when
the histogram is empty.

**Which pose the cloud is measured against matters.** The cloud lives in the
SLAM (loop-corrected) frame, while PX4 flies on the raw VIO pose; a loop closure
shifts one relative to the other by the 10-34 cm measured in
HANDOFF_LOOP_CLOSURE.md. So the transform from cloud to body frame uses the
*SLAM* pose (`/rtabmap/pose`) — same frame as the cloud, so the relative
geometry stays consistent no matter what the loop closure does. Only the
resulting relative bearing is handed to the flight node, which re-references it
to PX4's own heading. Nothing here ever mixes the two frames' positions.

Height slab: only points within `z_below` below and `z_above` above the vehicle
can be hit by it.

**`z_below` must be smaller than the hover height, with margin.** The slab is
measured from the vehicle, so `z_below` 0.35 at a 0.30 m hover reaches below the
floor and every ground return within `max_range` arrives as an obstacle — a
symmetric red wall across the entire forward arc with nothing actually there.
RTAB-Map is supposed to segment the floor into `/rtabmap/ground_cloud`, but
near-field ground routinely leaks into the obstacle cloud, so this filter is the
one that has to hold. The default is 0.15 m for the 0.30 m hover this project
flies; raise it only as far as `hover_height - 0.15`.
"""
import math
import time

import numpy as np
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2, PointField


class ObstacleSnapshot:
    """What the planner gets: samples plus the numbers safety decisions use."""

    def __init__(
        self,
        samples=(),
        nearest_range=math.inf,
        nearest_bearing=None,
        point_count=0,
        kept_count=0,
        cloud_age=math.inf,
        pose_age=math.inf,
        current_point_count=0,
        memory_point_count=0,
    ):
        self.samples = samples
        # Taken from every point that passed the slab/range filters, BEFORE
        # decimation, so the emergency-stop distance is exact even when the
        # histogram itself is built from a subsample.
        self.nearest_range = nearest_range
        self.nearest_bearing = nearest_bearing
        self.point_count = point_count
        self.kept_count = kept_count
        self.cloud_age = cloud_age
        self.pose_age = pose_age
        self.current_point_count = current_point_count
        self.memory_point_count = memory_point_count

    @property
    def valid(self):
        return self.kept_count > 0 or self.point_count > 0


def pointcloud_xyz(msg):
    """Nx3 float32 view of a PointCloud2's x/y/z, without ros2_numpy.

    Reads the field offsets from the message rather than assuming the layout of
    the publisher we happen to have.
    """
    if msg.is_bigendian:
        raise ValueError("big-endian PointCloud2 is not supported")
    offsets = {}
    for f in msg.fields:
        if f.name in ("x", "y", "z"):
            if f.datatype != PointField.FLOAT32 or f.count != 1:
                raise ValueError(f"field '{f.name}' is not a single float32")
            offsets[f.name] = f.offset
    if len(offsets) != 3:
        raise ValueError("PointCloud2 is missing float32 x/y/z fields")

    count = int(msg.width) * int(msg.height)
    dtype = np.dtype(
        {
            "names": ["x", "y", "z"],
            "formats": ["<f4"] * 3,
            "offsets": [offsets["x"], offsets["y"], offsets["z"]],
            "itemsize": int(msg.point_step),
        }
    )
    raw = np.frombuffer(bytes(msg.data), dtype=dtype, count=count)
    return np.stack((raw["x"], raw["y"], raw["z"]), axis=1).astype(np.float64)


class WorldObstacleMemory:
    """Bounded voxel map of recently observed world-frame obstacle points.

    Each voxel holds the complete point batch from the newest cloud that saw
    it. Replacing that batch instead of appending every frame is important:
    current-cloud point density is preserved, but VFH density cannot grow just
    because the camera stared at a surface. A finite lifetime also lets moved
    objects and bad stereo returns disappear without free-space ray tracing.
    """

    def __init__(
        self,
        *,
        duration=30.0,
        voxel_size=0.10,
        max_points=20000,
        clock=time.monotonic,
    ):
        self.duration = float(duration)
        self.voxel_size = float(voxel_size)
        self.max_points = int(max_points)
        self.clock = clock
        if self.duration < 0.0:
            raise ValueError("obstacle memory duration must be non-negative")
        if self.voxel_size <= 0.0:
            raise ValueError("obstacle memory voxel size must be positive")
        if self.max_points < 1:
            raise ValueError("obstacle memory max_points must be positive")
        self._voxels = {}
        self._snapshot_cache = None

    @property
    def enabled(self):
        return self.duration > 0.0

    def clear(self):
        self._voxels.clear()
        self._snapshot_cache = None

    def _prune(self, now):
        if not self.enabled:
            self.clear()
            return
        cutoff = now - self.duration
        expired = [key for key, value in self._voxels.items() if value[1] < cutoff]
        for key in expired:
            del self._voxels[key]
        if expired:
            self._snapshot_cache = None

    def update(self, points, now=None):
        """Merge one cloud, keeping only the newest sample in each voxel."""
        now = self.clock() if now is None else float(now)
        self._prune(now)
        if not self.enabled:
            return
        points = np.asarray(points, dtype=np.float64).reshape(-1, 3)
        points = points[np.isfinite(points).all(axis=1)]
        if points.size == 0:
            return
        self._merge(points, now)
        self._enforce_limit()

    def _merge(self, points, timestamp):
        keys = np.floor(points / self.voxel_size).astype(np.int64)
        # Group once. The old np.unique(..., return_inverse=True) followed by
        # `points[inverse == index]` scanned the whole cloud for every voxel:
        # O(points * voxels), which took ~0.5 s for a 2,800-point live cloud on
        # the Pi. Sorting makes the grouping O(points log points).
        order = np.lexsort((keys[:, 2], keys[:, 1], keys[:, 0]))
        sorted_keys = keys[order]
        sorted_points = points[order]
        boundaries = np.flatnonzero(
            np.any(sorted_keys[1:] != sorted_keys[:-1], axis=1)
        ) + 1
        starts = np.concatenate(([0], boundaries))
        ends = np.concatenate((boundaries, [len(sorted_points)]))
        for start, end in zip(starts, ends):
            key = sorted_keys[start]
            self._voxels[tuple(key.tolist())] = (
                np.array(sorted_points[start:end], dtype=np.float64, copy=True),
                float(timestamp),
            )
        self._snapshot_cache = None

    def _enforce_limit(self):
        point_count = sum(len(value[0]) for value in self._voxels.values())
        if point_count > self.max_points:
            oldest = sorted(self._voxels, key=lambda key: self._voxels[key][1])
            for key in oldest:
                point_count -= len(self._voxels[key][0])
                del self._voxels[key]
                self._snapshot_cache = None
                if point_count <= self.max_points:
                    break

    def snapshot(self, now=None):
        now = self.clock() if now is None else float(now)
        self._prune(now)
        if not self._voxels:
            return np.empty((0, 3), dtype=np.float64)
        if self._snapshot_cache is None:
            self._snapshot_cache = np.concatenate(
                [value[0] for value in self._voxels.values()], axis=0
            ).astype(np.float64, copy=False)
        return self._snapshot_cache

    def __len__(self):
        return sum(len(value[0]) for value in self._voxels.values())


def cloud_to_samples(
    points_enu,
    origin_enu,
    yaw_enu,
    *,
    min_range,
    max_range,
    z_below,
    z_above,
    max_samples=1200,
):
    """Filter an ENU cloud to a height slab and reduce it to (range, bearing).

    Returns `(samples, nearest_range, nearest_bearing, kept_count)` where
    `samples` is at most `max_samples` long. The nearest values are computed
    from the full filtered set, not the decimated one.
    """
    points = np.asarray(points_enu, dtype=np.float64).reshape(-1, 3)
    if points.size == 0:
        return [], math.inf, None, 0

    delta = points - np.asarray(origin_enu, dtype=np.float64).reshape(1, 3)
    finite = np.isfinite(delta).all(axis=1)
    slab = (delta[:, 2] >= -abs(z_below)) & (delta[:, 2] <= abs(z_above))
    ranges = np.hypot(delta[:, 0], delta[:, 1])
    band = (ranges >= min_range) & (ranges <= max_range)
    keep = finite & slab & band
    if not np.any(keep):
        return [], math.inf, None, 0

    ranges = ranges[keep]
    delta = delta[keep]
    # Vectorised vfh2d.relative_bearing_enu: 0 ahead, positive to the right.
    point_angle = np.arctan2(delta[:, 1], delta[:, 0])
    bearings = np.arctan2(
        np.sin(yaw_enu - point_angle), np.cos(yaw_enu - point_angle)
    )

    nearest_i = int(np.argmin(ranges))
    nearest_range = float(ranges[nearest_i])
    nearest_bearing = float(bearings[nearest_i])
    kept = int(ranges.size)

    if kept > max_samples:
        # Uniform stride, not "the closest N": keeping only near points would
        # bias the density histogram toward whatever the vehicle is already
        # closest to. The exact nearest range is reported separately above.
        stride = int(math.ceil(kept / max_samples))
        ranges = ranges[::stride]
        bearings = bearings[::stride]

    samples = list(zip(ranges.tolist(), bearings.tolist()))
    return samples, nearest_range, nearest_bearing, kept


def yaw_enu_from_quaternion(q):
    """ENU yaw (counter-clockwise from east) from a ROS quaternion."""
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class ObstacleField:
    """Subscribes to the cloud and the SLAM pose; hands out snapshots."""

    def __init__(
        self,
        node,
        *,
        cloud_topic="/rtabmap/obstacle_cloud",
        pose_topic="/rtabmap/pose",
        min_range=0.25,
        max_range=2.0,
        z_below=0.15,
        z_above=0.60,
        max_samples=1200,
        memory_duration=30.0,
        memory_voxel_size=0.10,
        memory_max_points=20000,
        memory_correction_topic="/vio/map_correction_target",
        memory_reset_correction_m=0.05,
        memory_reset_correction_deg=2.0,
        clock=None,
    ):
        self.node = node
        self.min_range = float(min_range)
        self.max_range = float(max_range)
        self.z_below = float(z_below)
        self.z_above = float(z_above)
        self.max_samples = int(max_samples)
        # OffboardHover exposes monotonic_time() so tests can freeze it; fall
        # back for any node that does not.
        self.clock = clock or getattr(node, "monotonic_time", time.monotonic)

        self.memory = WorldObstacleMemory(
            duration=memory_duration,
            voxel_size=memory_voxel_size,
            max_points=memory_max_points,
            clock=self.clock,
        )

        self.points = None
        self.current_point_count = 0
        self.memory_correction = None
        self.memory_reset_correction_m = float(memory_reset_correction_m)
        self.memory_reset_correction_yaw = math.radians(
            float(memory_reset_correction_deg)
        )
        self.cloud_time = None
        self.cloud_error = None
        self.origin = None          # (x, y, z) ENU, SLAM frame
        self.yaw_enu = None
        self.pose_time = None
        self.clouds_received = 0
        self.poses_received = 0

        # Both publishers use the default reliable depth-10 profile.
        node.create_subscription(PointCloud2, cloud_topic, self.on_cloud, 10)
        node.create_subscription(PoseStamped, pose_topic, self.on_pose, 10)
        if memory_correction_topic:
            node.create_subscription(
                PoseStamped, memory_correction_topic, self.on_memory_correction, 10
            )
        self.cloud_topic = cloud_topic
        self.pose_topic = pose_topic

    # --- subscriptions -----------------------------------------------------
    def on_cloud(self, msg):
        try:
            points = pointcloud_xyz(msg)
            self.cloud_error = None
        except (ValueError, TypeError) as exc:
            # A malformed cloud must not take the flight node down; it goes
            # stale instead, and staleness already has a defined response.
            self.cloud_error = str(exc)
            return
        self.current_point_count = int(len(points))
        if self.memory.enabled:
            self.memory.update(points)
            self.points = None
        else:
            self.points = points
        self.cloud_time = self.clock()
        self.clouds_received += 1

    def on_pose(self, msg):
        p = msg.pose.position
        self.origin = (float(p.x), float(p.y), float(p.z))
        self.yaw_enu = yaw_enu_from_quaternion(msg.pose.orientation)
        self.pose_time = self.clock()
        self.poses_received += 1

    def on_memory_correction(self, msg):
        """Invalidate memory after a real map shift, ignoring correction jitter."""
        p = msg.pose.position
        correction = (
            (float(p.x), float(p.y), float(p.z)),
            yaw_enu_from_quaternion(msg.pose.orientation),
        )
        if self.memory_correction is not None and self.memory.enabled:
            old_translation, old_yaw = self.memory_correction
            new_translation, new_yaw = correction
            translation_change = math.dist(old_translation, new_translation)
            yaw_change = abs(
                math.atan2(
                    math.sin(new_yaw - old_yaw), math.cos(new_yaw - old_yaw)
                )
            )
            if (
                translation_change >= self.memory_reset_correction_m
                or yaw_change >= self.memory_reset_correction_yaw
            ):
                # Correction and cloud callbacks are asynchronous, so rebasing
                # here can transform a cloud already in the new map frame a
                # second time. Clear and deliberately make input stale: VFH
                # holds until the next cloud repopulates the corrected frame.
                self.memory.clear()
                self.points = None
                self.current_point_count = 0
                self.cloud_time = None
                self.node.get_logger().warn(
                    "SLAM map correction changed by "
                    f"{translation_change:.3f}m/{math.degrees(yaw_change):.2f}deg; "
                    "cleared VFH memory pending a fresh cloud"
                )
                self.memory_correction = correction
        else:
            self.memory_correction = correction

    # --- output ------------------------------------------------------------
    def ages(self):
        now = self.clock()
        cloud_age = math.inf if self.cloud_time is None else now - self.cloud_time
        pose_age = math.inf if self.pose_time is None else now - self.pose_time
        return cloud_age, pose_age

    def stale_reason(self, timeout):
        """Why the obstacle picture cannot be trusted right now, or None."""
        cloud_age, pose_age = self.ages()
        if self.cloud_error is not None:
            return f"obstacle cloud unreadable: {self.cloud_error}"
        if self.cloud_time is None:
            return (
                f"no obstacle cloud on {self.cloud_topic} "
                "(is the stack running with slam_publish_clouds:=true?)"
            )
        if self.pose_time is None:
            return f"no SLAM pose on {self.pose_topic}"
        if cloud_age > timeout:
            return f"obstacle cloud stale for {cloud_age:.2f}s (>{timeout:.2f}s)"
        if pose_age > timeout:
            return f"SLAM pose stale for {pose_age:.2f}s (>{timeout:.2f}s)"
        return None

    def snapshot(self):
        cloud_age, pose_age = self.ages()
        points = self.memory.snapshot() if self.memory.enabled else self.points
        if points is None or self.origin is None or self.yaw_enu is None:
            return ObstacleSnapshot(cloud_age=cloud_age, pose_age=pose_age)
        samples, nearest_range, nearest_bearing, kept = cloud_to_samples(
            points,
            self.origin,
            self.yaw_enu,
            min_range=self.min_range,
            max_range=self.max_range,
            z_below=self.z_below,
            z_above=self.z_above,
            max_samples=self.max_samples,
        )
        return ObstacleSnapshot(
            samples=samples,
            nearest_range=nearest_range,
            nearest_bearing=nearest_bearing,
            point_count=int(len(points)),
            kept_count=kept,
            cloud_age=cloud_age,
            pose_age=pose_age,
            current_point_count=self.current_point_count,
            memory_point_count=int(len(points)) if self.memory.enabled else 0,
        )
