"""ROS-free geometry and state for a position-only polyline follower."""

import math
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple


Point2 = Tuple[float, float]
Correction4 = Tuple[float, float, float, float]


@dataclass(frozen=True)
class Projection:
    point: Point2
    along: float
    cross_track: float
    segment: int


@dataclass(frozen=True)
class FollowResult:
    status: str
    valid: bool
    desired_carrot: Point2
    commanded_carrot: Point2
    commanded_displacement: Point2
    path_progress: float
    progress: float
    remaining: float
    cross_track: float
    generation: int


class Polyline:
    def __init__(self, points: Sequence[Point2]):
        cleaned = []
        for point in points:
            value = (float(point[0]), float(point[1]))
            if not all(math.isfinite(v) for v in value):
                raise ValueError("path contains a non-finite point")
            if not cleaned or math.dist(cleaned[-1], value) > 1.0e-9:
                cleaned.append(value)
        if not cleaned:
            raise ValueError("path needs at least one point")
        self.points = tuple(cleaned)
        cumulative = [0.0]
        for first, second in zip(self.points, self.points[1:]):
            cumulative.append(cumulative[-1] + math.dist(first, second))
        self.cumulative = tuple(cumulative)
        self.length = cumulative[-1]

    def project(self, point: Point2) -> Projection:
        if len(self.points) == 1:
            return Projection(self.points[0], 0.0, math.dist(point, self.points[0]), 0)
        px, py = point
        best = None
        for index, (start, end) in enumerate(zip(self.points, self.points[1:])):
            dx, dy = end[0] - start[0], end[1] - start[1]
            length_sq = dx * dx + dy * dy
            fraction = max(0.0, min(1.0, ((px - start[0]) * dx + (py - start[1]) * dy) / length_sq))
            projected = (start[0] + fraction * dx, start[1] + fraction * dy)
            distance = math.dist(point, projected)
            along = self.cumulative[index] + fraction * math.sqrt(length_sq)
            candidate = Projection(projected, along, distance, index)
            if best is None or (candidate.cross_track, -candidate.along) < (best.cross_track, -best.along):
                best = candidate
        return best

    def point_at(self, along: float) -> Point2:
        if len(self.points) == 1:
            return self.points[0]
        distance = max(0.0, min(self.length, float(along)))
        for index in range(len(self.points) - 1):
            start_distance = self.cumulative[index]
            end_distance = self.cumulative[index + 1]
            if distance <= end_distance:
                fraction = (distance - start_distance) / (end_distance - start_distance)
                start, end = self.points[index], self.points[index + 1]
                return (
                    start[0] + fraction * (end[0] - start[0]),
                    start[1] + fraction * (end[1] - start[1]),
                )
        return self.points[-1]


def path_fingerprint(points: Sequence[Point2], precision: int = 4) -> Tuple[Point2, ...]:
    return tuple((round(float(x), precision), round(float(y), precision)) for x, y in points)


def _limit_norm(vector: Point2, limit: float) -> Point2:
    norm = math.hypot(*vector)
    if norm <= limit or norm <= 0.0:
        return vector
    scale = limit / norm
    return vector[0] * scale, vector[1] * scale


def _wrap_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _correction_delta(first: Correction4, second: Correction4) -> Tuple[float, float]:
    return (
        math.dist(first[:3], second[:3]),
        abs(_wrap_pi(second[3] - first[3])),
    )


def _blend_correction(first: Correction4, second: Correction4, fraction: float) -> Correction4:
    return (
        first[0] + fraction * (second[0] - first[0]),
        first[1] + fraction * (second[1] - first[1]),
        first[2] + fraction * (second[2] - first[2]),
        _wrap_pi(first[3] + fraction * _wrap_pi(second[3] - first[3])),
    )


class CorrectionReplanGate:
    """Coalesce noisy correction samples into infrequent replan barriers.

    The raw correction target can move by centimetres from one SLAM update to
    the next. Gating on each sample starves the follower. This detector filters
    the target, starts one barrier for a material correction episode, and then
    requires both a quiet interval and a path received after the last material
    movement. A cooldown prevents one graph optimization from becoming many
    separate barriers.
    """

    def __init__(
        self,
        translation_trigger: float = 0.05,
        yaw_trigger: float = math.radians(1.5),
        filter_time_constant: float = 0.35,
        material_translation: float = 0.03,
        material_yaw: float = math.radians(0.75),
        quiet_time: float = 0.40,
        cooldown: float = 8.0,
    ):
        values = (
            translation_trigger, yaw_trigger, filter_time_constant,
            material_translation, material_yaw, quiet_time, cooldown,
        )
        if any(not math.isfinite(value) or value <= 0.0 for value in values):
            raise ValueError("correction gate limits must be finite and positive")
        self.translation_trigger = translation_trigger
        self.yaw_trigger = yaw_trigger
        self.filter_time_constant = filter_time_constant
        self.material_translation = material_translation
        self.material_yaw = material_yaw
        self.quiet_time = quiet_time
        self.cooldown = cooldown
        self.filtered: Optional[Correction4] = None
        self.baseline: Optional[Correction4] = None
        self.event_reference: Optional[Correction4] = None
        self.last_observation: Optional[float] = None
        self.last_material_change = 0.0
        self.path_after_change = False
        self.pending = False
        self.cooldown_until = 0.0
        self.last_trigger_delta = (0.0, 0.0)

    def observe(self, correction: Correction4, now: float) -> bool:
        if not all(math.isfinite(value) for value in (*correction, now)):
            return False
        if self.filtered is None or self.last_observation is None or now < self.last_observation:
            self.filtered = correction
            self.baseline = correction
            self.event_reference = correction
            self.last_observation = now
            return False

        dt = now - self.last_observation
        self.last_observation = now
        fraction = 1.0 - math.exp(-dt / self.filter_time_constant)
        self.filtered = _blend_correction(self.filtered, correction, fraction)

        if self.pending:
            translation, yaw = _correction_delta(self.event_reference, self.filtered)
            if translation > self.material_translation or yaw > self.material_yaw:
                self.event_reference = self.filtered
                self.last_material_change = now
                self.path_after_change = False
            return False

        if now < self.cooldown_until:
            return False
        translation, yaw = _correction_delta(self.baseline, self.filtered)
        if translation <= self.translation_trigger and yaw <= self.yaw_trigger:
            return False
        self.pending = True
        self.event_reference = self.filtered
        self.last_material_change = now
        self.path_after_change = False
        self.last_trigger_delta = (translation, yaw)
        return True

    def path_received(self, now: float) -> None:
        if self.pending and math.isfinite(now) and now >= self.last_material_change:
            self.path_after_change = True

    def waiting(self, now: float) -> bool:
        if (
            self.pending
            and self.path_after_change
            and now - self.last_material_change >= self.quiet_time
        ):
            self.pending = False
            self.baseline = self.filtered
            self.cooldown_until = now + self.cooldown
        return self.pending


class PositionRouteFollower:
    """Follow a 2D route with a smoothed position displacement.

    The stored command is relative to the current SLAM pose. A translational
    loop closure therefore moves pose and world-frame carrot together without
    injecting the correction into the relative command intended for a future
    PX4 adapter.
    """

    def __init__(
        self,
        lookahead: float = 0.60,
        max_carrot_speed: float = 0.25,
        max_carrot_acceleration: float = 0.50,
        max_cross_track: float = 0.60,
        arrival_tolerance: float = 0.12,
    ):
        if min(lookahead, max_carrot_speed, max_carrot_acceleration, max_cross_track, arrival_tolerance) <= 0.0:
            raise ValueError("follower limits must be positive")
        self.lookahead = lookahead
        self.max_carrot_speed = max_carrot_speed
        self.max_carrot_acceleration = max_carrot_acceleration
        self.max_cross_track = max_cross_track
        self.arrival_tolerance = arrival_tolerance
        self.path: Optional[Polyline] = None
        self.fingerprint = ()
        self.generation = 0
        # progress is cumulative for one requested route. path_progress is the
        # coordinate on the current A* polyline and may re-anchor on a replan.
        self.progress = 0.0
        self.path_progress = 0.0
        self.commanded_displacement: Point2 = (0.0, 0.0)
        self.command_velocity: Point2 = (0.0, 0.0)

    def clear_path(self) -> None:
        self.path = None
        self.fingerprint = ()
        self.path_progress = 0.0
        self.command_velocity = (0.0, 0.0)

    def reset_route_progress(self) -> None:
        self.progress = 0.0
        if self.path is not None:
            self.path_progress = 0.0

    def set_path(self, points: Sequence[Point2], pose: Point2) -> bool:
        fingerprint = path_fingerprint(points)
        if fingerprint == self.fingerprint and self.path is not None:
            return False
        self.path = Polyline(points)
        self.fingerprint = fingerprint
        self.generation += 1
        self.path_progress = self.path.project(pose).along
        return True

    def update(self, pose: Point2, dt: float) -> FollowResult:
        if self.path is None:
            raise RuntimeError("no path")
        if not all(math.isfinite(v) for v in (*pose, dt)) or dt <= 0.0:
            raise ValueError("pose and dt must be finite, and dt positive")

        projection = self.path.project(pose)
        new_path_progress = max(self.path_progress, projection.along)
        self.progress += new_path_progress - self.path_progress
        self.path_progress = new_path_progress
        remaining = max(0.0, self.path.length - self.path_progress)
        desired = self.path.point_at(self.path_progress + self.lookahead)
        desired_offset = desired[0] - pose[0], desired[1] - pose[1]

        if projection.cross_track > self.max_cross_track:
            self.command_velocity = (0.0, 0.0)
            carrot = (pose[0] + self.commanded_displacement[0], pose[1] + self.commanded_displacement[1])
            return FollowResult(
                "CROSS_TRACK_EXCEEDED", False, desired, carrot,
                self.commanded_displacement, self.path_progress, self.progress, remaining,
                projection.cross_track, self.generation,
            )

        error = (
            desired_offset[0] - self.commanded_displacement[0],
            desired_offset[1] - self.commanded_displacement[1],
        )
        desired_velocity = _limit_norm((error[0] / dt, error[1] / dt), self.max_carrot_speed)
        velocity_change = _limit_norm(
            (desired_velocity[0] - self.command_velocity[0], desired_velocity[1] - self.command_velocity[1]),
            self.max_carrot_acceleration * dt,
        )
        velocity = _limit_norm(
            (self.command_velocity[0] + velocity_change[0], self.command_velocity[1] + velocity_change[1]),
            self.max_carrot_speed,
        )
        step = (velocity[0] * dt, velocity[1] * dt)
        if math.hypot(*step) >= math.hypot(*error):
            self.commanded_displacement = desired_offset
            self.command_velocity = (0.0, 0.0)
        else:
            self.commanded_displacement = (
                self.commanded_displacement[0] + step[0],
                self.commanded_displacement[1] + step[1],
            )
            self.command_velocity = velocity

        carrot = (pose[0] + self.commanded_displacement[0], pose[1] + self.commanded_displacement[1])
        at_goal = remaining <= self.arrival_tolerance and math.dist(pose, self.path.points[-1]) <= self.arrival_tolerance
        return FollowResult(
            "GOAL_REACHED" if at_goal else "FOLLOWING", True, desired, carrot,
            self.commanded_displacement, self.path_progress, self.progress, remaining,
            projection.cross_track, self.generation,
        )
