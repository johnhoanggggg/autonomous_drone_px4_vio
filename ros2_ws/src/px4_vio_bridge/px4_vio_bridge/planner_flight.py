"""ROS-free position-command limiting for global-planner flight."""

import math
from typing import Optional, Tuple

from px4_vio_bridge.path_follower import Polyline, path_fingerprint


Point2 = Tuple[float, float]


def _limit(vector: Point2, maximum: float) -> Point2:
    magnitude = math.hypot(*vector)
    if magnitude <= maximum or magnitude <= 0.0:
        return vector
    scale = maximum / magnitude
    return vector[0] * scale, vector[1] * scale


def vio_enu_displacement_to_ned(displacement: Point2) -> Point2:
    """Convert a continuous-VIO ENU vector to PX4 NED horizontal axes."""
    x, y = (float(value) for value in displacement)
    if not (math.isfinite(x) and math.isfinite(y)):
        raise ValueError("VIO displacement must be finite")
    return y, x


def vio_displacement_to_map(displacement: Point2, correction_yaw: float) -> Point2:
    """Rotate a continuous-VIO vector into the SLAM map frame."""
    x, y = (float(value) for value in displacement)
    yaw = float(correction_yaw)
    if not all(math.isfinite(value) for value in (x, y, yaw)):
        raise ValueError("displacement and correction yaw must be finite")
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return cosine * x - sine * y, sine * x + cosine * y


def _wrap_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def ned_track_heading(displacement: Point2, min_distance: float) -> Optional[float]:
    """PX4 NED yaw that points along a horizontal NED displacement.

    Returns None when the vector is shorter than ``min_distance``. Holding, and
    the last centimetres before the goal, leave a carrot offset whose bearing is
    dominated by follower and VIO noise; steering to it would spin the airframe.
    """
    x, y = (float(value) for value in displacement)
    if not (math.isfinite(x) and math.isfinite(y)):
        raise ValueError("displacement must be finite")
    if not math.isfinite(min_distance) or min_distance <= 0.0:
        raise ValueError("min_distance must be finite and positive")
    if math.hypot(x, y) < min_distance:
        return None
    # NED yaw is measured from +x (north) toward +y (east).
    return math.atan2(y, x)


def track_yaw_target(
    current: Optional[float], heading: Optional[float], deadband: float
) -> Optional[float]:
    """Latch a new yaw target only once the path heading leaves the deadband.

    The published setpoint is slewed toward this target, so re-latching every
    sample would keep the yaw controller chasing bearing noise instead of
    settling on the leg heading.
    """
    if heading is None or not math.isfinite(heading):
        return current
    if not math.isfinite(deadband) or deadband < 0.0:
        raise ValueError("deadband must be finite and non-negative")
    if current is None or abs(_wrap_pi(heading - current)) > deadband:
        return heading
    return current


def clamp_to_disc(point: Point2, center: Point2, radius: float) -> Point2:
    if not math.isfinite(radius) or radius <= 0.0:
        raise ValueError("geofence radius must be finite and positive")
    offset = point[0] - center[0], point[1] - center[1]
    distance = math.hypot(*offset)
    if distance <= radius or distance <= 0.0:
        return point
    scale = radius / distance
    return center[0] + offset[0] * scale, center[1] + offset[1] * scale


class HorizontalCommandLimiter:
    """Speed- and acceleration-limit the final PX4 horizontal setpoint."""

    def __init__(self, max_speed: float = 0.20, max_acceleration: float = 0.40):
        if not math.isfinite(max_speed) or max_speed <= 0.0:
            raise ValueError("max_speed must be finite and positive")
        if not math.isfinite(max_acceleration) or max_acceleration <= 0.0:
            raise ValueError("max_acceleration must be finite and positive")
        self.max_speed = float(max_speed)
        self.max_acceleration = float(max_acceleration)
        self.position: Optional[Point2] = None
        self.velocity: Point2 = (0.0, 0.0)

    def reset(self, position: Point2) -> None:
        point = tuple(float(value) for value in position)
        if not all(math.isfinite(value) for value in point):
            raise ValueError("reset position must be finite")
        self.position = point
        self.velocity = (0.0, 0.0)

    def update(self, target: Point2, dt: float) -> Point2:
        target = tuple(float(value) for value in target)
        if not all(math.isfinite(value) for value in (*target, dt)) or dt <= 0.0:
            raise ValueError("target and dt must be finite, with positive dt")
        if self.position is None:
            self.reset(target)
            return self.position

        offset = target[0] - self.position[0], target[1] - self.position[1]
        distance = math.hypot(*offset)
        if distance <= 1.0e-9:
            desired_velocity = (0.0, 0.0)
        else:
            # The stopping-speed term makes the command decelerate as it closes
            # on a stationary target instead of arriving with a velocity step.
            speed = min(
                self.max_speed,
                distance / dt,
                math.sqrt(2.0 * self.max_acceleration * distance),
            )
            desired_velocity = offset[0] / distance * speed, offset[1] / distance * speed

        velocity_change = (
            desired_velocity[0] - self.velocity[0],
            desired_velocity[1] - self.velocity[1],
        )
        velocity_change = _limit(velocity_change, self.max_acceleration * dt)
        self.velocity = _limit(
            (
                self.velocity[0] + velocity_change[0],
                self.velocity[1] + velocity_change[1],
            ),
            self.max_speed,
        )
        step = self.velocity[0] * dt, self.velocity[1] * dt
        # Do not snap to a nearby target. The target is rebased from the latest
        # PX4 position and therefore moves slightly even while the airframe is
        # stationary. Snapping and zeroing velocity made the published position
        # derivative disagree with ``self.velocity`` and bypassed the
        # acceleration limit whenever that noisy target crossed the command.
        # A small, bounded overshoot is preferable: the next update reverses
        # toward the target through the same acceleration limit.
        self.position = self.position[0] + step[0], self.position[1] + step[1]
        return self.position

    def adopt(self, position: Point2, velocity: Point2 = (0.0, 0.0)) -> Point2:
        """Adopt an already-limited command without applying another free chord.

        The path limiter owns route motion.  This method keeps the common hold
        point and watchdog state synchronized with its exact output without
        moving that output away from the validated polyline.
        """
        point = tuple(float(value) for value in position)
        vector = tuple(float(value) for value in velocity)
        if not all(math.isfinite(value) for value in (*point, *vector)):
            raise ValueError("adopted position and velocity must be finite")
        if math.hypot(*vector) > self.max_speed + 1.0e-9:
            raise ValueError("adopted velocity exceeds max_speed")
        self.position = point
        self.velocity = vector
        return self.position


class PathCommandLimiter:
    """Advance the final command on a polyline or its bounded rejoin band.

    Progress is one-dimensional arc length.  Each bend is treated as a stop
    point: the command decelerates to the vertex before accelerating down the
    next segment, avoiding an instantaneous velocity-direction change there.

    A replacement path is adopted by the first of these that applies:

    1. It shares a tail with the accepted path and the command already sits in
       that tail, so progress remaps by arc length and the command does not
       move at all.  Ordinary replanning only rewrites the head, so this is the
       common case.
    2. Its projection of the last command is inside `max_projection_error`, and
       the command rejoins it continuously through that band.
    3. Its projection is inside `max_connector_error` and the caller's
       clearance check passes on the connector, so the wider rejoin is known to
       be free rather than merely short.

    Route entry is measured against `max_entry_error` instead: there the anchor
    is the vehicle, whose own cross-track is far larger than any command-to-
    command offset.
    """

    def __init__(
        self,
        max_speed: float = 0.10,
        max_acceleration: float = 0.30,
        max_projection_error: float = 0.10,
        corner_tolerance: float = 0.05,
        max_entry_error: float = 0.30,
        max_connector_error: float = 0.20,
        suffix_tolerance: float = 0.01,
    ):
        values = (
            max_speed,
            max_acceleration,
            max_projection_error,
            corner_tolerance,
            max_entry_error,
            max_connector_error,
            suffix_tolerance,
        )
        if any(not math.isfinite(value) or value <= 0.0 for value in values):
            raise ValueError("path command limits must be finite and positive")
        if max_connector_error < max_projection_error:
            raise ValueError(
                "connector tolerance must be at least the projection tolerance"
            )
        self.max_speed = float(max_speed)
        self.max_acceleration = float(max_acceleration)
        self.max_projection_error = float(max_projection_error)
        self.corner_tolerance = float(corner_tolerance)
        self.max_entry_error = float(max_entry_error)
        self.max_connector_error = float(max_connector_error)
        self.suffix_tolerance = float(suffix_tolerance)
        self.path: Optional[Polyline] = None
        self.fingerprint = ()
        self.progress = 0.0
        self.speed = 0.0
        self.position: Optional[Point2] = None
        self.velocity: Point2 = (0.0, 0.0)
        self.join_target: Optional[Point2] = None
        self.join_limit = 0.0
        self.waiting_vertex: Optional[float] = None
        self.bends = ()

    def clear(self) -> None:
        self.path = None
        self.fingerprint = ()
        self.progress = 0.0
        self.speed = 0.0
        self.position = None
        self.velocity = (0.0, 0.0)
        self.join_target = None
        self.join_limit = 0.0
        self.waiting_vertex = None
        self.bends = ()

    def snapshot(self):
        """Capture every mutable field so a rejected tick can be undone."""
        return (
            self.path,
            self.fingerprint,
            self.progress,
            self.speed,
            self.position,
            self.velocity,
            self.join_target,
            self.join_limit,
            self.waiting_vertex,
            self.bends,
        )

    def restore(self, state) -> None:
        """Reinstate a snapshot, leaving the last published command in force."""
        (
            self.path,
            self.fingerprint,
            self.progress,
            self.speed,
            self.position,
            self.velocity,
            self.join_target,
            self.join_limit,
            self.waiting_vertex,
            self.bends,
        ) = state

    @staticmethod
    def _bends_of(path: Polyline):
        """Arc lengths of the vertices the command must stop at."""
        bends = []
        for index in range(1, len(path.points) - 1):
            first = (
                path.points[index][0] - path.points[index - 1][0],
                path.points[index][1] - path.points[index - 1][1],
            )
            second = (
                path.points[index + 1][0] - path.points[index][0],
                path.points[index + 1][1] - path.points[index][1],
            )
            cross = first[0] * second[1] - first[1] * second[0]
            dot = first[0] * second[0] + first[1] * second[1]
            if abs(cross) > 1.0e-9 or dot <= 0.0:
                bends.append(path.cumulative[index])
        return tuple(bends)

    def _shared_suffix_offset(self, new_path: Polyline) -> Optional[float]:
        """Arc-length shift onto an identical tail, or None if there is none.

        Replanning normally rewrites only the head of the route, so the segment
        the command is currently on survives verbatim.  Carrying progress
        across by arc length keeps the command exactly where it is and keeps
        its speed, instead of treating an unchanged geometry as a discontinuity.
        """
        if self.path is None:
            return None
        old_points = self.path.points
        new_points = new_path.points
        shared = 0
        while (
            shared < len(old_points)
            and shared < len(new_points)
            and math.dist(old_points[-1 - shared], new_points[-1 - shared])
            <= self.suffix_tolerance
        ):
            shared += 1
        # One common endpoint is a shared point, not a shared segment.
        if shared < 2:
            return None
        old_start = self.path.cumulative[len(old_points) - shared]
        if self.progress < old_start - 1.0e-9:
            return None
        new_start = new_path.cumulative[len(new_points) - shared]
        return new_start - old_start

    def set_path(self, points, reference: Point2, *, clearance_check=None) -> bool:
        """Install a path, carrying the command onto it without a jump.

        `clearance_check(start, end)`, when given, decides whether the wider
        connector rejoin is permitted; it is the caller's occupancy test, so a
        rejoin longer than the projection band is still known to be free.
        """
        fingerprint = path_fingerprint(points)
        if self.path is not None and fingerprint == self.fingerprint:
            return False
        new_path = Polyline(points)

        if self.position is None:
            # Route entry.  The anchor is the vehicle, whose own cross-track is
            # far larger than any command-to-command offset, so this is a
            # different question from adopting a replacement.
            anchor = tuple(float(value) for value in reference)
            if not all(math.isfinite(value) for value in anchor):
                raise ValueError("path command reference must be finite")
            projection = new_path.project(anchor)
            if projection.cross_track > self.max_entry_error + 1.0e-9:
                raise ValueError(
                    "route entry is %.3fm from the path (limit %.3fm)"
                    % (projection.cross_track, self.max_entry_error)
                )
            limit = self.max_entry_error
        else:
            anchor = self.position
            offset = self._shared_suffix_offset(new_path)
            if offset is not None:
                # Identical geometry ahead: keep the command point and its speed
                # and only renumber the arc length it is measured against.
                waiting = (
                    self.waiting_vertex + offset
                    if self.waiting_vertex is not None
                    and self.waiting_vertex + offset >= -1.0e-9
                    else None
                )
                self._install(new_path, fingerprint, self.progress + offset)
                self.position = new_path.point_at(self.progress)
                self.waiting_vertex = waiting
                return True

            projection = new_path.project(anchor)
            limit = self.max_projection_error
            if projection.cross_track > limit + 1.0e-9:
                connector_ok = (
                    clearance_check is not None
                    and projection.cross_track <= self.max_connector_error + 1.0e-9
                    and clearance_check(anchor, projection.point)
                )
                if not connector_ok:
                    raise ValueError(
                        "path is %.3fm from final command (limit %.3fm, "
                        "connector limit %.3fm)"
                        % (
                            projection.cross_track,
                            self.max_projection_error,
                            self.max_connector_error,
                        )
                    )
                limit = self.max_connector_error

        self._install(new_path, fingerprint, projection.along)
        # Keep the exact previously-published point.  If the path is offset,
        # rejoin it through the bounded band; snapping directly to
        # projection.point would bypass the speed limit.
        self.position = tuple(float(value) for value in anchor)
        if projection.cross_track > 1.0e-9:
            self.join_target = projection.point
            self.join_limit = limit
            self.speed = 0.0
        else:
            self.position = projection.point
            self.speed = min(self.max_speed, math.hypot(*self.velocity))
        return True

    def _install(self, new_path: Polyline, fingerprint, progress: float) -> None:
        """Adopt `new_path` at `progress`; the caller places the command point."""
        self.path = new_path
        self.fingerprint = fingerprint
        self.progress = min(max(0.0, float(progress)), new_path.length)
        self.bends = self._bends_of(new_path)
        self.join_target = None
        self.join_limit = 0.0
        self.waiting_vertex = None

    def _update_join(self, dt: float, advance: bool) -> Point2:
        """Move continuously from an in-band command onto a replacement path."""
        old_position = self.position
        offset = (
            self.join_target[0] - self.position[0],
            self.join_target[1] - self.position[1],
        )
        distance = math.hypot(*offset)
        if not advance or distance <= 1.0e-9:
            desired_velocity = (0.0, 0.0)
        else:
            speed = min(
                self.max_speed,
                distance / dt,
                math.sqrt(2.0 * self.max_acceleration * distance),
            )
            desired_velocity = offset[0] / distance * speed, offset[1] / distance * speed
        change = _limit(
            (
                desired_velocity[0] - self.velocity[0],
                desired_velocity[1] - self.velocity[1],
            ),
            self.max_acceleration * dt,
        )
        self.velocity = _limit(
            (self.velocity[0] + change[0], self.velocity[1] + change[1]),
            self.max_speed,
        )
        self.position = (
            self.position[0] + self.velocity[0] * dt,
            self.position[1] + self.velocity[1] * dt,
        )
        projection = self.path.project(self.position)
        if projection.cross_track > self.join_limit + 1.0e-9:
            self.position = old_position
            raise ValueError("path rejoin would leave the projection tolerance band")
        self.progress = projection.along
        if (
            math.dist(self.position, self.join_target) <= 1.0e-5
            and math.hypot(*self.velocity) <= self.max_acceleration * dt + 1.0e-9
        ):
            self.position = self.join_target
            self.progress = self.path.project(self.position).along
            self.velocity = (0.0, 0.0)
            self.speed = 0.0
            self.join_target = None
        return self.position

    def _next_motion_target(self, desired_progress: float) -> float:
        """Stop at the next vertex before proceeding onto another segment."""
        for vertex in (*self.bends, self.path.length):
            if self.progress + 1.0e-9 < vertex < desired_progress - 1.0e-9:
                return vertex
        return desired_progress

    def update(
        self,
        desired_point: Point2,
        dt: float,
        *,
        advance: bool = True,
        reference_point: Optional[Point2] = None,
    ) -> Point2:
        if self.path is None or self.position is None:
            raise RuntimeError("no path command has been initialized")
        desired_point = tuple(float(value) for value in desired_point)
        if not all(math.isfinite(value) for value in (*desired_point, dt)) or dt <= 0.0:
            raise ValueError("desired point and dt must be finite, with positive dt")

        old_position = self.position
        if self.join_target is not None:
            return self._update_join(dt, advance)
        if self.waiting_vertex is not None:
            vertex = self.path.point_at(self.waiting_vertex)
            if (
                reference_point is None
                or math.dist(reference_point, vertex) > self.corner_tolerance
            ):
                self.speed = 0.0
                self.velocity = (0.0, 0.0)
                return self.position
            self.waiting_vertex = None
        if not advance:
            # A yaw-alignment pause brakes forward along the route; it cannot
            # produce a shortcut or reverse through a corner.
            next_speed = max(0.0, self.speed - self.max_acceleration * dt)
            step = next_speed * dt
            next_vertex = next(
                (
                    vertex
                    for vertex in self.path.cumulative[1:]
                    if vertex > self.progress + 1.0e-9
                ),
                self.path.length,
            )
            self.progress = min(self.progress + step, next_vertex)
            if self.progress >= next_vertex - 1.0e-9:
                next_speed = 0.0
            self.speed = next_speed
        else:
            projection = self.path.project(desired_point)
            desired_progress = max(self.progress, projection.along)
            target_progress = self._next_motion_target(desired_progress)
            remaining = max(0.0, target_progress - self.progress)
            if remaining <= 1.0e-9:
                self.speed = 0.0
            else:
                # Bound this step and the remaining braking distance.  The
                # discrete form prevents overshooting a corner at finite dt.
                braking_speed = max(
                    0.0,
                    -self.max_acceleration * dt
                    + math.sqrt(
                        (self.max_acceleration * dt) ** 2
                        + 2.0 * self.max_acceleration * remaining
                    ),
                )
                desired_speed = min(
                    self.max_speed,
                    remaining / dt,
                    braking_speed,
                )
                delta = max(
                    -self.max_acceleration * dt,
                    min(self.max_acceleration * dt, desired_speed - self.speed),
                )
                self.speed = max(0.0, min(self.max_speed, self.speed + delta))
                step = min(remaining, self.speed * dt)
                self.progress += step
                if remaining - step <= 1.0e-9:
                    self.progress = target_progress
                    self.speed = 0.0
                    if any(
                        abs(target_progress - vertex) <= 1.0e-9
                        for vertex in self.bends
                    ):
                        self.waiting_vertex = target_progress

        self.position = self.path.point_at(self.progress)
        self.velocity = (
            (self.position[0] - old_position[0]) / dt,
            (self.position[1] - old_position[1]) / dt,
        )
        return self.position
