"""ROS-free position-command limiting for global-planner flight."""

import math
from typing import Optional, Tuple


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
