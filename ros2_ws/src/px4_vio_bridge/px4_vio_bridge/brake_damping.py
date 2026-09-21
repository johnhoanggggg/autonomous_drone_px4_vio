"""Bounded velocity-error feedback for the lateral-stop experiment only."""
from dataclasses import dataclass
import math


def limit_vector(vector, limit):
    length = math.hypot(*vector)
    if length <= limit:
        return tuple(vector)
    return tuple(value * limit / length for value in vector)


@dataclass(frozen=True)
class DampingResult:
    acceleration: tuple
    raw_correction: tuple
    filtered_correction: tuple
    applied_correction: tuple
    reason: str


class BrakeDamping:
    """Keep reference tracking separate from extra damping and total clipping.

    Velocity/reference/acceleration vectors must all use the same NED frame.
    Normal phase exits decay toward zero. Invalid input and flight-state exits
    clear immediately so stale feedback cannot enter landing or a later flight.
    """

    def __init__(self, enabled=False, gain=.5, limit=.10, time_constant=.10,
                 acceleration_limit=.30):
        if not math.isfinite(gain) or gain < 0:
            raise ValueError("damping gain must be finite and nonnegative")
        if not all(math.isfinite(v) and v > 0
                   for v in (limit, time_constant, acceleration_limit)):
            raise ValueError("damping limits and time constant must be finite and positive")
        if enabled and gain > 0 and limit > acceleration_limit:
            raise ValueError("damping limit must not exceed total acceleration limit")
        self.enabled = enabled
        self.gain = gain
        self.limit = limit
        self.time_constant = time_constant
        self.acceleration_limit = acceleration_limit
        self.reset()

    def reset(self):
        self.previous_time = None
        self.correction = (0., 0.)

    def update(self, reference_velocity, reference_acceleration, measured_velocity,
               now, *, active, braking):
        reference_acceleration = tuple(reference_acceleration)

        def unchanged(reason):
            self.reset()
            return DampingResult(reference_acceleration, (0., 0.), (0., 0.),
                                 (0., 0.), reason)

        if not self.enabled or self.gain == 0:
            return unchanged("disabled" if not self.enabled else "zero_gain")
        if not active:
            return unchanged("inactive_state")
        if not all(math.isfinite(v) for v in (*reference_velocity, *reference_acceleration)):
            return unchanged("reference_unavailable")
        if measured_velocity is None or not all(math.isfinite(v) for v in measured_velocity):
            return unchanged("velocity_unavailable")
        if not math.isfinite(now):
            return unchanged("invalid_time")
        dt = 0. if self.previous_time is None else now - self.previous_time
        if self.previous_time is not None and not 0 < dt <= .15:
            return unchanged("callback_gap")
        self.previous_time = now
        raw = tuple(self.gain * (ref - measured)
                    for ref, measured in zip(reference_velocity, measured_velocity)) if braking else (0., 0.)
        target = limit_vector(raw, self.limit)
        alpha = -math.expm1(-dt / self.time_constant)
        self.correction = tuple(old + alpha * (new - old)
                                for old, new in zip(self.correction, target))
        acceleration = limit_vector(
            tuple(ref + extra for ref, extra in zip(reference_acceleration, self.correction)),
            self.acceleration_limit)
        applied = tuple(final - ref for final, ref in zip(acceleration, reference_acceleration))
        return DampingResult(acceleration, raw, self.correction, applied,
                             "braking_or_hold" if braking else "phase_release")
