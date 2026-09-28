"""Fixed-heading, open-area braking experiment along body-right (axis=lateral) or the nose (axis=forward). No map/planner input."""
import json
import math

import rclpy
from px4_msgs.msg import TrajectorySetpoint
from std_msgs.msg import String

from px4_vio_bridge.offboard_hover import OffboardHover, wrap_pi
from px4_vio_bridge.brake_damping import BrakeDamping


class StopProfile:
    """Analytic position/velocity/acceleration with cosine velocity ramps.

    Acceleration is continuous at rest/cruise boundaries. Short legs reduce
    peak velocity so acceleration and distance remain bounded.
    """
    def __init__(self, distance, speed, acceleration):
        if not all(math.isfinite(x) and x > 0 for x in (distance, speed, acceleration)):
            raise ValueError("distance, speed and acceleration must be positive and finite")
        self.distance = distance
        self.speed = min(speed, math.sqrt(2 * acceleration * distance / math.pi))
        self.ramp = math.pi * self.speed / (2 * acceleration)
        self.cruise = max(0.0, distance / self.speed - self.ramp)
        self.brake_start = self.ramp + self.cruise
        self.duration = 2 * self.ramp + self.cruise

    def sample(self, time):
        if time <= 0:
            return 0.0, 0.0, 0.0, "ACCEL"
        if time >= self.duration:
            return self.distance, 0.0, 0.0, "HOLD"
        if time < self.ramp:
            w = math.pi * time / self.ramp
            return (self.speed / 2 * (time - self.ramp / math.pi * math.sin(w)),
                    self.speed / 2 * (1 - math.cos(w)),
                    self.speed * math.pi / (2 * self.ramp) * math.sin(w), "ACCEL")
        if time < self.brake_start:
            return self.speed * (time - self.ramp / 2), self.speed, 0.0, "CRUISE"
        # Time reversal of acceleration, preserving exact endpoint distance.
        remaining = self.duration - time
        w = math.pi * remaining / self.ramp
        p = self.speed / 2 * (remaining - self.ramp / math.pi * math.sin(w))
        v = self.speed / 2 * (1 - math.cos(w))
        a = self.speed * math.pi / (2 * self.ramp) * math.sin(w)
        return self.distance - p, v, -a, "BRAKE"


AXES = ("lateral", "forward")


def axis_unit(yaw0, axis="lateral"):
    """NED unit vector of the test axis: body-right ("lateral") or nose ("forward")."""
    if axis == "forward":
        return math.cos(yaw0), math.sin(yaw0)
    if axis == "lateral":
        return -math.sin(yaw0), math.cos(yaw0)
    raise ValueError(f"axis must be one of {AXES}")


def axis_xy(x0, y0, yaw0, displacement, axis="lateral"):
    ux, uy = axis_unit(yaw0, axis)
    return x0 + ux * displacement, y0 + uy * displacement


def lateral_xy(x0, y0, yaw0, displacement):
    return axis_xy(x0, y0, yaw0, displacement, "lateral")


class OffboardLateralStop(OffboardHover):
    def __init__(self):
        super().__init__("offboard_lateral_stop")
        defaults = dict(distance=0.35, speed=0.20, acceleration=0.30,
                        settle_hold=3.0, settle_dwell=1.0, settle_timeout=10.0,
                        settle_position=0.10, settle_speed=0.06, geofence_radius=1.0)
        for key, value in defaults.items():
            self.declare_parameter(key, value)
            value = float(self.get_parameter(key).value)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{key} must be positive and finite")
            setattr(self, key, value)
        # "lateral" moves along the latched heading's body-right axis (roll does
        # the braking); "forward" moves along the nose axis (pitch brakes).
        self.declare_parameter("axis", "lateral")
        self.axis = str(self.get_parameter("axis").value)
        if self.axis not in AXES:
            raise ValueError(f"axis must be one of {AXES}")
        # Post-climb altitude gate: the first leg waits until the climb has
        # really settled (the 4S takeoff creeps the last few cm while the hover
        # thrust estimate catches up). Inter-leg holds keep reach_tol.
        # climb_settle_hold: seconds at height (counted from arrival) before leg 0,
        # so PX4's velocity integrator trims out the lean before the test starts.
        # climb_settle_position/speed: leg 0 also waits until the drone actually
        # HOLDS its point, i.e. PX4 has finished learning the in-air tilt it needs
        # (the drone doesn't rest level, so a ground level check can't do this).
        for key, value in dict(settle_altitude=0.02, settle_vz=0.05, climb_settle_hold=4.0,
                               climb_settle_position=0.03, climb_settle_speed=0.04).items():
            self.declare_parameter(key, value)
            value = float(self.get_parameter(key).value)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{key} must be positive and finite")
            setattr(self, key, value)
        self.declare_parameter("brake_damping", False)
        damping = dict(damping_gain=.5, damping_limit=.10, damping_time_constant=.10)
        for key, value in damping.items():
            self.declare_parameter(key, value)
            damping[key] = float(self.get_parameter(key).value)
        self.brake_damper = BrakeDamping(
            enabled=bool(self.get_parameter("brake_damping").value),
            gain=damping["damping_gain"], limit=damping["damping_limit"],
            time_constant=damping["damping_time_constant"],
            acceleration_limit=self.acceleration)
        if self.distance + self.max_horizontal_error >= self.geofence_radius:
            raise ValueError("geofence must exceed distance plus tracking-error allowance")
        if self.settle_timeout <= max(self.settle_hold, self.settle_dwell):
            raise ValueError("settle_timeout must exceed hold and dwell")
        self.profile = StopProfile(self.distance, self.speed, self.acceleration)
        budget = (4 * self.profile.duration + 4 * max(self.settle_hold, self.settle_dwell)
                  + max(self.climb_settle_hold, self.settle_dwell))
        if self.max_flight_time < budget + 5:
            raise ValueError("max_flight_time needs the nominal sequence plus 5 s for climb")
        self.endpoints = [self.distance, 0.0, -self.distance, 0.0]
        self.leg = -1
        self.start_offset = self.offset = self.velocity = self.acceleration_sp = 0.0
        self.phase_started = self.stable_since = self.last_update = None
        self.climb_reached_at = None
        self.local_received = None
        self.phase = "INITIAL_HOLD"
        self.abort_xy = None
        self.debug_pub = self.create_publisher(String, "/lateral_stop/debug", 10)
        self.config = dict(defaults, **{k: getattr(self, k) for k in defaults})
        self.config.update(damping, brake_damping=self.brake_damper.enabled,
                           damping_velocity_timeout=.10)
        self.config.update(profile="cosine_velocity_ramps", endpoints=self.endpoints,
                           peak_speed=self.profile.speed, ramp_s=self.profile.ramp,
                           brake_start_s=self.profile.brake_start,
                           leg_duration_s=self.profile.duration, heading="latched_initial_NED",
                           axis=self.axis, settle_altitude=self.settle_altitude,
                           settle_vz=self.settle_vz, climb_settle_hold=self.climb_settle_hold,
                           climb_settle_position=self.climb_settle_position,
                           climb_settle_speed=self.climb_settle_speed,
                           hover_height=self.hover_height, rate_hz=self.rate_hz,
                           max_flight_time=self.max_flight_time)
        self.get_logger().warn("lateral stop test: " + json.dumps(self.config))

    @property
    def hold_point(self):
        if getattr(self, "abort_xy", None) is not None:
            return self.abort_xy
        if self.x0 is None or self.yaw0 is None:
            return self.x0, self.y0
        return axis_xy(self.x0, self.y0, self.yaw0, getattr(self, "offset", 0.0),
                       getattr(self, "axis", "lateral"))

    def on_local_position(self, msg):
        super().on_local_position(msg)
        self.local_received = self.monotonic_time()

    def enter_phase(self, state):
        self.set_state(state)
        self.phase_started = self.monotonic_time()
        self.stable_since = None

    def fail(self, reason):
        if self.pos_valid():
            self.abort_xy = (float(self.pos.x), float(self.pos.y))
        self.velocity = self.acceleration_sp = 0.0
        self.brake_damper.reset()
        if self.is_armed:
            self.trigger_landing(reason)
        else:
            self.get_logger().error(reason)
            self.set_state("ABORT")

    def settled(self):
        if not self.auto_arm:
            return True
        if self.pos is None or not self.pos.v_xy_valid:
            return False
        x, y = self.hold_point
        altitude_tol = self.reach_tol
        position_tol, speed_tol = self.settle_position, self.settle_speed
        if self.state == "CLIMB_HOLD":
            altitude_tol = min(self.reach_tol, getattr(self, "settle_altitude", self.reach_tol))
            vz = getattr(self.pos, "vz", 0.0)
            if not math.isfinite(vz) or abs(vz) > getattr(self, "settle_vz", math.inf):
                return False
            position_tol = min(position_tol, getattr(self, "climb_settle_position", position_tol))
            speed_tol = min(speed_tol, getattr(self, "climb_settle_speed", speed_tol))
        return (abs(-self.pos.z - self.hover_height) <= altitude_tol and
                math.hypot(self.pos.x - x, self.pos.y - y) <= position_tol and
                math.hypot(self.pos.vx, self.pos.vy) <= speed_tol and
                abs(wrap_pi(self.pos.heading - self.yaw0)) <= math.radians(5))

    def publish_setpoint(self, z_up, yaw=None):
        # No rebase against live map/vehicle position during the experiment.
        x, y = self.hold_point
        z, vz = self.ramp_z(float(z_up))
        active = self.state in ("LATERAL_MOVE", "LATERAL_HOLD")
        v = self.velocity if active else 0.0
        a = self.acceleration_sp if active else 0.0
        ux, uy = axis_unit(self.yaw0, getattr(self, "axis", "lateral"))
        reference_velocity = (ux * v, uy * v)
        reference_acceleration = (ux * a, uy * a)
        now = self.monotonic_time()
        age = None if self.local_received is None else now - self.local_received
        measured = None
        if (age is not None and 0 <= age <= .10 and self.pos is not None and
                self.pos.v_xy_valid and all(math.isfinite(x) for x in (self.pos.vx, self.pos.vy))):
            measured = (float(self.pos.vx), float(self.pos.vy))
        damping = self.brake_damper.update(
            reference_velocity, reference_acceleration, measured, now,
            active=active, braking=self.phase in ("BRAKE", "HOLD"))
        m = TrajectorySetpoint()
        m.timestamp = self.now_us()
        m.position = [float(x), float(y), float(-z)]
        m.velocity = [*reference_velocity, vz]
        m.acceleration = [*damping.acceleration, math.nan]
        m.yaw, m.yawspeed = float(self.yaw0), math.nan
        self.sp_pub.publish(m)
        if hasattr(self, "debug_pub"):
            row = dict(timestamp_us=m.timestamp, state=self.state, phase=self.phase,
                       leg=self.leg, offset=self.offset, velocity=v, acceleration=a,
                       origin=[self.x0, self.y0], yaw0=self.yaw0, config=self.config)
            row["damping"] = dict(
                reason=damping.reason, velocity_age_s=age,
                measured_velocity_ned=measured,
                reference_acceleration_ned=reference_acceleration,
                raw_correction_ned=damping.raw_correction,
                filtered_correction_ned=damping.filtered_correction,
                applied_correction_ned=damping.applied_correction,
                published_acceleration_ned=[float(value) for value in m.acceleration[:2]])
            if self.pos is not None:
                row.update(actual_offset=(self.pos.x-self.x0)*ux + (self.pos.y-self.y0)*uy,
                           actual_velocity=self.pos.vx*ux + self.pos.vy*uy)
            message = String()
            message.data = json.dumps(row)
            self.debug_pub.publish(message)

    def handle_flight_state(self):
        if self.state not in ("CLIMB_HOLD", "LATERAL_MOVE", "LATERAL_HOLD"):
            return False
        now = self.monotonic_time()
        if self.phase_started is None:
            self.phase_started = now
        if self.auto_arm:
            if (not self.pos_valid() or not self.pos.v_xy_valid or
                    not all(math.isfinite(v) for v in
                            (self.pos.x, self.pos.y, self.pos.z, self.pos.vx,
                             self.pos.vy, self.pos.heading)) or self.local_received is None or
                    now - self.local_received > 0.25):
                self.fail("local position/velocity invalid or stale for lateral test")
                return True
            if math.hypot(self.pos.x-self.x0, self.pos.y-self.y0) > self.geofence_radius:
                self.fail("lateral test geofence exceeded")
                return True
            if self.last_update is not None and now - self.last_update > 0.15:
                self.fail("lateral test control callback gap exceeded 150 ms")
                return True
        self.last_update = now
        elapsed = now - self.phase_started
        if self.state == "LATERAL_MOVE":
            p, v, a, self.phase = self.profile.sample(elapsed)
            sign = 1 if self.endpoints[self.leg] > self.start_offset else -1
            self.offset = self.start_offset + sign * p
            self.velocity, self.acceleration_sp = sign * v, sign * a
            self.publish_setpoint(self.hover_height)
            if self.phase == "HOLD":
                self.enter_phase("LATERAL_HOLD")
            return True
        self.velocity = self.acceleration_sp = 0.0
        self.publish_setpoint(self.hover_height)
        hold_needed, hold_elapsed = self.settle_hold, elapsed
        if self.state == "CLIMB_HOLD":
            # Time the post-climb hold from ARRIVAL at height, not from the start
            # of the climb (2026-09-28: the ~3 s climb used up settle_hold, so leg 0
            # began ~1 s after arrival with PX4's horizontal trim still settling).
            if getattr(self, "climb_reached_at", None) is None and self.pos is not None:
                tol = getattr(self, "settle_altitude", self.reach_tol)
                if abs(-self.pos.z - self.hover_height) <= tol:
                    self.climb_reached_at = now
            reached = getattr(self, "climb_reached_at", None)
            hold_needed = getattr(self, "climb_settle_hold", self.settle_hold)
            hold_elapsed = now - reached if reached is not None else -1.0
        if self.settled():
            if self.stable_since is None:
                self.stable_since = now
        else:
            self.stable_since = None
        if (hold_elapsed >= hold_needed and self.stable_since is not None and
                now - self.stable_since >= self.settle_dwell):
            self.leg += 1
            if self.leg == len(self.endpoints):
                self.set_state("LAND")
            else:
                self.start_offset = self.offset
                self.phase = "ACCEL"
                self.enter_phase("LATERAL_MOVE")
        elif elapsed > (self.climb_timeout if self.state == "CLIMB_HOLD" else self.settle_timeout):
            self.fail("lateral test failed to settle before next leg")
        return True


def main(args=None):
    rclpy.init(args=args)
    node = OffboardLateralStop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.on_shutdown()
    finally:
        node.restore_terminal()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
