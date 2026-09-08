"""Fixed-heading, open-area lateral braking experiment. No map/planner input."""
import json
import math

import rclpy
from px4_msgs.msg import TrajectorySetpoint
from std_msgs.msg import String

from px4_vio_bridge.offboard_hover import OffboardHover, wrap_pi


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


def lateral_xy(x0, y0, yaw0, displacement):
    return x0 - math.sin(yaw0) * displacement, y0 + math.cos(yaw0) * displacement


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
        if self.distance + self.max_horizontal_error >= self.geofence_radius:
            raise ValueError("geofence must exceed distance plus tracking-error allowance")
        if self.settle_timeout <= max(self.settle_hold, self.settle_dwell):
            raise ValueError("settle_timeout must exceed hold and dwell")
        self.profile = StopProfile(self.distance, self.speed, self.acceleration)
        budget = 4 * self.profile.duration + 5 * max(self.settle_hold, self.settle_dwell)
        if self.max_flight_time < budget + 5:
            raise ValueError("max_flight_time needs the nominal sequence plus 5 s for climb")
        self.endpoints = [self.distance, 0.0, -self.distance, 0.0]
        self.leg = -1
        self.start_offset = self.offset = self.velocity = self.acceleration_sp = 0.0
        self.phase_started = self.stable_since = self.last_update = None
        self.local_received = None
        self.phase = "INITIAL_HOLD"
        self.abort_xy = None
        self.debug_pub = self.create_publisher(String, "/lateral_stop/debug", 10)
        self.config = dict(defaults, **{k: getattr(self, k) for k in defaults})
        self.config.update(profile="cosine_velocity_ramps", endpoints=self.endpoints,
                           peak_speed=self.profile.speed, ramp_s=self.profile.ramp,
                           brake_start_s=self.profile.brake_start,
                           leg_duration_s=self.profile.duration, heading="latched_initial_NED",
                           hover_height=self.hover_height, rate_hz=self.rate_hz,
                           max_flight_time=self.max_flight_time)
        self.get_logger().warn("lateral stop test: " + json.dumps(self.config))

    @property
    def hold_point(self):
        if getattr(self, "abort_xy", None) is not None:
            return self.abort_xy
        if self.x0 is None or self.yaw0 is None:
            return self.x0, self.y0
        return lateral_xy(self.x0, self.y0, self.yaw0, getattr(self, "offset", 0.0))

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
        return (abs(-self.pos.z - self.hover_height) <= self.reach_tol and
                math.hypot(self.pos.x - x, self.pos.y - y) <= self.settle_position and
                math.hypot(self.pos.vx, self.pos.vy) <= self.settle_speed and
                abs(wrap_pi(self.pos.heading - self.yaw0)) <= math.radians(5))

    def publish_setpoint(self, z_up, yaw=None):
        # No rebase against live map/vehicle position during the experiment.
        x, y = self.hold_point
        z, vz = self.ramp_z(float(z_up))
        active = self.state in ("LATERAL_MOVE", "LATERAL_HOLD")
        v = self.velocity if active else 0.0
        a = self.acceleration_sp if active else 0.0
        m = TrajectorySetpoint()
        m.timestamp = self.now_us()
        m.position = [float(x), float(y), float(-z)]
        m.velocity = [-math.sin(self.yaw0) * v, math.cos(self.yaw0) * v, vz]
        m.acceleration = [-math.sin(self.yaw0) * a, math.cos(self.yaw0) * a, math.nan]
        m.yaw, m.yawspeed = float(self.yaw0), math.nan
        self.sp_pub.publish(m)
        if hasattr(self, "debug_pub"):
            row = dict(timestamp_us=m.timestamp, state=self.state, phase=self.phase,
                       leg=self.leg, offset=self.offset, velocity=v, acceleration=a,
                       origin=[self.x0, self.y0], yaw0=self.yaw0, config=self.config)
            if self.pos is not None:
                row.update(actual_offset=-(self.pos.x-self.x0)*math.sin(self.yaw0) +
                           (self.pos.y-self.y0)*math.cos(self.yaw0),
                           actual_velocity=-self.pos.vx*math.sin(self.yaw0) +
                           self.pos.vy*math.cos(self.yaw0))
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
        if self.settled():
            if self.stable_since is None:
                self.stable_since = now
        else:
            self.stable_since = None
        if (elapsed >= self.settle_hold and self.stable_since is not None and
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
