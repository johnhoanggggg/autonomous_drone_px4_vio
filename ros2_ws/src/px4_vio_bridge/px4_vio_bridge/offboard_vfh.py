"""PARKED experimental VFH2D flight mode; not part of the current flight stack.

NEVER ARMED. Fly `vfh_monitor` first (observation only, cannot move the
vehicle) until the histogram and chosen direction match the room, then dry-run
this node props-off, then `sides`-style incremental flights. The whole point of
splitting the two nodes is that the algorithm can be judged before it is given
authority.

Built on OffboardWaypoint, so the setpoint rate limiter, the `hold_point`
re-pointing, the settled/transit error-gate split, the VIO tracking-loss
watchdogs, K/L and `max_flight_time` all carry over unchanged. What changes is
where the target comes from: instead of the click *being* the setpoint, the
click is a **goal**, and the setpoint is a carrot placed `lookahead` metres
along the direction VFH picks each planning cycle. The vehicle therefore curves
around obstacles instead of driving through them.

Three things about this specific vehicle shape the design:

- **The camera sees ~70 deg and nothing else.** Sectors outside the field of
  view are unknown, not free, so `max_steer` (35 deg) bounds every chosen
  direction and `yaw_follows_direction` turns the vehicle so the direction it is
  travelling is the direction it can see. A goal off to the side is reached by
  curving, not by strafing blind.
- **Obstacle bearings are measured in the SLAM frame and flown in PX4's NED.**
  That is only safe while the two headings agree, which the inherited
  `max_vio_yaw_error_deg` watchdog (20 deg) already enforces by landing.
- **Reactive, with bounded world-frame memory.** Voxelised obstacles remain for
  `memory_duration` after leaving the camera view, preventing a yaw-away then
  snap-back oscillation. It still will not reverse out of a dead end; it holds
  position and lands. Do not fly it in a corridor narrower than the turning
  envelope.

Safety layers on top of the inherited ones, cheapest first:

| condition | response |
|---|---|
| `nearest < stop_distance` | freeze the carrot; hold position |
| planner blocked | freeze the carrot; hold position |
| held blocked for `blocked_timeout` | AUTO.LAND |
| obstacle data stale > `obstacle_timeout` | freeze the carrot |
| stale > `obstacle_stale_land_time` | AUTO.LAND |
| `nearest < abort_distance` for `abort_time` | AUTO.LAND |
"""
import math

import rclpy

from px4_vio_bridge.offboard_waypoint import OffboardWaypoint, swap_enu_ned_xy
from px4_vio_bridge.vfh2d import (
    Vfh2D,
    VfhConfig,
    histogram_bar,
    relative_bearing_ned,
    wrap_pi,
)
from px4_vio_bridge.vfh_obstacles import ObstacleField
from px4_vio_bridge.vfh_telemetry import AMBER, DIM, RED, VfhTelemetry


class OffboardVfh(OffboardWaypoint):
    def __init__(self, node_name="offboard_vfh"):
        super().__init__(node_name)

        self.declare_parameter("lookahead", 0.60)          # m, carrot ahead of the vehicle
        self.declare_parameter("plan_period", 0.20)        # s between VFH updates
        self.declare_parameter("cloud_topic", "/rtabmap/obstacle_cloud")
        self.declare_parameter("obstacle_pose_topic", "/rtabmap/pose")
        self.declare_parameter("obstacle_timeout", 1.0)    # s without data -> hold
        self.declare_parameter("obstacle_stale_land_time", 2.0)  # s without data -> LAND
        self.declare_parameter("stop_distance", 0.90)      # m, freeze the carrot
        self.declare_parameter("abort_distance", 0.50)     # m, LAND
        self.declare_parameter("abort_time", 0.50)         # s at abort_distance
        self.declare_parameter("blocked_timeout", 10.0)    # s with no way forward -> LAND
        self.declare_parameter("yaw_follows_direction", True)
        self.declare_parameter("goal_tol", 0.20)           # m, goal counts as reached
        # Populate world-frame obstacle memory before allowing translation:
        # original heading (0) -> -90 -> +90 -> original heading.
        self.declare_parameter("startup_sweep_min_deg", -90.0)
        self.declare_parameter("startup_sweep_max_deg", 90.0)
        self.declare_parameter("startup_sweep_rate_deg", 15.0)
        self.declare_parameter("startup_sweep_settle_time", 1.0)
        self.declare_parameter("startup_sweep_heading_tol_deg", 5.0)
        self.declare_parameter("startup_sweep_return_timeout", 5.0)

        self.declare_parameter("sectors", 72)
        self.declare_parameter("vfh_min_range", 0.25)
        self.declare_parameter("vfh_max_range", 2.0)
        self.declare_parameter("min_points", 4)
        self.declare_parameter("tau_high", 6.0)
        self.declare_parameter("tau_low", 3.0)
        self.declare_parameter("smoothing", 3)
        self.declare_parameter("robot_radius", 0.30)
        self.declare_parameter("safety_margin", 0.10)
        self.declare_parameter("max_steer_deg", 35.0)
        self.declare_parameter("display_fov_deg", 90.0)
        self.declare_parameter("wide_valley_deg", 40.0)
        self.declare_parameter("mu_target", 5.0)
        self.declare_parameter("mu_heading", 2.0)
        self.declare_parameter("mu_previous", 2.0)
        self.declare_parameter("z_below", 0.15)   # MUST stay below hover_height
        self.declare_parameter("z_above", 0.60)
        self.declare_parameter("max_samples", 1200)
        self.declare_parameter("memory_duration", 30.0)
        self.declare_parameter("memory_voxel_size", 0.10)
        self.declare_parameter("memory_max_points", 20000)
        self.declare_parameter(
            "memory_correction_topic", "/vio/map_correction_target"
        )
        self.declare_parameter("memory_reset_correction_m", 0.05)
        self.declare_parameter("memory_reset_correction_deg", 2.0)

        self.lookahead = float(self.get_parameter("lookahead").value)
        self.plan_period = float(self.get_parameter("plan_period").value)
        self.obstacle_timeout = float(self.get_parameter("obstacle_timeout").value)
        self.obstacle_stale_land_time = float(
            self.get_parameter("obstacle_stale_land_time").value
        )
        self.stop_distance = float(self.get_parameter("stop_distance").value)
        self.abort_distance = float(self.get_parameter("abort_distance").value)
        self.abort_time = float(self.get_parameter("abort_time").value)
        self.blocked_timeout = float(self.get_parameter("blocked_timeout").value)
        self.yaw_follows_direction = bool(
            self.get_parameter("yaw_follows_direction").value
        )
        self.goal_tol = float(self.get_parameter("goal_tol").value)
        self.startup_sweep_min = math.radians(
            float(self.get_parameter("startup_sweep_min_deg").value)
        )
        self.startup_sweep_max = math.radians(
            float(self.get_parameter("startup_sweep_max_deg").value)
        )
        if not (
            -math.pi <= self.startup_sweep_min <= 0.0
            and 0.0 <= self.startup_sweep_max <= math.pi
        ):
            raise ValueError(
                "startup sweep bounds must satisfy -180 <= min <= 0 <= max <= 180"
            )
        self.startup_sweep_enabled = (
            self.startup_sweep_max - self.startup_sweep_min > 1e-9
        )
        requested_sweep_rate = math.radians(
            abs(float(self.get_parameter("startup_sweep_rate_deg").value))
        )
        if self.startup_sweep_enabled and requested_sweep_rate <= 0.0:
            raise ValueError(
                "startup_sweep_rate_deg must be positive when the sweep is enabled"
            )
        # ramp_yaw is the final authority. Do not let the generated target run
        # away by more than 180 deg and make its shortest-path wrap reverse.
        self.startup_sweep_rate = requested_sweep_rate
        if self.commanded_yaw_rate > 0.0:
            self.startup_sweep_rate = min(
                self.startup_sweep_rate, self.commanded_yaw_rate
            )
        self.startup_sweep_settle_time = max(
            0.0, float(self.get_parameter("startup_sweep_settle_time").value)
        )
        self.startup_sweep_heading_tol = math.radians(
            max(0.0, float(self.get_parameter("startup_sweep_heading_tol_deg").value))
        )
        self.startup_sweep_return_timeout = max(
            0.1, float(self.get_parameter("startup_sweep_return_timeout").value)
        )
        self.display_fov = math.radians(
            max(0.0, min(180.0, float(self.get_parameter("display_fov_deg").value)))
        )

        self.config = self.build_config()
        self.vfh = Vfh2D(self.config)
        self.obstacles = ObstacleField(
            self,
            cloud_topic=str(self.get_parameter("cloud_topic").value),
            pose_topic=str(self.get_parameter("obstacle_pose_topic").value),
            min_range=self.config.min_range,
            max_range=self.config.max_range,
            z_below=self.checked_z_below(),
            z_above=float(self.get_parameter("z_above").value),
            max_samples=int(self.get_parameter("max_samples").value),
            memory_duration=float(self.get_parameter("memory_duration").value),
            memory_voxel_size=float(
                self.get_parameter("memory_voxel_size").value
            ),
            memory_max_points=int(self.get_parameter("memory_max_points").value),
            memory_correction_topic=str(
                self.get_parameter("memory_correction_topic").value
            ),
            memory_reset_correction_m=float(
                self.get_parameter("memory_reset_correction_m").value
            ),
            memory_reset_correction_deg=float(
                self.get_parameter("memory_reset_correction_deg").value
            ),
        )

        self.goal = None                # (x, y) NED — where the click wants us
        # Pre-armed so the first VFH tick plans immediately instead of holding
        # station for a plan_period before it will consider moving.
        self.plan_t = self.plan_period
        self.blocked_t = 0.0            # s with no usable direction
        self.stale_t = 0.0              # s without usable obstacle data
        self.abort_t = 0.0              # s inside abort_distance
        self.last_result = None
        self.last_snapshot = None
        self.last_hold_reason = "not started"
        self.holding = True
        self.previous_direction_ned = None   # last committed direction, absolute NED
        self.plans = 0
        self.holds = 0
        self.sweep_offset = 0.0
        self.sweep_travelled = 0.0
        self.sweep_phase = "idle"
        self.sweep_settle_t = 0.0
        self.sweep_return_t = 0.0
        self.sweep_start_heading = None

        # Identical display to vfh_monitor: markers, the sample cloud and the
        # scalar topics. Drawn in the cloud's own (SLAM) frame, so the fan lines
        # up with /rtabmap/obstacle_cloud rather than with PX4's NED origin.
        self.telemetry = VfhTelemetry(self, frame_id=self.waypoint_frame)
        self.last_target_bearing = None

        self.get_logger().warn(
            f"offboard_vfh: lookahead={self.lookahead:.2f}m "
            f"speed={self.waypoint_speed:.2f}m/s "
            f"steer=+/-{math.degrees(self.config.max_steer):.0f}deg "
            f"display=+/-{math.degrees(self.display_fov):.0f}deg "
            f"stop={self.stop_distance:.2f}m abort={self.abort_distance:.2f}m "
            f"yaw_follows={self.yaw_follows_direction} memory="
            f"{float(self.get_parameter('memory_duration').value):.0f}s "
            f"startup_sweep={math.degrees(self.startup_sweep_min):+.0f}.."
            f"{math.degrees(self.startup_sweep_max):+.0f}deg@"
            f"{math.degrees(self.startup_sweep_rate):.0f}deg/s"
        )

    def checked_z_below(self):
        """Keep the height slab off the floor, whatever the operator asked for.

        The slab is measured from the vehicle, so a z_below at or above
        hover_height admits the ground as an obstacle and the planner reports a
        wall across the whole forward arc with nothing in front of it. That is a
        silent failure — the histogram looks plausible — so it is clamped here
        rather than left to be discovered in flight.
        """
        requested = float(self.get_parameter("z_below").value)
        allowed = max(0.05, self.hover_height - 0.15)
        if requested <= allowed:
            return requested
        self.get_logger().error(
            f"z_below {requested:.2f}m reaches the floor at a "
            f"{self.hover_height:.2f}m hover and would make the ground an "
            f"obstacle; clamping to {allowed:.2f}m"
        )
        return allowed

    def build_config(self):
        def value(name, cast=float):
            return cast(self.get_parameter(name).value)

        return VfhConfig(
            sectors=value("sectors", int),
            min_range=value("vfh_min_range"),
            max_range=value("vfh_max_range"),
            min_points=value("min_points", int),
            tau_high=value("tau_high"),
            tau_low=value("tau_low"),
            smoothing=value("smoothing", int),
            robot_radius=value("robot_radius"),
            safety_margin=value("safety_margin"),
            max_steer=math.radians(value("max_steer_deg")),
            wide_valley=math.radians(value("wide_valley_deg")),
            mu_target=value("mu_target"),
            mu_heading=value("mu_heading"),
            mu_previous=value("mu_previous"),
        )

    # --- goal intake -------------------------------------------------------
    def accept_click(self, frame_id, x_enu, y_enu, yaw_enu):
        """A click is a goal here, not a setpoint.

        The inherited validation (frame, finiteness, geofence clamp, logging) is
        exactly what a goal needs, so it runs first; the accepted point is then
        moved out of `x_target` — which this node owns as the VFH carrot — and
        into `goal`.
        """
        before = self.waypoints_accepted
        super().accept_click(frame_id, x_enu, y_enu, yaw_enu)
        if self.waypoints_accepted == before:
            return   # rejected; x_target untouched
        self.goal = (self.x_target, self.y_target)
        self.x_target, self.y_target = self.x_cmd, self.y_cmd
        self.blocked_t = 0.0
        self.vfh.reset()
        self.previous_direction_ned = None
        self.get_logger().warn(
            f"VFH goal set: ned=({self.goal[0]:.2f}, {self.goal[1]:.2f}), "
            f"{self.goal_distance():.2f}m away"
        )

    def goal_distance(self):
        if self.goal is None:
            return math.inf
        x, y = (self.pos.x, self.pos.y) if (self.auto_arm and self.pos) else (
            self.x_cmd,
            self.y_cmd,
        )
        if x is None:
            return math.inf
        return math.hypot(self.goal[0] - x, self.goal[1] - y)

    def heading_now(self):
        """PX4 heading when armed, commanded yaw otherwise (dry runs have no motion)."""
        if self.auto_arm and self.pos is not None and math.isfinite(self.pos.heading):
            return float(self.pos.heading)
        return float(self.yaw_cmd if self.yaw_cmd is not None else self.yaw0)

    # --- planning ----------------------------------------------------------
    def freeze_carrot(self, reason):
        """Stop advancing: the setpoint stays where it is and PX4 holds there."""
        self.holds += 1
        self.holding = True
        self.x_target, self.y_target = self.x_cmd, self.y_cmd
        self.last_hold_reason = reason

    def plan(self):
        """One VFH cycle: place the carrot, or refuse to move."""
        snapshot = self.obstacles.snapshot()
        self.last_snapshot = snapshot
        heading = self.heading_now()

        if self.goal is None:
            self.last_target_bearing = None
            self.freeze_carrot("no goal yet")
            return

        target_bearing = relative_bearing_ned(
            heading, self.goal[0] - self.x_cmd, self.goal[1] - self.y_cmd
        )
        target_distance = math.hypot(
            self.goal[0] - self.x_cmd, self.goal[1] - self.y_cmd
        )
        self.last_target_bearing = target_bearing
        previous = None
        if self.previous_direction_ned is not None:
            previous = wrap_pi(self.previous_direction_ned - heading)

        result = self.vfh.update(
            snapshot.samples,
            target_bearing,
            previous,
            target_distance=target_distance,
        )
        self.last_result = result
        self.plans += 1

        if self.goal_distance() <= self.goal_tol:
            self.freeze_carrot("goal reached")
            return
        if result.blocked:
            self.freeze_carrot(result.reason)
            return
        if math.isfinite(snapshot.nearest_range) and (
            snapshot.nearest_range < self.stop_distance
        ):
            self.freeze_carrot(
                f"obstacle at {snapshot.nearest_range:.2f}m inside "
                f"stop_distance {self.stop_distance:.2f}m"
            )
            return

        direction_ned = wrap_pi(heading + result.direction)
        self.previous_direction_ned = direction_ned
        # Carrot from the *commanded* point, never from the measured position:
        # feeding tracking error back into the setpoint is how a rate-limited
        # setpoint starts chasing its own noise.
        reach = min(self.lookahead, max(self.goal_distance(), 0.0))
        x = self.x_cmd + reach * math.cos(direction_ned)
        y = self.y_cmd + reach * math.sin(direction_ned)
        x, y, clamped = self.clamp_to_geofence(x, y)
        self.x_target, self.y_target = x, y
        self.holding = False
        self.last_hold_reason = ""
        if clamped:
            self.get_logger().warn(
                "VFH carrot clamped to the geofence", throttle_duration_sec=2.0
            )
        if self.yaw_follows_direction:
            self.waypoint_yaw = direction_ned

    # --- safety ------------------------------------------------------------
    def obstacle_watchdogs(self):
        """Stale-data and proximity aborts. True when the planner may run."""
        stale = self.obstacles.stale_reason(self.obstacle_timeout)
        if stale is not None:
            self.stale_t += self.dt
            self.freeze_carrot(stale)
            if self.stale_t >= self.obstacle_stale_land_time:
                self.trigger_landing(
                    f"obstacle data unusable for {self.stale_t:.1f}s: {stale}"
                )
            else:
                self.get_logger().warn(stale, throttle_duration_sec=1.0)
            return False
        self.stale_t = 0.0

        nearest = (
            self.last_snapshot.nearest_range
            if self.last_snapshot is not None
            else math.inf
        )
        if math.isfinite(nearest) and nearest < self.abort_distance:
            self.abort_t += self.dt
            if self.abort_t >= self.abort_time:
                self.trigger_landing(
                    f"obstacle {nearest:.2f}m away, inside abort_distance "
                    f"{self.abort_distance:.2f}m for {self.abort_time:.2f}s"
                )
                return False
        else:
            self.abort_t = 0.0
        return True

    # --- startup scan -----------------------------------------------------
    def begin_startup_sweep(self):
        """Hold the takeoff point and start a fresh world-map scan."""
        if not self.startup_sweep_enabled:
            self.get_logger().warn(
                "stable hover reached; startup yaw sweep disabled; VFH active"
            )
            self.set_state("VFH")
            return

        self.ensure_commanded_position()
        self.x_target, self.y_target = self.x_cmd, self.y_cmd
        self.cmd_vx = self.cmd_vy = 0.0
        # This is a stationary phase, so use the normal tight horizontal hold
        # watchdog rather than the looser in-transit tracking allowance.
        self.settled_t = self.transit_settle_time
        self.sweep_offset = 0.0
        self.sweep_travelled = 0.0
        self.sweep_phase = "to_min"
        self.sweep_settle_t = 0.0
        self.sweep_return_t = 0.0
        self.sweep_start_heading = float(
            self.yaw_cmd if self.yaw_cmd is not None else self.yaw0
        )
        # Discard chair/ground-level observations collected before takeoff. A
        # fresh cloud callback repopulates memory at the verified hover height.
        if self.obstacles.memory.enabled:
            self.obstacles.memory.clear()
        self.vfh.reset()
        self.last_result = None
        self.last_snapshot = None
        self.previous_direction_ned = None
        self.freeze_carrot("startup yaw sweep 0%")
        self.plan_t = self.plan_period
        self.get_logger().warn(
            "stable hover reached; scanning in place "
            f"0 -> {math.degrees(self.startup_sweep_min):+.0f} -> "
            f"{math.degrees(self.startup_sweep_max):+.0f} -> 0 deg at "
            f"{math.degrees(self.startup_sweep_rate):.0f}deg/s before VFH motion"
        )
        self.set_state("VFH_SWEEP")

    def update_sweep_observation(self):
        """Update the displayed histogram without generating a flight carrot."""
        snapshot = self.obstacles.snapshot()
        self.last_snapshot = snapshot
        self.last_target_bearing = None
        self.last_result = self.vfh.update(snapshot.samples, target_bearing=0.0)
        self.plans += 1

    def advance_sweep_offset(self, target):
        """Move the relative yaw target toward one sweep boundary."""
        previous = self.sweep_offset
        delta = target - previous
        step = self.startup_sweep_rate * self.dt
        if abs(delta) <= step:
            self.sweep_offset = target
            reached = True
        else:
            self.sweep_offset = previous + math.copysign(step, delta)
            reached = False
        self.sweep_travelled += abs(self.sweep_offset - previous)
        return reached

    def handle_startup_sweep(self):
        """Scan min-to-max relative yaw, return to zero, then settle."""
        self.plan_t += self.dt
        if self.plan_t >= self.plan_period:
            self.plan_t = 0.0
            self.update_sweep_observation()

        if not self.obstacle_watchdogs():
            # Hold both position and the current yaw while waiting for a brief
            # cloud interruption. The normal stale timeout escalates to LAND.
            if self.state == "VFH_SWEEP":
                self.publish_setpoint(self.hover_height, self.yaw_cmd)
                self.publish_vfh()
            return True

        if self.sweep_phase == "to_min":
            if self.advance_sweep_offset(self.startup_sweep_min):
                self.sweep_phase = "scan"
        elif self.sweep_phase == "scan":
            if self.advance_sweep_offset(self.startup_sweep_max):
                self.sweep_phase = "return"
        elif self.sweep_phase == "return":
            if self.advance_sweep_offset(0.0):
                self.sweep_phase = "settle"

        yaw_target = wrap_pi(self.sweep_start_heading + self.sweep_offset)
        total_travel = (
            abs(self.startup_sweep_min)
            + (self.startup_sweep_max - self.startup_sweep_min)
            + abs(self.startup_sweep_max)
        )
        percent = 100.0 * min(1.0, self.sweep_travelled / total_travel)
        self.freeze_carrot(
            f"startup yaw sweep {self.sweep_phase} {percent:.0f}% "
            f"at {math.degrees(self.sweep_offset):+.0f}deg"
        )

        if self.sweep_phase == "settle":
            self.sweep_return_t += self.dt
            heading_error = abs(wrap_pi(self.heading_now() - self.sweep_start_heading))
            horizontal_ok = (
                self.horizontal_error is not None
                and self.horizontal_error <= self.pre_waypoint_max_horizontal_error
            )
            if heading_error <= self.startup_sweep_heading_tol and horizontal_ok:
                self.sweep_settle_t += self.dt
            else:
                self.sweep_settle_t = 0.0
            self.freeze_carrot(
                f"startup sweep settling yaw_err={math.degrees(heading_error):.1f}deg"
            )

            if self.sweep_settle_t >= self.startup_sweep_settle_time:
                self.waypoint_yaw = None
                self.previous_direction_ned = None
                self.plan_t = self.plan_period
                self.get_logger().warn(
                    "startup yaw sweep "
                    f"{math.degrees(self.startup_sweep_min):+.0f}.."
                    f"{math.degrees(self.startup_sweep_max):+.0f}deg complete; "
                    "VFH avoidance active, click a goal"
                )
                self.set_state("VFH")
                return True
            if self.sweep_return_t >= self.startup_sweep_return_timeout:
                reason = (
                    "startup yaw sweep did not settle at its starting heading "
                    f"within {self.startup_sweep_return_timeout:.1f}s"
                )
                if self.auto_arm:
                    self.trigger_landing(reason)
                else:
                    self.get_logger().warn(f"dry run: {reason}; ending sweep")
                    self.set_state("VFH")
                return True

        self.waypoint_yaw = yaw_target
        self.publish_setpoint(self.hover_height, yaw_target)
        if not self.check_flight_position():
            return True
        self.publish_vfh()
        return True

    # --- state machine -----------------------------------------------------
    def handle_flight_state(self):
        if self.state == "CLIMB_HOLD":
            self.publish_setpoint(self.hover_height, self.yaw0)
            if not self.check_flight_position():
                return True
            if self.auto_arm:
                altitude_ok = self.pos is not None and (
                    abs((-self.pos.z) - self.hover_height) <= self.reach_tol
                )
                horizontal_ok = (
                    self.horizontal_error is not None
                    and self.horizontal_error <= self.pre_waypoint_max_horizontal_error
                )
                if altitude_ok and horizontal_ok:
                    self.begin_startup_sweep()
                elif self.t > self.climb_timeout:
                    self.trigger_landing(
                        "climb timeout without a stable hover "
                        f"(altitude_ok={altitude_ok}, horizontal_error="
                        f"{float('nan') if self.horizontal_error is None else self.horizontal_error:.2f}m"
                        f"/{self.pre_waypoint_max_horizontal_error:.2f}m)"
                    )
            elif self.t > self.climb_timeout:
                self.get_logger().warn(
                    "dry run: climb timeout; starting the VFH startup sequence"
                )
                self.begin_startup_sweep()
            return True

        if self.state == "VFH_SWEEP":
            return self.handle_startup_sweep()

        if self.state != "VFH":
            return False

        self.plan_t += self.dt
        if self.obstacle_watchdogs() and self.plan_t >= self.plan_period:
            self.plan_t = 0.0
            self.plan()

        self.slew_commanded_position()
        self.publish_setpoint(self.hover_height, self.current_yaw_target())
        if not self.check_flight_position():
            return True

        blocked = self.last_result is None or self.last_result.blocked
        if blocked and self.goal is not None and self.goal_distance() > self.goal_tol:
            self.blocked_t += self.dt
            if self.blocked_t >= self.blocked_timeout:
                self.trigger_landing(
                    f"no way forward for {self.blocked_timeout:.0f}s "
                    f"({'' if self.last_result is None else self.last_result.reason})"
                )
                return True
        else:
            self.blocked_t = 0.0

        # Goal arrival reuses the inherited latch and idle timeout, so parking
        # at the goal with nothing else clicked lands the vehicle exactly as a
        # waypoint flight does.
        if not self.arrived and self.goal is not None and (
            self.goal_distance() <= self.goal_tol
        ):
            self.arrived = True
            self.get_logger().warn(
                f"goal reached; idle timeout {self.idle_timeout:.0f}s running"
            )
        self.idle_t = self.idle_t + self.dt if self.arrived else 0.0
        if self.idle_t >= self.idle_timeout:
            self.get_logger().warn(
                f"no new goal for {self.idle_timeout:.0f}s -> LAND"
            )
            self.set_state("LAND")
            return True

        self.publish_vfh()
        return True

    # --- telemetry ---------------------------------------------------------
    def publish_vfh(self):
        self.commanded_pub.publish(
            self.enu_pose(self.x_cmd, self.y_cmd, self.current_yaw_target())
        )
        self.publish_target()

        result = self.last_result
        nearest = (
            self.last_snapshot.nearest_range
            if self.last_snapshot is not None
            else math.inf
        )
        if result is not None:
            steer_text = (
                "BLOCKED" if result.direction is None
                else f"{math.degrees(result.direction):+.0f}deg"
            )
            self.telemetry.publish(
                result,
                self.last_snapshot,
                self.obstacles.origin,
                self.obstacles.yaw_enu,
                self.config.max_range,
                label=(
                    f"VFH {steer_text}  "
                    f"nearest {'clear' if not math.isfinite(nearest) else f'{nearest:.2f}m'}"
                ),
                goal_enu=(
                    None if self.goal is None else swap_enu_ned_xy(*self.goal)
                ),
                goal_bearing=self.last_target_bearing,
                goal_distance=None if self.goal is None else self.goal_distance(),
                max_steer=self.config.max_steer,
                display_fov=self.display_fov,
                # The two rings that decide whether it holds or lands, drawn at
                # the same scale as the obstacles they are compared against.
                rings=(
                    (self.stop_distance, AMBER),
                    (self.abort_distance, RED),
                    (self.config.max_range, DIM),
                ),
            )

        if self.holding:
            steer = f"HOLD({self.last_hold_reason})"
        elif result is not None and result.direction is not None:
            steer = f"{math.degrees(result.direction):+.0f}deg"
        else:
            steer = "HOLD"
        self.publish_status(
            f"VFH steer={steer} "
            f"goal={'none' if self.goal is None else f'{self.goal_distance():.2f}m'} "
            f"nearest={'clear' if not math.isfinite(nearest) else f'{nearest:.2f}m'} "
            f"memory={0 if self.last_snapshot is None else self.last_snapshot.memory_point_count} "
            f"hold_err={float('nan') if self.horizontal_error is None else self.horizontal_error:.2f}m"
            f"/{self.horizontal_error_limit:.2f}m "
            f"blocked={self.blocked_t:.1f}/{self.blocked_timeout:.0f}s "
            f"arrived={self.arrived} idle={self.idle_t:.0f}/{self.idle_timeout:.0f}s"
        )
        if result is not None:
            self.get_logger().info(
                f"{histogram_bar(result)}  steer={steer}",
                throttle_duration_sec=1.0,
            )


def main(args=None):
    rclpy.init(args=args)
    node = OffboardVfh()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.on_shutdown()
    finally:
        node.restore_terminal()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
