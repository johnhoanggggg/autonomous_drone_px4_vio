"""Node-level tests for the VFH flight logic, with no ROS running.

`StubVfh` subclasses the real node but skips `Node.__init__`, so every method
under test is the one that will fly; only the ROS-touching edges (publishers,
logger, state changes, landing) are replaced with recorders. That is what lets
`accept_click` — which delegates to `OffboardWaypoint` via `super()` — be
exercised for real rather than reimplemented in the test.
"""
import math

from px4_vio_bridge.offboard_vfh import OffboardVfh
from px4_vio_bridge.vfh2d import VfhConfig, Vfh2D
from px4_vio_bridge.vfh_obstacles import ObstacleSnapshot


class FakeLogger:
    def __init__(self):
        self.messages = []

    def _record(self, msg, **kwargs):
        self.messages.append(str(msg))

    warn = error = info = fatal = debug = _record


class FakeObstacles:
    """Stands in for ObstacleField: whatever the test says the world looks like."""

    def __init__(self, samples=(), nearest=math.inf, stale=None):
        self.samples = list(samples)
        self.nearest = nearest
        self.stale = stale
        self.memory = type(
            "Memory",
            (),
            {
                "enabled": True,
                "clear_count": 0,
                "clear": lambda memory: setattr(
                    memory, "clear_count", memory.clear_count + 1
                ),
            },
        )()

    def stale_reason(self, timeout):
        return self.stale

    def snapshot(self):
        return ObstacleSnapshot(
            samples=self.samples,
            nearest_range=self.nearest,
            nearest_bearing=0.0 if math.isfinite(self.nearest) else None,
            point_count=len(self.samples),
            kept_count=len(self.samples),
            cloud_age=0.0,
            pose_age=0.0,
        )


class StubVfh(OffboardVfh):
    def __init__(self, **overrides):
        # Deliberately does NOT call Node.__init__ / OffboardVfh.__init__.
        self.config = VfhConfig(
            sectors=72,
            min_range=0.25,
            max_range=3.0,
            min_points=3,
            tau_high=2.0,
            tau_low=1.0,
            smoothing=3,
            robot_radius=0.25,
            safety_margin=0.25,
            max_steer=math.radians(35.0),
            wide_valley=math.radians(40.0),
        )
        self.vfh = Vfh2D(self.config)
        self.obstacles = FakeObstacles()
        self.logger = FakeLogger()

        # OffboardHover / OffboardWaypoint state.
        self.dt = 0.02
        self.state = "VFH"
        self.t = 0.0
        self.auto_arm = True
        self.pos = None
        self.x0 = self.y0 = 0.0
        self.yaw0 = 0.0
        self.yaw_cmd = 0.0
        self.x_cmd = self.y_cmd = 0.0
        self.x_target = self.y_target = 0.0
        self.cmd_vx = self.cmd_vy = 0.0
        self.settled_t = 0.0
        self.idle_t = 0.0
        self.idle_timeout = 20.0
        self.arrived = False
        self.waypoints_accepted = 0
        self.waypoints_rejected = 0
        self.waypoint_yaw = None
        self.waypoint_speed = 0.25
        self.waypoint_frame = "world"
        self.accept_waypoint_yaw = False
        self.geofence_radius = 1.5
        self.arrival_tol = 0.12
        self.hover_height = 0.30
        self.reach_tol = 0.07
        self.climb_timeout = 15.0
        self.horizontal_error = 0.05
        self.max_horizontal_error = 0.35
        self.transit_horizontal_error = 0.60
        self.transit_settle_time = 1.0
        self.pre_waypoint_max_horizontal_error = 0.15

        # OffboardVfh state.
        self.lookahead = 0.60
        self.plan_period = 0.20
        self.plan_t = self.plan_period
        self.obstacle_timeout = 1.0
        self.obstacle_stale_land_time = 2.0
        self.stop_distance = 0.90
        self.abort_distance = 0.50
        self.abort_time = 0.50
        self.blocked_timeout = 10.0
        self.yaw_follows_direction = True
        self.goal_tol = 0.20
        self.commanded_yaw_rate = math.radians(15.0)
        self.startup_sweep_min = math.radians(-90.0)
        self.startup_sweep_max = math.radians(90.0)
        self.startup_sweep_enabled = True
        self.startup_sweep_rate = math.radians(15.0)
        self.startup_sweep_settle_time = 1.0
        self.startup_sweep_heading_tol = math.radians(5.0)
        self.startup_sweep_return_timeout = 5.0
        self.goal = None
        self.blocked_t = 0.0
        self.stale_t = 0.0
        self.abort_t = 0.0
        self.last_result = None
        self.last_snapshot = None
        self.last_hold_reason = "not started"
        self.holding = True
        self.previous_direction_ned = None
        self.plans = 0
        self.holds = 0
        self.sweep_offset = 0.0
        self.sweep_travelled = 0.0
        self.sweep_phase = "idle"
        self.sweep_settle_t = 0.0
        self.sweep_return_t = 0.0
        self.sweep_start_heading = None

        # Recorders.
        self.landing_reasons = []
        self.states = []
        self.setpoints = []
        self.published = 0

        self.__dict__.update(overrides)

    # --- ROS edges ---------------------------------------------------------
    def get_logger(self):
        return self.logger

    def trigger_landing(self, reason):
        self.landing_reasons.append(reason)
        self.state = "LAND"

    def set_state(self, name):
        self.states.append(name)
        self.state = name
        self.t = 0.0

    def publish_setpoint(self, z_up, yaw=None):
        self.setpoints.append((self.x_cmd, self.y_cmd, z_up, yaw))
        if yaw is not None:
            self.yaw_cmd = yaw

    def publish_target(self):
        pass

    def publish_status(self, text, force=False):
        pass

    def publish_vfh(self):
        self.published += 1

    def check_flight_position(self):
        return True


def hover_at(node, x, y, heading=0.0):
    """Put the (armed) vehicle exactly on its commanded point."""
    node.x_cmd = node.x_target = x
    node.y_cmd = node.y_target = y
    node.yaw_cmd = heading
    node.pos = type("Pos", (), {"x": x, "y": y, "z": -0.30, "heading": heading})()


def wall(bearing_deg, distance, spread_deg=20.0, points=120):
    half = math.radians(spread_deg) / 2.0
    center = math.radians(bearing_deg)
    return [
        (distance, center - half + 2.0 * half * i / max(1, points - 1))
        for i in range(points)
    ]


# --- carrot placement -------------------------------------------------------
def test_clear_room_puts_the_carrot_a_lookahead_toward_the_goal() -> None:
    node = StubVfh()
    hover_at(node, 0.0, 0.0, heading=0.0)   # facing north
    node.goal = (2.0, 0.0)                  # 2 m north

    node.plan()

    assert not node.holding
    assert math.isclose(node.x_target, node.lookahead, abs_tol=1e-6)
    assert math.isclose(node.y_target, 0.0, abs_tol=1e-6)


def test_carrot_never_overshoots_a_close_goal() -> None:
    node = StubVfh()
    hover_at(node, 0.0, 0.0, heading=0.0)
    node.goal = (0.35, 0.0)                 # closer than lookahead

    node.plan()

    assert math.isclose(math.hypot(node.x_target, node.y_target), 0.35, abs_tol=1e-6)


def test_obstacle_beyond_short_goal_does_not_freeze_carrot() -> None:
    """The flight node must pass goal distance into finite-path VFH geometry."""
    node = StubVfh()
    hover_at(node, 0.0, 0.0, heading=0.0)
    node.goal = (0.88, 0.0)
    node.obstacles = FakeObstacles(
        samples=wall(23.0, 1.37, spread_deg=5.0), nearest=1.37
    )

    node.plan()

    assert not node.holding
    assert math.isclose(node.x_target, node.lookahead, abs_tol=1e-6)
    assert math.isclose(node.y_target, 0.0, abs_tol=1e-6)


def test_goal_off_to_the_side_steers_and_turns_the_vehicle() -> None:
    node = StubVfh()
    hover_at(node, 0.0, 0.0, heading=0.0)
    node.goal = (0.0, 2.0)                  # due east: 90 deg to the right

    node.plan()

    # Steering is capped by the camera field of view, so it curves rather than
    # strafing, and the yaw setpoint follows the direction actually flown.
    assert 0.0 < node.y_target
    assert math.isclose(
        node.waypoint_yaw, node.config.max_steer, abs_tol=1e-6
    )


def test_carrot_is_clamped_into_the_geofence() -> None:
    node = StubVfh(geofence_radius=1.0)
    hover_at(node, 0.95, 0.0, heading=0.0)
    node.goal = (5.0, 0.0)

    node.plan()

    assert math.hypot(node.x_target, node.y_target) <= 1.0 + 1e-9


def test_yaw_is_left_alone_when_yaw_follows_direction_is_off() -> None:
    node = StubVfh(yaw_follows_direction=False)
    hover_at(node, 0.0, 0.0, heading=0.0)
    node.goal = (0.0, 2.0)

    node.plan()

    assert node.waypoint_yaw is None


# --- holding ----------------------------------------------------------------
def test_a_wall_in_the_way_freezes_the_carrot_on_the_commanded_point() -> None:
    node = StubVfh()
    hover_at(node, 0.4, -0.2, heading=0.0)
    node.goal = (2.0, -0.2)
    node.obstacles = FakeObstacles(samples=wall(0.0, 1.0), nearest=1.0)

    node.plan()

    assert node.holding
    assert (node.x_target, node.y_target) == (node.x_cmd, node.y_cmd)
    assert node.last_result.blocked


def test_an_obstacle_inside_stop_distance_holds_even_when_a_gap_exists() -> None:
    """Proximity outranks the planner: it is measured, not inferred."""
    node = StubVfh()
    hover_at(node, 0.0, 0.0, heading=0.0)
    node.goal = (2.0, 0.0)
    # Off to the side, so VFH itself would happily fly forward.
    node.obstacles = FakeObstacles(samples=wall(90.0, 0.6), nearest=0.6)

    node.plan()

    assert node.holding
    assert "stop_distance" in node.last_hold_reason
    assert (node.x_target, node.y_target) == (node.x_cmd, node.y_cmd)


def test_no_goal_means_hold_not_wander() -> None:
    node = StubVfh()
    hover_at(node, 0.1, 0.2, heading=0.0)

    node.plan()

    assert node.holding
    assert (node.x_target, node.y_target) == (0.1, 0.2)


def test_reaching_the_goal_freezes_the_carrot() -> None:
    node = StubVfh()
    hover_at(node, 1.0, 0.0, heading=0.0)
    node.goal = (1.05, 0.0)

    node.plan()

    assert node.holding
    assert node.last_hold_reason == "goal reached"


# --- goal intake ------------------------------------------------------------
def test_a_click_sets_the_goal_and_leaves_the_carrot_alone() -> None:
    node = StubVfh()
    hover_at(node, 0.3, 0.1, heading=0.0)
    node.previous_direction_ned = 1.0

    # Foxglove publishes ENU; x_enu=east, y_enu=north -> NED (north, east).
    node.accept_click("world", 0.5, 1.2, None)

    assert node.goal == (1.2, 0.5)
    assert (node.x_target, node.y_target) == (0.3, 0.1)
    assert node.previous_direction_ned is None
    assert node.waypoints_accepted == 1


def test_a_click_in_the_wrong_frame_is_rejected_and_no_goal_is_set() -> None:
    node = StubVfh()
    hover_at(node, 0.0, 0.0)

    node.accept_click("map", 1.0, 1.0, None)

    assert node.goal is None
    assert node.waypoints_rejected == 1


def test_a_click_outside_the_geofence_becomes_a_goal_on_the_fence() -> None:
    node = StubVfh(geofence_radius=1.5)
    hover_at(node, 0.0, 0.0)

    node.accept_click("world", 0.0, 40.0, None)

    assert math.isclose(math.hypot(*node.goal), 1.5, abs_tol=1e-6)


# --- watchdogs --------------------------------------------------------------
def test_stale_obstacle_data_holds_first_and_lands_later() -> None:
    node = StubVfh()
    hover_at(node, 0.2, 0.0)
    node.obstacles = FakeObstacles(stale="obstacle cloud stale for 1.50s")

    assert node.obstacle_watchdogs() is False
    assert node.holding
    assert node.landing_reasons == []

    node.stale_t = node.obstacle_stale_land_time
    node.obstacle_watchdogs()

    assert len(node.landing_reasons) == 1
    assert "unusable" in node.landing_reasons[0]


def test_an_obstacle_inside_abort_distance_lands_after_abort_time() -> None:
    node = StubVfh()
    hover_at(node, 0.0, 0.0)
    node.last_snapshot = FakeObstacles(nearest=0.4).snapshot()

    ticks = int(node.abort_time / node.dt)
    for _ in range(ticks - 1):
        assert node.obstacle_watchdogs() is True
    assert node.obstacle_watchdogs() is False

    assert len(node.landing_reasons) == 1
    assert "abort_distance" in node.landing_reasons[0]


def test_the_abort_timer_resets_when_the_obstacle_clears() -> None:
    node = StubVfh()
    hover_at(node, 0.0, 0.0)
    node.last_snapshot = FakeObstacles(nearest=0.4).snapshot()
    node.obstacle_watchdogs()
    assert node.abort_t > 0.0

    node.last_snapshot = FakeObstacles(nearest=2.0).snapshot()
    node.obstacle_watchdogs()

    assert node.abort_t == 0.0
    assert node.landing_reasons == []


# --- state machine ----------------------------------------------------------
def test_stable_hover_starts_the_yaw_sweep_before_vfh_navigation() -> None:
    node = StubVfh(state="CLIMB_HOLD")
    hover_at(node, 0.0, 0.0, heading=0.0)

    node.handle_flight_state()

    assert node.state == "VFH_SWEEP"
    assert node.obstacles.memory.clear_count == 1
    assert (node.x_target, node.y_target) == (0.0, 0.0)


def test_startup_sweep_holds_xy_and_first_turns_to_minus_90() -> None:
    node = StubVfh(auto_arm=False, pos=None, state="CLIMB_HOLD")
    node.begin_startup_sweep()
    node.goal = (2.0, 0.0)  # a click during the scan must not enable translation

    node.handle_flight_state()

    assert node.state == "VFH_SWEEP"
    assert node.sweep_offset < 0.0
    assert node.sweep_phase == "to_min"
    assert node.setpoints[-1][0:2] == (0.0, 0.0)
    assert node.setpoints[-1][3] < 0.0
    assert (node.x_target, node.y_target) == (0.0, 0.0)


def test_startup_sweep_runs_min_to_max_then_returns_zero_before_vfh() -> None:
    node = StubVfh(
        auto_arm=False,
        pos=None,
        state="CLIMB_HOLD",
        dt=0.10,
        startup_sweep_min=math.radians(-10.0),
        startup_sweep_max=math.radians(10.0),
        startup_sweep_rate=math.radians(10.0),
        startup_sweep_settle_time=0.20,
    )
    node.begin_startup_sweep()

    phases = []
    offsets = []
    for _ in range(60):
        phases.append(node.sweep_phase)
        node.handle_flight_state()
        offsets.append(node.sweep_offset)
        if node.state == "VFH":
            break

    assert "scan" in phases
    assert "return" in phases
    assert math.isclose(min(offsets), node.startup_sweep_min, abs_tol=1e-9)
    assert math.isclose(max(offsets), node.startup_sweep_max, abs_tol=1e-9)
    assert math.isclose(node.sweep_offset, 0.0, abs_tol=1e-9)
    assert node.state == "VFH"
    assert math.isclose(node.yaw_cmd, node.sweep_start_heading, abs_tol=1e-9)


def test_startup_sweep_pauses_when_obstacle_data_is_stale() -> None:
    node = StubVfh(auto_arm=False, pos=None, state="CLIMB_HOLD")
    node.begin_startup_sweep()
    node.obstacles.stale = "obstacle cloud stale"

    node.handle_flight_state()

    assert node.sweep_offset == 0.0
    assert node.sweep_travelled == 0.0
    assert node.state == "VFH_SWEEP"
    assert "stale" in node.last_hold_reason


def test_staying_blocked_eventually_lands() -> None:
    node = StubVfh(blocked_timeout=1.0)
    hover_at(node, 0.0, 0.0, heading=0.0)
    node.goal = (2.0, 0.0)
    node.obstacles = FakeObstacles(samples=wall(0.0, 1.0), nearest=1.0)

    for _ in range(int(1.0 / node.dt) + 2):
        node.handle_flight_state()

    assert node.landing_reasons
    assert "no way forward" in node.landing_reasons[0]


def test_a_clear_run_advances_the_setpoint_at_the_slew_rate() -> None:
    node = StubVfh()
    hover_at(node, 0.0, 0.0, heading=0.0)
    node.goal = (2.0, 0.0)

    for _ in range(50):    # 1 s at 50 Hz
        node.handle_flight_state()

    # The inherited limiter, not the planner, sets how fast the setpoint moves.
    assert math.isclose(node.x_cmd, node.waypoint_speed * 1.0, abs_tol=0.01)
    assert node.landing_reasons == []
    assert node.published > 0


def test_arriving_at_the_goal_latches_and_the_idle_timeout_lands() -> None:
    node = StubVfh(idle_timeout=0.2)
    hover_at(node, 1.0, 0.0, heading=0.0)
    node.goal = (1.0, 0.0)

    for _ in range(int(0.2 / node.dt) + 2):
        node.handle_flight_state()

    assert node.arrived
    assert node.states[-1] == "LAND"


def test_a_dry_run_uses_the_commanded_yaw_instead_of_a_measured_heading() -> None:
    node = StubVfh(auto_arm=False, pos=None, yaw_cmd=math.radians(90.0))

    assert math.isclose(node.heading_now(), math.radians(90.0))


# --- height slab ------------------------------------------------------------
def test_z_below_is_clamped_off_the_floor() -> None:
    """A slab reaching the ground makes the floor an obstacle, silently."""
    node = StubVfh(hover_height=0.30)
    node.get_parameter = lambda name: type("P", (), {"value": 0.35})()

    # 0.35 m below a 0.30 m hover is under the floor; only 0.15 m is allowed.
    assert math.isclose(node.checked_z_below(), 0.15, abs_tol=1e-9)
    assert any("clamping" in m for m in node.logger.messages)


def test_a_z_below_that_clears_the_floor_is_left_alone() -> None:
    node = StubVfh(hover_height=0.60)
    node.get_parameter = lambda name: type("P", (), {"value": 0.30})()

    assert math.isclose(node.checked_z_below(), 0.30, abs_tol=1e-9)
    assert not any("clamping" in m for m in node.logger.messages)
