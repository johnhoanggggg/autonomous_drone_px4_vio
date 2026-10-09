"""Autonomous short-hover offboard sequencer for PX4 over uXRCE-DDS.

Sequence: latch current NED x/y/yaw -> stream position setpoints (disarmed) ->
request OFFBOARD -> (optionally) ARM -> climb to hover_height -> hold hold_time
seconds -> AUTO.LAND (auto-disarms on ground detect).

Safety:
- auto_arm defaults to FALSE. With it false the node runs the ENTIRE flow but
  never sends an arm command, so you can dry-run (props off) and confirm setpoint
  streaming + the OFFBOARD request without the vehicle flying.
- Guards: requires valid local position before engaging; aborts (never arms) if
  OFFBOARD+ARM is not confirmed within engage_timeout; hard max_flight_time
  watchdog forces LAND; climb_timeout forces the hold/land timeline to proceed so
  the vehicle always reaches the LAND phase.
- When run in an interactive terminal, L requests AUTO.LAND and K immediately
  sends PX4's forced-disarm command repeatedly. K stops the motors; it does NOT
  attempt to land.
- Up/Down arrow keys raise/lower the hover height by height_step during
  CLIMB_HOLD, clamped to [min_hover_height, max_hover_height]. The change goes
  through the same climb ramp as takeoff. Plain offboard_hover only: subclasses
  that replace handle_flight_state never apply the keypresses.
- While armed, stale RTAB-Map VIO poses, stale feature-count data, or a sustained
  low tracked-feature count request AUTO.LAND.
- While armed, a sustained low battery (loaded cell voltage, PX4 remaining, or
  any PX4 battery warning) or stale battery telemetry requests AUTO.LAND. PX4's
  own COM_LOW_BAT_ACT is 0 (warn only) on this airframe and will not land.
- On shutdown while armed it commands AUTO.LAND rather than disarming in air.
"""
import math
import os
import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from px4_msgs.msg import (
    BatteryStatus,
    OffboardControlMode,
    SensorCombined,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleControlMode,
    VehicleLocalPosition,
    VehicleOdometry,
)
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int32
from px4_vio_bridge.process_singleton import ProcessSingleton


# Vehicle within this of the altitude target (with the ramp finished) ends the
# climb feedforward when climb_ff_latch is on.
CLIMB_FF_LATCH_TOL = 0.01


def wrap_pi(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(q):
    """Return NED yaw from a PX4 [w, x, y, z] quaternion."""
    w, x, y, z = q
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


class OffboardHover(Node):
    # Subclasses whose battery authority is PX4 alone opt out of the default.
    BATTERY_LAND_DEFAULT = True

    def __init__(self, node_name="offboard_hover"):
        super().__init__(node_name)
        self._flight_authority = ProcessSingleton("flight_authority", lock_directory="/tmp")

        self.declare_parameter("hover_height", 0.30)      # meters up
        self.declare_parameter("hold_time", 10.0)         # seconds at altitude
        self.declare_parameter("yaw_rate_deg", 5.0)       # deg/s slew of commanded yaw (<=0 disables)
        self.declare_parameter("yaw_feedforward", False)  # publish yawspeed matching the slew (NaN when off)
        self.declare_parameter("climb_rate", 0.25)        # m/s slew of commanded altitude (<=0 = raw step)
        self.declare_parameter("climb_leash", 0.12)       # m the ramp may lead the vehicle (<=0 disables)
        self.declare_parameter("climb_release", 0.05)     # m band where the vz feedforward tapers out
        self.declare_parameter("climb_feedforward", True) # publish velocity[2] matching the ramp (NaN when off)
        # Stop the vz feedforward once the vehicle has reached the target (within
        # CLIMB_FF_LATCH_TOL) instead of tracking the altitude error all flight.
        # 2026-09-28 (4S): left running, 0.25 m/s per 0.05 m acted as a second
        # altitude P loop (gain ~5) on noisy VIO height and pumped a ~1 Hz bob.
        self.declare_parameter("climb_ff_latch", True)
        self.declare_parameter("rate_hz", 50.0)
        self.declare_parameter("stream_time", 1.0)        # setpoint pre-stream before engage
        self.declare_parameter("engage_timeout", 5.0)     # arm+offboard must confirm within
        # 2026-07-25 flight: the PX4 takeoff ramp winds the vz integrator down
        # and thrust needs ~10 s to reach hover after arm; 8 s aborted a healthy
        # climb. Armed yaw tests LAND on this timeout, so longer is safe.
        # 2026-08-28 (ULog 209): that ~10 s was measured at 20.4 s for 0.30 m and
        # root-caused — see ramp_z. Kept at 15 s as a safety bound, not as the
        # expected climb time: with the ramp+feedforward the climb is ~2 s.
        self.declare_parameter("climb_timeout", 15.0)     # start hold clock by here regardless
        self.declare_parameter("reach_tol", 0.07)         # m, "reached" altitude band
        self.declare_parameter("max_flight_time", 40.0)   # armed watchdog -> force land
        self.declare_parameter("auto_arm", False)         # MUST be true to actually fly
        self.declare_parameter("keyboard_kill", True)     # K force-disarms (interactive TTY only)
        self.declare_parameter("keyboard_land", True)     # L requests AUTO.LAND (interactive TTY only)
        self.declare_parameter("keyboard_height", True)   # Up/Down arrows nudge hover_height (CLIMB_HOLD only)
        self.declare_parameter("height_step", 0.02)       # m per arrow press
        self.declare_parameter("min_hover_height", 0.20)  # m, arrow-key floor
        self.declare_parameter("max_hover_height", 1.50)  # m, arrow-key ceiling
        # Foxglove's built-in Teleop panel publishes Twist. This is deliberately
        # opt-in and decoded as discrete safety commands, never as vehicle
        # velocity: linear.z=-1 requests LAND and linear.z=-2 requests KILL.
        self.declare_parameter("foxglove_teleop", False)
        self.declare_parameter("foxglove_teleop_topic", "/planner/flight/teleop")
        self.declare_parameter("tracking_loss_land", True)
        self.declare_parameter("vio_pose_topic", "/rtabmap/vio_pose")
        self.declare_parameter("vio_feature_topic", "/rtabmap/vio_feature_count")
        self.declare_parameter(
            "vio_odometry_topic", "/fmu/in/vehicle_visual_odometry"
        )
        self.declare_parameter("vio_pose_timeout", 0.75)       # seconds without a VIO pose
        self.declare_parameter("vio_feature_timeout", 0.5)     # seconds without feature data
        self.declare_parameter("vio_odometry_timeout", 0.5)    # bridge output must keep reaching PX4
        self.declare_parameter("min_vio_features", 160)        # 40% of the 400-feature target
        self.declare_parameter("vio_feature_loss_time", 0.25)  # low-count persistence before LAND
        self.declare_parameter("max_vio_yaw_error_deg", 20.0)  # EV yaw vs EKF propagated heading
        self.declare_parameter("vio_yaw_error_time", 0.20)
        self.declare_parameter("max_yaw_rate_deg", 60.0)       # catch violent yaw reversals
        self.declare_parameter("yaw_rate_loss_time", 0.10)
        self.declare_parameter("max_horizontal_error", 0.35)   # distance from latched hold point
        self.declare_parameter("horizontal_error_time", 0.25)
        self.declare_parameter("tracking_arm_grace", 1.0)      # settle time after arm confirmation
        # VIO relocalization-reset detection: the pose keeps publishing but snaps
        # back to the exact origin, which staleness/feature checks miss. A reset
        # writes bit-exact 0.0; real poses are always noisy, so (0,0,0) is a
        # sufficient signature on its own.
        self.declare_parameter("vio_reset_persist", 0.2)         # s at origin before it counts as a reset
        # Low-battery LAND. PX4 COM_LOW_BAT_ACT=0 only warns, so without this a
        # long hold flies the pack flat. Any one condition held for
        # battery_land_time lands. Cell volts are measured UNDER hover load:
        # 3.50 V loaded is ~3.65 V resting at R=0.0082 ohm/cell and ~17 A (269 W hover).
        self.declare_parameter("battery_land", self.BATTERY_LAND_DEFAULT)
        self.declare_parameter("battery_topic", "/fmu/out/battery_status_v1")
        self.declare_parameter("battery_land_cell_volts", 3.50)
        self.declare_parameter("battery_land_percent", 20.0)     # PX4 `remaining`, 0..100
        self.declare_parameter("battery_land_on_px4_warning", True)  # WARNING_LOW or worse
        self.declare_parameter("battery_land_time", 3.0)         # s below threshold before LAND
        self.declare_parameter("battery_timeout", 5.0)           # PX4 publishes at ~1 Hz
        self.declare_parameter("battery_log_period", 15.0)       # armed battery log line, s

        self.hover_height = float(self.get_parameter("hover_height").value)
        self.hold_time = float(self.get_parameter("hold_time").value)
        self.keyboard_height = bool(self.get_parameter("keyboard_height").value)
        self.height_step = abs(float(self.get_parameter("height_step").value))
        # Never clamp the configured start height: widen the band to include it.
        self.min_hover_height = min(float(self.get_parameter("min_hover_height").value),
                                    self.hover_height)
        self.max_hover_height = max(float(self.get_parameter("max_hover_height").value),
                                    self.hover_height)
        self.pending_height_steps = 0  # arrow presses not yet applied (+ up)
        # Configured yaw-setpoint slew rate. Immutable after init: the measured
        # gyro rate lives in self.measured_yaw_rate and must NEVER be written
        # here (a past collision fed the gyro back into ramp_yaw -> runaway yaw).
        self.commanded_yaw_rate = math.radians(float(self.get_parameter("yaw_rate_deg").value))
        self.yaw_feedforward = bool(self.get_parameter("yaw_feedforward").value)
        # Same contract as the yaw slew: a configured command rate, never a
        # measured one. See ramp_z for why the altitude is ramped at all.
        self.commanded_climb_rate = float(self.get_parameter("climb_rate").value)
        self.climb_leash = float(self.get_parameter("climb_leash").value)
        self.climb_release = float(self.get_parameter("climb_release").value)
        self.climb_feedforward = bool(self.get_parameter("climb_feedforward").value)
        self.climb_ff_latch = bool(self.get_parameter("climb_ff_latch").value)
        self.climb_ff_done_target = None  # target whose climb feedforward has finished
        self.rate_hz = float(self.get_parameter("rate_hz").value)
        self.stream_time = float(self.get_parameter("stream_time").value)
        self.engage_timeout = float(self.get_parameter("engage_timeout").value)
        self.climb_timeout = float(self.get_parameter("climb_timeout").value)
        self.reach_tol = float(self.get_parameter("reach_tol").value)
        self.max_flight_time = float(self.get_parameter("max_flight_time").value)
        self.auto_arm = bool(self.get_parameter("auto_arm").value)
        self.keyboard_kill = bool(self.get_parameter("keyboard_kill").value)
        self.keyboard_land = bool(self.get_parameter("keyboard_land").value)
        self.foxglove_teleop = bool(
            self.get_parameter("foxglove_teleop").value
        )
        self.foxglove_teleop_topic = str(
            self.get_parameter("foxglove_teleop_topic").value
        )
        self.tracking_loss_land = bool(self.get_parameter("tracking_loss_land").value)
        self.vio_pose_timeout = float(self.get_parameter("vio_pose_timeout").value)
        self.vio_feature_timeout = float(self.get_parameter("vio_feature_timeout").value)
        self.vio_odometry_timeout = float(
            self.get_parameter("vio_odometry_timeout").value
        )
        self.min_vio_features = int(self.get_parameter("min_vio_features").value)
        self.vio_feature_loss_time = float(self.get_parameter("vio_feature_loss_time").value)
        self.max_vio_yaw_error = math.radians(
            float(self.get_parameter("max_vio_yaw_error_deg").value)
        )
        self.vio_yaw_error_time = float(
            self.get_parameter("vio_yaw_error_time").value
        )
        self.max_yaw_rate = math.radians(
            float(self.get_parameter("max_yaw_rate_deg").value)
        )
        self.yaw_rate_loss_time = float(
            self.get_parameter("yaw_rate_loss_time").value
        )
        self.max_horizontal_error = float(
            self.get_parameter("max_horizontal_error").value
        )
        self.horizontal_error_time = float(
            self.get_parameter("horizontal_error_time").value
        )
        self.tracking_arm_grace = float(self.get_parameter("tracking_arm_grace").value)
        self.vio_reset_persist = float(self.get_parameter("vio_reset_persist").value)
        self.battery_land = bool(self.get_parameter("battery_land").value)
        self.battery_topic = str(self.get_parameter("battery_topic").value)
        self.battery_land_cell_volts = float(
            self.get_parameter("battery_land_cell_volts").value
        )
        self.battery_land_percent = float(self.get_parameter("battery_land_percent").value)
        self.battery_land_on_px4_warning = bool(
            self.get_parameter("battery_land_on_px4_warning").value
        )
        self.battery_land_time = float(self.get_parameter("battery_land_time").value)
        self.battery_timeout = float(self.get_parameter("battery_timeout").value)
        self.battery_log_period = float(self.get_parameter("battery_log_period").value)

        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.ocm_pub = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", pub_qos)
        self.sp_pub = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", pub_qos)
        self.cmd_pub = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", pub_qos)

        self.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position_v1",
                                 self.on_local_position, sub_qos)
        # This PX4 build's dds_topics.yaml publishes vehicle_control_mode (not vehicle_status),
        # so arm/offboard state is confirmed from its flags.
        self.create_subscription(VehicleControlMode, "/fmu/out/vehicle_control_mode",
                                 self.on_control_mode, sub_qos)
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter("vio_pose_topic").value),
            self.on_vio_pose,
            10,
        )
        self.create_subscription(
            Int32,
            str(self.get_parameter("vio_feature_topic").value),
            self.on_vio_feature_count,
            10,
        )
        self.create_subscription(
            VehicleOdometry,
            str(self.get_parameter("vio_odometry_topic").value),
            self.on_vio_odometry,
            pub_qos,
        )
        self.create_subscription(
            SensorCombined,
            "/fmu/out/sensor_combined",
            self.on_sensor_combined,
            pub_qos,
        )
        self.create_subscription(
            BatteryStatus, self.battery_topic, self.on_battery, sub_qos
        )
        if self.foxglove_teleop:
            self.create_subscription(
                Twist,
                self.foxglove_teleop_topic,
                self.on_foxglove_teleop,
                10,
            )
            self.get_logger().warn(
                f"FOXGLOVE FLIGHT CONTROLS: {self.foxglove_teleop_topic} "
                "linear.z=-1 LAND, linear.z=-2 EMERGENCY KILL"
            )

        self.pos = None            # latest VehicleLocalPosition
        self.vcm = None            # latest VehicleControlMode
        self.x0 = self.y0 = self.yaw0 = None
        self.yaw_cmd = None        # rate-limited yaw setpoint actually published
        self.z_cmd = None          # rate-limited altitude setpoint actually published (m up)

        self.state = "WAIT_POS"
        self.t = 0.0               # seconds in current state
        self.armed_t = 0.0         # seconds since arm confirmed
        self.hold_t = 0.0
        self.reached = False
        self.last_cmd_t = 0.0
        self.stdin_fd = None
        self.stdin_fd_owned = False
        self.stdin_termios = None
        self.last_vio_pose_time = None
        self.last_vio_feature_time = None
        self.last_vio_odometry_time = None
        self.vio_feature_count = None
        self.low_features_since = None
        self.vio_at_origin_since = None  # when the pose snapped back to exact origin
        self.vio_yaw_error = None
        self.vio_yaw_error_since = None
        self.measured_yaw_rate = None   # abs gyro Z from sensor_combined; monitoring only
        self.excessive_yaw_rate_since = None
        self.horizontal_error = None
        self.horizontal_error_since = None
        self.battery = None             # latest connected BatteryStatus
        self.last_battery_time = None
        self.battery_low_since = None
        self.battery_low_detail = None
        self.battery_at_arm = None      # first BatteryStatus seen armed, for the summary
        self.last_battery_log_t = None

        self.dt = 1.0 / self.rate_hz
        self.timer = self.create_timer(self.dt, self.tick)
        self.get_logger().warn(
            f"offboard_hover: auto_arm={self.auto_arm} hover={self.hover_height}m hold={self.hold_time}s "
            f"({'ARMED FLIGHT' if self.auto_arm else 'DRY RUN - will not arm'})"
        )
        self.setup_keyboard_controls()

    # --- keyboard flight controls ----------------------------------------
    def destroy_node(self):
        try:
            return super().destroy_node()
        finally:
            self._flight_authority.close()

    def setup_keyboard_controls(self):
        if not self.keyboard_kill and not self.keyboard_land and not self.keyboard_height:
            self.get_logger().warn("keyboard kill, land and height controls disabled by parameters")
            return
        try:
            if sys.stdin.isatty():
                self.stdin_fd = sys.stdin.fileno()
            else:
                # ros2 launch does not forward its stdin to child nodes. Opening
                # the controlling terminal keeps K/L available from that shell.
                self.stdin_fd = os.open(
                    "/dev/tty", os.O_RDONLY | os.O_NONBLOCK
                )
                self.stdin_fd_owned = True
            self.stdin_termios = termios.tcgetattr(self.stdin_fd)
            tty.setcbreak(self.stdin_fd)
        except (OSError, termios.error) as exc:
            if self.stdin_fd_owned and self.stdin_fd is not None:
                os.close(self.stdin_fd)
            self.stdin_fd = None
            self.stdin_fd_owned = False
            self.stdin_termios = None
            self.get_logger().error(
                f"could not enable K/L keyboard controls: {exc}; keep the RC kill ready"
            )
            return

        self.get_logger().warn(
            "KEYBOARD CONTROLS: press L for AUTO.LAND; press K to FORCE-DISARM immediately"
        )
        if self.keyboard_height:
            self.get_logger().warn(
                f"KEYBOARD HEIGHT: Up/Down arrows change hover height by {self.height_step:.2f} m "
                f"(range {self.min_hover_height:.2f}-{self.max_hover_height:.2f} m, while hovering)"
            )

    def restore_terminal(self):
        if self.stdin_fd is not None and self.stdin_termios is not None:
            try:
                termios.tcsetattr(self.stdin_fd, termios.TCSADRAIN, self.stdin_termios)
            except (OSError, termios.error):
                pass
        if self.stdin_fd_owned and self.stdin_fd is not None:
            try:
                os.close(self.stdin_fd)
            except OSError:
                pass
        self.stdin_fd = None
        self.stdin_fd_owned = False
        self.stdin_termios = None

    def poll_keyboard_controls(self):
        if self.stdin_fd is None:
            return
        try:
            readable, _, _ = select.select([self.stdin_fd], [], [], 0.0)
            if readable:
                raw = os.read(self.stdin_fd, 64)
                keys = raw.lower()
                # K takes precedence if both keys are present in one read.
                if self.keyboard_kill and b"k" in keys:
                    self.trigger_kill()
                elif self.keyboard_land and b"l" in keys:
                    self.trigger_landing("LAND KEY PRESSED")
                elif self.keyboard_height and self.state == "CLIMB_HOLD":
                    self.pending_height_steps += self.decode_height_keys(raw)
        except OSError as exc:
            self.get_logger().error(f"keyboard control read failed: {exc}")
            self.restore_terminal()

    @staticmethod
    def decode_height_keys(raw):
        """Net Up minus Down arrow presses in one terminal read.

        Covers both the normal (ESC [ A) and application-cursor (ESC O A)
        encodings. Holding a key auto-repeats, so a read can carry several.
        """
        up = raw.count(b"\x1b[A") + raw.count(b"\x1bOA")
        down = raw.count(b"\x1b[B") + raw.count(b"\x1bOB")
        return up - down

    def apply_height_keys(self):
        """Move hover_height by the queued arrow presses, clamped to the band."""
        steps = self.pending_height_steps
        self.pending_height_steps = 0
        if steps == 0:
            return
        old = self.hover_height
        new = old + steps * self.height_step
        new = max(self.min_hover_height, min(self.max_hover_height, new))
        if abs(new - old) < 1e-9:
            self.get_logger().warn(
                f"hover height already at {'max' if steps > 0 else 'min'} {old:.2f} m")
            return
        self.hover_height = round(new, 3)
        self.get_logger().warn(f"HEIGHT KEY: hover height {old:.2f} -> {self.hover_height:.2f} m")

    @staticmethod
    def decode_foxglove_teleop(msg):
        """Decode the discrete Twist contract used by Foxglove's Teleop panel."""
        value = float(msg.linear.z)
        if not math.isfinite(value):
            return None
        if value <= -1.5:
            return "KILL"
        if value <= -0.5:
            return "LAND"
        return None

    def on_foxglove_teleop(self, msg):
        command = self.decode_foxglove_teleop(msg)
        if command == "KILL":
            # Do not let a retained/stale browser command terminate a newly
            # started dry run before it can reach the normal arming gates.
            if not self.is_armed:
                self.get_logger().warn(
                    "FOXGLOVE EMERGENCY KILL ignored because vehicle is disarmed"
                )
                return
            self.trigger_kill("FOXGLOVE EMERGENCY KILL PRESSED")
        elif command == "LAND":
            self.trigger_landing("FOXGLOVE LAND PRESSED")

    def trigger_kill(self, reason="KILL KEY PRESSED"):
        if self.state == "KILL":
            return
        self.get_logger().fatal(f"{reason} -> PX4 FORCED DISARM; MOTORS STOPPING")
        self.set_state("KILL")
        # Send an immediate burst as well as repeating in tick(), since this is a
        # BEST_EFFORT link and a single command must not be relied upon.
        for _ in range(5):
            self.force_disarm()

    # --- subscriptions -----------------------------------------------------
    def on_local_position(self, msg):
        self.pos = msg
        if self.x0 is None or self.y0 is None:
            return
        now = self.monotonic_time()
        hold_x, hold_y = self.hold_point
        self.horizontal_error = math.hypot(msg.x - hold_x, msg.y - hold_y)
        if self.horizontal_error > self.horizontal_error_limit:
            if self.horizontal_error_since is None:
                self.horizontal_error_since = now
        else:
            self.horizontal_error_since = None

    @property
    def hold_point(self):
        """(x, y) NED that the horizontal-error watchdog measures against.

        The latched takeoff point for a station-keeping flight. Subclasses that
        deliberately translate must override this with the position they are
        currently commanding, or the watchdog will land them for successfully
        doing what they were told.
        """
        return self.x0, self.y0

    @property
    def horizontal_error_limit(self):
        """Distance from hold_point that trips the watchdog, in meters."""
        return self.max_horizontal_error

    def on_control_mode(self, msg):
        self.vcm = msg

    def monotonic_time(self):
        return time.monotonic()

    def on_vio_pose(self, msg):
        now = self.monotonic_time()
        self.last_vio_pose_time = now
        # A relocalization reset writes exactly (0, 0, 0); real poses are always
        # noisy and never land on bit-exact zero, so this alone flags the reset.
        p = msg.pose.position
        if p.x == 0.0 and p.y == 0.0 and p.z == 0.0:
            if self.vio_at_origin_since is None:
                self.vio_at_origin_since = now
        else:
            self.vio_at_origin_since = None

    def on_vio_feature_count(self, msg):
        now = self.monotonic_time()
        self.last_vio_feature_time = now
        self.vio_feature_count = int(msg.data)
        if self.vio_feature_count < self.min_vio_features:
            if self.low_features_since is None:
                self.low_features_since = now
        else:
            self.low_features_since = None

    def on_vio_odometry(self, msg):
        now = self.monotonic_time()
        self.last_vio_odometry_time = now
        if self.pos is None or not math.isfinite(self.pos.heading):
            self.vio_yaw_error = None
            self.vio_yaw_error_since = None
            return
        q = tuple(float(value) for value in msg.q)
        if not all(math.isfinite(value) for value in q):
            self.vio_yaw_error = None
            self.vio_yaw_error_since = None
            return
        self.vio_yaw_error = abs(wrap_pi(self.pos.heading - yaw_from_quaternion(q)))
        if self.vio_yaw_error > self.max_vio_yaw_error:
            if self.vio_yaw_error_since is None:
                self.vio_yaw_error_since = now
        else:
            self.vio_yaw_error_since = None

    def on_sensor_combined(self, msg):
        now = self.monotonic_time()
        self.measured_yaw_rate = abs(float(msg.gyro_rad[2]))
        if self.measured_yaw_rate > self.max_yaw_rate:
            if self.excessive_yaw_rate_since is None:
                self.excessive_yaw_rate_since = now
        else:
            self.excessive_yaw_rate_since = None

    def on_battery(self, msg):
        if not msg.connected:
            return
        now = self.monotonic_time()
        self.battery = msg
        self.last_battery_time = now
        detail = self.battery_low_condition(msg)
        if detail is None:
            self.battery_low_since = None
        elif self.battery_low_since is None:
            self.battery_low_since = now
        self.battery_low_detail = detail

    def battery_low_condition(self, msg):
        """Which land threshold this sample is under, or None."""
        cells = int(msg.cell_count)
        if msg.voltage_v > 0.0 and cells > 0:
            cell_v = msg.voltage_v / cells
            if cell_v < self.battery_land_cell_volts:
                return f"cell voltage {cell_v:.2f}V < {self.battery_land_cell_volts:.2f}V"
        # PX4 reports remaining=-1 when it has no estimate.
        if 0.0 <= msg.remaining and msg.remaining * 100.0 < self.battery_land_percent:
            return f"remaining {msg.remaining * 100.0:.0f}% < {self.battery_land_percent:.0f}%"
        if (self.battery_land_on_px4_warning
                and msg.warning >= BatteryStatus.WARNING_LOW):
            return f"PX4 battery warning {int(msg.warning)}"
        return None

    def battery_land_reason(self):
        """LAND reason from battery state, only while armed."""
        if not self.battery_land or not self.is_armed:
            return None
        now = self.monotonic_time()
        if self.last_battery_time is None or now - self.last_battery_time > self.battery_timeout:
            return f"battery telemetry stale for >{self.battery_timeout:.1f}s"
        if (self.battery_low_since is not None
                and now - self.battery_low_since >= self.battery_land_time):
            return (
                f"LOW BATTERY: {self.battery_low_detail} for {self.battery_land_time:.1f}s "
                f"({self.battery_text(self.battery)})"
            )
        return None

    @staticmethod
    def battery_text(msg):
        if msg is None:
            return "no battery data"
        parts = [f"{msg.voltage_v:.2f}V"]
        if int(msg.cell_count) > 0:
            parts.append(f"({msg.voltage_v / int(msg.cell_count):.2f}V/cell)")
        if msg.current_a >= 0.0:
            parts.append(f"{msg.current_a:.1f}A")
        if msg.remaining >= 0.0:
            parts.append(f"{msg.remaining * 100.0:.0f}%")
        return " ".join(parts)

    def battery_summary(self):
        """One line comparing the pack at arm with now, for the flight log."""
        start, end = self.battery_at_arm, self.battery
        if start is None or end is None:
            return None
        text = (
            f"BATTERY SUMMARY armed {self.armed_t:.0f}s: "
            f"{self.battery_text(start)} -> {self.battery_text(end)}"
        )
        if start.discharged_mah >= 0.0 and end.discharged_mah >= 0.0:
            text += f", {end.discharged_mah - start.discharged_mah:.0f} mAh used"
        return text

    def log_battery(self):
        """Periodic armed battery line so the log alone is a discharge record."""
        if not self.is_armed or self.battery is None:
            return
        if self.battery_at_arm is None:
            self.battery_at_arm = self.battery
        now = self.monotonic_time()
        if (self.last_battery_log_t is not None
                and now - self.last_battery_log_t < self.battery_log_period):
            return
        self.last_battery_log_t = now
        used = ""
        if self.battery.discharged_mah >= 0.0 and self.battery_at_arm.discharged_mah >= 0.0:
            used = f" {self.battery.discharged_mah - self.battery_at_arm.discharged_mah:.0f}mAh"
        self.get_logger().info(
            f"battery armed={self.armed_t:.0f}s {self.battery_text(self.battery)}{used}"
        )

    @property
    def is_armed(self):
        return self.vcm is not None and self.vcm.flag_armed

    @property
    def is_offboard(self):
        return self.vcm is not None and self.vcm.flag_control_offboard_enabled

    def pos_valid(self):
        return self.pos is not None and self.pos.xy_valid and self.pos.z_valid

    # --- publishers --------------------------------------------------------
    def now_us(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    def publish_offboard_mode(self):
        m = OffboardControlMode()
        m.timestamp = self.now_us()
        m.position = True
        m.velocity = False
        m.acceleration = False
        m.attitude = False
        m.body_rate = False
        self.ocm_pub.publish(m)

    def publish_setpoint(self, z_up, yaw=None):
        target = float(self.yaw0 if yaw is None else yaw)
        yaw_sp, yawspeed = self.ramp_yaw(target)
        z_sp, vz_sp = self.ramp_z(float(z_up))
        m = TrajectorySetpoint()
        m.timestamp = self.now_us()
        m.position = [float(self.x0), float(self.y0), float(-z_sp)]  # NED, z down-negative-up
        m.velocity = [math.nan, math.nan, vz_sp]
        m.acceleration = [math.nan, math.nan, math.nan]
        m.yaw = yaw_sp
        m.yawspeed = yawspeed
        self.sp_pub.publish(m)

    def ramp_z(self, target_up):
        """Slew the published altitude toward target_up at commanded_climb_rate.

        Returns (altitude_up_m, vz_ned) ready to drop into TrajectorySetpoint;
        vz is NaN unless climb_feedforward is on. Mirrors ramp_yaw, and like it
        uses ONLY the configured rate — never a measured climb rate.

        Why the altitude is ramped at all (ULog 209, 2026-08-28): PX4's takeoff
        ramp hands the position controller a collective of 0.475 against a true
        hover of 0.554, so the vz integrator starts ~0.08 low. A hard z step
        gives the controller nothing but a position error, and MPC_Z_VEL_P_ACC=4
        converts 0.12 m/s of velocity error into only ~0.03 of collective — a
        third of the deficit. The climb then runs entirely on MPC_Z_VEL_I_ACC
        unwinding at 0.004 collective/s: 20.4 s to cover 0.30 m, against a
        commanded 48 mm/s that only ever delivered 7.8 mm/s.

        The ramp alone would not fix that — a small position error commands an
        even smaller climb. The fix is the feedforward: PX4 sums
        TrajectorySetpoint.velocity with the position-P output, so publishing
        vz puts the full commanded rate into the velocity loop and drives the
        integrator with a real error instead of a shrinking one. The ramp is
        what keeps the position setpoint honest while that happens.

        climb_leash caps how far the ramp may lead the vehicle, so a drone that
        cannot follow gets a bounded error rather than the same step delivered
        late. climb_rate <= 0 restores the raw step behavior exactly.
        """
        if self.commanded_climb_rate <= 0.0:
            self.z_cmd = target_up
            return target_up, math.nan
        if self.z_cmd is None:
            self.z_cmd = target_up
        # Advance toward the target, clamped to the remaining distance so the
        # ramp lands exactly on it and never overshoots.
        step = self.commanded_climb_rate * self.dt
        self.z_cmd += max(-step, min(step, target_up - self.z_cmd))
        # The leash is applied on every path, including the first call: it is
        # what bounds the setpoint if z_cmd was seeded straight at the target.
        if self.climb_leash > 0.0 and self.pos is not None:
            z_now = -float(self.pos.z)
            self.z_cmd = max(min(self.z_cmd, z_now + self.climb_leash),
                             z_now - self.climb_leash)
        if not self.climb_feedforward:
            return self.z_cmd, math.nan
        latch = getattr(self, "climb_ff_latch", False)
        done = getattr(self, "climb_ff_done_target", None)
        if latch and done is not None:
            if abs(done - target_up) <= 1e-6:
                # Climb finished: PX4's own altitude loop holds from here.
                return self.z_cmd, 0.0
            self.climb_ff_done_target = None  # a new target re-arms the feedforward
        # Release on the VEHICLE's remaining distance, not the ramp's. ULog
        # 03_53_16 (2026-08-28): the ramp reached the target at t=4 s while the
        # vehicle was still 0.09 m low -- inside climb_leash, so the leash never
        # clamped -- the feedforward switched off, and the last 0.09 m reverted
        # to exactly the slow position-P crawl this exists to avoid: 18 s of it.
        z_ref = -float(self.pos.z) if self.pos is not None else self.z_cmd
        remaining = target_up - z_ref
        if (latch and abs(self.z_cmd - target_up) <= 1e-9 and
                abs(remaining) <= CLIMB_FF_LATCH_TOL):
            self.climb_ff_done_target = target_up
            return self.z_cmd, 0.0
        # Taper inside the release band rather than switching off, so there is
        # no discontinuity and measurement noise near the target cannot chatter
        # the feedforward between +/- full rate.
        span = max(self.climb_release, 1e-6)
        rate = self.commanded_climb_rate * min(1.0, abs(remaining) / span)
        if rate < 1e-3:
            return self.z_cmd, 0.0
        # Deliberately the commanded rate, not d(z_cmd)/dt: while the leash is
        # holding the setpoint back the vehicle is failing to keep up, which is
        # exactly when the velocity loop needs a real error to wind against.
        # TrajectorySetpoint.velocity is NED, so climbing is negative vz.
        return self.z_cmd, -math.copysign(rate, remaining)

    def ramp_yaw(self, target):
        """Slew the published yaw toward target at the configured commanded_yaw_rate.

        Sending PX4 a bounded yaw setpoint avoids the step-yaw torque spike that
        saturates the mixer (a motor drops out) when a new heading is commanded.
        Uses ONLY self.commanded_yaw_rate — never the measured gyro rate.
        yaw_feedforward=False publishes yawspeed=NaN (PX4 derives its own rate);
        commanded_yaw_rate <= 0 restores the raw step behavior.
        """
        if self.yaw_cmd is None:
            self.yaw_cmd = target
        if self.commanded_yaw_rate <= 0.0:
            self.yaw_cmd = target
            return target, math.nan
        step = self.commanded_yaw_rate * self.dt
        delta = wrap_pi(target - self.yaw_cmd)
        if abs(delta) <= step:
            self.yaw_cmd = target
            return target, 0.0 if self.yaw_feedforward else math.nan
        self.yaw_cmd = wrap_pi(self.yaw_cmd + math.copysign(step, delta))
        if not self.yaw_feedforward:
            return self.yaw_cmd, math.nan
        return self.yaw_cmd, math.copysign(self.commanded_yaw_rate, delta)

    def send_command(self, command, p1=0.0, p2=0.0):
        m = VehicleCommand()
        m.timestamp = self.now_us()
        m.command = command
        m.param1 = float(p1)
        m.param2 = float(p2)
        m.target_system = 1
        m.target_component = 1
        m.source_system = 1
        m.source_component = 1
        m.from_external = True
        self.cmd_pub.publish(m)

    def request_offboard(self):
        # DO_SET_MODE: base_mode custom(1), custom_main_mode OFFBOARD(6)
        self.send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)

    def arm(self):
        self.send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

    def force_disarm(self):
        # PX4/MAVLink magic value 21196 bypasses the normal in-air disarm denial.
        self.send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0, 21196.0)

    def request_land(self):
        self.send_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)

    def trigger_landing(self, reason):
        if self.state in ("LAND", "KILL", "DONE"):
            return
        if not self.is_armed:
            self.get_logger().warn(f"{reason}, but vehicle is not armed; LAND ignored")
            return
        self.get_logger().error(f"{reason} -> AUTO.LAND")
        self.request_land()
        self.set_state("LAND")

    def check_flight_position(self):
        """False (and LAND) once an armed flight has lost its position estimate."""
        if self.auto_arm and not self.pos_valid():
            self.trigger_landing("lost local position in flight")
            return False
        return True

    def tracking_loss_reason(self):
        """Fault reason gated by the conditions under which we actually LAND."""
        if not self.tracking_loss_land or not self.is_armed:
            return None
        if self.armed_t < self.tracking_arm_grace:
            return None
        reason = self.vio_fault_reason()
        if reason is not None:
            return reason
        now = self.monotonic_time()
        if (
            self.excessive_yaw_rate_since is not None
            and now - self.excessive_yaw_rate_since >= self.yaw_rate_loss_time
        ):
            return (
                f"yaw rate exceeded {math.degrees(self.max_yaw_rate):.0f} deg/s "
                f"for {self.yaw_rate_loss_time:.2f}s "
                f"(latest={math.degrees(self.measured_yaw_rate):.0f} deg/s)"
            )
        if (
            self.horizontal_error_since is not None
            and now - self.horizontal_error_since >= self.horizontal_error_time
        ):
            return (
                f"horizontal hold error exceeded {self.horizontal_error_limit:.2f}m "
                f"for {self.horizontal_error_time:.2f}s "
                f"(latest={self.horizontal_error:.2f}m)"
            )
        return None

    def vio_fault_reason(self):
        """Raw VIO fault detection, independent of arm state or grace.

        Kept separate so a dry run (auto_arm=false) can observe that the checks
        fire without arming, even though LAND itself only happens when armed.
        """
        now = self.monotonic_time()
        if self.last_vio_pose_time is None or now - self.last_vio_pose_time > self.vio_pose_timeout:
            return f"RTAB-Map VIO pose stale for >{self.vio_pose_timeout:.2f}s"
        if (self.last_vio_feature_time is None
                or now - self.last_vio_feature_time > self.vio_feature_timeout):
            return f"RTAB-Map feature data stale for >{self.vio_feature_timeout:.2f}s"
        if (
            self.last_vio_odometry_time is None
            or now - self.last_vio_odometry_time > self.vio_odometry_timeout
        ):
            return f"PX4 visual odometry input stale for >{self.vio_odometry_timeout:.2f}s"
        if (self.low_features_since is not None
                and now - self.low_features_since >= self.vio_feature_loss_time):
            return (
                f"RTAB-Map tracking features stayed below {self.min_vio_features} "
                f"for {self.vio_feature_loss_time:.2f}s (latest={self.vio_feature_count})"
            )
        if (self.vio_at_origin_since is not None
                and now - self.vio_at_origin_since >= self.vio_reset_persist):
            return (
                f"RTAB-Map VIO pose reset to exact origin "
                f"for {self.vio_reset_persist:.2f}s"
            )
        if (
            self.vio_yaw_error_since is not None
            and now - self.vio_yaw_error_since >= self.vio_yaw_error_time
        ):
            return (
                f"VIO yaw disagreed with PX4 by more than "
                f"{math.degrees(self.max_vio_yaw_error):.0f}deg "
                f"for {self.vio_yaw_error_time:.2f}s "
                f"(latest={math.degrees(self.vio_yaw_error):.1f}deg)"
            )
        return None

    # --- state machine -----------------------------------------------------
    def set_state(self, name):
        self.get_logger().warn(f"-> {name}")
        self.state = name
        self.t = 0.0

    def handle_flight_state(self):
        """Run the flight-specific portion of the sequence.

        Subclasses can replace this while retaining engagement, safety watchdogs,
        landing, abort, and keyboard behavior.
        """
        if self.state != "CLIMB_HOLD":
            return False

        self.apply_height_keys()
        self.publish_setpoint(self.hover_height)
        if self.auto_arm and not self.pos_valid():
            self.get_logger().error("lost local position in flight -> LAND")
            self.set_state("LAND")
            return True
        if not self.reached and self.pos is not None:
            if abs((-self.pos.z) - self.hover_height) <= self.reach_tol:
                self.reached = True
                self.get_logger().warn("reached hover altitude, starting hold clock")
        if self.reached or self.t > self.climb_timeout:
            self.hold_t += self.dt
            if self.hold_t >= self.hold_time:
                self.set_state("LAND")
        return True

    def tick(self):
        self.t += self.dt
        self.poll_keyboard_controls()

        if self.state == "KILL":
            # Stop offboard streaming and repeat for one second for transport
            # robustness. Forced disarm is intentionally effective in the air.
            self.force_disarm()
            if self.t >= 1.0:
                self.set_state("DONE")
            return

        if self.is_armed:
            self.armed_t += self.dt

        tracking_loss = self.tracking_loss_reason()
        if tracking_loss is not None and self.state not in ("LAND", "KILL", "DONE"):
            self.trigger_landing(tracking_loss)
        elif (self.tracking_loss_land and not self.is_armed
                and self.state not in ("WAIT_POS", "STREAM", "ENGAGE",
                                       "LAND", "KILL", "DONE", "ABORT")):
            # Dry run: surface that the detector fires, without arming or landing.
            dry_fault = self.vio_fault_reason()
            if dry_fault is not None:
                self.get_logger().warn(
                    f"[dry run] tracking loss detected (disarmed, no LAND): {dry_fault}",
                    throttle_duration_sec=1.0,
                )

        battery_low = self.battery_land_reason()
        if battery_low is not None and self.state not in ("LAND", "KILL", "DONE"):
            self.trigger_landing(battery_low)
        self.log_battery()

        # global armed watchdog
        if self.is_armed and self.armed_t > self.max_flight_time and self.state not in ("LAND", "DONE"):
            self.get_logger().error("max_flight_time exceeded -> LAND")
            self.set_state("LAND")

        if self.state == "WAIT_POS":
            if self.pos_valid() and self.vcm is not None:
                self.x0 = self.pos.x
                self.y0 = self.pos.y
                self.yaw0 = self.pos.heading
                self.yaw_cmd = self.pos.heading
                self.get_logger().warn(
                    f"latched hold x={self.x0:.2f} y={self.y0:.2f} yaw={math.degrees(self.yaw0):.0f}deg")
                self.set_state("STREAM")
            elif self.t > 10.0:
                self.get_logger().error("no valid local position in 10s -> ABORT")
                self.set_state("ABORT")
            return

        # From STREAM onward, ALWAYS keep the offboard heartbeat + setpoint flowing.
        self.publish_offboard_mode()

        if self.handle_flight_state():
            return

        if self.state == "STREAM":
            self.publish_setpoint(0.0)  # hold on ground
            if not self.pos_valid():
                self.get_logger().error("lost local position validity -> ABORT")
                self.set_state("ABORT")
                return
            if self.t >= self.stream_time:
                if self.battery_land and self.auto_arm:
                    if self.battery is None:
                        self.get_logger().error(
                            f"no battery telemetry on {self.battery_topic} -> ABORT "
                            "(battery_land:=false flies without the low-battery LAND)")
                        self.set_state("ABORT")
                        return
                    if self.battery_low_detail is not None:
                        self.get_logger().error(
                            f"battery already below land threshold: {self.battery_low_detail} "
                            f"({self.battery_text(self.battery)}) -> ABORT")
                        self.set_state("ABORT")
                        return
                    self.get_logger().warn(f"battery at arm: {self.battery_text(self.battery)}")
                self.request_offboard()
                if self.auto_arm:
                    self.arm()
                self.last_cmd_t = 0.0
                self.set_state("ENGAGE")
            return

        if self.state == "ENGAGE":
            self.publish_setpoint(0.0)
            self.last_cmd_t += self.dt
            if self.last_cmd_t >= 0.5:  # resend requests periodically
                self.request_offboard()
                if self.auto_arm:
                    self.arm()
                self.last_cmd_t = 0.0
                self.get_logger().info(
                    f"ENGAGE t={self.t:.1f}s offboard={self.is_offboard} armed={self.is_armed}")
            ready = self.is_offboard and (self.is_armed or not self.auto_arm)
            if ready:
                self.set_state("CLIMB_HOLD")
            elif self.t > self.engage_timeout:
                self.get_logger().error(
                    f"engage timeout (offboard={self.is_offboard} armed={self.is_armed}) -> ABORT")
                self.set_state("ABORT")
            return

        if self.state == "LAND":
            # keep streaming a couple cycles then hand to AUTO.LAND
            if self.x0 is not None:
                self.publish_setpoint(self.hover_height)
            if self.t < 0.2:
                self.request_land()
            if not self.is_armed and self.t > 1.0:
                summary = self.battery_summary()
                if summary is not None:
                    self.get_logger().warn(summary)
                self.get_logger().warn("disarmed on ground -> DONE")
                self.set_state("DONE")
            elif self.t > 20.0:
                self.get_logger().warn("land phase timeout -> DONE")
                self.set_state("DONE")
            return

        if self.state == "ABORT":
            # never flew (or engage failed). If somehow armed, land; else just stop.
            if self.is_armed:
                self.request_land()
            if self.t > 1.0:
                self.set_state("DONE")
            return

        if self.state == "DONE":
            self.timer.cancel()
            self.get_logger().warn("sequence complete")
            rclpy.try_shutdown()
            return

    def on_shutdown(self):
        self.restore_terminal()
        if self.is_armed:
            self.get_logger().error("shutdown while armed -> commanding AUTO.LAND")
            for _ in range(5):
                self.request_land()


def main(args=None):
    rclpy.init(args=args)
    node = OffboardHover()
    if node.hold_time + node.climb_timeout > node.max_flight_time:
        node.get_logger().warn(
            f"max_flight_time={node.max_flight_time:.0f}s will LAND before "
            f"hold_time={node.hold_time:.0f}s completes; raise max_flight_time for a long hold"
        )
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
