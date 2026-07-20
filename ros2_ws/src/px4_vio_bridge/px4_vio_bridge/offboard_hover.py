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
- While armed, stale RTAB-Map VIO poses, stale feature-count data, or a sustained
  low tracked-feature count request AUTO.LAND.
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
from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleControlMode,
    VehicleLocalPosition,
)
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int32


def wrap_pi(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class OffboardHover(Node):
    def __init__(self, node_name="offboard_hover"):
        super().__init__(node_name)

        self.declare_parameter("hover_height", 0.30)      # meters up
        self.declare_parameter("hold_time", 10.0)         # seconds at altitude
        self.declare_parameter("yaw_rate_deg", 20.0)      # deg/s slew of commanded yaw (<=0 disables)
        self.declare_parameter("rate_hz", 50.0)
        self.declare_parameter("stream_time", 1.0)        # setpoint pre-stream before engage
        self.declare_parameter("engage_timeout", 5.0)     # arm+offboard must confirm within
        self.declare_parameter("climb_timeout", 8.0)      # start hold clock by here regardless
        self.declare_parameter("reach_tol", 0.07)         # m, "reached" altitude band
        self.declare_parameter("max_flight_time", 40.0)   # armed watchdog -> force land
        self.declare_parameter("auto_arm", False)         # MUST be true to actually fly
        self.declare_parameter("keyboard_kill", True)     # K force-disarms (interactive TTY only)
        self.declare_parameter("keyboard_land", True)     # L requests AUTO.LAND (interactive TTY only)
        self.declare_parameter("tracking_loss_land", True)
        self.declare_parameter("vio_pose_topic", "/rtabmap/vio_pose")
        self.declare_parameter("vio_feature_topic", "/rtabmap/vio_feature_count")
        self.declare_parameter("vio_pose_timeout", 0.75)       # seconds without a VIO pose
        self.declare_parameter("vio_feature_timeout", 1.0)     # seconds without feature data
        self.declare_parameter("min_vio_features", 15)         # sustained count below this is lost
        self.declare_parameter("vio_feature_loss_time", 1.0)   # low-count persistence before LAND
        self.declare_parameter("tracking_arm_grace", 1.0)      # settle time after arm confirmation
        # VIO relocalization-reset detection: the pose keeps publishing but snaps
        # back to the exact origin, which staleness/feature checks miss. A reset
        # writes bit-exact 0.0; real poses are always noisy, so (0,0,0) is a
        # sufficient signature on its own.
        self.declare_parameter("vio_reset_persist", 0.2)         # s at origin before it counts as a reset

        self.hover_height = float(self.get_parameter("hover_height").value)
        self.hold_time = float(self.get_parameter("hold_time").value)
        self.yaw_rate = math.radians(float(self.get_parameter("yaw_rate_deg").value))
        self.rate_hz = float(self.get_parameter("rate_hz").value)
        self.stream_time = float(self.get_parameter("stream_time").value)
        self.engage_timeout = float(self.get_parameter("engage_timeout").value)
        self.climb_timeout = float(self.get_parameter("climb_timeout").value)
        self.reach_tol = float(self.get_parameter("reach_tol").value)
        self.max_flight_time = float(self.get_parameter("max_flight_time").value)
        self.auto_arm = bool(self.get_parameter("auto_arm").value)
        self.keyboard_kill = bool(self.get_parameter("keyboard_kill").value)
        self.keyboard_land = bool(self.get_parameter("keyboard_land").value)
        self.tracking_loss_land = bool(self.get_parameter("tracking_loss_land").value)
        self.vio_pose_timeout = float(self.get_parameter("vio_pose_timeout").value)
        self.vio_feature_timeout = float(self.get_parameter("vio_feature_timeout").value)
        self.min_vio_features = int(self.get_parameter("min_vio_features").value)
        self.vio_feature_loss_time = float(self.get_parameter("vio_feature_loss_time").value)
        self.tracking_arm_grace = float(self.get_parameter("tracking_arm_grace").value)
        self.vio_reset_persist = float(self.get_parameter("vio_reset_persist").value)

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

        self.pos = None            # latest VehicleLocalPosition
        self.vcm = None            # latest VehicleControlMode
        self.x0 = self.y0 = self.yaw0 = None
        self.yaw_cmd = None        # rate-limited yaw setpoint actually published

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
        self.vio_feature_count = None
        self.low_features_since = None
        self.vio_at_origin_since = None  # when the pose snapped back to exact origin

        self.dt = 1.0 / self.rate_hz
        self.timer = self.create_timer(self.dt, self.tick)
        self.get_logger().warn(
            f"offboard_hover: auto_arm={self.auto_arm} hover={self.hover_height}m hold={self.hold_time}s "
            f"({'ARMED FLIGHT' if self.auto_arm else 'DRY RUN - will not arm'})"
        )
        self.setup_keyboard_controls()

    # --- keyboard flight controls ----------------------------------------
    def setup_keyboard_controls(self):
        if not self.keyboard_kill and not self.keyboard_land:
            self.get_logger().warn("keyboard kill and land controls disabled by parameters")
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
                keys = os.read(self.stdin_fd, 64).lower()
                # K takes precedence if both keys are present in one read.
                if self.keyboard_kill and b"k" in keys:
                    self.trigger_kill()
                elif self.keyboard_land and b"l" in keys:
                    self.trigger_landing("LAND KEY PRESSED")
        except OSError as exc:
            self.get_logger().error(f"keyboard control read failed: {exc}")
            self.restore_terminal()

    def trigger_kill(self):
        if self.state == "KILL":
            return
        self.get_logger().fatal("KILL KEY PRESSED -> PX4 FORCED DISARM; MOTORS STOPPING")
        self.set_state("KILL")
        # Send an immediate burst as well as repeating in tick(), since this is a
        # BEST_EFFORT link and a single command must not be relied upon.
        for _ in range(5):
            self.force_disarm()

    # --- subscriptions -----------------------------------------------------
    def on_local_position(self, msg):
        self.pos = msg

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
        m = TrajectorySetpoint()
        m.timestamp = self.now_us()
        m.position = [float(self.x0), float(self.y0), float(-z_up)]  # NED, z down-negative-up
        m.velocity = [math.nan, math.nan, math.nan]
        m.acceleration = [math.nan, math.nan, math.nan]
        m.yaw = yaw_sp
        m.yawspeed = yawspeed
        self.sp_pub.publish(m)

    def ramp_yaw(self, target):
        """Slew the published yaw toward target at yaw_rate, feeding the rate forward.

        Sending PX4 a bounded yaw setpoint plus a matching yawspeed avoids the
        step-yaw torque spike that saturates the mixer (a motor drops out) when a
        new heading is commanded. yaw_rate <= 0 restores the raw step behavior.
        """
        if self.yaw_cmd is None:
            self.yaw_cmd = target
        if self.yaw_rate <= 0.0:
            self.yaw_cmd = target
            return target, math.nan
        step = self.yaw_rate * self.dt
        delta = wrap_pi(target - self.yaw_cmd)
        if abs(delta) <= step:
            self.yaw_cmd = target
            return target, 0.0
        self.yaw_cmd = wrap_pi(self.yaw_cmd + math.copysign(step, delta))
        return self.yaw_cmd, math.copysign(self.yaw_rate, delta)

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

    def tracking_loss_reason(self):
        """Fault reason gated by the conditions under which we actually LAND."""
        if not self.tracking_loss_land or not self.is_armed:
            return None
        if self.armed_t < self.tracking_arm_grace:
            return None
        return self.vio_fault_reason()

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
