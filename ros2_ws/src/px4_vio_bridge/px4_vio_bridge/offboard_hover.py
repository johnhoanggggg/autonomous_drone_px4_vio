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
- On shutdown while armed it commands AUTO.LAND rather than disarming in air.
"""
import math

import rclpy
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleControlMode,
    VehicleLocalPosition,
)
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


class OffboardHover(Node):
    def __init__(self):
        super().__init__("offboard_hover")

        self.declare_parameter("hover_height", 0.30)      # meters up
        self.declare_parameter("hold_time", 10.0)         # seconds at altitude
        self.declare_parameter("rate_hz", 50.0)
        self.declare_parameter("stream_time", 1.0)        # setpoint pre-stream before engage
        self.declare_parameter("engage_timeout", 5.0)     # arm+offboard must confirm within
        self.declare_parameter("climb_timeout", 8.0)      # start hold clock by here regardless
        self.declare_parameter("reach_tol", 0.07)         # m, "reached" altitude band
        self.declare_parameter("max_flight_time", 40.0)   # armed watchdog -> force land
        self.declare_parameter("auto_arm", False)         # MUST be true to actually fly

        self.hover_height = float(self.get_parameter("hover_height").value)
        self.hold_time = float(self.get_parameter("hold_time").value)
        self.rate_hz = float(self.get_parameter("rate_hz").value)
        self.stream_time = float(self.get_parameter("stream_time").value)
        self.engage_timeout = float(self.get_parameter("engage_timeout").value)
        self.climb_timeout = float(self.get_parameter("climb_timeout").value)
        self.reach_tol = float(self.get_parameter("reach_tol").value)
        self.max_flight_time = float(self.get_parameter("max_flight_time").value)
        self.auto_arm = bool(self.get_parameter("auto_arm").value)

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

        self.pos = None            # latest VehicleLocalPosition
        self.vcm = None            # latest VehicleControlMode
        self.x0 = self.y0 = self.yaw0 = None

        self.state = "WAIT_POS"
        self.t = 0.0               # seconds in current state
        self.armed_t = 0.0         # seconds since arm confirmed
        self.hold_t = 0.0
        self.reached = False
        self.last_cmd_t = 0.0

        self.dt = 1.0 / self.rate_hz
        self.timer = self.create_timer(self.dt, self.tick)
        self.get_logger().warn(
            f"offboard_hover: auto_arm={self.auto_arm} hover={self.hover_height}m hold={self.hold_time}s "
            f"({'ARMED FLIGHT' if self.auto_arm else 'DRY RUN - will not arm'})"
        )

    # --- subscriptions -----------------------------------------------------
    def on_local_position(self, msg):
        self.pos = msg

    def on_control_mode(self, msg):
        self.vcm = msg

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

    def publish_setpoint(self, z_up):
        m = TrajectorySetpoint()
        m.timestamp = self.now_us()
        m.position = [float(self.x0), float(self.y0), float(-z_up)]  # NED, z down-negative-up
        m.velocity = [math.nan, math.nan, math.nan]
        m.acceleration = [math.nan, math.nan, math.nan]
        m.yaw = float(self.yaw0)
        m.yawspeed = math.nan
        self.sp_pub.publish(m)

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

    def land(self):
        self.send_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)

    # --- state machine -----------------------------------------------------
    def set_state(self, name):
        self.get_logger().warn(f"-> {name}")
        self.state = name
        self.t = 0.0

    def tick(self):
        self.t += self.dt
        if self.is_armed:
            self.armed_t += self.dt

        # global armed watchdog
        if self.is_armed and self.armed_t > self.max_flight_time and self.state not in ("LAND", "DONE"):
            self.get_logger().error("max_flight_time exceeded -> LAND")
            self.set_state("LAND")

        if self.state == "WAIT_POS":
            if self.pos_valid() and self.vcm is not None:
                self.x0 = self.pos.x
                self.y0 = self.pos.y
                self.yaw0 = self.pos.heading
                self.get_logger().warn(
                    f"latched hold x={self.x0:.2f} y={self.y0:.2f} yaw={math.degrees(self.yaw0):.0f}deg")
                self.set_state("STREAM")
            elif self.t > 10.0:
                self.get_logger().error("no valid local position in 10s -> ABORT")
                self.set_state("ABORT")
            return

        # From STREAM onward, ALWAYS keep the offboard heartbeat + setpoint flowing.
        self.publish_offboard_mode()

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

        if self.state == "CLIMB_HOLD":
            self.publish_setpoint(self.hover_height)
            if self.auto_arm and not self.pos_valid():
                self.get_logger().error("lost local position in flight -> LAND")
                self.set_state("LAND")
                return
            if not self.reached and self.pos is not None:
                if abs((-self.pos.z) - self.hover_height) <= self.reach_tol:
                    self.reached = True
                    self.get_logger().warn("reached hover altitude, starting hold clock")
            if self.reached or self.t > self.climb_timeout:
                self.hold_t += self.dt
                if self.hold_t >= self.hold_time:
                    self.set_state("LAND")
            return

        if self.state == "LAND":
            # keep streaming a couple cycles then hand to AUTO.LAND
            self.publish_setpoint(self.hover_height)
            if self.t < 0.2:
                self.land()
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
                self.land()
            if self.t > 1.0:
                self.set_state("DONE")
            return

        if self.state == "DONE":
            self.timer.cancel()
            self.get_logger().warn("sequence complete")
            rclpy.try_shutdown()
            return

    def on_shutdown(self):
        if self.is_armed:
            self.get_logger().error("shutdown while armed -> commanding AUTO.LAND")
            for _ in range(5):
                self.land()


def main(args=None):
    rclpy.init(args=args)
    node = OffboardHover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.on_shutdown()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
