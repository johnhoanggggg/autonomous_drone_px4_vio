"""Interactive waypoint flight driven by Foxglove 3D-panel clicks.

Foxglove has no RViz-style `visualization_msgs/InteractiveMarker` support, so
there are no draggable 6-DOF handles. What it does have is the 3D panel's
**Publish** tool, which publishes a `geometry_msgs/PointStamped` (or
`PoseStamped`) wherever you click in the scene. This node turns those clicks
into PX4 position setpoints.

Sequence: the OffboardHover engagement path -> climb to `hover_height` ->
WAYPOINT, where the commanded position slews toward the most recently accepted
click. When no waypoint has been pending for `idle_timeout` seconds it requests
AUTO.LAND. Everything else (arm/offboard confirmation, VIO tracking-loss
watchdogs, K/L keys, max_flight_time) is inherited unchanged.

A click arrives over the network from a browser, so nothing it says is trusted
before it reaches PX4:

- the `frame_id` must equal `waypoint_frame`, and the point is converted from
  the ENU `world` frame that everything ROS-side publishes in into PX4 NED;
- the target is clamped into a `geofence_radius` disc around the *latched
  takeoff point*, so no click can send the vehicle across the room;
- the click's z is **ignored**. The Foxglove publish tool projects onto the
  display frame's z=0 ground plane, so its z is always 0 and honoring it would
  fly the vehicle into the floor. Altitude is always `hover_height`;
- the commanded position slews at `waypoint_speed` rather than stepping, so PX4
  never sees a position discontinuity.

The horizontal-error watchdog is re-pointed at the commanded position via
`hold_point`; measuring against the takeoff point (the OffboardHover default)
would land the vehicle the moment it deliberately translated.
"""
import math

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from px4_msgs.msg import TrajectorySetpoint
from std_msgs.msg import String

from px4_vio_bridge.offboard_hover import OffboardHover, wrap_pi


def yaw_from_ros_quaternion(q):
    """Yaw from a ROS geometry_msgs quaternion (x, y, z, w fields)."""
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def yaw_to_ros_quaternion(yaw):
    half_yaw = yaw * 0.5
    return 0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)


def swap_enu_ned_xy(x, y):
    """Convert between the ENU `world` frame and PX4 NED, either direction.

    `px4_local_position_to_ros` publishes x_enu=y_ned (east) and y_enu=x_ned
    (north); swapping the pair is its own inverse.
    """
    return float(y), float(x)


def swap_enu_ned_yaw(yaw):
    """Convert a heading between ENU and NED, either direction (also an involution)."""
    return wrap_pi(math.pi * 0.5 - yaw)


class OffboardWaypoint(OffboardHover):
    def __init__(self, node_name="offboard_waypoint"):
        super().__init__(node_name)

        self.declare_parameter("waypoint_speed", 0.25)     # m/s slew of the commanded point
        self.declare_parameter("geofence_radius", 1.5)     # m from the latched takeoff point
        self.declare_parameter("arrival_tol", 0.12)        # m, "at the waypoint"
        self.declare_parameter("idle_timeout", 20.0)       # s parked at a waypoint -> LAND
        self.declare_parameter("waypoint_frame", "world")  # frame_id a click must carry
        self.declare_parameter("accept_waypoint_yaw", False)
        self.declare_parameter("velocity_feedforward", False)
        # Transit lag is real and expected: PX4 trails a moving position
        # setpoint by roughly waypoint_speed / MPC_XY_P. At 0.25 m/s against the
        # default 0.95 that is ~0.26 m, which would sit right on top of the
        # 0.35 m hold gate. The tight gate therefore applies only once the
        # commanded point has been stationary for transit_settle_time.
        self.declare_parameter("transit_horizontal_error", 0.60)
        self.declare_parameter("transit_settle_time", 1.0)
        # As in offboard_hold_yaw: the interactive phase may only begin from a
        # verified stable hover, not from the edge of the abort envelope.
        self.declare_parameter("pre_waypoint_max_horizontal_error", 0.15)
        self.declare_parameter("waypoint_point_topic", "/waypoint/clicked")
        self.declare_parameter("waypoint_pose_topic", "/waypoint/clicked_pose")

        self.waypoint_speed = float(self.get_parameter("waypoint_speed").value)
        self.geofence_radius = float(self.get_parameter("geofence_radius").value)
        self.arrival_tol = float(self.get_parameter("arrival_tol").value)
        self.idle_timeout = float(self.get_parameter("idle_timeout").value)
        self.waypoint_frame = str(self.get_parameter("waypoint_frame").value)
        self.accept_waypoint_yaw = bool(
            self.get_parameter("accept_waypoint_yaw").value
        )
        self.velocity_feedforward = bool(
            self.get_parameter("velocity_feedforward").value
        )
        self.transit_horizontal_error = float(
            self.get_parameter("transit_horizontal_error").value
        )
        self.transit_settle_time = float(
            self.get_parameter("transit_settle_time").value
        )
        self.pre_waypoint_max_horizontal_error = float(
            self.get_parameter("pre_waypoint_max_horizontal_error").value
        )

        # Commanded (slewed) point actually published to PX4, and the accepted
        # target it is slewing toward. Both NED; None until the takeoff latch.
        self.x_cmd = self.y_cmd = None
        self.x_target = self.y_target = None
        self.waypoint_yaw = None       # NED yaw to hold; None = keep yaw0
        self.cmd_vx = self.cmd_vy = 0.0
        self.settled_t = 0.0           # s since the commanded point last moved
        self.idle_t = 0.0              # s parked with nothing left to fly to
        # Latched on first arrival, cleared by the next accepted click. It is
        # deliberately NOT the instantaneous at_waypoint() test: the 2026-07-27
        # flight held station 0.135 m from target on average against an
        # arrival_tol of 0.12, so an instantaneous test flickered false, reset
        # idle_t every few ticks, and the idle timeout could never fire.
        self.arrived = False
        self.waypoints_accepted = 0
        self.waypoints_rejected = 0
        self.last_status = None
        self.status_t = 0.0

        self.create_subscription(
            PointStamped,
            str(self.get_parameter("waypoint_point_topic").value),
            self.on_waypoint_point,
            10,
        )
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter("waypoint_pose_topic").value),
            self.on_waypoint_pose,
            10,
        )
        # Visualization back to Foxglove. PoseStamped, never TransformStamped:
        # Foxglove grafts any TransformStamped topic into its transform tree and
        # a second root detaches the point clouds from `world`.
        self.target_pub = self.create_publisher(PoseStamped, "/waypoint/target", 10)
        self.commanded_pub = self.create_publisher(
            PoseStamped, "/waypoint/commanded", 10
        )
        self.status_pub = self.create_publisher(String, "/waypoint/status", 10)

        self.get_logger().warn(
            f"waypoint flight: speed={self.waypoint_speed:.2f}m/s "
            f"geofence={self.geofence_radius:.2f}m hover={self.hover_height:.2f}m "
            f"idle_timeout={self.idle_timeout:.0f}s "
            f"yaw_from_click={self.accept_waypoint_yaw}"
        )

    # --- watchdog re-pointing ---------------------------------------------
    @property
    def hold_point(self):
        if self.x_cmd is None:
            return self.x0, self.y0
        return self.x_cmd, self.y_cmd

    @property
    def horizontal_error_limit(self):
        if self.settled_t < self.transit_settle_time:
            return self.transit_horizontal_error
        return self.max_horizontal_error

    # --- click intake ------------------------------------------------------
    def accepting_waypoints(self):
        return (
            self.x0 is not None
            and self.state not in ("WAIT_POS", "LAND", "KILL", "DONE", "ABORT")
        )

    def reject(self, reason):
        self.waypoints_rejected += 1
        self.get_logger().warn(f"waypoint REJECTED: {reason}")
        self.publish_status(f"rejected: {reason}", force=True)

    def on_waypoint_point(self, msg):
        self.accept_click(msg.header.frame_id, msg.point.x, msg.point.y, None)

    def on_waypoint_pose(self, msg):
        yaw_enu = None
        if self.accept_waypoint_yaw:
            yaw_enu = yaw_from_ros_quaternion(msg.pose.orientation)
        self.accept_click(
            msg.header.frame_id, msg.pose.position.x, msg.pose.position.y, yaw_enu
        )

    def accept_click(self, frame_id, x_enu, y_enu, yaw_enu):
        """Validate, clamp and latch one click. The click's z is never used."""
        if not self.accepting_waypoints():
            self.reject(f"not accepting waypoints in state {self.state}")
            return
        if frame_id != self.waypoint_frame:
            self.reject(
                f"frame_id '{frame_id}' != waypoint_frame '{self.waypoint_frame}' "
                "(set the Foxglove 3D panel's display frame to "
                f"'{self.waypoint_frame}')"
            )
            return
        if not (math.isfinite(x_enu) and math.isfinite(y_enu)):
            self.reject(f"non-finite click ({x_enu}, {y_enu})")
            return

        x_ned, y_ned = swap_enu_ned_xy(x_enu, y_enu)
        x_ned, y_ned, clamped = self.clamp_to_geofence(x_ned, y_ned)
        self.x_target, self.y_target = x_ned, y_ned
        if yaw_enu is not None and math.isfinite(yaw_enu):
            self.waypoint_yaw = swap_enu_ned_yaw(yaw_enu)
        self.idle_t = 0.0
        self.arrived = False
        self.waypoints_accepted += 1

        distance = math.hypot(x_ned - self.x_cmd, y_ned - self.y_cmd) if self.x_cmd is not None else 0.0
        note = " (CLAMPED to geofence)" if clamped else ""
        self.get_logger().warn(
            f"waypoint #{self.waypoints_accepted}: ned=({x_ned:.2f}, {y_ned:.2f}) "
            f"{distance:.2f}m away, "
            f"{distance / self.waypoint_speed if self.waypoint_speed > 0 else 0.0:.1f}s "
            f"of travel{note}"
        )
        self.publish_target()

    def clamp_to_geofence(self, x_ned, y_ned):
        """Pull a target inside the geofence disc around the latched takeoff point."""
        dx = x_ned - self.x0
        dy = y_ned - self.y0
        distance = math.hypot(dx, dy)
        if distance <= self.geofence_radius or distance == 0.0:
            return x_ned, y_ned, False
        scale = self.geofence_radius / distance
        return self.x0 + dx * scale, self.y0 + dy * scale, True

    # --- commanded-position slew ------------------------------------------
    def ensure_commanded_position(self):
        if self.x_cmd is None and self.x0 is not None:
            self.x_cmd, self.y_cmd = self.x0, self.y0

    def slew_commanded_position(self):
        """Step the commanded point toward the target at waypoint_speed.

        Rate-limiting the *setpoint* is what keeps this safe: PX4 is given a
        continuously moving goal it can track, never a jump it would chase at
        MPC_XY_VEL_MAX.
        """
        self.ensure_commanded_position()
        if self.x_cmd is None:
            return
        if self.x_target is None:
            self.x_target, self.y_target = self.x_cmd, self.y_cmd

        dx = self.x_target - self.x_cmd
        dy = self.y_target - self.y_cmd
        distance = math.hypot(dx, dy)
        step = self.waypoint_speed * self.dt
        if distance <= step or step <= 0.0:
            moved = distance > 0.0
            self.x_cmd, self.y_cmd = self.x_target, self.y_target
            self.cmd_vx = self.cmd_vy = 0.0
        else:
            self.x_cmd += dx / distance * step
            self.y_cmd += dy / distance * step
            self.cmd_vx = dx / distance * self.waypoint_speed
            self.cmd_vy = dy / distance * self.waypoint_speed
            moved = True

        self.settled_t = 0.0 if moved else self.settled_t + self.dt

    def setpoint_velocity(self, vz=math.nan):
        """Optional feedforward that cancels most of the position-tracking lag.

        vz comes from ramp_z and is independent of the horizontal feedforward:
        the altitude ramp stays active even with velocity_feedforward off.
        """
        if not self.velocity_feedforward:
            return [math.nan, math.nan, vz]
        return [float(self.cmd_vx), float(self.cmd_vy), vz]

    def publish_setpoint(self, z_up, yaw=None):
        self.ensure_commanded_position()
        target_yaw = float(self.yaw0 if yaw is None else yaw)
        yaw_sp, yawspeed = self.ramp_yaw(target_yaw)
        z_sp, vz_sp = self.ramp_z(float(z_up))
        m = TrajectorySetpoint()
        m.timestamp = self.now_us()
        m.position = [float(self.x_cmd), float(self.y_cmd), float(-z_sp)]
        m.velocity = self.setpoint_velocity(vz_sp)
        m.acceleration = [math.nan, math.nan, math.nan]
        m.yaw = yaw_sp
        m.yawspeed = yawspeed
        self.sp_pub.publish(m)

    def at_waypoint(self):
        """True once the vehicle (armed) or the commanded point (dry run) has arrived."""
        if self.x_target is None or self.x_cmd is None:
            return False
        if math.hypot(self.x_target - self.x_cmd, self.y_target - self.y_cmd) > 1e-6:
            return False
        if not self.auto_arm:
            # A props-off dry run cannot physically follow the setpoint; verify
            # the slew and state machine against the commanded point instead.
            return True
        if self.pos is None:
            return False
        return (
            math.hypot(self.pos.x - self.x_target, self.pos.y - self.y_target)
            <= self.arrival_tol
        )

    # --- visualization -----------------------------------------------------
    def enu_pose(self, x_ned, y_ned, yaw_ned):
        x_enu, y_enu = swap_enu_ned_xy(x_ned, y_ned)
        qx, qy, qz, qw = yaw_to_ros_quaternion(swap_enu_ned_yaw(yaw_ned))
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.waypoint_frame
        pose.pose.position.x = x_enu
        pose.pose.position.y = y_enu
        pose.pose.position.z = float(self.hover_height)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def current_yaw_target(self):
        return self.yaw0 if self.waypoint_yaw is None else self.waypoint_yaw

    def publish_target(self):
        if self.x_target is None:
            return
        self.target_pub.publish(
            self.enu_pose(self.x_target, self.y_target, self.current_yaw_target())
        )

    def publish_status(self, text, force=False):
        if not force and text == self.last_status and self.status_t < 1.0:
            return
        self.last_status = text
        self.status_t = 0.0
        m = String()
        m.data = text
        self.status_pub.publish(m)

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
                    self.get_logger().warn(
                        "stable hover reached; accepting waypoints from Foxglove"
                    )
                    self.set_state("WAYPOINT")
                elif self.t > self.climb_timeout:
                    self.trigger_landing(
                        "climb timeout without a stable hover "
                        f"(altitude_ok={altitude_ok}, horizontal_error="
                        f"{float('nan') if self.horizontal_error is None else self.horizontal_error:.2f}m"
                        f"/{self.pre_waypoint_max_horizontal_error:.2f}m)"
                    )
            elif self.t > self.climb_timeout:
                self.get_logger().warn(
                    "dry run: climb timeout; accepting waypoints (will not fly)"
                )
                self.set_state("WAYPOINT")
            return True

        if self.state == "WAYPOINT":
            self.slew_commanded_position()
            self.publish_setpoint(self.hover_height, self.current_yaw_target())
            if not self.check_flight_position():
                return True

            if not self.arrived and self.at_waypoint():
                self.arrived = True
                self.get_logger().warn(
                    f"arrived at waypoint #{self.waypoints_accepted}; "
                    f"idle timeout {self.idle_timeout:.0f}s running"
                )
            self.idle_t = self.idle_t + self.dt if self.arrived else 0.0

            self.commanded_pub.publish(
                self.enu_pose(self.x_cmd, self.y_cmd, self.current_yaw_target())
            )
            self.publish_target()
            remaining = (
                math.hypot(self.x_target - self.x_cmd, self.y_target - self.y_cmd)
                if self.x_target is not None
                else 0.0
            )
            self.publish_status(
                f"WAYPOINT accepted={self.waypoints_accepted} "
                f"rejected={self.waypoints_rejected} "
                f"to_go={remaining:.2f}m "
                f"hold_err={float('nan') if self.horizontal_error is None else self.horizontal_error:.2f}m"
                f"/{self.horizontal_error_limit:.2f}m "
                f"arrived={self.arrived} "
                f"idle={self.idle_t:.0f}/{self.idle_timeout:.0f}s"
            )

            if self.idle_t >= self.idle_timeout:
                self.get_logger().warn(
                    f"no new waypoint for {self.idle_timeout:.0f}s -> LAND"
                )
                self.set_state("LAND")
            return True

        return False

    def tick(self):
        self.status_t += self.dt
        super().tick()


def main(args=None):
    rclpy.init(args=args)
    node = OffboardWaypoint()
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
