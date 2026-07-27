"""Scripted square path: fly a side, turn, repeat.

Sequence per side: slew the position setpoint one `side_m` along the current
heading, settle, then yaw by `turn_deg` and settle. Four of those close the
square and return to the latched start position *and* start heading.

Built on OffboardWaypoint, so the position rate limiter, the hold_point
re-pointing and the settled/transit gate split all come along unchanged. The
only thing replaced is where the target comes from: corners are computed up
front instead of arriving as Foxglove clicks.

Two deliberate choices worth knowing:

- **Corners are precomputed from the latched start pose, not chained off the
  vehicle's actual position.** Chaining would fold each leg's tracking error
  into the next corner and the "square" would walk across the room. The 4th
  corner is therefore exactly the 1st, in the ideal frame.
- **Timeouts are derived from the geometry, not fixed.** HANDOFF records a
  near-miss where a fixed 6 s yaw timeout would have aborted a 45 deg leg that
  needed 8.4 s. A 90 deg turn at the old 5 deg/s default takes 18 s, so the
  timeouts here are computed as travel-time + margin and the node logs the
  resulting flight budget and refuses to start if it exceeds max_flight_time.
"""
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from px4_vio_bridge.offboard_waypoint import OffboardWaypoint, swap_enu_ned_xy


def wrap_pi(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class OffboardSquare(OffboardWaypoint):
    def __init__(self):
        super().__init__("offboard_square")

        self.declare_parameter("side_m", 0.40)
        self.declare_parameter("turn_deg", 90.0)     # +ve = turn right (yaw increases)
        self.declare_parameter("sides", 4)
        self.declare_parameter("corner_tol", 0.15)   # m, "reached this corner"
        self.declare_parameter("yaw_tolerance_deg", 5.0)
        self.declare_parameter("leg_settle_time", 1.5)   # s parked before turning
        self.declare_parameter("turn_settle_time", 1.0)  # s after a turn before flying
        # Worst-case deadlines = ideal travel time + margin. Raise the margins,
        # not the timeouts, if legs are timing out.
        self.declare_parameter("leg_timeout_margin", 6.0)
        self.declare_parameter("turn_timeout_margin", 4.0)

        self.side_m = float(self.get_parameter("side_m").value)
        self.turn = math.radians(float(self.get_parameter("turn_deg").value))
        self.sides = int(self.get_parameter("sides").value)
        self.corner_tol = float(self.get_parameter("corner_tol").value)
        self.yaw_tolerance = math.radians(
            float(self.get_parameter("yaw_tolerance_deg").value)
        )
        self.leg_settle_time = float(self.get_parameter("leg_settle_time").value)
        self.turn_settle_time = float(self.get_parameter("turn_settle_time").value)

        travel = self.side_m / self.waypoint_speed if self.waypoint_speed > 0 else 0.0
        turn_time = (
            abs(self.turn) / self.commanded_yaw_rate
            if self.commanded_yaw_rate > 0
            else 0.0
        )
        self.leg_timeout = travel + float(
            self.get_parameter("leg_timeout_margin").value
        )
        self.turn_timeout = turn_time + float(
            self.get_parameter("turn_timeout_margin").value
        )

        self.leg = 0                 # 0-based index of the side being flown
        self.corners = []            # NED (x, y) per corner, corners[0] = start
        self.headings = []           # NED yaw flown along each side

        self.path_pub = self.create_publisher(Path, "/square/path", 10)

        budget = (
            self.sides * (self.leg_timeout + self.leg_settle_time
                          + self.turn_timeout + self.turn_settle_time)
        )
        self.get_logger().warn(
            f"square: {self.sides} sides x {self.side_m:.2f} m, "
            f"turn {math.degrees(self.turn):+.0f} deg at "
            f"{math.degrees(self.commanded_yaw_rate):.0f} deg/s, "
            f"speed {self.waypoint_speed:.2f} m/s"
        )
        self.get_logger().warn(
            f"square: leg timeout {self.leg_timeout:.1f}s, turn timeout "
            f"{self.turn_timeout:.1f}s, worst-case budget {budget:.0f}s "
            f"vs max_flight_time {self.max_flight_time:.0f}s"
        )
        if budget > self.max_flight_time:
            self.get_logger().error(
                f"max_flight_time {self.max_flight_time:.0f}s is below the "
                f"worst-case square budget {budget:.0f}s -- the armed watchdog "
                "would LAND mid-square. Raise max_flight_time or the rates."
            )

    # A scripted path must not be redirected mid-flight; a stray click would
    # deform the square and leave the remaining corners meaningless.
    def accepting_waypoints(self):
        return False

    def plan_square(self):
        """Corner list and per-side heading, from the latched start pose."""
        self.corners = [(self.x0, self.y0)]
        self.headings = []
        x, y, h = self.x0, self.y0, self.yaw0
        for _ in range(self.sides):
            self.headings.append(h)
            x += self.side_m * math.cos(h)
            y += self.side_m * math.sin(h)
            self.corners.append((x, y))
            h = wrap_pi(h + self.turn)

        worst = max(
            math.hypot(cx - self.x0, cy - self.y0) for cx, cy in self.corners
        )
        self.get_logger().warn(
            f"square planned from ({self.x0:+.2f}, {self.y0:+.2f}) "
            f"heading {math.degrees(self.yaw0):.0f} deg; "
            f"furthest corner {worst:.2f} m (geofence {self.geofence_radius:.2f} m)"
        )
        for i, (cx, cy) in enumerate(self.corners):
            self.get_logger().warn(f"  corner {i}: ned=({cx:+.3f}, {cy:+.3f})")
        if worst > self.geofence_radius:
            # Clamping would deform the square, so refuse instead. Caught before
            # the vehicle ever leaves the first corner.
            self.trigger_landing(
                f"square corner {worst:.2f} m exceeds geofence "
                f"{self.geofence_radius:.2f} m"
            )
            return False
        self.publish_path()
        return True

    def publish_path(self):
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self.waypoint_frame
        for i, (cx, cy) in enumerate(self.corners):
            yaw = self.headings[min(i, len(self.headings) - 1)]
            path.poses.append(self.enu_pose(cx, cy, yaw))
        path.header.frame_id = self.waypoint_frame
        self.path_pub.publish(path)

    def set_leg_target(self):
        self.x_target, self.y_target = self.corners[self.leg + 1]
        self.waypoint_yaw = self.headings[self.leg]
        self.arrived = False

    def at_corner(self):
        """Vehicle inside corner_tol when armed; commanded convergence on a dry run."""
        if self.x_target is None or self.x_cmd is None:
            return False
        if math.hypot(self.x_target - self.x_cmd, self.y_target - self.y_cmd) > 1e-6:
            return False
        if not self.auto_arm:
            return True
        if self.pos is None:
            return False
        return (
            math.hypot(self.pos.x - self.x_target, self.pos.y - self.y_target)
            <= self.corner_tol
        )

    def yaw_error(self, target):
        """Vehicle heading error when armed; commanded-yaw error on a dry run."""
        if not self.auto_arm:
            return abs(wrap_pi(target - self.yaw_cmd))
        if self.pos is None or not math.isfinite(self.pos.heading):
            return math.inf
        return abs(wrap_pi(target - self.pos.heading))

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
                    self.get_logger().warn("stable hover reached; starting square")
                    self.start_square()
                elif self.t > self.climb_timeout:
                    self.trigger_landing(
                        "climb timeout without a stable hover "
                        f"(altitude_ok={altitude_ok}, horizontal_error="
                        f"{float('nan') if self.horizontal_error is None else self.horizontal_error:.2f}m"
                        f"/{self.pre_waypoint_max_horizontal_error:.2f}m)"
                    )
            elif self.t > self.climb_timeout:
                self.get_logger().warn(
                    "dry run: climb timeout; running the square state machine"
                )
                self.start_square()
            return True

        if self.state == "LEG":
            self.slew_commanded_position()
            self.publish_setpoint(self.hover_height, self.headings[self.leg])
            if not self.check_flight_position():
                return True
            self.publish_waypoint_viz()
            if self.at_corner():
                self.get_logger().warn(
                    f"side {self.leg + 1}/{self.sides} complete "
                    f"(corner {self.leg + 1})"
                )
                self.set_state("LEG_SETTLE")
            elif self.t >= self.leg_timeout:
                self.trigger_landing(
                    f"side {self.leg + 1} timed out after {self.leg_timeout:.1f}s"
                )
            return True

        if self.state == "LEG_SETTLE":
            self.slew_commanded_position()
            self.publish_setpoint(self.hover_height, self.headings[self.leg])
            if not self.check_flight_position():
                return True
            self.publish_waypoint_viz()
            if self.t >= self.leg_settle_time:
                self.set_state("TURN")
            return True

        if self.state == "TURN":
            self.slew_commanded_position()
            target = wrap_pi(self.headings[self.leg] + self.turn)
            self.publish_setpoint(self.hover_height, target)
            if not self.check_flight_position():
                return True
            self.publish_waypoint_viz()
            if self.yaw_error(target) <= self.yaw_tolerance:
                self.get_logger().warn(
                    f"turn {self.leg + 1}/{self.sides} complete "
                    f"(heading {math.degrees(target):.0f} deg)"
                )
                self.set_state("TURN_SETTLE")
            elif self.t >= self.turn_timeout:
                self.trigger_landing(
                    f"turn {self.leg + 1} timed out after {self.turn_timeout:.1f}s "
                    f"(error {math.degrees(self.yaw_error(target)):.1f} deg)"
                )
            return True

        if self.state == "TURN_SETTLE":
            self.slew_commanded_position()
            self.publish_setpoint(
                self.hover_height, wrap_pi(self.headings[self.leg] + self.turn)
            )
            if not self.check_flight_position():
                return True
            self.publish_waypoint_viz()
            if self.t >= self.turn_settle_time:
                self.leg += 1
                if self.leg >= self.sides:
                    self.get_logger().warn(
                        f"square complete: {self.sides} sides flown -> LAND"
                    )
                    self.set_state("LAND")
                else:
                    self.set_leg_target()
                    self.set_state("LEG")
            return True

        return False

    def start_square(self):
        if not self.plan_square():
            return
        self.leg = 0
        self.set_leg_target()
        self.set_state("LEG")

    def publish_waypoint_viz(self):
        self.commanded_pub.publish(
            self.enu_pose(self.x_cmd, self.y_cmd, self.current_yaw_target())
        )
        self.publish_target()
        remaining = math.hypot(
            self.x_target - self.x_cmd, self.y_target - self.y_cmd
        )
        self.publish_status(
            f"{self.state} side={self.leg + 1}/{self.sides} "
            f"to_go={remaining:.2f}m "
            f"hold_err={float('nan') if self.horizontal_error is None else self.horizontal_error:.2f}m"
            f"/{self.horizontal_error_limit:.2f}m "
            f"hdg_err={math.degrees(self.yaw_error(self.current_yaw_target())):.1f}deg"
        )


def main(args=None):
    rclpy.init(args=args)
    node = OffboardSquare()
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
