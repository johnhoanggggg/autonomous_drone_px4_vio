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

from px4_vio_bridge.offboard_waypoint import (
    OffboardWaypoint,
    swap_enu_ned_xy,
    swap_enu_ned_yaw,
    yaw_to_ros_quaternion,
)
from px4_vio_bridge.path_follower import (
    correction_rejection_reason,
    map_displacement_to_vio,
    yaw_from_quaternion,
)
from px4_vio_bridge.planner_flight import vio_displacement_to_map


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
        # Opt-in: keep the scripted square fixed in the loop-corrected map while
        # continuing to send smooth commands in PX4's continuous local frame.
        self.declare_parameter("use_loop_corrections", False)
        self.declare_parameter("correction_topic", "/rtabmap/odom_correction")
        self.declare_parameter("map_pose_topic", "/rtabmap/body_pose")
        self.declare_parameter("correction_timeout", 1.0)
        self.declare_parameter("map_pose_timeout", 1.0)
        self.declare_parameter("max_correction_m", 0.50)
        self.declare_parameter("max_correction_yaw_deg", 15.0)
        self.declare_parameter("loop_command_tolerance", 0.02)

        self.side_m = float(self.get_parameter("side_m").value)
        self.turn = math.radians(float(self.get_parameter("turn_deg").value))
        self.sides = int(self.get_parameter("sides").value)
        self.corner_tol = float(self.get_parameter("corner_tol").value)
        self.yaw_tolerance = math.radians(
            float(self.get_parameter("yaw_tolerance_deg").value)
        )
        self.leg_settle_time = float(self.get_parameter("leg_settle_time").value)
        self.turn_settle_time = float(self.get_parameter("turn_settle_time").value)
        self.use_loop_corrections = bool(
            self.get_parameter("use_loop_corrections").value
        )
        self.correction_timeout = float(
            self.get_parameter("correction_timeout").value
        )
        self.map_pose_timeout = float(self.get_parameter("map_pose_timeout").value)
        self.max_correction_m = float(self.get_parameter("max_correction_m").value)
        self.max_correction_yaw = math.radians(
            float(self.get_parameter("max_correction_yaw_deg").value)
        )
        self.loop_command_tolerance = float(
            self.get_parameter("loop_command_tolerance").value
        )

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
        self.map_corners = []        # canonical ENU world corners when correction is on
        self.map_headings = []       # canonical ENU yaw along each corrected side
        self.map_target = None
        self.correction = None       # map<-VIO (x, y, z, yaw)
        self.correction_valid = False
        self.correction_reason = "not received"
        self.correction_received = None
        self.map_pose = None         # corrected body (x, y, yaw) in world
        self.map_pose_valid = False
        self.map_pose_reason = "not received"
        self.map_pose_received = None

        if self.use_loop_corrections:
            self.create_subscription(
                PoseStamped,
                str(self.get_parameter("correction_topic").value),
                self.on_correction,
                10,
            )
            self.create_subscription(
                PoseStamped,
                str(self.get_parameter("map_pose_topic").value),
                self.on_map_pose,
                10,
            )

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
        if self.use_loop_corrections:
            self.get_logger().warn(
                "square loop correction ENABLED: fixed in corrected world; "
                f"limits={self.max_correction_m:.2f}m/"
                f"{math.degrees(self.max_correction_yaw):.1f}deg"
            )

    def on_correction(self, msg):
        """Validate and retain RTAB-Map's native map<-VIO correction."""
        now = self.monotonic_time()
        position = msg.pose.position
        orientation = msg.pose.orientation
        correction = (
            float(position.x),
            float(position.y),
            float(position.z),
            yaw_from_quaternion(
                (
                    float(orientation.w),
                    float(orientation.x),
                    float(orientation.y),
                    float(orientation.z),
                )
            ),
        )
        reason = correction_rejection_reason(
            correction, self.max_correction_m, self.max_correction_yaw
        )
        self.correction_valid = reason is None
        self.correction_reason = reason or ""
        self.correction = correction if reason is None else None
        self.correction_received = now

    def on_map_pose(self, msg):
        """Validate and retain the loop-corrected body pose used for feedback."""
        now = self.monotonic_time()
        position = msg.pose.position
        orientation = msg.pose.orientation
        yaw = yaw_from_quaternion(
            (
                float(orientation.w),
                float(orientation.x),
                float(orientation.y),
                float(orientation.z),
            )
        )
        values = (float(position.x), float(position.y), yaw)
        if msg.header.frame_id != self.waypoint_frame:
            reason = (
                f"map pose frame '{msg.header.frame_id}' != "
                f"'{self.waypoint_frame}'"
            )
        elif not all(math.isfinite(value) for value in values):
            reason = "map pose contains a non-finite value or invalid quaternion"
        else:
            reason = None
        self.map_pose_valid = reason is None
        self.map_pose_reason = reason or ""
        self.map_pose = values if reason is None else None
        self.map_pose_received = now

    def loop_correction_health_reason(self):
        if not self.use_loop_corrections:
            return None
        now = self.monotonic_time()
        if (
            self.correction_received is None
            or now - self.correction_received > self.correction_timeout
        ):
            return f"native correction stale for >{self.correction_timeout:.2f}s"
        if not self.correction_valid or self.correction is None:
            return f"native correction rejected: {self.correction_reason}"
        if (
            self.map_pose_received is None
            or now - self.map_pose_received > self.map_pose_timeout
        ):
            return f"corrected body pose stale for >{self.map_pose_timeout:.2f}s"
        if not self.map_pose_valid or self.map_pose is None:
            return f"corrected body pose rejected: {self.map_pose_reason}"
        return None

    def arm(self):
        reason = self.loop_correction_health_reason()
        if reason is not None:
            self.get_logger().error(
                f"ARM INHIBITED: {reason}", throttle_duration_sec=1.0
            )
            return
        super().arm()

    def vio_fault_reason(self):
        reason = super().vio_fault_reason()
        if reason is not None:
            return reason
        return self.loop_correction_health_reason()

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

        if self.use_loop_corrections:
            reason = self.loop_correction_health_reason()
            if reason is not None:
                self.get_logger().error(f"cannot plan corrected square: {reason}")
                return False
            self.map_corners = [(self.map_pose[0], self.map_pose[1])]
            self.map_headings = []
            x, y, h = self.map_pose
            for _ in range(self.sides):
                self.map_headings.append(h)
                x += self.side_m * math.cos(h)
                y += self.side_m * math.sin(h)
                self.map_corners.append((x, y))
                # Positive NED yaw is a right turn; ENU yaw has the opposite sign.
                h = wrap_pi(h - self.turn)

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
        if self.use_loop_corrections:
            for i, (cx, cy) in enumerate(self.map_corners):
                yaw = self.map_headings[min(i, len(self.map_headings) - 1)]
                path.poses.append(self.map_pose_message(cx, cy, yaw))
        else:
            for i, (cx, cy) in enumerate(self.corners):
                yaw = self.headings[min(i, len(self.headings) - 1)]
                path.poses.append(self.enu_pose(cx, cy, yaw))
        path.header.frame_id = self.waypoint_frame
        self.path_pub.publish(path)

    def set_leg_target(self):
        if self.use_loop_corrections:
            self.map_target = self.map_corners[self.leg + 1]
            if not self.refresh_corrected_target(self.map_headings[self.leg]):
                return False
        else:
            self.x_target, self.y_target = self.corners[self.leg + 1]
            self.waypoint_yaw = self.headings[self.leg]
        self.arrived = False
        return True

    def corrected_ned_yaw(self, map_yaw):
        """Map ENU heading expressed in PX4's continuous NED frame."""
        vio_yaw = wrap_pi(map_yaw - self.correction[3])
        return swap_enu_ned_yaw(vio_yaw)

    def refresh_corrected_target(self, map_yaw):
        """Rebase a fixed map goal from the current PX4 local position.

        Translation cancels from the map displacement because both the current
        corrected body pose and fixed target are in `world`. Only the native
        correction yaw rotates that displacement back into continuous VIO; the
        ENU/NED swap then makes it a PX4-local offset. The existing waypoint
        slew limiter absorbs any correction step before it reaches PX4.
        """
        if not self.use_loop_corrections:
            return True
        reason = self.loop_correction_health_reason()
        if reason is not None or self.pos is None or self.map_target is None:
            reason = reason or "PX4 local position or corrected target unavailable"
            if self.is_armed:
                self.trigger_landing(f"loop-corrected square lost input: {reason}")
            else:
                self.get_logger().error(f"corrected square stopped: {reason}")
                self.set_state("ABORT")
            return False
        map_displacement = (
            self.map_target[0] - self.map_pose[0],
            self.map_target[1] - self.map_pose[1],
        )
        vio_displacement = map_displacement_to_vio(
            map_displacement, self.correction[3]
        )
        ned_displacement = swap_enu_ned_xy(*vio_displacement)
        target = (
            float(self.pos.x) + ned_displacement[0],
            float(self.pos.y) + ned_displacement[1],
        )
        distance_from_start = math.dist(target, (self.x0, self.y0))
        if distance_from_start > self.geofence_radius:
            reason = (
                f"loop-corrected target {distance_from_start:.2f}m exceeds "
                f"geofence {self.geofence_radius:.2f}m"
            )
            if self.is_armed:
                self.trigger_landing(reason)
            else:
                self.get_logger().error(reason)
                self.set_state("ABORT")
            return False
        self.x_target, self.y_target = target
        self.waypoint_yaw = self.corrected_ned_yaw(map_yaw)
        return True

    def at_corner(self):
        """Vehicle inside corner_tol when armed; commanded convergence on a dry run."""
        if self.x_target is None or self.x_cmd is None:
            return False
        command_tolerance = (
            self.loop_command_tolerance if self.use_loop_corrections else 1e-6
        )
        if (
            math.hypot(self.x_target - self.x_cmd, self.y_target - self.y_cmd)
            > command_tolerance
        ):
            return False
        if not self.auto_arm:
            return True
        if self.use_loop_corrections:
            return (
                self.map_pose is not None
                and self.map_target is not None
                and math.dist(self.map_pose[:2], self.map_target) <= self.corner_tol
            )
        if self.pos is None:
            return False
        return (
            math.hypot(self.pos.x - self.x_target, self.pos.y - self.y_target)
            <= self.corner_tol
        )

    def leg_heading(self):
        if self.use_loop_corrections:
            return self.corrected_ned_yaw(self.map_headings[self.leg])
        return self.headings[self.leg]

    def turned_heading(self):
        if self.use_loop_corrections:
            return self.corrected_ned_yaw(
                wrap_pi(self.map_headings[self.leg] - self.turn)
            )
        return wrap_pi(self.headings[self.leg] + self.turn)

    def refresh_for_state(self, map_yaw):
        if not self.use_loop_corrections:
            return True
        if self.refresh_corrected_target(map_yaw):
            return True
        # Keep the last continuous setpoint flowing until LAND/ABORT takes over.
        self.publish_setpoint(self.hover_height, self.current_yaw_target())
        return False

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
            if not self.refresh_for_state(
                self.map_headings[self.leg] if self.use_loop_corrections else 0.0
            ):
                return True
            self.slew_commanded_position()
            self.publish_setpoint(self.hover_height, self.leg_heading())
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
            if not self.refresh_for_state(
                self.map_headings[self.leg] if self.use_loop_corrections else 0.0
            ):
                return True
            self.slew_commanded_position()
            self.publish_setpoint(self.hover_height, self.leg_heading())
            if not self.check_flight_position():
                return True
            self.publish_waypoint_viz()
            if self.t >= self.leg_settle_time:
                self.set_state("TURN")
            return True

        if self.state == "TURN":
            turned_map_yaw = (
                wrap_pi(self.map_headings[self.leg] - self.turn)
                if self.use_loop_corrections
                else 0.0
            )
            if not self.refresh_for_state(turned_map_yaw):
                return True
            self.slew_commanded_position()
            target = self.turned_heading()
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
            turned_map_yaw = (
                wrap_pi(self.map_headings[self.leg] - self.turn)
                if self.use_loop_corrections
                else 0.0
            )
            if not self.refresh_for_state(turned_map_yaw):
                return True
            self.slew_commanded_position()
            self.publish_setpoint(self.hover_height, self.turned_heading())
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
                    if self.set_leg_target():
                        self.set_state("LEG")
            return True

        return False

    def start_square(self):
        if not self.plan_square():
            if self.is_armed:
                self.trigger_landing("corrected square could not be planned")
            else:
                self.set_state("ABORT")
            return
        self.leg = 0
        if self.set_leg_target():
            self.set_state("LEG")

    def map_pose_message(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.waypoint_frame
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(self.hover_height)
        qx, qy, qz, qw = yaw_to_ros_quaternion(yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def local_command_map_pose(self):
        if (
            not self.use_loop_corrections
            or self.pos is None
            or self.map_pose is None
            or self.correction is None
        ):
            return self.enu_pose(self.x_cmd, self.y_cmd, self.current_yaw_target())
        ned_displacement = (
            self.x_cmd - float(self.pos.x), self.y_cmd - float(self.pos.y)
        )
        vio_displacement = swap_enu_ned_xy(*ned_displacement)
        map_displacement = vio_displacement_to_map(
            vio_displacement, self.correction[3]
        )
        point = (
            self.map_pose[0] + map_displacement[0],
            self.map_pose[1] + map_displacement[1],
        )
        vio_yaw = swap_enu_ned_yaw(self.current_yaw_target())
        map_yaw = wrap_pi(vio_yaw + self.correction[3])
        return self.map_pose_message(*point, map_yaw)

    def publish_target(self):
        if self.use_loop_corrections and self.map_target is not None:
            map_yaw = self.map_headings[self.leg]
            if self.state in ("TURN", "TURN_SETTLE"):
                map_yaw = wrap_pi(map_yaw - self.turn)
            self.target_pub.publish(self.map_pose_message(*self.map_target, map_yaw))
            return
        super().publish_target()

    def publish_waypoint_viz(self):
        self.commanded_pub.publish(self.local_command_map_pose())
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
