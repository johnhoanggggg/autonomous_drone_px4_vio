"""Position-only PX4 adapter for the validated global-planner follower.

This node never consumes an absolute SLAM-world carrot. It rebases the
follower's continuous-VIO-frame displacement from PX4's current local position,
then applies an independent speed/acceleration limiter before publishing a NED
position setpoint. Invalid planner data latches a stationary HOLD; persistent
faults request AUTO.LAND.

The published yaw tracks the heading of that same commanded displacement, so
the airframe (and the forward-facing camera the VIO depends on) points along
the route. Translation pauses while a turn larger than yaw_align_error_deg
slews in, and the tracked heading is dropped whenever PX4 resets its own.
"""

import math

import rclpy
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from px4_msgs.msg import TrajectorySetpoint
from std_msgs.msg import Bool, String

from px4_vio_bridge.offboard_hover import OffboardHover, wrap_pi
from px4_vio_bridge.path_follower import (
    correction_rejection_reason,
    yaw_from_quaternion,
)
from px4_vio_bridge.planner_flight import (
    HorizontalCommandLimiter,
    clamp_to_disc,
    ned_track_heading,
    track_yaw_target,
    vio_enu_displacement_to_ned,
)


class OffboardGlobalPlanner(OffboardHover):
    def __init__(self):
        super().__init__("offboard_global_planner")

        self.declare_parameter(
            "follower_displacement_topic", "/planner/follower/vio_displacement"
        )
        self.declare_parameter("follower_valid_topic", "/planner/follower/valid")
        self.declare_parameter(
            "follower_goal_topic", "/planner/follower/goal_reached"
        )
        self.declare_parameter("correction_topic", "/rtabmap/odom_correction")
        self.declare_parameter("follower_timeout", 0.30)
        self.declare_parameter("correction_timeout", 1.0)
        self.declare_parameter("planner_fault_land_time", 3.0)
        self.declare_parameter("goal_hold_time", 3.0)
        self.declare_parameter("max_follower_displacement", 1.0)
        self.declare_parameter("max_correction_m", 0.25)
        self.declare_parameter("max_correction_yaw_deg", 5.0)
        self.declare_parameter("command_speed", 0.20)
        self.declare_parameter("command_acceleration", 0.40)
        self.declare_parameter("geofence_radius", 1.0)
        self.declare_parameter("geofence_tolerance", 0.15)
        self.declare_parameter("transit_horizontal_error", 0.60)
        self.declare_parameter("pre_route_max_horizontal_error", 0.15)
        # Heading tracking. The published yaw still slews at the base class's
        # yaw_rate_deg, so these only choose WHICH heading is commanded.
        self.declare_parameter("yaw_follows_heading", True)
        self.declare_parameter("yaw_track_min_displacement", 0.15)
        self.declare_parameter("yaw_track_deadband_deg", 15.0)
        # Translation pauses above yaw_align_error_deg and resumes below
        # yaw_resume_error_deg, so a leg is flown forward rather than sideways.
        # yaw_align_error_deg <= 0 turns and translates simultaneously instead.
        self.declare_parameter("yaw_align_error_deg", 40.0)
        self.declare_parameter("yaw_resume_error_deg", 15.0)

        self.follower_timeout = float(self.get_parameter("follower_timeout").value)
        self.correction_timeout = float(
            self.get_parameter("correction_timeout").value
        )
        self.planner_fault_land_time = float(
            self.get_parameter("planner_fault_land_time").value
        )
        self.goal_hold_time = float(self.get_parameter("goal_hold_time").value)
        self.max_follower_displacement = float(
            self.get_parameter("max_follower_displacement").value
        )
        self.max_correction_m = float(
            self.get_parameter("max_correction_m").value
        )
        self.max_correction_yaw = math.radians(
            float(self.get_parameter("max_correction_yaw_deg").value)
        )
        self.geofence_radius = float(self.get_parameter("geofence_radius").value)
        self.geofence_tolerance = float(
            self.get_parameter("geofence_tolerance").value
        )
        self.transit_horizontal_error = float(
            self.get_parameter("transit_horizontal_error").value
        )
        self.pre_route_max_horizontal_error = float(
            self.get_parameter("pre_route_max_horizontal_error").value
        )
        self.yaw_track = bool(self.get_parameter("yaw_follows_heading").value)
        self.yaw_track_min_displacement = float(
            self.get_parameter("yaw_track_min_displacement").value
        )
        self.yaw_track_deadband = math.radians(
            float(self.get_parameter("yaw_track_deadband_deg").value)
        )
        self.yaw_align_error = math.radians(
            float(self.get_parameter("yaw_align_error_deg").value)
        )
        self.yaw_resume_error = math.radians(
            float(self.get_parameter("yaw_resume_error_deg").value)
        )
        if self.yaw_resume_error > self.yaw_align_error:
            self.yaw_resume_error = self.yaw_align_error
        self.limiter = HorizontalCommandLimiter(
            max_speed=float(self.get_parameter("command_speed").value),
            max_acceleration=float(
                self.get_parameter("command_acceleration").value
            ),
        )

        self.follower_valid = False
        self.follower_valid_received = None
        self.goal_reached = False
        self.goal_received = None
        self.vio_displacement = None
        self.displacement_received = None
        self.correction_valid = False
        self.correction_reason = "not received"
        self.correction_received = None
        self.planner_fault_since = None
        self.planner_fault_reason_text = ""
        self.goal_since = None
        self.yaw_target = None      # latched NED heading of the current leg
        self.yaw_holding = False    # translation paused while turning onto it
        self.holding_for_fault = True
        self.require_follower_after = 0.0
        self.local_reset_counters = None
        self.last_route_status = None
        self.last_route_status_time = 0.0
        self.last_route_status_kind = None

        self.follower_displacement_topic = str(
            self.get_parameter("follower_displacement_topic").value
        )
        self.follower_valid_topic = str(
            self.get_parameter("follower_valid_topic").value
        )
        self.follower_goal_topic = str(
            self.get_parameter("follower_goal_topic").value
        )
        self.create_subscription(
            Vector3Stamped,
            self.follower_displacement_topic,
            self.on_follower_displacement,
            10,
        )
        self.create_subscription(
            Bool,
            self.follower_valid_topic,
            self.on_follower_valid,
            10,
        )
        self.create_subscription(
            Bool,
            self.follower_goal_topic,
            self.on_goal_reached,
            10,
        )
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter("correction_topic").value),
            self.on_correction,
            10,
        )
        self.route_status_pub = self.create_publisher(
            String, "/planner/flight/status", 10
        )
        self.get_logger().warn(
            "GLOBAL PLANNER FLIGHT ADAPTER: position-only, "
            f"auto_arm={self.auto_arm}, speed={self.limiter.max_speed:.2f}m/s, "
            f"geofence={self.geofence_radius:.2f}m, correction gate="
            f"{self.max_correction_m:.2f}m/"
            f"{math.degrees(self.max_correction_yaw):.1f}deg"
        )
        self.get_logger().warn(
            "BATTERY AUTHORITY: PX4 arming checks and battery failsafes; "
            "companion battery topics are telemetry only"
        )
        if not self.yaw_track:
            self.get_logger().warn("yaw tracking disabled; holding latched takeoff yaw")
        elif self.yaw_align_error > 0.0:
            self.get_logger().warn(
                "YAW FOLLOWS PATH HEADING: slew "
                f"{math.degrees(self.commanded_yaw_rate):.0f}deg/s, translation pauses "
                f"above {math.degrees(self.yaw_align_error):.0f}deg error and resumes "
                f"below {math.degrees(self.yaw_resume_error):.0f}deg"
            )
        else:
            self.get_logger().warn(
                "YAW FOLLOWS PATH HEADING: slew "
                f"{math.degrees(self.commanded_yaw_rate):.0f}deg/s, turning while "
                "translating (align gate disabled)"
            )

    @property
    def hold_point(self):
        if self.limiter.position is not None:
            return self.limiter.position
        return self.x0, self.y0

    @property
    def horizontal_error_limit(self):
        moving = math.hypot(*self.limiter.velocity) > 0.01
        return self.transit_horizontal_error if moving else self.max_horizontal_error

    def on_follower_valid(self, msg):
        self.follower_valid = bool(msg.data)
        self.follower_valid_received = self.monotonic_time()

    def on_goal_reached(self, msg):
        self.goal_reached = bool(msg.data)
        self.goal_received = self.monotonic_time()

    def on_follower_displacement(self, msg):
        now = self.monotonic_time()
        vector = float(msg.vector.x), float(msg.vector.y)
        if (
            msg.header.frame_id != "vio"
            or not all(math.isfinite(value) for value in vector)
            or math.hypot(*vector) > self.max_follower_displacement
        ):
            self.vio_displacement = None
        else:
            self.vio_displacement = vector
        self.displacement_received = now

    def on_correction(self, msg):
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
        self.correction_received = now

    def on_local_position(self, msg):
        counters = (
            int(msg.xy_reset_counter),
            int(msg.z_reset_counter),
            int(msg.heading_reset_counter),
        )
        if self.local_reset_counters is not None and counters != self.local_reset_counters:
            now = self.monotonic_time()
            if self.x0 is not None and math.isfinite(msg.x) and math.isfinite(msg.y):
                self.limiter.reset((float(msg.x), float(msg.y)))
                self.holding_for_fault = True
                self.require_follower_after = now
                self.planner_fault_since = now
                self.planner_fault_reason_text = (
                    f"PX4 local reset counters {self.local_reset_counters}->{counters}"
                )
            if counters[2] != self.local_reset_counters[2] and math.isfinite(msg.heading):
                # The estimator's heading jumped; the tracked leg heading was
                # expressed in the old frame, so re-latch on the new one.
                self.yaw0 = float(msg.heading)
                self.yaw_cmd = float(msg.heading)
                self.yaw_target = None
                self.yaw_holding = False
            self.get_logger().error(self.planner_fault_reason_text)
        self.local_reset_counters = counters
        super().on_local_position(msg)

    def ensure_limiter(self):
        if self.limiter.position is None and self.x0 is not None:
            self.limiter.reset((self.x0, self.y0))

    def planner_health_reason(self):
        now = self.monotonic_time()
        for topic, label in (
            ("/planner/status", "global planner status"),
            ("/planner/path", "global planner path"),
            ("/planner/inflated_map", "global planner costmap"),
            (self.follower_valid_topic, "follower validity"),
            (self.follower_displacement_topic, "follower displacement"),
            (self.follower_goal_topic, "follower goal state"),
        ):
            publisher_count = self.count_publishers(topic)
            if publisher_count != 1:
                return (
                    f"{label} publisher count is {publisher_count}, expected exactly 1"
                )
        if (
            self.follower_valid_received is None
            or now - self.follower_valid_received > self.follower_timeout
        ):
            return f"follower validity stale for >{self.follower_timeout:.2f}s"
        # Report invalidity before the remaining staleness checks.  An invalid
        # follower stops publishing displacement and goal state, so those go
        # stale a fraction of a second later and would otherwise overwrite the
        # real reason in the logs with a VIO fault that never happened.
        if not self.follower_valid:
            return "follower validity is false"
        for received, label in (
            (self.goal_received, "follower goal state"),
            (self.displacement_received, "VIO displacement"),
        ):
            if received is None or now - received > self.follower_timeout:
                return f"{label} stale for >{self.follower_timeout:.2f}s"
        if self.vio_displacement is None:
            return "VIO displacement is invalid"
        if self.displacement_received <= self.require_follower_after:
            return "waiting for follower data after PX4 local reset"
        if (
            self.correction_received is None
            or now - self.correction_received > self.correction_timeout
        ):
            return f"native correction stale for >{self.correction_timeout:.2f}s"
        if not self.correction_valid:
            return f"native correction rejected: {self.correction_reason}"
        return None

    def preflight_reason(self):
        if not self.pos_valid():
            return "PX4 local position invalid"
        reason = self.vio_fault_reason()
        if reason is not None:
            return reason
        reason = self.planner_health_reason()
        if reason is not None:
            return reason
        if self.goal_reached:
            return "planner goal is already reached; provide a new route"
        return None

    def arm(self):
        reason = self.preflight_reason()
        if reason is not None:
            self.get_logger().error(
                f"ARM INHIBITED: {reason}", throttle_duration_sec=1.0
            )
            return
        super().arm()

    def publish_setpoint(self, z_up, yaw=None):
        self.ensure_limiter()
        if self.limiter.position is None:
            return
        # Callers that do not name a yaw (the base class LAND and ground-hold
        # states) must hold the heading the route reached, NOT snap back to the
        # takeoff latch: reverting would command a large turn during a descent
        # if PX4 ever stayed in OFFBOARD instead of taking AUTO.LAND.
        target_yaw = float(self.route_yaw() if yaw is None else yaw)
        yaw_sp, yawspeed = self.ramp_yaw(target_yaw)
        msg = TrajectorySetpoint()
        msg.timestamp = self.now_us()
        msg.position = [
            float(self.limiter.position[0]),
            float(self.limiter.position[1]),
            float(-z_up),
        ]
        msg.velocity = [math.nan, math.nan, math.nan]
        msg.acceleration = [math.nan, math.nan, math.nan]
        msg.yaw = yaw_sp
        msg.yawspeed = yawspeed
        self.sp_pub.publish(msg)

    def publish_route_status(self, text, kind):
        now = self.monotonic_time()
        if kind == self.last_route_status_kind and now - self.last_route_status_time < 0.2:
            return
        if text == self.last_route_status and now - self.last_route_status_time < 1.0:
            return
        self.last_route_status = text
        self.last_route_status_time = now
        self.route_status_pub.publish(String(data=text))
        if kind != self.last_route_status_kind:
            self.last_route_status_kind = kind
            self.get_logger().warn(text)

    def latch_fault_hold(self, reason):
        now = self.monotonic_time()
        if not self.holding_for_fault:
            if self.pos is not None and math.isfinite(self.pos.x) and math.isfinite(self.pos.y):
                self.limiter.reset((float(self.pos.x), float(self.pos.y)))
            self.holding_for_fault = True
        if self.planner_fault_since is None:
            self.planner_fault_since = now
        self.planner_fault_reason_text = reason
        elapsed = now - self.planner_fault_since
        self.publish_route_status(
            f"HOLD planner fault: {reason}; land in "
            f"{max(0.0, self.planner_fault_land_time - elapsed):.1f}s",
            "HOLD",
        )
        if self.is_armed and elapsed >= self.planner_fault_land_time:
            self.trigger_landing(f"planner fault persisted: {reason}")

    def clear_planner_fault(self):
        self.planner_fault_since = None
        self.planner_fault_reason_text = ""
        self.holding_for_fault = False

    # --- heading tracking --------------------------------------------------
    def route_yaw(self):
        """Yaw setpoint for the route: the tracked leg heading, else takeoff yaw."""
        if not self.yaw_track or self.yaw_target is None:
            return self.yaw0
        return self.yaw_target

    def yaw_track_error(self):
        """Signed error from the estimated heading to the tracked target."""
        if not self.yaw_track or self.yaw_target is None:
            return None
        if self.pos is None or not math.isfinite(self.pos.heading):
            return None
        return wrap_pi(self.yaw_target - float(self.pos.heading))

    def update_yaw_target(self, ned_displacement):
        """Latch the leg heading and update the turn-before-translate gate."""
        if not self.yaw_track:
            return
        self.yaw_target = track_yaw_target(
            self.yaw_target,
            ned_track_heading(ned_displacement, self.yaw_track_min_displacement),
            self.yaw_track_deadband,
        )
        if self.yaw_align_error <= 0.0:
            self.yaw_holding = False
            return
        error = self.yaw_track_error()
        if error is None:
            self.yaw_holding = False
        elif abs(error) > self.yaw_align_error:
            self.yaw_holding = True
        elif abs(error) <= self.yaw_resume_error:
            self.yaw_holding = False

    def freeze_yaw_target(self):
        """Stop turning where the slew currently is, for a fault or reset hold."""
        self.yaw_holding = False
        if self.yaw_track and self.yaw_cmd is not None:
            self.yaw_target = float(self.yaw_cmd)

    def update_route_command(self):
        ned_displacement = vio_enu_displacement_to_ned(self.vio_displacement)
        self.update_yaw_target(ned_displacement)
        if self.yaw_holding:
            # Translating with a large heading error flies the vehicle sideways
            # and points the camera off the path. Decelerate to a stop through
            # the same limiter and let the yaw slew catch up first.
            self.limiter.update(self.hold_point, self.dt)
            return
        target = (
            float(self.pos.x) + ned_displacement[0],
            float(self.pos.y) + ned_displacement[1],
        )
        target = clamp_to_disc(target, (self.x0, self.y0), self.geofence_radius)
        self.limiter.update(target, self.dt)

    def geofence_breached(self):
        if self.pos is None or self.x0 is None:
            return False
        return math.hypot(self.pos.x - self.x0, self.pos.y - self.y0) > (
            self.geofence_radius + self.geofence_tolerance
        )

    def handle_flight_state(self):
        if self.state not in ("CLIMB_HOLD", "ROUTE"):
            return False

        self.ensure_limiter()
        if not self.check_flight_position():
            return True
        if self.geofence_breached():
            self.trigger_landing("vehicle crossed planner-flight geofence")
            return True

        planner_reason = self.planner_health_reason()
        fault = planner_reason
        if fault is not None:
            self.latch_fault_hold(fault)
        else:
            self.clear_planner_fault()

        if self.state == "CLIMB_HOLD":
            self.publish_setpoint(self.hover_height, self.yaw0)
            if fault is not None:
                return True
            if self.auto_arm:
                altitude_ok = self.pos is not None and (
                    abs((-self.pos.z) - self.hover_height) <= self.reach_tol
                )
                horizontal_ok = (
                    self.horizontal_error is not None
                    and self.horizontal_error <= self.pre_route_max_horizontal_error
                )
                if altitude_ok and horizontal_ok:
                    self.get_logger().warn("stable hover reached; enabling route command")
                    self.set_state("ROUTE")
                elif self.t > self.climb_timeout:
                    self.trigger_landing(
                        "climb timeout without stable pre-route hover"
                    )
            elif self.t > self.climb_timeout:
                self.get_logger().warn("dry run: enabling route command without arming")
                self.set_state("ROUTE")
            return True

        if fault is None:
            self.update_route_command()
        else:
            self.freeze_yaw_target()
        self.publish_setpoint(self.hover_height, self.route_yaw())

        if self.goal_reached and fault is None:
            if self.goal_since is None:
                self.goal_since = self.monotonic_time()
            goal_elapsed = self.monotonic_time() - self.goal_since
            self.publish_route_status(
                f"GOAL_REACHED holding {goal_elapsed:.1f}/{self.goal_hold_time:.1f}s",
                "GOAL_REACHED",
            )
            if self.is_armed and goal_elapsed >= self.goal_hold_time:
                self.trigger_landing("planner goal reached")
        else:
            self.goal_since = None
            if fault is None:
                self.publish_route_status(self.route_status_text(), self.route_status_kind())
        return True

    def yaw_status_text(self):
        if not self.yaw_track or self.yaw_target is None:
            return " yaw=hold"
        error = self.yaw_track_error()
        text = f" yaw_target={math.degrees(self.yaw_target):.0f}deg"
        if error is not None:
            text += f" yaw_error={math.degrees(error):.0f}deg"
        return text

    def route_status_kind(self):
        return "YAW_ALIGN" if self.yaw_holding else "ROUTE"

    def route_status_text(self):
        if self.yaw_holding:
            return f"YAW_ALIGN translation paused while turning;{self.yaw_status_text()}"
        return (
            f"ROUTE valid displacement={math.hypot(*self.vio_displacement):.2f}m "
            f"command_speed={math.hypot(*self.limiter.velocity):.2f}m/s"
            f"{self.yaw_status_text()}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = OffboardGlobalPlanner()
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
