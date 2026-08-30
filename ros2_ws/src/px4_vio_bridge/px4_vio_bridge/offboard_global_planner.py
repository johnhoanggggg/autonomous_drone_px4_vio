"""Position-only PX4 adapter for the validated global-planner follower.

This node rebases the follower's continuous-VIO-frame displacement from PX4's
current local position, advances the final command along the accepted map-frame
polyline, and validates that exact output against the raw occupancy map before
publishing a NED position setpoint. Invalid planner data latches a stationary
HOLD; persistent faults request AUTO.LAND.

The published yaw tracks the heading of that same commanded displacement, so
the airframe (and the forward-facing camera the VIO depends on) points along
the route. Translation pauses while a turn larger than yaw_align_error_deg
slews in, and the tracked heading is dropped whenever PX4 resets its own.
"""

import json
import math

import rclpy
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from nav_msgs.msg import OccupancyGrid, Path
from px4_msgs.msg import TrajectorySetpoint
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from px4_vio_bridge.grid_planner import GridMap, segment_has_clearance
from px4_vio_bridge.offboard_hover import OffboardHover, wrap_pi
from px4_vio_bridge.path_follower import (
    correction_rejection_reason,
    map_displacement_to_vio,
    yaw_from_quaternion,
)
from px4_vio_bridge.planner_flight import (
    HorizontalCommandLimiter,
    PathCommandLimiter,
    ned_track_heading,
    track_yaw_target,
    vio_displacement_to_map,
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
        self.declare_parameter("path_topic", "/planner/path")
        self.declare_parameter("map_topic", "/rtabmap/grid")
        self.declare_parameter("map_pose_topic", "/rtabmap/body_pose")
        self.declare_parameter("follower_config_topic", "/planner/follower/config")
        self.declare_parameter("frame_id", "world")
        self.declare_parameter("follower_timeout", 0.30)
        self.declare_parameter("correction_timeout", 1.0)
        self.declare_parameter("path_timeout", 3.0)
        self.declare_parameter("map_timeout", 3.0)
        self.declare_parameter("map_pose_timeout", 1.0)
        self.declare_parameter("planner_fault_land_time", 3.0)
        self.declare_parameter("goal_hold_time", 3.0)
        self.declare_parameter("max_follower_displacement", 1.0)
        self.declare_parameter("max_correction_m", 0.25)
        self.declare_parameter("max_correction_yaw_deg", 5.0)
        self.declare_parameter("command_speed", 0.10)
        self.declare_parameter("command_acceleration", 0.30)
        # Publish the limiter's own velocity as a PX4 feedforward instead of
        # making the position loop rediscover it. Default since the 2026-08-28
        # 03:5x runs; set false to fall back to the position-only command.
        self.declare_parameter("horizontal_feedforward", True)
        # Corner blending: carry speed through a bend at the speed that keeps
        # the airframe within junction_deviation of the vertex, instead of the
        # full stop-and-wait the limiter takes by default. Off by default -- it
        # removes a stop the route has always had, so it wants a deliberate
        # first flight. junction_deviation must stay inside the follower's
        # cross-track allowance, since that is what the vehicle is permitted
        # to cut off the corner.
        self.declare_parameter("corner_blending", False)
        self.declare_parameter("junction_deviation", 0.05)
        self.declare_parameter("path_command_projection_tolerance", 0.05)
        self.declare_parameter("path_command_entry_tolerance", 0.30)
        self.declare_parameter("path_command_connector_tolerance", 0.20)
        self.declare_parameter("path_command_suffix_tolerance", 0.01)
        self.declare_parameter("path_corner_tolerance", 0.05)
        self.declare_parameter("route_command_grace", 2.0)
        self.declare_parameter("replan_during_yaw_align", False)
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
        self.path_timeout = float(self.get_parameter("path_timeout").value)
        self.map_timeout = float(self.get_parameter("map_timeout").value)
        self.map_pose_timeout = float(
            self.get_parameter("map_pose_timeout").value
        )
        self.frame_id = str(self.get_parameter("frame_id").value)
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
        self.horizontal_feedforward = bool(
            self.get_parameter("horizontal_feedforward").value
        )
        self.limiter = HorizontalCommandLimiter(
            max_speed=float(self.get_parameter("command_speed").value),
            max_acceleration=float(
                self.get_parameter("command_acceleration").value
            ),
        )
        self.route_limiter = PathCommandLimiter(
            max_speed=self.limiter.max_speed,
            max_acceleration=self.limiter.max_acceleration,
            max_projection_error=float(
                self.get_parameter("path_command_projection_tolerance").value
            ),
            corner_tolerance=float(
                self.get_parameter("path_corner_tolerance").value
            ),
            max_entry_error=float(
                self.get_parameter("path_command_entry_tolerance").value
            ),
            max_connector_error=float(
                self.get_parameter("path_command_connector_tolerance").value
            ),
            suffix_tolerance=float(
                self.get_parameter("path_command_suffix_tolerance").value
            ),
            corner_blending=bool(self.get_parameter("corner_blending").value),
            junction_deviation=float(
                self.get_parameter("junction_deviation").value
            ),
        )
        self.route_command_grace = float(
            self.get_parameter("route_command_grace").value
        )
        self.replan_during_yaw_align = bool(
            self.get_parameter("replan_during_yaw_align").value
        )

        self.last_ff_velocity = None
        self.follower_valid = False
        self.follower_valid_received = None
        self.goal_reached = False
        self.goal_received = None
        self.vio_displacement = None
        self.displacement_received = None
        self.correction_valid = False
        self.correction_reason = "not received"
        self.correction_received = None
        self.correction = None
        self.path_points = None
        self.path_received = None
        self.grid = None
        self.map_received = None
        self.map_pose = None
        self.map_pose_received = None
        self.follower_config_received = None
        self.follower_required_clearance = None
        self.follower_occupied_threshold = None
        self.follower_command_speed = None
        self.follower_config_reason = "not received"
        self.planner_fault_since = None
        self.planner_fault_reason_text = ""
        self.route_command_stall_since = None
        self.route_command_holding = False
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
        self.path_topic = str(self.get_parameter("path_topic").value)
        self.map_topic = str(self.get_parameter("map_topic").value)
        self.map_pose_topic = str(self.get_parameter("map_pose_topic").value)
        self.follower_config_topic = str(
            self.get_parameter("follower_config_topic").value
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
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, self.map_topic, self.on_map, map_qos)
        self.create_subscription(Path, self.path_topic, self.on_path, 10)
        self.create_subscription(PoseStamped, self.map_pose_topic, self.on_map_pose, 10)
        self.create_subscription(
            String, self.follower_config_topic, self.on_follower_config, map_qos
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

    def on_follower_config(self, msg):
        self.follower_config_received = self.monotonic_time()
        try:
            config = json.loads(msg.data)
            if config.get("frame_id") != self.frame_id:
                raise ValueError(
                    f"frame_id {config.get('frame_id')!r} != {self.frame_id!r}"
                )
            radius = float(config["robot_radius"])
            margin = float(config["safety_margin"])
            threshold = int(config["occupied_threshold"])
            speed = float(config["max_carrot_speed"])
            if not all(math.isfinite(value) for value in (radius, margin, speed)):
                raise ValueError("clearance and speed values must be finite")
            if radius < 0.0 or margin < 0.0 or radius + margin <= 0.0:
                raise ValueError("robot_radius + safety_margin must be positive")
            if not 0 <= threshold <= 100 or speed <= 0.0:
                raise ValueError("occupied threshold or speed is invalid")
            self.follower_required_clearance = radius + margin
            self.follower_occupied_threshold = threshold
            self.follower_command_speed = speed
            self.follower_config_reason = ""
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            self.follower_required_clearance = None
            self.follower_occupied_threshold = None
            self.follower_command_speed = None
            self.follower_config_reason = str(exc)

    def on_path(self, msg):
        if msg.header.frame_id != self.frame_id:
            return
        points = tuple(
            (float(pose.pose.position.x), float(pose.pose.position.y))
            for pose in msg.poses
        )
        if points and all(math.isfinite(value) for point in points for value in point):
            self.path_points = points
            self.path_received = self.monotonic_time()

    def on_map_pose(self, msg):
        if msg.header.frame_id != self.frame_id:
            return
        point = float(msg.pose.position.x), float(msg.pose.position.y)
        if all(math.isfinite(value) for value in point):
            self.map_pose = point
            self.map_pose_received = self.monotonic_time()

    def on_map(self, msg):
        if msg.header.frame_id != self.frame_id:
            return
        q = msg.info.origin.orientation
        if (
            abs(q.x) > 1.0e-6
            or abs(q.y) > 1.0e-6
            or abs(q.z) > 1.0e-6
            or abs(q.w - 1.0) > 1.0e-6
        ):
            return
        try:
            values = tuple(int(value) for value in msg.data)
            if any(value < -1 or value > 100 for value in values):
                raise ValueError("values must be -1 or 0..100")
            self.grid = GridMap(
                int(msg.info.width),
                int(msg.info.height),
                float(msg.info.resolution),
                float(msg.info.origin.position.x),
                float(msg.info.origin.position.y),
                values,
            )
        except ValueError:
            return
        self.map_received = self.monotonic_time()

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
        self.correction = correction if reason is None else None
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
                self.route_limiter.clear()
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
            (self.follower_config_topic, "follower configuration"),
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
        if self.follower_config_received is None:
            return "follower configuration not received"
        if self.follower_config_reason:
            return f"follower configuration rejected: {self.follower_config_reason}"
        if abs(self.follower_command_speed - self.limiter.max_speed) > 1.0e-6:
            return (
                f"follower speed {self.follower_command_speed:.2f}m/s does not match "
                f"final command speed {self.limiter.max_speed:.2f}m/s"
            )
        for received, timeout, label in (
            (self.path_received, self.path_timeout, "path"),
            (self.map_received, self.map_timeout, "raw map"),
            (self.map_pose_received, self.map_pose_timeout, "map pose"),
        ):
            if received is None or now - received > timeout:
                return f"{label} stale for >{timeout:.2f}s"
        if self.path_points is None or self.grid is None or self.map_pose is None:
            return "path-clearance validation inputs are invalid"
        if self.displacement_received <= self.require_follower_after:
            return "waiting for follower data after PX4 local reset"
        if (
            self.correction_received is None
            or now - self.correction_received > self.correction_timeout
        ):
            return f"native correction stale for >{self.correction_timeout:.2f}s"
        if not self.correction_valid:
            return f"native correction rejected: {self.correction_reason}"
        if self.correction is None:
            return "native correction is invalid"
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
        z_sp, vz_sp = self.ramp_z(float(z_up))
        vx_sp, vy_sp = self.setpoint_velocity_xy()
        ax_sp, ay_sp = self.setpoint_acceleration_xy(vx_sp, vy_sp)
        msg = TrajectorySetpoint()
        msg.timestamp = self.now_us()
        msg.position = [
            float(self.limiter.position[0]),
            float(self.limiter.position[1]),
            float(-z_sp),
        ]
        msg.velocity = [vx_sp, vy_sp, vz_sp]
        msg.acceleration = [ax_sp, ay_sp, math.nan]
        msg.yaw = yaw_sp
        msg.yawspeed = yawspeed
        self.sp_pub.publish(msg)

    def setpoint_velocity_xy(self):
        """Horizontal feedforward for the published command, or NaN when off.

        `HorizontalCommandLimiter.velocity` is already the speed- and
        acceleration-limited velocity of the point being published, in the same
        NED frame, and by construction it is that point's own derivative
        (`update` advances position by `velocity * dt`; `adopt` takes the route
        limiter's velocity through the same map->VIO->NED transform as the
        position). Publishing it costs nothing to compute and is the value PX4
        would otherwise have to rediscover from position error alone.

        Why it matters (bag 20260828T021240Z): every accepted path replacement
        restarts the carrot from zero speed, and MPC_XY_P=0.95 on the few
        centimetres of position error that leaves asks for far less than the
        0.20 m/s the limiter actually wants. The route ran at 0.106 m/s
        effective against a 0.20 m/s cruise, in accelerate/coast/stall cycles
        every ~3.9 s. This is the same fix as ramp_z, one axis over.

        Gated to an advancing route on purpose. In a fault hold latch_fault_hold
        resets the limiter, but a command hold does not, and the base-class LAND
        state publishes through here too — in either case the last route
        velocity is stale and would keep pushing the vehicle. NaN restores pure
        position control for that tick.
        """
        if not self.horizontal_feedforward:
            return math.nan, math.nan
        if self.state != "ROUTE" or self.holding_for_fault or self.route_command_holding:
            return math.nan, math.nan
        return float(self.limiter.velocity[0]), float(self.limiter.velocity[1])

    def setpoint_acceleration_xy(self, vx, vy):
        """Acceleration feedforward: the derivative of the velocity just published.

        PX4 sums TrajectorySetpoint.acceleration into the velocity loop the same
        way it sums velocity into the position loop, so this completes the p/v/a
        triple and removes the last stage of lag from the command. It is safe to
        differentiate because the limiter has already acceleration-bounded that
        velocity -- this is reading back a quantity it computed, not amplifying
        noise from an estimate.

        Returns NaN whenever the velocity feedforward itself is NaN, so a hold
        or a LAND never carries a stale acceleration onto the wire.
        """
        if math.isnan(vx) or math.isnan(vy) or self.last_ff_velocity is None:
            self.last_ff_velocity = None if math.isnan(vx) else (vx, vy)
            return math.nan, math.nan
        limit = self.limiter.max_acceleration
        ax = (vx - self.last_ff_velocity[0]) / self.dt
        ay = (vy - self.last_ff_velocity[1]) / self.dt
        self.last_ff_velocity = (vx, vy)
        magnitude = math.hypot(ax, ay)
        if magnitude > limit and magnitude > 0.0:
            scale = limit / magnitude
            ax, ay = ax * scale, ay * scale
        return ax, ay

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
        # Never let an unvalidated route command advance internally while the
        # published command is holding. A recovery restarts from actual pose.
        self.route_limiter.clear()
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

    def hold_route_command(self, reason):
        """Hold the published command through a transient route-command reject.

        The accepted path stays installed and the last cleared setpoint stays on
        the wire, so ordinary replanning jitter cannot start the land timer.
        Only a stall that outlives the grace window becomes a flight fault.
        """
        now = self.monotonic_time()
        if self.route_command_stall_since is None:
            self.route_command_stall_since = now
        elapsed = now - self.route_command_stall_since
        if elapsed >= self.route_command_grace:
            return f"route command stalled {elapsed:.1f}s: {reason}"
        self.route_command_holding = True
        self.publish_route_status(
            f"COMMAND_HOLD {reason}; fault in "
            f"{max(0.0, self.route_command_grace - elapsed):.1f}s",
            "COMMAND_HOLD",
        )
        return None

    def clear_route_command_stall(self):
        self.route_command_stall_since = None

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

    def map_segment_has_clearance(self, start, end):
        """Occupancy test for a map-frame segment, at the follower's clearance."""
        return segment_has_clearance(
            self.grid,
            start,
            end,
            self.follower_required_clearance,
            occupied_threshold=self.follower_occupied_threshold,
        )

    def update_route_command(self):
        """Return `(fault, deferral)` for this tick's final command.

        `fault` is a flight fault and lands the aircraft on the usual timer.
        `deferral` is a transient the accepted route survives: the installed
        path and the last validated setpoint both stand, and the caller holds
        the command instead of starting that timer.
        """
        ned_displacement = vio_enu_displacement_to_ned(self.vio_displacement)
        self.update_yaw_target(ned_displacement)
        deferral = None

        # Replanning mid-slew is what generates unjoinable paths: translation is
        # paused, the vehicle drifts as it pivots, and each republished route
        # comes out shifted from the command that is standing still.  Keep the
        # accepted path until the nose is back on the leg.
        if self.yaw_holding and not self.replan_during_yaw_align:
            if self.route_limiter.path is None:
                return None, "waiting for yaw alignment before route entry"
        else:
            try:
                self.route_limiter.set_path(
                    self.path_points,
                    self.map_pose,
                    clearance_check=self.map_segment_has_clearance,
                )
            except (RuntimeError, ValueError) as exc:
                # An unjoinable replacement is not a flight fault.  The accepted
                # path is still installed and still validated, so keep flying it
                # and retry when the planner republishes.
                deferral = f"path replacement deferred: {exc}"

        if self.route_limiter.path is None:
            return None, deferral or "waiting for a joinable route path"

        restore_point = self.route_limiter.snapshot()
        try:
            map_displacement = vio_displacement_to_map(
                self.vio_displacement, self.correction[3]
            )
            desired_map = (
                self.map_pose[0] + map_displacement[0],
                self.map_pose[1] + map_displacement[1],
            )
            final_map = self.route_limiter.update(
                desired_map,
                self.dt,
                advance=not self.yaw_holding,
                reference_point=self.map_pose,
            )
        except (RuntimeError, ValueError) as exc:
            self.route_limiter.restore(restore_point)
            return None, f"path-constrained command rejected: {exc}"

        # This is deliberately after every limiting/projection operation: the
        # exact point and swept segment that PX4 will receive must be safe.
        # Rewinding leaves last tick's cleared command on the wire.
        if not self.map_segment_has_clearance(self.map_pose, final_map):
            self.route_limiter.restore(restore_point)
            return None, "post-limiter command has insufficient clearance"

        final_map_displacement = (
            final_map[0] - self.map_pose[0],
            final_map[1] - self.map_pose[1],
        )
        final_vio_displacement = map_displacement_to_vio(
            final_map_displacement, self.correction[3]
        )
        final_ned_displacement = vio_enu_displacement_to_ned(final_vio_displacement)
        final_ned = (
            float(self.pos.x) + final_ned_displacement[0],
            float(self.pos.y) + final_ned_displacement[1],
        )
        if math.dist(final_ned, (self.x0, self.y0)) > self.geofence_radius:
            # The vehicle itself is still inside; holding the last command is the
            # recovery.  An actual breach is caught by geofence_breached().
            self.route_limiter.restore(restore_point)
            return None, "path-constrained command lies outside flight geofence"

        velocity_vio = map_displacement_to_vio(
            self.route_limiter.velocity, self.correction[3]
        )
        velocity_ned = vio_enu_displacement_to_ned(velocity_vio)
        self.limiter.adopt(final_ned, velocity_ned)
        return None, deferral

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

        fault = self.planner_health_reason()

        if self.state == "CLIMB_HOLD":
            if fault is not None:
                self.latch_fault_hold(fault)
            else:
                self.clear_planner_fault()
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

        self.route_command_holding = False
        if fault is None:
            command_fault, deferral = self.update_route_command()
            if command_fault is not None:
                fault = command_fault
            elif deferral is not None:
                fault = self.hold_route_command(deferral)
            else:
                self.clear_route_command_stall()
        if fault is not None:
            self.latch_fault_hold(fault)
            self.freeze_yaw_target()
            self.clear_route_command_stall()
        else:
            self.clear_planner_fault()
        self.publish_setpoint(self.hover_height, self.route_yaw())

        if self.goal_reached and fault is None and not self.route_command_holding:
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
            if fault is None and not self.route_command_holding:
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
        if self.yaw_holding:
            return "YAW_ALIGN"
        if self.route_limiter.waiting_vertex is not None:
            return "CORNER_HOLD"
        return "ROUTE"

    def route_status_text(self):
        if self.yaw_holding:
            return f"YAW_ALIGN translation paused while turning;{self.yaw_status_text()}"
        if self.route_limiter.waiting_vertex is not None:
            vertex = self.route_limiter.path.point_at(
                self.route_limiter.waiting_vertex
            )
            return (
                "CORNER_HOLD final command waiting for vehicle "
                f"distance={math.dist(self.map_pose, vertex):.2f}m"
                f"{self.yaw_status_text()}"
            )
        path_offset = math.nan
        if self.route_limiter.path is not None and self.route_limiter.position is not None:
            path_offset = self.route_limiter.path.project(
                self.route_limiter.position
            ).cross_track
        return (
            f"ROUTE valid displacement={math.hypot(*self.vio_displacement):.2f}m "
            f"command_speed={math.hypot(*self.limiter.velocity):.2f}m/s "
            f"path_offset={path_offset:.3f}m"
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
