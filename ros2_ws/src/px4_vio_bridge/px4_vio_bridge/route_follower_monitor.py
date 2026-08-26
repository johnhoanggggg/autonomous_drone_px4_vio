"""Observation-only position route follower for the accepted global path."""

import json
import math
import time

import rclpy
from geometry_msgs.msg import Point, PointStamped, PoseStamped, Vector3Stamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32, Int32, String
from visualization_msgs.msg import Marker, MarkerArray

from px4_vio_bridge.path_follower import (
    CorrectionReplanGate,
    PositionRouteFollower,
    correction_rejection_reason,
    map_displacement_to_vio,
    requested_goal_reached,
    yaw_from_quaternion,
)
from px4_vio_bridge.grid_planner import GridMap, segment_has_clearance
from px4_vio_bridge.process_singleton import ProcessSingleton
from px4_vio_bridge.vio_to_px4_odometry import pose_rejection_reason


class RouteFollowerMonitor(Node):
    CONFIG_PARAMETERS = (
        "path_topic",
        "map_topic",
        "pose_topic",
        "raw_vio_topic",
        "goal_topic",
        "goal_terminal_topic",
        "correction_topic",
        "frame_id",
        "rate_hz",
        "path_timeout",
        "map_timeout",
        "pose_timeout",
        "vio_timeout",
        "correction_timeout",
        "max_correction_m",
        "max_correction_yaw_deg",
        "lookahead",
        "lookahead_step",
        "min_lookahead",
        "occupied_threshold",
        "robot_radius",
        "safety_margin",
        "max_carrot_speed",
        "max_carrot_acceleration",
        "max_cross_track",
        "cross_track_resume",
        "cross_track_recovery_time",
        "path_start_tolerance",
        "arrival_tolerance",
        "arrival_release_tolerance",
        "correction_translation_trigger",
        "correction_yaw_trigger_deg",
        "correction_filter_time_constant",
        "correction_material_translation",
        "correction_material_yaw_deg",
        "correction_settle_time",
        "correction_cooldown",
    )

    def __init__(self):
        super().__init__("route_follower_monitor")
        self.declare_parameter("path_topic", "/planner/path")
        self.declare_parameter("map_topic", "/rtabmap/grid")
        self.declare_parameter("pose_topic", "/rtabmap/pose")
        self.declare_parameter("raw_vio_topic", "/rtabmap/vio_pose")
        self.declare_parameter("goal_topic", "/waypoint/clicked")
        self.declare_parameter("goal_terminal_topic", "/planner/goal_terminal")
        self.declare_parameter("correction_topic", "/rtabmap/odom_correction")
        self.declare_parameter("frame_id", "world")
        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("path_timeout", 3.0)
        self.declare_parameter("map_timeout", 3.0)
        self.declare_parameter("pose_timeout", 1.0)
        self.declare_parameter("vio_timeout", 0.5)
        self.declare_parameter("correction_timeout", 1.0)
        self.declare_parameter("max_correction_m", 0.50)
        self.declare_parameter("max_correction_yaw_deg", 15.0)
        self.declare_parameter("lookahead", 0.60)
        self.declare_parameter("lookahead_step", 0.05)
        self.declare_parameter("min_lookahead", 0.05)
        self.declare_parameter("occupied_threshold", 65)
        self.declare_parameter("robot_radius", 0.25)
        self.declare_parameter("safety_margin", 0.05)
        self.declare_parameter("max_carrot_speed", 0.10)
        self.declare_parameter("max_carrot_acceleration", 0.30)
        self.declare_parameter("max_cross_track", 0.60)
        self.declare_parameter("cross_track_resume", 0.05)
        self.declare_parameter("cross_track_recovery_time", 1.0)
        self.declare_parameter("path_start_tolerance", 0.75)
        self.declare_parameter("arrival_tolerance", 0.12)
        self.declare_parameter("arrival_release_tolerance", 0.20)
        self.declare_parameter("correction_translation_trigger", 0.05)
        self.declare_parameter("correction_yaw_trigger_deg", 1.5)
        self.declare_parameter("correction_filter_time_constant", 0.35)
        self.declare_parameter("correction_material_translation", 0.03)
        self.declare_parameter("correction_material_yaw_deg", 0.75)
        self.declare_parameter("correction_settle_time", 0.40)
        self.declare_parameter("correction_cooldown", 8.0)

        self.frame_id = str(self.get_parameter("frame_id").value)
        self.path_timeout = float(self.get_parameter("path_timeout").value)
        self.map_timeout = float(self.get_parameter("map_timeout").value)
        self.pose_timeout = float(self.get_parameter("pose_timeout").value)
        self.vio_timeout = float(self.get_parameter("vio_timeout").value)
        self.correction_timeout = float(
            self.get_parameter("correction_timeout").value
        )
        self.max_correction_m = float(
            self.get_parameter("max_correction_m").value
        )
        self.max_correction_yaw = math.radians(
            float(self.get_parameter("max_correction_yaw_deg").value)
        )
        self.path_start_tolerance = float(self.get_parameter("path_start_tolerance").value)
        self.lookahead_step = max(
            0.01, float(self.get_parameter("lookahead_step").value)
        )
        self.min_lookahead = max(
            0.0, float(self.get_parameter("min_lookahead").value)
        )
        self.occupied_threshold = int(
            self.get_parameter("occupied_threshold").value
        )
        self.required_clearance = (
            float(self.get_parameter("robot_radius").value)
            + float(self.get_parameter("safety_margin").value)
        )
        self.correction_gate = CorrectionReplanGate(
            translation_trigger=float(
                self.get_parameter("correction_translation_trigger").value
            ),
            yaw_trigger=math.radians(
                float(self.get_parameter("correction_yaw_trigger_deg").value)
            ),
            filter_time_constant=float(
                self.get_parameter("correction_filter_time_constant").value
            ),
            material_translation=float(
                self.get_parameter("correction_material_translation").value
            ),
            material_yaw=math.radians(
                float(self.get_parameter("correction_material_yaw_deg").value)
            ),
            quiet_time=float(self.get_parameter("correction_settle_time").value),
            cooldown=float(self.get_parameter("correction_cooldown").value),
        )
        self.follower = PositionRouteFollower(
            lookahead=float(self.get_parameter("lookahead").value),
            max_carrot_speed=float(self.get_parameter("max_carrot_speed").value),
            max_carrot_acceleration=float(
                self.get_parameter("max_carrot_acceleration").value
            ),
            max_cross_track=float(self.get_parameter("max_cross_track").value),
            cross_track_resume=float(
                self.get_parameter("cross_track_resume").value
            ),
            cross_track_recovery_time=float(
                self.get_parameter("cross_track_recovery_time").value
            ),
            arrival_tolerance=float(self.get_parameter("arrival_tolerance").value),
            arrival_release_tolerance=float(
                self.get_parameter("arrival_release_tolerance").value
            ),
        )
        if self.required_clearance <= 0.0:
            raise ValueError("robot_radius + safety_margin must be positive")
        if self.min_lookahead > self.follower.lookahead:
            raise ValueError("min_lookahead must not exceed lookahead")

        self.pose = None
        self.pose_received = 0.0
        self.raw_vio_seen = False
        self.raw_vio_valid = False
        self.raw_vio_reason = ""
        self.raw_vio_received = 0.0
        self.correction_seen = False
        self.correction_valid = False
        self.correction_reason = ""
        self.correction_received = 0.0
        self.correction = None
        self.goal_terminal = False
        self.path_received = 0.0
        self.path_start = None
        self.grid = None
        self.map_received = 0.0
        self.last_tick = self.monotonic_time()

        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter("map_topic").value),
            self.on_map,
            map_qos,
        )
        self.create_subscription(
            Path, str(self.get_parameter("path_topic").value), self.on_path, 10
        )
        self.create_subscription(
            PoseStamped, str(self.get_parameter("pose_topic").value), self.on_pose, 10
        )
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter("raw_vio_topic").value),
            self.on_raw_vio,
            10,
        )
        self.create_subscription(
            PointStamped, str(self.get_parameter("goal_topic").value), self.on_goal, 10
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("goal_terminal_topic").value),
            self.on_goal_terminal,
            10,
        )
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter("correction_topic").value),
            self.on_correction,
            10,
        )
        self.carrot_pub = self.create_publisher(
            PoseStamped, "/planner/follower/carrot", 10
        )
        self.lookahead_pub = self.create_publisher(
            PoseStamped, "/planner/follower/lookahead", 10
        )
        self.displacement_pub = self.create_publisher(
            Vector3Stamped, "/planner/follower/displacement", 10
        )
        self.vio_displacement_pub = self.create_publisher(
            Vector3Stamped, "/planner/follower/vio_displacement", 10
        )
        self.status_pub = self.create_publisher(
            String, "/planner/follower/status", 10
        )
        self.valid_pub = self.create_publisher(
            Bool, "/planner/follower/valid", 10
        )
        self.goal_reached_pub = self.create_publisher(
            Bool, "/planner/follower/goal_reached", 10
        )
        self.progress_pub = self.create_publisher(
            Float32, "/planner/follower/progress", 10
        )
        self.path_progress_pub = self.create_publisher(
            Float32, "/planner/follower/path_progress", 10
        )
        self.remaining_pub = self.create_publisher(
            Float32, "/planner/follower/remaining", 10
        )
        self.cross_track_pub = self.create_publisher(
            Float32, "/planner/follower/cross_track", 10
        )
        self.generation_pub = self.create_publisher(
            Int32, "/planner/follower/path_generation", 10
        )
        self.markers_pub = self.create_publisher(
            MarkerArray, "/planner/follower/markers", 10
        )
        config_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.config_pub = self.create_publisher(
            String, "/planner/follower/config", config_qos
        )
        config = {
            name: self.get_parameter(name).value
            for name in self.CONFIG_PARAMETERS
        }
        self.config_pub.publish(
            String(data=json.dumps(config, sort_keys=True, separators=(",", ":")))
        )
        rate = max(1.0, float(self.get_parameter("rate_hz").value))
        self.create_timer(1.0 / rate, self.tick)
        self.get_logger().warn(
            "route_follower_monitor: OBSERVATION ONLY; position proposals only, "
            "publishes no PX4 commands"
        )

    @staticmethod
    def monotonic_time():
        return time.monotonic()

    def publish_status(self, status, valid=False, goal_reached=False):
        self.status_pub.publish(String(data=status))
        self.valid_pub.publish(Bool(data=valid))
        self.goal_reached_pub.publish(Bool(data=goal_reached))

    def on_pose(self, msg):
        if msg.header.frame_id != self.frame_id:
            return
        point = (float(msg.pose.position.x), float(msg.pose.position.y), float(msg.pose.position.z))
        if all(math.isfinite(value) for value in point):
            self.pose = point
            self.pose_received = self.monotonic_time()

    def on_map(self, msg):
        if msg.header.frame_id != self.frame_id:
            self.get_logger().error(
                f"map rejected: frame '{msg.header.frame_id}' != '{self.frame_id}'",
                throttle_duration_sec=5.0,
            )
            return
        q = msg.info.origin.orientation
        if (
            abs(q.x) > 1.0e-6
            or abs(q.y) > 1.0e-6
            or abs(q.z) > 1.0e-6
            or abs(q.w - 1.0) > 1.0e-6
        ):
            self.get_logger().error(
                "map rejected: rotated occupancy-grid origins are unsupported",
                throttle_duration_sec=5.0,
            )
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
        except ValueError as exc:
            self.get_logger().error(
                f"map rejected: {exc}", throttle_duration_sec=5.0
            )
            return
        self.map_received = self.monotonic_time()

    def command_has_clearance(self, start, end):
        return segment_has_clearance(
            self.grid,
            start,
            end,
            self.required_clearance,
            occupied_threshold=self.occupied_threshold,
        )

    def safe_lookahead(self, pose):
        projection = self.follower.path.project(pose)
        candidate = self.follower.lookahead
        while candidate + 1.0e-12 >= self.min_lookahead:
            target = self.follower.path.point_at(projection.along + candidate)
            if self.command_has_clearance(pose, target):
                return candidate
            candidate -= self.lookahead_step
        return None

    def on_raw_vio(self, msg):
        now = self.monotonic_time()
        position = msg.pose.position
        orientation = msg.pose.orientation
        reason = pose_rejection_reason(
            (float(position.x), float(position.y), float(position.z)),
            (
                float(orientation.w),
                float(orientation.x),
                float(orientation.y),
                float(orientation.z),
            ),
        )
        self.raw_vio_seen = True
        self.raw_vio_received = now
        self.raw_vio_valid = reason is None
        self.raw_vio_reason = reason or ""
        if reason:
            self.get_logger().error(
                f"raw VIO rejected: {reason}", throttle_duration_sec=1.0
            )

    def on_goal(self, msg):
        if msg.header.frame_id != self.frame_id:
            return
        # A new requested route has its own cumulative progress. Clear the old
        # path so it cannot be followed while the planner computes the new one.
        self.follower.reset_route_progress()
        self.follower.clear_path()
        self.path_start = None
        self.goal_terminal = False

    def on_goal_terminal(self, msg):
        self.goal_terminal = bool(msg.data)

    def on_path(self, msg):
        if msg.header.frame_id != self.frame_id:
            self.get_logger().warn(
                f"path rejected: frame '{msg.header.frame_id}' != '{self.frame_id}'",
                throttle_duration_sec=5.0,
            )
            return
        now = self.monotonic_time()
        self.path_received = now
        if not msg.poses:
            self.follower.clear_path()
            self.path_start = None
            return
        points = tuple(
            (float(pose.pose.position.x), float(pose.pose.position.y))
            for pose in msg.poses
        )
        anchor = self.pose[:2] if self.pose is not None else points[0]
        try:
            changed = self.follower.set_path(points, anchor)
        except ValueError as exc:
            self.get_logger().error(f"path rejected: {exc}")
            self.follower.clear_path()
            self.path_start = None
            return
        self.correction_gate.path_received(now)
        self.path_start = points[0]
        if changed:
            self.get_logger().info(
                f"accepted follower route generation {self.follower.generation}"
            )

    def on_correction(self, msg):
        now = self.monotonic_time()
        position = msg.pose.position
        orientation = msg.pose.orientation
        correction = (
            float(position.x), float(position.y), float(position.z),
            yaw_from_quaternion((orientation.w, orientation.x, orientation.y, orientation.z)),
        )
        reason = correction_rejection_reason(
            correction, self.max_correction_m, self.max_correction_yaw
        )
        self.correction_seen = True
        self.correction_received = now
        self.correction_valid = reason is None
        self.correction_reason = reason or ""
        if reason:
            self.get_logger().error(
                f"native correction rejected: {reason}", throttle_duration_sec=1.0
            )
            return
        self.correction = correction
        if self.correction_gate.observe(correction, now):
            translation, yaw = self.correction_gate.last_trigger_delta
            self.get_logger().warn(
                f"persistent map correction {translation:.3f}m/"
                f"{math.degrees(yaw):.2f}deg; coalescing optimization and "
                "waiting for one fresh path"
            )

    def pose_message(self, xy, z, stamp):
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = xy[0], xy[1], z
        msg.pose.orientation.w = 1.0
        return msg

    def publish_markers(self, result, stamp):
        array = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        array.markers.append(clear)
        for marker_id, namespace, point, color in (
            (0, "desired_lookahead", result.desired_carrot, (1.0, 0.7, 0.0, 1.0)),
            (1, "commanded_carrot", result.commanded_carrot, (0.0, 1.0, 1.0, 1.0)),
        ):
            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.header.stamp = stamp
            marker.ns = namespace
            marker.id = marker_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x, marker.pose.position.y = point
            marker.pose.position.z = self.pose[2]
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = marker.scale.z = 0.14
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
            array.markers.append(marker)
        line = Marker()
        line.header.frame_id = self.frame_id
        line.header.stamp = stamp
        line.ns = "commanded_displacement"
        line.id = 2
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = 0.025
        line.color.r, line.color.g, line.color.b, line.color.a = (0.0, 1.0, 1.0, 1.0)
        line.points = [
            Point(x=self.pose[0], y=self.pose[1], z=self.pose[2]),
            Point(x=result.commanded_carrot[0], y=result.commanded_carrot[1], z=self.pose[2]),
        ]
        array.markers.append(line)
        self.markers_pub.publish(array)

    def tick(self):
        now = self.monotonic_time()
        dt = max(1.0e-3, min(0.5, now - self.last_tick))
        self.last_tick = now
        if not self.raw_vio_seen:
            self.follower.interrupt_cross_track_recovery()
            self.publish_status("WAITING_FOR_VIO")
            return
        if not self.raw_vio_valid:
            self.follower.interrupt_cross_track_recovery()
            self.publish_status(f"INVALID_VIO reason={self.raw_vio_reason}")
            return
        if now - self.raw_vio_received > self.vio_timeout:
            self.follower.interrupt_cross_track_recovery()
            self.publish_status(f"STALE_VIO age={now - self.raw_vio_received:.2f}s")
            return
        if not self.correction_seen:
            self.follower.interrupt_cross_track_recovery()
            self.publish_status("WAITING_FOR_CORRECTION")
            return
        if not self.correction_valid:
            self.follower.interrupt_cross_track_recovery()
            self.publish_status(
                f"CORRECTION_REJECTED reason={self.correction_reason}"
            )
            return
        if now - self.correction_received > self.correction_timeout:
            self.follower.interrupt_cross_track_recovery()
            self.publish_status(
                f"STALE_CORRECTION age={now - self.correction_received:.2f}s"
            )
            return
        if self.pose is None:
            self.follower.interrupt_cross_track_recovery()
            self.publish_status("WAITING_FOR_POSE")
            return
        if now - self.pose_received > self.pose_timeout:
            self.follower.interrupt_cross_track_recovery()
            self.publish_status(f"STALE_POSE age={now - self.pose_received:.2f}s")
            return
        if self.grid is None:
            self.follower.interrupt_cross_track_recovery()
            self.publish_status("WAITING_FOR_MAP")
            return
        if now - self.map_received > self.map_timeout:
            self.follower.interrupt_cross_track_recovery()
            self.publish_status(f"STALE_MAP age={now - self.map_received:.2f}s")
            return
        if self.follower.path is None:
            self.follower.interrupt_cross_track_recovery()
            self.publish_status("WAITING_FOR_PATH")
            return
        if now - self.path_received > self.path_timeout:
            self.follower.interrupt_cross_track_recovery()
            self.publish_status(f"STALE_PATH age={now - self.path_received:.2f}s")
            return
        was_waiting_for_correction = self.correction_gate.pending
        if self.correction_gate.waiting(now):
            self.follower.interrupt_cross_track_recovery()
            self.publish_status("WAITING_FOR_POST_CORRECTION_PATH")
            return
        if was_waiting_for_correction:
            self.get_logger().info("map correction settled and fresh path received; follower resumed")
        start_error = math.dist(self.pose[:2], self.path_start)
        if start_error > self.path_start_tolerance:
            self.follower.interrupt_cross_track_recovery()
            self.publish_status(f"PATH_START_MISMATCH distance={start_error:.2f}m")
            return

        selected_lookahead = self.safe_lookahead(self.pose[:2])
        if selected_lookahead is None:
            self.follower.hold_command()
            pose_safe = self.command_has_clearance(self.pose[:2], self.pose[:2])
            reason = "NO_SAFE_LOOKAHEAD" if pose_safe else "POSE_INSIDE_CLEARANCE"
            self.publish_status(
                f"CLEARANCE_BLOCKED reason={reason} "
                f"required={self.required_clearance:.2f}m"
            )
            return

        result = self.follower.update(
            self.pose[:2],
            dt,
            lookahead=selected_lookahead,
            command_validator=lambda carrot: self.command_has_clearance(
                self.pose[:2], carrot
            ),
        )
        stamp = self.get_clock().now().to_msg()
        self.lookahead_pub.publish(
            self.pose_message(result.desired_carrot, self.pose[2], stamp)
        )
        self.carrot_pub.publish(
            self.pose_message(result.commanded_carrot, self.pose[2], stamp)
        )
        displacement = Vector3Stamped()
        displacement.header.stamp = stamp
        displacement.header.frame_id = self.frame_id
        displacement.vector.x, displacement.vector.y = result.commanded_displacement
        self.displacement_pub.publish(displacement)
        vio_displacement = Vector3Stamped()
        vio_displacement.header.stamp = stamp
        vio_displacement.header.frame_id = "vio"
        (
            vio_displacement.vector.x,
            vio_displacement.vector.y,
        ) = map_displacement_to_vio(
            result.commanded_displacement, self.correction[3]
        )
        self.vio_displacement_pub.publish(vio_displacement)
        self.progress_pub.publish(Float32(data=float(result.progress)))
        self.path_progress_pub.publish(Float32(data=float(result.path_progress)))
        self.remaining_pub.publish(Float32(data=float(result.remaining)))
        self.cross_track_pub.publish(Float32(data=float(result.cross_track)))
        self.generation_pub.publish(Int32(data=result.generation))
        self.publish_status(
            f"{result.status} generation={result.generation} progress={result.progress:.2f}m "
            f"path_progress={result.path_progress:.2f}m "
            f"remaining={result.remaining:.2f}m cross_track={result.cross_track:.2f}m "
            f"lookahead={selected_lookahead:.2f}/{self.follower.lookahead:.2f}m",
            valid=result.valid,
            goal_reached=requested_goal_reached(
                result.status, self.goal_terminal
            ),
        )
        self.publish_markers(result, stamp)


def main(args=None):
    try:
        singleton = ProcessSingleton("route_follower_monitor")
    except RuntimeError as exc:
        raise SystemExit(f"FATAL: {exc}") from None
    with singleton:
        rclpy.init(args=args)
        node = RouteFollowerMonitor()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == "__main__":
    main()
