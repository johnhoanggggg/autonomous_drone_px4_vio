"""Observation-only position route follower for the accepted global path."""

import math
import time

import rclpy
from geometry_msgs.msg import Point, PointStamped, PoseStamped, Vector3Stamped
from nav_msgs.msg import Path
from rclpy.node import Node
from std_msgs.msg import Float32, Int32, String
from visualization_msgs.msg import Marker, MarkerArray

from px4_vio_bridge.map_correction import yaw_from_quaternion
from px4_vio_bridge.path_follower import CorrectionReplanGate, PositionRouteFollower


class RouteFollowerMonitor(Node):
    def __init__(self):
        super().__init__("route_follower_monitor")
        self.declare_parameter("path_topic", "/planner/path")
        self.declare_parameter("pose_topic", "/rtabmap/pose")
        self.declare_parameter("goal_topic", "/waypoint/clicked")
        self.declare_parameter("correction_topic", "/vio/map_correction_target")
        self.declare_parameter("frame_id", "world")
        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("path_timeout", 3.0)
        self.declare_parameter("pose_timeout", 1.0)
        self.declare_parameter("lookahead", 0.60)
        self.declare_parameter("max_carrot_speed", 0.25)
        self.declare_parameter("max_carrot_acceleration", 0.50)
        self.declare_parameter("max_cross_track", 0.60)
        self.declare_parameter("path_start_tolerance", 0.75)
        self.declare_parameter("arrival_tolerance", 0.12)
        self.declare_parameter("correction_translation_trigger", 0.05)
        self.declare_parameter("correction_yaw_trigger_deg", 1.5)
        self.declare_parameter("correction_filter_time_constant", 0.35)
        self.declare_parameter("correction_material_translation", 0.03)
        self.declare_parameter("correction_material_yaw_deg", 0.75)
        self.declare_parameter("correction_settle_time", 0.40)
        self.declare_parameter("correction_cooldown", 8.0)

        self.frame_id = str(self.get_parameter("frame_id").value)
        self.path_timeout = float(self.get_parameter("path_timeout").value)
        self.pose_timeout = float(self.get_parameter("pose_timeout").value)
        self.path_start_tolerance = float(self.get_parameter("path_start_tolerance").value)
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
            arrival_tolerance=float(self.get_parameter("arrival_tolerance").value),
        )

        self.pose = None
        self.pose_received = 0.0
        self.path_received = 0.0
        self.path_start = None
        self.last_tick = self.monotonic_time()

        self.create_subscription(
            Path, str(self.get_parameter("path_topic").value), self.on_path, 10
        )
        self.create_subscription(
            PoseStamped, str(self.get_parameter("pose_topic").value), self.on_pose, 10
        )
        self.create_subscription(
            PointStamped, str(self.get_parameter("goal_topic").value), self.on_goal, 10
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
        self.status_pub = self.create_publisher(
            String, "/planner/follower/status", 10
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
        rate = max(1.0, float(self.get_parameter("rate_hz").value))
        self.create_timer(1.0 / rate, self.tick)
        self.get_logger().warn(
            "route_follower_monitor: OBSERVATION ONLY; position proposals only, "
            "publishes no PX4 commands"
        )

    @staticmethod
    def monotonic_time():
        return time.monotonic()

    def publish_status(self, status):
        self.status_pub.publish(String(data=status))

    def on_pose(self, msg):
        if msg.header.frame_id != self.frame_id:
            return
        point = (float(msg.pose.position.x), float(msg.pose.position.y), float(msg.pose.position.z))
        if all(math.isfinite(value) for value in point):
            self.pose = point
            self.pose_received = self.monotonic_time()

    def on_goal(self, msg):
        if msg.header.frame_id != self.frame_id:
            return
        # A new requested route has its own cumulative progress. Clear the old
        # path so it cannot be followed while the planner computes the new one.
        self.follower.reset_route_progress()
        self.follower.clear_path()
        self.path_start = None

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
        position = msg.pose.position
        orientation = msg.pose.orientation
        correction = (
            float(position.x), float(position.y), float(position.z),
            yaw_from_quaternion((orientation.w, orientation.x, orientation.y, orientation.z)),
        )
        if not all(math.isfinite(value) for value in correction):
            return
        if self.correction_gate.observe(correction, self.monotonic_time()):
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
        if self.pose is None:
            self.publish_status("WAITING_FOR_POSE")
            return
        if now - self.pose_received > self.pose_timeout:
            self.publish_status(f"STALE_POSE age={now - self.pose_received:.2f}s")
            return
        if self.follower.path is None:
            self.publish_status("WAITING_FOR_PATH")
            return
        if now - self.path_received > self.path_timeout:
            self.publish_status(f"STALE_PATH age={now - self.path_received:.2f}s")
            return
        was_waiting_for_correction = self.correction_gate.pending
        if self.correction_gate.waiting(now):
            self.publish_status("WAITING_FOR_POST_CORRECTION_PATH")
            return
        if was_waiting_for_correction:
            self.get_logger().info("map correction settled and fresh path received; follower resumed")
        start_error = math.dist(self.pose[:2], self.path_start)
        if start_error > self.path_start_tolerance:
            self.publish_status(f"PATH_START_MISMATCH distance={start_error:.2f}m")
            return

        result = self.follower.update(self.pose[:2], dt)
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
        self.progress_pub.publish(Float32(data=float(result.progress)))
        self.path_progress_pub.publish(Float32(data=float(result.path_progress)))
        self.remaining_pub.publish(Float32(data=float(result.remaining)))
        self.cross_track_pub.publish(Float32(data=float(result.cross_track)))
        self.generation_pub.publish(Int32(data=result.generation))
        self.publish_status(
            f"{result.status} generation={result.generation} progress={result.progress:.2f}m "
            f"path_progress={result.path_progress:.2f}m "
            f"remaining={result.remaining:.2f}m cross_track={result.cross_track:.2f}m"
        )
        self.publish_markers(result, stamp)


def main(args=None):
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
