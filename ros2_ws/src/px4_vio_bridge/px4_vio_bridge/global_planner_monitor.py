"""Observation-only global costmap and repeated-A* monitor.

LEGACY. src/global_planner_node.cpp (``cpp_astar_planner``) is the flown
implementation. This module is kept for reference and for the parity tests that
still cover it; it does NOT have the 2026-08-29 stability work -- correction-
canonical accepted paths, goal-mode hysteresis, or the generation telemetry the
follower pairs its correction episodes against. Run the planner with
``cpp_astar:=true`` (implied by ``cpp_nodes:=true``).

This node publishes no PX4 topics. It plans in the loop-corrected RTAB-Map
``world`` frame and exposes both the latest candidate and the stable accepted
path for Foxglove inspection.
"""

import json
import math
import time

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32, Int32, String
from visualization_msgs.msg import Marker, MarkerArray

from px4_vio_bridge.grid_planner import (
    GridMap,
    astar,
    classify_goal,
    closest_reachable_goal,
    grid_lethal_radius,
    inflate_occupancy,
    inflation_display_data,
    line_is_clear,
    path_length,
    path_projection,
    recover_start,
    should_replace_path,
    simplify_path,
    segment_has_clearance,
    traversable,
    trim_path_to,
)
from px4_vio_bridge.process_singleton import ProcessSingleton


class GlobalPlannerMonitor(Node):
    CONFIG_PARAMETERS = (
        "map_topic",
        "pose_topic",
        "goal_topic",
        "frame_id",
        "rate_hz",
        "map_timeout",
        "pose_timeout",
        "occupied_threshold",
        "robot_radius",
        "safety_margin",
        "inflation_extra",
        "inflation_cost_scaling",
        "start_recovery_radius",
        "heuristic_weight",
        "cost_weight",
        "planning_timeout_ms",
        "switch_improvement",
        "path_retain_tolerance",
        "path_head_margin",
    )

    def __init__(self):
        super().__init__("global_planner_monitor")
        self.declare_parameter("map_topic", "/rtabmap/grid")
        self.declare_parameter("pose_topic", "/rtabmap/body_pose")
        self.declare_parameter("goal_topic", "/waypoint/clicked")
        self.declare_parameter("frame_id", "world")
        self.declare_parameter("rate_hz", 2.0)
        self.declare_parameter("map_timeout", 3.0)
        self.declare_parameter("pose_timeout", 1.0)
        self.declare_parameter("occupied_threshold", 65)
        self.declare_parameter("robot_radius", 0.25)
        self.declare_parameter("safety_margin", 0.05)
        self.declare_parameter("inflation_extra", 0.20)
        self.declare_parameter("inflation_cost_scaling", 3.0)
        self.declare_parameter("start_recovery_radius", 0.30)
        self.declare_parameter("heuristic_weight", 1.0)
        self.declare_parameter("cost_weight", 2.0)
        self.declare_parameter("planning_timeout_ms", 100.0)
        self.declare_parameter("switch_improvement", 0.10)
        # Distance the vehicle may sit off the accepted path before the path is
        # rebuilt. Keep it below the follower's max_cross_track so the planner
        # gives up on a corridor before the follower faults on it.
        self.declare_parameter("path_retain_tolerance", 0.35)
        # How far the retained path's head may fall behind the vehicle before it
        # is advanced onto it. Must stay under the follower's
        # path_start_tolerance, which rejects a path that starts too far away.
        self.declare_parameter("path_head_margin", 0.50)

        self.frame_id = str(self.get_parameter("frame_id").value)
        self.map_timeout = float(self.get_parameter("map_timeout").value)
        self.pose_timeout = float(self.get_parameter("pose_timeout").value)
        self.occupied_threshold = int(self.get_parameter("occupied_threshold").value)
        self.lethal_radius = (
            float(self.get_parameter("robot_radius").value)
            + float(self.get_parameter("safety_margin").value)
        )
        self.inflation_radius = self.lethal_radius + float(
            self.get_parameter("inflation_extra").value
        )
        self.inflation_cost_scaling = float(
            self.get_parameter("inflation_cost_scaling").value
        )
        self.start_recovery_radius = max(
            0.0, float(self.get_parameter("start_recovery_radius").value)
        )
        self.heuristic_weight = float(self.get_parameter("heuristic_weight").value)
        self.cost_weight = float(self.get_parameter("cost_weight").value)
        self.planning_timeout_ms = float(
            self.get_parameter("planning_timeout_ms").value
        )
        self.switch_improvement = float(
            self.get_parameter("switch_improvement").value
        )
        self.path_retain_tolerance = max(
            0.0, float(self.get_parameter("path_retain_tolerance").value)
        )
        self.path_head_margin = max(
            0.0, float(self.get_parameter("path_head_margin").value)
        )

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
            PoseStamped,
            str(self.get_parameter("pose_topic").value),
            self.on_pose,
            10,
        )
        self.create_subscription(
            PointStamped,
            str(self.get_parameter("goal_topic").value),
            self.on_goal,
            10,
        )

        self.path_pub = self.create_publisher(Path, "/planner/path", 10)
        self.candidate_pub = self.create_publisher(
            Path, "/planner/candidate_path", 10
        )
        self.inflated_pub = self.create_publisher(
            OccupancyGrid, "/planner/inflated_map", map_qos
        )
        self.markers_pub = self.create_publisher(
            MarkerArray, "/planner/markers", 10
        )
        self.status_pub = self.create_publisher(String, "/planner/status", 10)
        self.planning_ms_pub = self.create_publisher(
            Float32, "/planner/planning_ms", 10
        )
        self.path_length_pub = self.create_publisher(
            Float32, "/planner/path_length", 10
        )
        self.expanded_pub = self.create_publisher(
            Int32, "/planner/expanded_cells", 10
        )
        self.goal_exact_pub = self.create_publisher(
            Bool, "/planner/goal_exact", 10
        )
        self.goal_terminal_pub = self.create_publisher(
            Bool, "/planner/goal_terminal", 10
        )
        self.effective_goal_pub = self.create_publisher(
            PointStamped, "/planner/effective_goal", 10
        )
        # A flight recorder normally starts after this long-running planner.
        # TRANSIENT_LOCAL makes the effective launch overrides available to that
        # late subscriber, so every flight bag is self-describing.
        self.config_pub = self.create_publisher(
            String, "/planner/config", map_qos
        )
        config = {
            name: self.get_parameter(name).value
            for name in self.CONFIG_PARAMETERS
        }
        config.update({
            "lethal_radius": self.lethal_radius,
            "inflation_radius": self.inflation_radius,
        })
        self.config_pub.publish(
            String(data=json.dumps(config, sort_keys=True, separators=(",", ":")))
        )

        self.grid = None
        self.grid_msg = None
        self.grid_generation = 0
        self.inflated = None
        self.grid_lethal_radius = None
        self.grid_inflation_radius = None
        self.inflated_generation = -1
        self.pose = None
        self.goal = None
        self.goal_generation = 0
        self.map_received = 0.0
        self.pose_received = 0.0
        self.accepted_points = ()
        self.accepted_goal_generation = -1
        self.accepted_effective_goal = None
        self.accepted_generation = 0
        self.last_planned_key = None
        rate = max(0.2, float(self.get_parameter("rate_hz").value))
        self.create_timer(1.0 / rate, self.tick)
        self.get_logger().warn(
            "global_planner_monitor: OBSERVATION ONLY; publishes no PX4 commands. "
            f"continuous_clearance={self.lethal_radius:.2f}m unknown=blocked"
        )

    @staticmethod
    def monotonic_time():
        return time.monotonic()

    def publish_status(self, text):
        self.status_pub.publish(String(data=text))

    def on_map(self, msg):
        if msg.header.frame_id != self.frame_id:
            self.get_logger().error(
                f"map rejected: frame '{msg.header.frame_id}' != '{self.frame_id}'",
                throttle_duration_sec=5.0,
            )
            return
        q = msg.info.origin.orientation
        if abs(q.x) > 1e-6 or abs(q.y) > 1e-6 or abs(q.z) > 1e-6 or abs(q.w - 1.0) > 1e-6:
            self.get_logger().error(
                "map rejected: rotated occupancy-grid origins are unsupported",
                throttle_duration_sec=5.0,
            )
            return
        try:
            values = tuple(int(value) for value in msg.data)
            if any(value < -1 or value > 100 for value in values):
                raise ValueError("values must be -1 or 0..100")
            grid = GridMap(
                int(msg.info.width),
                int(msg.info.height),
                float(msg.info.resolution),
                float(msg.info.origin.position.x),
                float(msg.info.origin.position.y),
                values,
            )
        except (TypeError, ValueError) as exc:
            self.get_logger().error(f"map rejected: {exc}")
            return
        self.grid = grid
        self.grid_msg = msg
        self.grid_generation += 1
        self.map_received = self.monotonic_time()

    def on_pose(self, msg):
        if msg.header.frame_id != self.frame_id:
            return
        self.pose = (float(msg.pose.position.x), float(msg.pose.position.y), float(msg.pose.position.z))
        self.pose_received = self.monotonic_time()

    def on_goal(self, msg):
        if msg.header.frame_id != self.frame_id:
            self.get_logger().warn(
                f"goal rejected: frame '{msg.header.frame_id}' != '{self.frame_id}'"
            )
            return
        goal = (float(msg.point.x), float(msg.point.y))
        if not all(math.isfinite(value) for value in goal):
            self.get_logger().warn("goal rejected: coordinates must be finite")
            return
        self.goal = goal
        self.goal_generation += 1
        self.get_logger().warn(f"planner goal=({self.goal[0]:.2f}, {self.goal[1]:.2f})")

    def ensure_inflated(self):
        if self.inflated_generation == self.grid_generation:
            return
        self.grid_lethal_radius = grid_lethal_radius(
            self.lethal_radius, self.grid.resolution
        )
        self.grid_inflation_radius = grid_lethal_radius(
            self.inflation_radius, self.grid.resolution
        )
        self.inflated = inflate_occupancy(
            self.grid,
            occupied_threshold=self.occupied_threshold,
            lethal_radius=self.grid_lethal_radius,
            inflation_radius=self.grid_inflation_radius,
            cost_scaling=self.inflation_cost_scaling,
        )
        self.inflated_generation = self.grid_generation
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.info = self.grid_msg.info
        msg.info.map_load_time = msg.header.stamp
        msg.data = inflation_display_data(
            self.grid,
            self.inflated,
            occupied_threshold=self.occupied_threshold,
        )
        self.inflated_pub.publish(msg)

    def path_valid(self, points):
        if not points:
            return False
        cells = [self.inflated.world_to_cell(point) for point in points]
        if any(cell is None for cell in cells):
            return False
        if len(points) == 1:
            return segment_has_clearance(
                self.grid,
                points[0],
                points[0],
                self.lethal_radius,
                occupied_threshold=self.occupied_threshold,
            )
        return all(
            line_is_clear(self.inflated, a, b)
            and segment_has_clearance(
                self.grid,
                first,
                second,
                self.lethal_radius,
                occupied_threshold=self.occupied_threshold,
            )
            for a, b, first, second in zip(
                cells, cells[1:], points, points[1:]
            )
        )

    def make_path(self, points, z):
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        for x, y in points:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        return msg

    def clear_path(self, status, effective_goal=None):
        self.accepted_points = ()
        self.accepted_effective_goal = None
        empty = self.make_path((), self.pose[2] if self.pose else 0.0)
        self.candidate_pub.publish(empty)
        self.path_pub.publish(empty)
        self.goal_exact_pub.publish(Bool(data=False))
        self.goal_terminal_pub.publish(Bool(data=False))
        self.publish_status(status)
        self.publish_markers(status, (), effective_goal)

    def publish_markers(self, status, candidate_points, effective_goal=None):
        now = self.get_clock().now().to_msg()
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        def sphere(marker_id, namespace, xyz, color):
            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.header.stamp = now
            marker.ns = namespace
            marker.id = marker_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = xyz
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = marker.scale.z = 0.16
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
            return marker

        if self.pose is not None:
            markers.markers.append(sphere(0, "planner_start", self.pose, (0.1, 1.0, 0.1, 1.0)))
        if self.goal is not None:
            z = self.pose[2] if self.pose else 0.0
            markers.markers.append(sphere(1, "planner_goal", (*self.goal, z), (1.0, 1.0, 1.0, 1.0)))
        if effective_goal is not None:
            z = self.pose[2] if self.pose else 0.0
            markers.markers.append(
                sphere(
                    3,
                    "planner_effective_goal",
                    (*effective_goal, z),
                    (1.0, 0.5, 0.0, 1.0),
                )
            )
        text = Marker()
        text.header.frame_id = self.frame_id
        text.header.stamp = now
        text.ns = "planner_status"
        text.id = 2
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        if self.pose:
            text.pose.position.x, text.pose.position.y = self.pose[:2]
            text.pose.position.z = self.pose[2] + 0.35
        text.pose.orientation.w = 1.0
        text.scale.z = 0.12
        text.color.r = text.color.g = text.color.b = text.color.a = 1.0
        text.text = status
        markers.markers.append(text)
        self.markers_pub.publish(markers)

    def tick(self):
        now = self.monotonic_time()
        if self.grid is None:
            self.publish_status("WAITING_FOR_MAP")
            return
        if now - self.map_received > self.map_timeout:
            self.publish_status(f"STALE_MAP age={now - self.map_received:.2f}s")
            return
        # Inflation is a property of the map, not of a requested route.  Keep
        # its visualization current before a goal exists so clicking in
        # Foxglove does not appear to turn nearby free space into obstacles.
        self.ensure_inflated()
        if self.pose is None or now - self.pose_received > self.pose_timeout:
            self.publish_status("WAITING_FOR_POSE")
            return
        if self.goal is None:
            self.publish_status("WAITING_FOR_GOAL")
            return
        start = self.inflated.world_to_cell(self.pose[:2])
        if start is None:
            self.clear_path("START_OUTSIDE_MAP")
            return
        planning_started = time.monotonic()
        # Brushing past an obstacle puts the vehicle inside its own lethal
        # inflation.  Dropping the route for that would arm the flight adapter's
        # land timer while the aircraft is merely close to something, so shift
        # the search start to the nearest cell outside the envelope instead and
        # keep the offset visible in the status line.
        start_offset = 0.0
        if not traversable(self.inflated, start):
            recovery = recover_start(
                self.grid,
                self.inflated,
                start,
                occupied_threshold=self.occupied_threshold,
                max_radius=self.start_recovery_radius,
            )
            if recovery is None:
                self.clear_path("START_BLOCKED")
                return
            start = recovery.cell
            start_offset = recovery.distance
        selection = closest_reachable_goal(self.inflated, start, self.goal)
        if selection is None:
            self.clear_path("START_BLOCKED")
            return
        goal = selection.cell
        exact, terminal = classify_goal(
            self.grid, self.inflated, self.goal, goal
        )
        self.goal_exact_pub.publish(Bool(data=exact))
        self.goal_terminal_pub.publish(Bool(data=terminal))
        effective_goal = self.inflated.cell_center(goal)
        effective_msg = PointStamped()
        effective_msg.header.stamp = self.get_clock().now().to_msg()
        effective_msg.header.frame_id = self.frame_id
        effective_msg.point.x, effective_msg.point.y = effective_goal
        effective_msg.point.z = self.pose[2]
        self.effective_goal_pub.publish(effective_msg)
        key = (self.grid_generation, self.goal_generation, start, goal)
        if key == self.last_planned_key:
            return
        self.last_planned_key = key
        selection_ms = (time.monotonic() - planning_started) * 1000.0
        remaining_ms = self.planning_timeout_ms - selection_ms
        if self.planning_timeout_ms > 0.0 and remaining_ms <= 0.0:
            self.planning_ms_pub.publish(Float32(data=float(selection_ms)))
            self.expanded_pub.publish(Int32(data=selection.reachable_cells))
            self.clear_path(
                f"TIMEOUT reachable={selection.reachable_cells} "
                f"plan={selection_ms:.1f}ms",
                effective_goal,
            )
            return
        result = astar(
            self.inflated,
            start,
            goal,
            heuristic_weight=self.heuristic_weight,
            cost_weight=self.cost_weight,
            timeout_ms=max(0.0, remaining_ms),
        )
        total_ms = (time.monotonic() - planning_started) * 1000.0
        self.planning_ms_pub.publish(Float32(data=float(total_ms)))
        self.expanded_pub.publish(Int32(data=result.expanded))
        if not result.found:
            self.accepted_points = ()
            self.path_pub.publish(self.make_path((), self.pose[2]))
            status = (
                f"{result.reason} expanded={result.expanded} "
                f"plan={total_ms:.1f}ms"
            )
            self.publish_status(status)
            self.publish_markers(status, (), effective_goal)
            return

        candidate_cells = simplify_path(
            self.inflated,
            result.cells,
            preserve_cost=True,
            source_grid=self.grid,
            required_clearance=self.lethal_radius,
            occupied_threshold=self.occupied_threshold,
        )
        if not candidate_cells:
            self.clear_path(
                "UNSAFE_SEGMENT continuous clearance validation failed",
                effective_goal,
            )
            return
        candidate_points = tuple(self.inflated.cell_center(cell) for cell in candidate_cells)
        candidate_length = path_length(candidate_points)
        self.candidate_pub.publish(self.make_path(candidate_points, self.pose[2]))
        accepted_valid = self.path_valid(self.accepted_points)
        projection = (
            path_projection(self.accepted_points, self.pose[:2])
            if accepted_valid
            else None
        )
        goal_changed = self.accepted_goal_generation != self.goal_generation
        effective_goal_changed = self.accepted_effective_goal != goal
        replace = (
            goal_changed
            or effective_goal_changed
            or should_replace_path(
                projection,
                candidate_length,
                retain_tolerance=self.path_retain_tolerance,
                switch_improvement=self.switch_improvement,
            )
        )
        if replace:
            self.accepted_points = candidate_points
            self.accepted_goal_generation = self.goal_generation
            self.accepted_effective_goal = goal
            self.accepted_generation += 1
        else:
            self.accepted_points = trim_path_to(
                self.accepted_points, self.pose[:2], self.path_head_margin
            )
        self.path_pub.publish(self.make_path(self.accepted_points, self.pose[2]))
        accepted_length = path_length(self.accepted_points)
        self.path_length_pub.publish(Float32(data=float(accepted_length)))
        mode = (
            "PATH_VALID"
            if exact
            else ("SAFE_APPROACH" if terminal else "EXPLORING")
        )
        status = (
            f"{mode} length={accepted_length:.2f}m candidate={candidate_length:.2f}m "
            f"goal_distance={selection.distance:.2f}m "
            f"reachable={selection.reachable_cells} expanded={result.expanded} "
            f"plan={total_ms:.1f}ms "
            f"map_age={now - self.map_received:.2f}s "
            f"path_gen={self.accepted_generation}"
        )
        if projection is not None:
            status += f" off_path={projection.distance:.2f}m"
        if start_offset > 0.0:
            status += f" start_recovered={start_offset:.2f}m"
        self.publish_status(status)
        self.publish_markers(status, candidate_points, effective_goal)


def main(args=None):
    try:
        singleton = ProcessSingleton("global_planner_monitor")
    except RuntimeError as exc:
        raise SystemExit(f"FATAL: {exc}") from None
    with singleton:
        rclpy.init(args=args)
        node = GlobalPlannerMonitor()
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
