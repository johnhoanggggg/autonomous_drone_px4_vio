"""Run VFH2D against the live obstacle cloud and publish what it decides.

**This node cannot move the vehicle.** It publishes no `/fmu/in/*` topic and
holds no PX4 state — it only watches `/rtabmap/obstacle_cloud` and
`/rtabmap/pose`, runs the planner, and publishes the result for Foxglove. That
makes it the first step in testing VFH on this drone: carry the drone around by
hand, or hover it with `offboard_hover` in another terminal, and watch whether
the histogram and the chosen direction agree with the room.

    ros2 run px4_vio_bridge vfh_monitor

Requires the stack to be running with clouds enabled, which is NOT the default:

    ros2 launch px4_vio_bridge rtabmap_slam_px4.launch.py slam_publish_clouds:=true

Topics out (the full set is documented in `vfh_telemetry.py` and README; the
ones worth a panel):

| topic | type | Foxglove panel |
|---|---|---|
| `/vfh/markers` | `MarkerArray` | **3D** — the histogram fan, chosen arrow, candidates, label |
| `/vfh/samples` | `PointCloud2` | **3D** — the points the planner actually used |
| `/vfh/status` | `String` | Raw Message — one line, everything |
| `/vfh/blocked` | `Int32` | Indicator (0 clear, 1 blocked) |
| `/vfh/nearest` | `Float32` | Gauge — closest return, metres |
| `/vfh/heading_deg`, `/vfh/direction_heading_deg` | `Float32` | Plot — where it points vs where it wants to go |
| `/vfh/direction_deg`, `/vfh/goal_bearing_deg` | `Float32` | Plot — the same two, relative to the nose |
| `/vfh/opening_width_deg` | `Float32` | Gauge — how much room the chosen gap has |
| `/vfh/histogram`, `/vfh/binary` | `Float32MultiArray` | Plot — density and blocked/free |

A goal is optional: publish a `PointStamped` on `/waypoint/clicked` from the
Foxglove 3D panel's Publish tool (the bridge already whitelists that topic) and
the planner will steer toward it. With no goal it just answers "can I keep going
forward, and if not, which way should I turn".
"""
import math
import time

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from std_msgs.msg import Int32, String

from px4_vio_bridge.vfh2d import (
    Vfh2D,
    VfhConfig,
    histogram_bar,
    relative_bearing_enu,
    wrap_pi,
)
from px4_vio_bridge.vfh_obstacles import ObstacleField
from px4_vio_bridge.vfh_telemetry import DIM, VfhTelemetry


class VfhMonitor(Node):
    def __init__(self, node_name="vfh_monitor"):
        super().__init__(node_name)

        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("cloud_topic", "/rtabmap/obstacle_cloud")
        # The SLAM pose, not the raw VIO pose: it is the frame the cloud is
        # expressed in, so measuring against it keeps obstacle geometry
        # consistent across a loop closure.
        self.declare_parameter("pose_topic", "/rtabmap/pose")
        self.declare_parameter("goal_topic", "/waypoint/clicked")
        self.declare_parameter("goal_frame", "world")
        self.declare_parameter("obstacle_timeout", 1.0)
        self.declare_parameter("log_period", 1.0)

        # VFH tunables, all exposed because tuning them against a live histogram
        # is the entire point of this node.
        self.declare_parameter("sectors", 72)
        self.declare_parameter("min_range", 0.25)
        self.declare_parameter("max_range", 2.0)
        self.declare_parameter("min_points", 4)
        self.declare_parameter("tau_high", 6.0)
        self.declare_parameter("tau_low", 3.0)
        self.declare_parameter("smoothing", 3)
        self.declare_parameter("robot_radius", 0.30)
        self.declare_parameter("safety_margin", 0.10)
        self.declare_parameter("max_steer_deg", 35.0)
        self.declare_parameter("display_fov_deg", 90.0)
        self.declare_parameter("wide_valley_deg", 40.0)
        self.declare_parameter("mu_target", 5.0)
        self.declare_parameter("mu_heading", 2.0)
        self.declare_parameter("mu_previous", 2.0)
        self.declare_parameter("z_below", 0.15)   # MUST stay below hover_height
        self.declare_parameter("z_above", 0.60)
        self.declare_parameter("max_samples", 1200)
        self.declare_parameter("memory_duration", 30.0)
        self.declare_parameter("memory_voxel_size", 0.10)
        self.declare_parameter("memory_max_points", 20000)
        self.declare_parameter(
            "memory_correction_topic", "/rtabmap/odom_correction"
        )
        self.declare_parameter("memory_reset_correction_m", 0.05)
        self.declare_parameter("memory_reset_correction_deg", 2.0)

        self.config = self.build_config()
        self.vfh = Vfh2D(self.config)
        self.goal_frame = str(self.get_parameter("goal_frame").value)
        self.obstacle_timeout = float(self.get_parameter("obstacle_timeout").value)
        self.log_period = float(self.get_parameter("log_period").value)
        self.display_fov = math.radians(
            max(0.0, min(180.0, float(self.get_parameter("display_fov_deg").value)))
        )

        self.obstacles = ObstacleField(
            self,
            cloud_topic=str(self.get_parameter("cloud_topic").value),
            pose_topic=str(self.get_parameter("pose_topic").value),
            min_range=self.config.min_range,
            max_range=self.config.max_range,
            z_below=float(self.get_parameter("z_below").value),
            z_above=float(self.get_parameter("z_above").value),
            max_samples=int(self.get_parameter("max_samples").value),
            memory_duration=float(self.get_parameter("memory_duration").value),
            memory_voxel_size=float(
                self.get_parameter("memory_voxel_size").value
            ),
            memory_max_points=int(self.get_parameter("memory_max_points").value),
            memory_correction_topic=str(
                self.get_parameter("memory_correction_topic").value
            ),
            memory_reset_correction_m=float(
                self.get_parameter("memory_reset_correction_m").value
            ),
            memory_reset_correction_deg=float(
                self.get_parameter("memory_reset_correction_deg").value
            ),
        )

        self.create_subscription(
            PointStamped, str(self.get_parameter("goal_topic").value), self.on_goal, 10
        )

        # Markers, sample cloud and the scalar topics; shared with offboard_vfh
        # so the display is identical whether or not the vehicle is flying.
        self.telemetry = VfhTelemetry(self, frame_id=self.goal_frame)
        self.status_pub = self.create_publisher(String, "/vfh/status", 10)
        self.blocked_pub = self.create_publisher(Int32, "/vfh/blocked", 10)

        self.goal_enu = None
        self.previous_absolute = None   # last chosen direction as an ENU heading
        self.last_log = 0.0

        rate = float(self.get_parameter("rate_hz").value)
        self.dt = 1.0 / max(1.0, rate)
        self.timer = self.create_timer(self.dt, self.tick)
        self.get_logger().warn(
            f"vfh_monitor: OBSERVATION ONLY, publishes nothing to PX4. "
            f"{self.config.sectors} sectors, range "
            f"{self.config.min_range:.2f}-{self.config.max_range:.2f} m, "
            f"steer limit +/-{math.degrees(self.config.max_steer):.0f} deg, "
            f"Foxglove fan +/-{math.degrees(self.display_fov):.0f} deg, "
            f"obstacle memory "
            f"{float(self.get_parameter('memory_duration').value):.0f}s"
        )

    def monotonic_time(self):
        return time.monotonic()

    def build_config(self):
        def value(name, cast=float):
            return cast(self.get_parameter(name).value)

        return VfhConfig(
            sectors=value("sectors", int),
            min_range=value("min_range"),
            max_range=value("max_range"),
            min_points=value("min_points", int),
            tau_high=value("tau_high"),
            tau_low=value("tau_low"),
            smoothing=value("smoothing", int),
            robot_radius=value("robot_radius"),
            safety_margin=value("safety_margin"),
            max_steer=math.radians(value("max_steer_deg")),
            wide_valley=math.radians(value("wide_valley_deg")),
            mu_target=value("mu_target"),
            mu_heading=value("mu_heading"),
            mu_previous=value("mu_previous"),
        )

    # --- goal --------------------------------------------------------------
    def on_goal(self, msg):
        if msg.header.frame_id != self.goal_frame:
            self.get_logger().warn(
                f"goal rejected: frame_id '{msg.header.frame_id}' != "
                f"'{self.goal_frame}'"
            )
            return
        self.goal_enu = (float(msg.point.x), float(msg.point.y))
        self.get_logger().warn(
            f"goal set: enu=({self.goal_enu[0]:.2f}, {self.goal_enu[1]:.2f})"
        )

    def target_bearing(self):
        """Where the goal sits relative to the nose, or 0 (ahead) with no goal."""
        if self.goal_enu is None or self.obstacles.origin is None:
            return 0.0, None
        dx = self.goal_enu[0] - self.obstacles.origin[0]
        dy = self.goal_enu[1] - self.obstacles.origin[1]
        return (
            relative_bearing_enu(self.obstacles.yaw_enu, dx, dy),
            math.hypot(dx, dy),
        )

    # --- publishing --------------------------------------------------------
    def publish_status(self, text):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    # --- loop --------------------------------------------------------------
    def tick(self):
        stale = self.obstacles.stale_reason(self.obstacle_timeout)
        if stale is not None:
            self.publish_status(f"NO DATA: {stale}")
            self.blocked_pub.publish(Int32(data=1))
            self.get_logger().warn(stale, throttle_duration_sec=5.0)
            return

        snapshot = self.obstacles.snapshot()
        bearing, distance = self.target_bearing()
        previous = None
        if self.previous_absolute is not None:
            previous = wrap_pi(self.obstacles.yaw_enu - self.previous_absolute)

        result = self.vfh.update(
            snapshot.samples,
            bearing,
            previous,
            target_distance=distance,
        )

        if result.direction is not None:
            self.previous_absolute = wrap_pi(
                self.obstacles.yaw_enu - result.direction
            )

        direction_text = (
            "BLOCKED" if result.direction is None
            else f"{math.degrees(result.direction):+.0f}deg"
        )
        goal_text = (
            "none" if distance is None
            else f"{distance:.2f}m at {math.degrees(bearing):+.0f}deg"
        )
        nearest_text = (
            "clear" if not math.isfinite(snapshot.nearest_range)
            else f"{snapshot.nearest_range:.2f}m at "
            f"{math.degrees(snapshot.nearest_bearing):+.0f}deg"
        )
        status = (
            f"steer={direction_text} goal={goal_text} nearest={nearest_text} "
            f"points={snapshot.kept_count}/{snapshot.point_count} "
            f"current={snapshot.current_point_count} "
            f"memory={snapshot.memory_point_count} "
            f"obstacle_sectors={sum(result.obstacle_binary)}/{len(result.binary)} "
            f"nonflyable_sectors={sum(result.binary)}/{len(result.binary)} "
            f"cloud_age={snapshot.cloud_age:.2f}s"
        )
        self.publish_status(status)
        self.telemetry.publish(
            result,
            snapshot,
            self.obstacles.origin,
            self.obstacles.yaw_enu,
            self.config.max_range,
            label=f"VFH {direction_text}  nearest {nearest_text}",
            goal_enu=self.goal_enu,
            goal_bearing=bearing if self.goal_enu is not None else None,
            goal_distance=distance,
            max_steer=self.config.max_steer,
            display_fov=self.display_fov,
            rings=((self.config.max_range, DIM),),
        )

        now = self.monotonic_time()
        if now - self.last_log >= self.log_period:
            self.last_log = now
            # The bar is nose-centred, so the middle character is straight ahead.
            self.get_logger().info(f"{histogram_bar(result)}  {status}")


def main(args=None):
    rclpy.init(args=args)
    node = VfhMonitor()
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
