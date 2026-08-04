"""Everything the VFH planner knows, in a form Foxglove can draw.

Shared by `vfh_monitor` and `offboard_vfh` so the picture on screen is identical
whether the vehicle is flying or the algorithm is only watching — a monitor
session is then a genuine rehearsal of the flight display, not a different view
of the same data.

Three kinds of output, because Foxglove panels want different things:

- **3D markers** (`/vfh/markers`): the polar histogram drawn as a fan of rays
  around the vehicle at the range each sector actually measured, red where
  blocked and green where free, plus the chosen direction as a thick arrow, the
  rejected candidates as thin lines, and a text label. Overlay this on
  `/rtabmap/obstacle_cloud` and a wrong answer is obvious at a glance.
- **A point cloud** (`/vfh/samples`): exactly the points that survived the
  height slab and range band and went into the histogram — the difference
  between "the planner is wrong" and "the planner never saw it".
- **Scalars** (`/vfh/*_deg`, `/vfh/nearest`, ...): one number per topic so Gauge,
  Indicator and Plot panels can bind directly, the same pattern `battery_to_ros`
  uses. Headings are published in PX4's NED convention (0 = north, increasing to
  the right), because that is what every other number in this project means.

All geometry is published in the ENU `world` frame at the pose the obstacle
cloud was measured against, so it lines up with the cloud and the SLAM path.
"""
import math
import struct

from geometry_msgs.msg import Point, PoseStamped
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import ColorRGBA, Float32, Float32MultiArray, Int32
from visualization_msgs.msg import Marker, MarkerArray

from px4_vio_bridge.vfh2d import sector_center, wrap_pi

RED = ColorRGBA(r=0.90, g=0.20, b=0.15, a=0.85)      # blocked sector
GREEN = ColorRGBA(r=0.20, g=0.80, b=0.35, a=0.55)    # free sector
BLUE = ColorRGBA(r=0.20, g=0.55, b=0.95, a=0.95)     # chosen direction
YELLOW = ColorRGBA(r=0.95, g=0.80, b=0.20, a=0.70)   # rejected candidate
WHITE = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.9)
FOV = ColorRGBA(r=0.55, g=0.65, b=0.85, a=0.45)     # steer-limit wedge
AMBER = ColorRGBA(r=0.95, g=0.65, b=0.15, a=0.70)   # stop ring
DIM = ColorRGBA(r=0.55, g=0.55, b=0.60, a=0.35)     # sensor-range ring
UNKNOWN = ColorRGBA(r=0.50, g=0.52, b=0.56, a=0.38) # clear but not steerable


def enu_heading(yaw_enu, bearing):
    """ENU heading of a body-relative bearing (body-right is ENU-negative)."""
    return wrap_pi(yaw_enu - bearing)


def ned_heading_deg(yaw_enu, bearing=0.0):
    """Body-relative bearing as a PX4-style NED heading, in degrees.

    ENU yaw runs counter-clockwise from east, NED heading clockwise from north:
    heading = 90 deg - yaw_enu, and a body bearing adds on in NED.
    """
    return math.degrees(wrap_pi(math.pi / 2.0 - enu_heading(yaw_enu, bearing)))


def polar_to_enu(origin, yaw_enu, bearing, distance):
    heading = enu_heading(yaw_enu, bearing)
    return (
        origin[0] + distance * math.cos(heading),
        origin[1] + distance * math.sin(heading),
        origin[2],
    )


def sector_rays(result, origin, yaw_enu, max_range, display_fov=None):
    """(bearing, end_point, obstacle_blocked) for each displayed sector.

    A sector with returns is drawn out to its nearest return, so the fan traces
    the obstacle's actual shape; an empty sector is drawn to `max_range`, which
    is how far the planner was willing to look. Colours come from the physical
    obstacle histogram before the steering-FOV mask: blind space is a planning
    constraint, not a detected obstacle.
    """
    n = len(result.binary)
    obstacle_binary = (
        result.obstacle_binary
        if len(result.obstacle_binary) == n
        else result.binary
    )
    rays = []
    for i in range(n):
        bearing = sector_center(i, n)
        if display_fov is not None and abs(bearing) > display_fov + 1e-9:
            continue
        distance = result.sector_range[i]
        if not math.isfinite(distance):
            distance = max_range
        rays.append(
            (
                bearing,
                polar_to_enu(origin, yaw_enu, bearing, distance),
                obstacle_binary[i],
            )
        )
    return rays


def opening_width(binary, direction):
    """Angular width of the free run containing `direction`, in radians.

    The number that says how much room the chosen direction actually has —
    a direction threading a one-sector gap and one crossing an open room are
    otherwise indistinguishable on screen.
    """
    n = len(binary)
    if n == 0 or direction is None:
        return 0.0
    width = 2.0 * math.pi / n
    start = int((wrap_pi(direction) + math.pi) // width) % n
    if binary[start]:
        return 0.0
    count = 1
    for step in range(1, n):
        if binary[(start + step) % n]:
            break
        count += 1
    for step in range(1, n):
        if binary[(start - step) % n]:
            break
        count += 1
    return min(count, n) * width


def arc_points(origin, yaw_enu, start_bearing, end_bearing, radius, segments=32):
    """Points along a body-relative arc at a fixed radius, in ENU."""
    span = end_bearing - start_bearing
    return [
        polar_to_enu(
            origin, yaw_enu, start_bearing + span * i / max(1, segments), radius
        )
        for i in range(segments + 1)
    ]


def circle_points(origin, radius, segments=48):
    """A closed horizontal ring around the vehicle, in ENU."""
    points = [
        (
            origin[0] + radius * math.cos(2.0 * math.pi * i / segments),
            origin[1] + radius * math.sin(2.0 * math.pi * i / segments),
            origin[2],
        )
        for i in range(segments)
    ]
    return points + [points[0]]


def samples_cloud(stamp, frame_id, samples, origin, yaw_enu):
    """The samples the histogram was built from, as a PointCloud2 in `world`."""
    data = bytearray()
    for r, bearing in samples:
        x, y, z = polar_to_enu(origin, yaw_enu, bearing, r)
        data += struct.pack("<fff", float(x), float(y), float(z))

    msg = PointCloud2()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = 1
    msg.width = len(samples)
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * len(samples)
    msg.is_dense = True
    msg.data = bytes(data)
    return msg


class VfhTelemetry:
    """Publishes the planner's state. Owns its topics; holds no planner state."""

    def __init__(self, node, frame_id="world", namespace="/vfh"):
        self.node = node
        self.frame_id = frame_id

        def pub(msg_type, name):
            return node.create_publisher(msg_type, f"{namespace}/{name}", 10)

        self.markers_pub = pub(MarkerArray, "markers")
        self.samples_pub = pub(PointCloud2, "samples")
        self.direction_pub = pub(PoseStamped, "direction")
        self.goal_pub = pub(PoseStamped, "goal")
        self.histogram_pub = pub(Float32MultiArray, "histogram")
        self.binary_pub = pub(Float32MultiArray, "binary")
        self.obstacle_binary_pub = pub(Float32MultiArray, "obstacle_binary")
        self.blocked_pub = pub(Int32, "blocked")
        self.blocked_sectors_pub = pub(Int32, "blocked_sectors")
        self.obstacle_blocked_sectors_pub = pub(Int32, "obstacle_blocked_sectors")
        self.samples_count_pub = pub(Int32, "samples_count")
        self.memory_points_pub = pub(Int32, "memory_points")
        self.nearest_pub = pub(Float32, "nearest")
        self.nearest_bearing_pub = pub(Float32, "nearest_bearing_deg")
        self.heading_pub = pub(Float32, "heading_deg")
        self.steer_pub = pub(Float32, "direction_deg")
        self.steer_heading_pub = pub(Float32, "direction_heading_deg")
        self.opening_pub = pub(Float32, "opening_width_deg")
        self.goal_bearing_pub = pub(Float32, "goal_bearing_deg")
        self.goal_distance_pub = pub(Float32, "goal_distance")
        self.cost_pub = pub(Float32, "cost")

    # --- helpers -----------------------------------------------------------
    def stamp(self):
        return self.node.get_clock().now().to_msg()

    def pose_along(self, origin, yaw_enu, bearing):
        heading = enu_heading(yaw_enu, bearing)
        pose = PoseStamped()
        pose.header.stamp = self.stamp()
        pose.header.frame_id = self.frame_id
        pose.pose.position.x = float(origin[0])
        pose.pose.position.y = float(origin[1])
        pose.pose.position.z = float(origin[2])
        pose.pose.orientation.z = math.sin(heading / 2.0)
        pose.pose.orientation.w = math.cos(heading / 2.0)
        return pose

    def _marker(self, marker_type, ns, marker_id, scale, stamp):
        m = Marker()
        m.header.stamp = stamp
        m.header.frame_id = self.frame_id
        m.ns = ns
        m.id = marker_id
        m.type = marker_type
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = scale
        m.scale.y = scale
        m.scale.z = scale
        return m

    def build_markers(
        self,
        result,
        origin,
        yaw_enu,
        max_range,
        label,
        goal_enu,
        max_steer,
        display_fov,
        rings,
    ):
        stamp = self.stamp()
        markers = MarkerArray()
        base = Point(x=float(origin[0]), y=float(origin[1]), z=float(origin[2]))

        # The histogram fan: one ray per sector, coloured by blocked/free.
        fan = self._marker(Marker.LINE_LIST, "vfh_histogram", 0, 0.01, stamp)
        for bearing, end, blocked in sector_rays(
            result, origin, yaw_enu, max_range, display_fov
        ):
            fan.points.append(base)
            fan.points.append(Point(x=float(end[0]), y=float(end[1]), z=float(end[2])))
            # Outside the steering FOV, red still means a remembered physical
            # obstacle. A clear ray is grey, not green, because it is not a
            # legal output heading and may only mean "not in memory".
            steerable = max_steer is None or abs(bearing) <= max_steer + 1e-9
            color = RED if blocked else (GREEN if steerable else UNKNOWN)
            fan.colors.append(color)
            fan.colors.append(color)
        markers.markers.append(fan)

        # Candidates that lost, so a surprising choice can be explained.
        candidates = self._marker(Marker.LINE_LIST, "vfh_candidates", 1, 0.006, stamp)
        for bearing in result.candidates:
            if result.direction is not None and abs(
                wrap_pi(bearing - result.direction)
            ) < 1e-9:
                continue
            end = polar_to_enu(origin, yaw_enu, bearing, max_range * 0.5)
            candidates.points.append(base)
            candidates.points.append(
                Point(x=float(end[0]), y=float(end[1]), z=float(end[2]))
            )
            candidates.colors.append(YELLOW)
            candidates.colors.append(YELLOW)
        markers.markers.append(candidates)

        # The committed direction.
        chosen = self._marker(Marker.ARROW, "vfh_direction", 2, 0.03, stamp)
        chosen.scale.x = 0.03      # shaft diameter
        chosen.scale.y = 0.07      # head diameter
        chosen.scale.z = 0.10      # head length
        chosen.color = BLUE
        if result.direction is not None:
            end = polar_to_enu(origin, yaw_enu, result.direction, max_range * 0.6)
            chosen.points = [
                base,
                Point(x=float(end[0]), y=float(end[1]), z=float(end[2])),
            ]
        else:
            chosen.action = Marker.DELETE
        markers.markers.append(chosen)

        if goal_enu is not None:
            goal = self._marker(Marker.SPHERE, "vfh_goal", 3, 0.12, stamp)
            goal.color = WHITE
            goal.pose.position.x = float(goal_enu[0])
            goal.pose.position.y = float(goal_enu[1])
            goal.pose.position.z = float(origin[2])
            markers.markers.append(goal)

        text = self._marker(Marker.TEXT_VIEW_FACING, "vfh_label", 4, 0.10, stamp)
        text.color = WHITE
        text.pose.position.x = float(origin[0])
        text.pose.position.y = float(origin[1])
        text.pose.position.z = float(origin[2]) + 0.35
        text.text = label
        markers.markers.append(text)

        markers.markers.extend(
            self.build_context(origin, yaw_enu, max_range, max_steer, rings, stamp)
        )
        return markers

    def build_context(self, origin, yaw_enu, max_range, max_steer, rings, stamp):
        """The constraints the decision was made under, not the decision itself.

        Without these the fan is unreadable: a direction pinned at the edge of
        the field of view looks like a free choice, and "nearest 0.85 m" means
        nothing until you can see where the stop ring is.
        """
        out = []
        if max_steer is not None:
            # The wedge the planner may choose within — the camera's blind spot
            # is everything outside it, and unknown is not free.
            wedge = self._marker(Marker.LINE_STRIP, "vfh_fov", 5, 0.008, stamp)
            wedge.color = FOV
            arc = arc_points(origin, yaw_enu, -max_steer, max_steer, max_range)
            base = Point(x=float(origin[0]), y=float(origin[1]), z=float(origin[2]))
            wedge.points = (
                [base]
                + [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in arc]
                + [base]
            )
            out.append(wedge)

        for index, ring in enumerate(rings or ()):
            radius, color = ring[0], ring[1]
            marker = self._marker(
                Marker.LINE_STRIP, "vfh_rings", 6 + index, 0.006, stamp
            )
            marker.color = color
            marker.points = [
                Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))
                for p in circle_points(origin, radius)
            ]
            out.append(marker)
        return out

    # --- entry point -------------------------------------------------------
    def publish(
        self,
        result,
        snapshot,
        origin,
        yaw_enu,
        max_range,
        label="",
        goal_enu=None,
        goal_bearing=None,
        goal_distance=None,
        max_steer=None,
        display_fov=None,
        rings=(),
    ):
        """Publish one planning cycle. `origin`/`yaw_enu` are the cloud's frame."""
        if origin is None or yaw_enu is None:
            return

        self.markers_pub.publish(
            self.build_markers(
                result,
                origin,
                yaw_enu,
                max_range,
                label,
                goal_enu,
                max_steer,
                display_fov,
                rings,
            )
        )
        if snapshot is not None and snapshot.samples:
            self.samples_pub.publish(
                samples_cloud(
                    self.stamp(), self.frame_id, snapshot.samples, origin, yaw_enu
                )
            )

        if result.direction is not None:
            self.direction_pub.publish(
                self.pose_along(origin, yaw_enu, result.direction)
            )
            self.steer_pub.publish(Float32(data=float(math.degrees(result.direction))))
            self.steer_heading_pub.publish(
                Float32(data=float(ned_heading_deg(yaw_enu, result.direction)))
            )
            self.opening_pub.publish(
                Float32(
                    data=float(
                        math.degrees(opening_width(result.binary, result.direction))
                    )
                )
            )
        if result.cost is not None:
            self.cost_pub.publish(Float32(data=float(result.cost)))

        if goal_enu is not None:
            pose = PoseStamped()
            pose.header.stamp = self.stamp()
            pose.header.frame_id = self.frame_id
            pose.pose.position.x = float(goal_enu[0])
            pose.pose.position.y = float(goal_enu[1])
            pose.pose.position.z = float(origin[2])
            pose.pose.orientation.w = 1.0
            self.goal_pub.publish(pose)
        if goal_bearing is not None:
            self.goal_bearing_pub.publish(
                Float32(data=float(math.degrees(goal_bearing)))
            )
        if goal_distance is not None:
            self.goal_distance_pub.publish(Float32(data=float(goal_distance)))

        density = Float32MultiArray()
        density.data = [float(v) for v in result.density]
        self.histogram_pub.publish(density)
        binary = Float32MultiArray()
        binary.data = [float(v) for v in result.binary]
        self.binary_pub.publish(binary)
        obstacle_binary = Float32MultiArray()
        obstacle_binary.data = [float(v) for v in result.obstacle_binary]
        self.obstacle_binary_pub.publish(obstacle_binary)

        self.blocked_pub.publish(Int32(data=1 if result.blocked else 0))
        self.blocked_sectors_pub.publish(Int32(data=int(sum(result.binary))))
        self.obstacle_blocked_sectors_pub.publish(
            Int32(data=int(sum(result.obstacle_binary)))
        )
        self.heading_pub.publish(Float32(data=float(ned_heading_deg(yaw_enu))))

        nearest = math.inf if snapshot is None else snapshot.nearest_range
        # -1 rather than inf: a Gauge cannot render inf, and PX4's own
        # convention for "unknown" in this project is a negative sentinel.
        self.nearest_pub.publish(
            Float32(data=float(nearest if math.isfinite(nearest) else -1.0))
        )
        if snapshot is not None:
            self.samples_count_pub.publish(Int32(data=int(snapshot.kept_count)))
            self.memory_points_pub.publish(
                Int32(data=int(snapshot.memory_point_count))
            )
            if snapshot.nearest_bearing is not None:
                self.nearest_bearing_pub.publish(
                    Float32(data=float(math.degrees(snapshot.nearest_bearing)))
                )
