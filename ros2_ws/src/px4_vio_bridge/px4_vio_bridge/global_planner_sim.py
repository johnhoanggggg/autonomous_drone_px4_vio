"""Fake occupancy grid and SLAM pose for the global-planner monitor."""

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


class GlobalPlannerSim(Node):
    def __init__(self):
        super().__init__("global_planner_sim")
        self.declare_parameter("dynamic_obstacle", True)
        self.declare_parameter("dynamic_period", 8.0)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_pub = self.create_publisher(OccupancyGrid, "/rtabmap/grid", qos)
        self.pose_pub = self.create_publisher(PoseStamped, "/rtabmap/pose", 10)
        self.raw_vio_pub = self.create_publisher(PoseStamped, "/rtabmap/vio_pose", 10)
        # Simulated poses already represent the robot origin, so publish the
        # body-center interfaces directly as well as the legacy camera topics.
        self.body_pose_pub = self.create_publisher(
            PoseStamped, "/rtabmap/body_pose", 10
        )
        self.body_raw_vio_pub = self.create_publisher(
            PoseStamped, "/rtabmap/body_vio_pose", 10
        )
        self.correction_pub = self.create_publisher(
            PoseStamped, "/rtabmap/odom_correction", 10
        )
        self.dynamic = bool(self.get_parameter("dynamic_obstacle").value)
        self.period = max(2.0, float(self.get_parameter("dynamic_period").value))
        self.started = self.get_clock().now().nanoseconds / 1e9
        self.last_state = None
        self.create_timer(0.2, self.tick)
        self.get_logger().warn(
            "simulator only: click a world-frame goal in Foxglove; "
            "a central blockage toggles to exercise replanning"
        )

    def build_map(self, blocked):
        resolution = 0.10
        width = height = 100
        origin = -5.0
        data = [-1] * (width * height)

        def cell(x, y):
            return int((x - origin) / resolution), int((y - origin) / resolution)

        def fill(x0, y0, x1, y1, value):
            ax, ay = cell(x0, y0)
            bx, by = cell(x1, y1)
            for y in range(max(0, ay), min(height, by + 1)):
                for x in range(max(0, ax), min(width, bx + 1)):
                    data[y * width + x] = value

        # Explored room, perimeter, and a dividing wall with two usable gaps.
        fill(-4.5, -3.5, 4.5, 3.5, 0)
        fill(-4.5, -3.5, 4.5, -3.4, 100)
        fill(-4.5, 3.4, 4.5, 3.5, 100)
        fill(-4.5, -3.5, -4.4, 3.5, 100)
        fill(4.4, -3.5, 4.5, 3.5, 100)
        fill(-0.1, -3.5, 0.1, -1.1, 100)
        fill(-0.1, 0.4, 0.1, 1.5, 100)
        fill(-0.1, 2.8, 0.1, 3.5, 100)
        if blocked:
            # Close the lower gap; A* should switch to the upper gap.
            fill(-0.2, -1.2, 0.2, 0.5, 100)

        msg = OccupancyGrid()
        msg.header.frame_id = "world"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.map_load_time = msg.header.stamp
        msg.info.resolution = resolution
        msg.info.width = width
        msg.info.height = height
        msg.info.origin.position.x = origin
        msg.info.origin.position.y = origin
        msg.info.origin.orientation.w = 1.0
        msg.data = data
        return msg

    def tick(self):
        now = self.get_clock().now().nanoseconds / 1e9
        blocked = self.dynamic and int((now - self.started) / self.period) % 2 == 1
        # Publish continually so map freshness can be exercised without special cases.
        self.map_pub.publish(self.build_map(blocked))
        pose = PoseStamped()
        pose.header.frame_id = "world"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = -3.0
        pose.pose.position.y = 0.0
        pose.pose.position.z = 0.30
        pose.pose.orientation.w = 1.0
        self.pose_pub.publish(pose)
        self.raw_vio_pub.publish(pose)
        self.body_pose_pub.publish(pose)
        self.body_raw_vio_pub.publish(pose)
        correction = PoseStamped()
        correction.header.frame_id = "world"
        correction.header.stamp = pose.header.stamp
        correction.pose.orientation.w = 1.0
        self.correction_pub.publish(correction)
        if blocked != self.last_state:
            self.last_state = blocked
            self.get_logger().warn(f"dynamic lower gap blocked={blocked}")


def main(args=None):
    rclpy.init(args=args)
    node = GlobalPlannerSim()
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
