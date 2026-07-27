"""Estimate and slew-limit the SLAM loop-closure correction T_map<-odom.

RTAB-Map publishes two poses for the same instant: `/rtabmap/vio_pose`, the
continuous VIO track that PX4 consumes today, and `/rtabmap/pose`, the
loop-corrected SLAM pose.  The difference between them is the map correction.
It is a step function -- when a loop closes it jumps -- so feeding the corrected
pose straight to EKF2 would either be rejected as an innovation outlier or
produce a lurch.

This node keeps the correction as a 4-DOF gravity-aligned transform (x, y, z,
yaw) and moves the *applied* correction toward the *target* correction under a
rate limit, so the correction arrives as a slow drift instead of a jump.  Roll
and pitch are deliberately discarded: EKF2 already anchors attitude to gravity
and injecting a tilt correction through external vision is never wanted.

The node is observation-only.  It publishes the correction and a preview of the
pose PX4 *would* receive, but nothing here writes to the flight controller.
"""

import math
from collections import deque
from dataclasses import dataclass
from typing import Deque, Optional, Sequence, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Float32

from px4_vio_bridge.vio_to_px4_odometry import pose_rejection_reason


Vector3 = Tuple[float, float, float]
Quaternion = Tuple[float, float, float, float]  # w, x, y, z


def wrap_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def yaw_from_quaternion(q: Quaternion) -> float:
    """Yaw about +Z from an ENU quaternion given as (w, x, y, z)."""
    w, x, y, z = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quaternion_from_yaw(yaw: float) -> Quaternion:
    return (math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0))


def quaternion_multiply(a: Quaternion, b: Quaternion) -> Quaternion:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


@dataclass(frozen=True)
class Correction:
    """A gravity-aligned SE(2)+z correction: rotate by yaw about the origin, then translate."""

    tx: float = 0.0
    ty: float = 0.0
    tz: float = 0.0
    yaw: float = 0.0

    @property
    def translation_norm(self) -> float:
        return math.sqrt(self.tx * self.tx + self.ty * self.ty + self.tz * self.tz)

    def is_finite(self) -> bool:
        return all(math.isfinite(v) for v in (self.tx, self.ty, self.tz, self.yaw))


IDENTITY = Correction()


def apply_correction(
    correction: Correction, position: Vector3, orientation: Quaternion
) -> Tuple[Vector3, Quaternion]:
    """Map an odom-frame pose into the corrected (map) frame."""
    c = math.cos(correction.yaw)
    s = math.sin(correction.yaw)
    x, y, z = position
    corrected_position = (
        c * x - s * y + correction.tx,
        s * x + c * y + correction.ty,
        z + correction.tz,
    )
    corrected_orientation = quaternion_multiply(
        quaternion_from_yaw(correction.yaw), orientation
    )
    return corrected_position, corrected_orientation


def correction_from_pair(
    vio_position: Vector3,
    vio_orientation: Quaternion,
    slam_position: Vector3,
    slam_orientation: Quaternion,
) -> Correction:
    """Solve for the correction that carries a VIO pose onto its SLAM pose.

    Exact in position and yaw by construction: `apply_correction` on the VIO
    pose reproduces the SLAM position and the SLAM yaw.
    """
    yaw = wrap_pi(
        yaw_from_quaternion(slam_orientation) - yaw_from_quaternion(vio_orientation)
    )
    c = math.cos(yaw)
    s = math.sin(yaw)
    vx, vy, vz = vio_position
    rotated = (c * vx - s * vy, s * vx + c * vy, vz)
    return Correction(
        tx=slam_position[0] - rotated[0],
        ty=slam_position[1] - rotated[1],
        tz=slam_position[2] - rotated[2],
        yaw=yaw,
    )


def correction_residual(applied: Correction, target: Correction) -> Tuple[float, float]:
    """Remaining (translation metres, yaw radians) between applied and target."""
    translation = math.sqrt(
        (target.tx - applied.tx) ** 2
        + (target.ty - applied.ty) ** 2
        + (target.tz - applied.tz) ** 2
    )
    return translation, abs(wrap_pi(target.yaw - applied.yaw))


def slew_correction(
    applied: Correction,
    target: Correction,
    max_translation_step: float,
    max_yaw_step: float,
) -> Correction:
    """Step `applied` toward `target` without exceeding the per-step limits.

    Translation is capped on its Euclidean norm so the correction always
    approaches along a straight line rather than one axis at a time, and the
    step never overshoots: once the remainder is inside the limit the target is
    reached exactly.
    """
    dx = target.tx - applied.tx
    dy = target.ty - applied.ty
    dz = target.tz - applied.tz
    distance = math.sqrt(dx * dx + dy * dy + dz * dz)

    if distance <= max_translation_step or distance <= 0.0:
        tx, ty, tz = target.tx, target.ty, target.tz
    else:
        scale = max_translation_step / distance
        tx = applied.tx + dx * scale
        ty = applied.ty + dy * scale
        tz = applied.tz + dz * scale

    dyaw = wrap_pi(target.yaw - applied.yaw)
    if abs(dyaw) <= max_yaw_step:
        yaw = target.yaw
    else:
        yaw = wrap_pi(applied.yaw + math.copysign(max_yaw_step, dyaw))

    return Correction(tx=tx, ty=ty, tz=tz, yaw=yaw)


def correction_rejection_reason(
    correction: Correction, max_translation: float, max_yaw: float
) -> Optional[str]:
    """Return why a target correction is unsafe to accept, or None if usable.

    A correction larger than the gates is a fault -- a mis-association or a VIO
    reset the node did not catch -- not something to ramp through over the next
    half minute.
    """
    if not correction.is_finite():
        return "correction contains a non-finite value"
    if correction.translation_norm > max_translation:
        return (
            f"translation {correction.translation_norm:.2f}m exceeds "
            f"max_correction_m {max_translation:.2f}"
        )
    if abs(wrap_pi(correction.yaw)) > max_yaw:
        return (
            f"yaw {math.degrees(correction.yaw):.1f}deg exceeds "
            f"max_correction_yaw_deg {math.degrees(max_yaw):.1f}"
        )
    return None


def exceeds_deadband(
    candidate: Correction, current: Correction, translation_deadband: float, yaw_deadband: float
) -> bool:
    """Whether a freshly solved correction differs enough from the latched target.

    SLAM re-optimizes continuously, so the raw solution jitters by millimetres
    even with no loop closure.  Latching through a deadband keeps the applied
    correction from perpetually chasing that noise.
    """
    translation, yaw = correction_residual(current, candidate)
    return translation > translation_deadband or yaw > yaw_deadband


def pose_to_tuple(pose: PoseStamped) -> Tuple[Vector3, Quaternion]:
    return (
        (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z),
        (
            pose.pose.orientation.w,
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
        ),
    )


def stamp_seconds(pose: PoseStamped) -> float:
    return pose.header.stamp.sec + pose.header.stamp.nanosec * 1.0e-9


def nearest_sample(
    history: Sequence[Tuple[float, Vector3, Quaternion]], timestamp: float
) -> Optional[Tuple[float, Vector3, Quaternion]]:
    if not history:
        return None
    return min(history, key=lambda item: abs(item[0] - timestamp))


class MapCorrectionNode(Node):
    def __init__(self) -> None:
        super().__init__("map_correction")

        self.declare_parameter("vio_pose_topic", "/rtabmap/vio_pose")
        self.declare_parameter("slam_pose_topic", "/rtabmap/pose")
        self.declare_parameter("correction_topic", "/vio/map_correction")
        self.declare_parameter("target_topic", "/vio/map_correction_target")
        self.declare_parameter("preview_pose_topic", "/vio/map_correction/preview_pose")
        self.declare_parameter("residual_topic", "/vio/map_correction/residual_m")
        self.declare_parameter("residual_yaw_topic", "/vio/map_correction/residual_deg")
        # Slew limits. The translation rate is the number that matters most: it
        # is the fake velocity the ramp would inject if this were ever wired
        # into EKF2, so it must stay small against EKF2's velocity noise.
        self.declare_parameter("translation_rate", 0.03)
        self.declare_parameter("yaw_rate_deg", 1.0)
        self.declare_parameter("max_correction_m", 0.5)
        self.declare_parameter("max_correction_yaw_deg", 15.0)
        self.declare_parameter("max_pair_dt", 0.15)
        self.declare_parameter("update_deadband_m", 0.01)
        self.declare_parameter("update_deadband_deg", 0.2)
        self.declare_parameter("update_rate_hz", 50.0)
        self.declare_parameter("history_size", 240)
        self.declare_parameter("enabled", True)
        # Synthetic correction, added to whatever SLAM reports. Lets the ramp be
        # exercised on demand instead of waiting for a real loop closure:
        #   ros2 param set /map_correction inject_translation "[0.1, 0.0, 0.0]"
        self.declare_parameter("inject_translation", [0.0, 0.0, 0.0])
        self.declare_parameter("inject_yaw_deg", 0.0)

        self.applied = IDENTITY
        self.target = IDENTITY
        # The deadbanded SLAM solution, without any injected test offset.
        self.latched = IDENTITY
        self.history: Deque[Tuple[float, Vector3, Quaternion]] = deque(
            maxlen=max(1, int(self.get_parameter("history_size").value))
        )
        self.latest_vio: Optional[Tuple[Vector3, Quaternion]] = None
        self.last_slew_time = self.get_clock().now()
        self.rejected_count = 0
        self.unpaired_count = 0
        self.vio_reset_active = False
        self.last_report = self.get_clock().now()

        vio_topic = str(self.get_parameter("vio_pose_topic").value)
        slam_topic = str(self.get_parameter("slam_pose_topic").value)

        self.create_subscription(PoseStamped, vio_topic, self.on_vio_pose, 10)
        self.create_subscription(PoseStamped, slam_topic, self.on_slam_pose, 10)

        # Published as PoseStamped in the existing `world` frame, NOT as
        # TransformStamped: Foxglove grafts any TransformStamped topic into its
        # transform tree, and map/odom frames are a disconnected second root
        # next to this pipeline's world->camera tree, which breaks every 3D
        # panel that tries to resolve against them.
        self.correction_pub = self.create_publisher(
            PoseStamped, str(self.get_parameter("correction_topic").value), 10
        )
        self.target_pub = self.create_publisher(
            PoseStamped, str(self.get_parameter("target_topic").value), 10
        )
        self.preview_pub = self.create_publisher(
            PoseStamped, str(self.get_parameter("preview_pose_topic").value), 10
        )
        self.residual_pub = self.create_publisher(
            Float32, str(self.get_parameter("residual_topic").value), 10
        )
        self.residual_yaw_pub = self.create_publisher(
            Float32, str(self.get_parameter("residual_yaw_topic").value), 10
        )

        period = 1.0 / max(1.0, float(self.get_parameter("update_rate_hz").value))
        self.create_timer(period, self.on_timer)

        self.get_logger().info(
            f"Estimating map correction from {vio_topic} and {slam_topic} "
            f"(observation only, nothing is sent to PX4)"
        )

    # ---- subscriptions -------------------------------------------------

    def on_vio_pose(self, pose: PoseStamped) -> None:
        position, orientation = pose_to_tuple(pose)
        if pose_rejection_reason(position, orientation) is not None:
            # An occluded camera snaps the VIO pose to the reset sentinel and
            # restarts the odom frame at the origin, which invalidates every
            # pairing in the history and the correction solved from it.
            if not self.vio_reset_active:
                self.vio_reset_active = True
                self.history.clear()
                self.get_logger().warning(
                    "VIO reset detected: cleared pose history and froze the correction "
                    f"at {self.describe(self.applied)}"
                )
            return

        self.vio_reset_active = False
        self.latest_vio = (position, orientation)
        self.history.append((stamp_seconds(pose), position, orientation))

    def on_slam_pose(self, pose: PoseStamped) -> None:
        if self.vio_reset_active:
            return

        timestamp = stamp_seconds(pose)
        sample = nearest_sample(self.history, timestamp)
        if sample is None:
            return

        pair_dt = abs(sample[0] - timestamp)
        if pair_dt > float(self.get_parameter("max_pair_dt").value):
            self.unpaired_count += 1
            if self.unpaired_count == 1 or self.unpaired_count % 100 == 0:
                self.get_logger().warning(
                    f"No VIO pose within {pair_dt * 1000.0:.0f} ms of the SLAM pose "
                    f"(#{self.unpaired_count}); skipping this correction update"
                )
            return

        slam_position, slam_orientation = pose_to_tuple(pose)
        candidate = correction_from_pair(
            sample[1], sample[2], slam_position, slam_orientation
        )

        reason = correction_rejection_reason(
            candidate,
            float(self.get_parameter("max_correction_m").value),
            math.radians(float(self.get_parameter("max_correction_yaw_deg").value)),
        )
        if reason is not None:
            self.rejected_count += 1
            if self.rejected_count == 1 or self.rejected_count % 20 == 0:
                self.get_logger().warning(
                    f"Rejected map correction #{self.rejected_count}: {reason}"
                )
            return

        if not exceeds_deadband(
            candidate,
            self.latched,
            float(self.get_parameter("update_deadband_m").value),
            math.radians(float(self.get_parameter("update_deadband_deg").value)),
        ):
            return

        previous = self.latched
        self.latched = candidate
        translation, yaw = correction_residual(previous, candidate)
        if translation > 0.05 or yaw > math.radians(1.0):
            self.get_logger().info(
                f"Loop correction moved {translation * 100.0:.1f} cm / "
                f"{math.degrees(yaw):.2f} deg -> {self.describe(candidate)}"
            )

    # ---- periodic slew -------------------------------------------------

    def on_timer(self) -> None:
        now = self.get_clock().now()
        dt = (now - self.last_slew_time).nanoseconds * 1.0e-9
        self.last_slew_time = now
        if dt <= 0.0 or dt > 1.0:
            return

        # Rebuilt every tick so a live `inject_translation` change takes effect
        # even while SLAM is quiet.
        self.target = self.injected(self.latched)

        if bool(self.get_parameter("enabled").value):
            self.applied = slew_correction(
                self.applied,
                self.target,
                float(self.get_parameter("translation_rate").value) * dt,
                math.radians(float(self.get_parameter("yaw_rate_deg").value)) * dt,
            )

        stamp = now.to_msg()
        self.correction_pub.publish(self.to_pose(self.applied, stamp))
        self.target_pub.publish(self.to_pose(self.target, stamp))

        translation, yaw = correction_residual(self.applied, self.target)
        self.residual_pub.publish(Float32(data=float(translation)))
        self.residual_yaw_pub.publish(Float32(data=float(math.degrees(yaw))))

        if self.latest_vio is not None:
            position, orientation = apply_correction(self.applied, *self.latest_vio)
            preview = PoseStamped()
            preview.header.stamp = stamp
            preview.header.frame_id = "world"
            preview.pose.position.x = position[0]
            preview.pose.position.y = position[1]
            preview.pose.position.z = position[2]
            preview.pose.orientation.w = orientation[0]
            preview.pose.orientation.x = orientation[1]
            preview.pose.orientation.y = orientation[2]
            preview.pose.orientation.z = orientation[3]
            self.preview_pub.publish(preview)

        if (now - self.last_report).nanoseconds * 1.0e-9 >= 5.0:
            self.last_report = now
            self.get_logger().info(
                f"applied={self.describe(self.applied)} "
                f"target={self.describe(self.target)} "
                f"residual={translation * 100.0:.1f}cm/{math.degrees(yaw):.2f}deg"
            )

    # ---- helpers -------------------------------------------------------

    def injected(self, correction: Correction) -> Correction:
        offset = list(self.get_parameter("inject_translation").value)
        offset += [0.0] * (3 - len(offset))
        return Correction(
            tx=correction.tx + float(offset[0]),
            ty=correction.ty + float(offset[1]),
            tz=correction.tz + float(offset[2]),
            yaw=wrap_pi(
                correction.yaw
                + math.radians(float(self.get_parameter("inject_yaw_deg").value))
            ),
        )

    @staticmethod
    def describe(correction: Correction) -> str:
        return (
            f"({correction.tx:+.3f}, {correction.ty:+.3f}, {correction.tz:+.3f}) m, "
            f"{math.degrees(correction.yaw):+.2f} deg"
        )

    @staticmethod
    def to_pose(correction: Correction, stamp) -> PoseStamped:
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = "world"
        pose.pose.position.x = correction.tx
        pose.pose.position.y = correction.ty
        pose.pose.position.z = correction.tz
        quaternion = quaternion_from_yaw(correction.yaw)
        pose.pose.orientation.w = quaternion[0]
        pose.pose.orientation.x = quaternion[1]
        pose.pose.orientation.y = quaternion[2]
        pose.pose.orientation.z = quaternion[3]
        return pose


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MapCorrectionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
