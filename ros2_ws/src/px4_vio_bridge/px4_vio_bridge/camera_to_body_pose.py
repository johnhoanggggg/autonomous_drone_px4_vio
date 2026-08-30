"""Publish planner poses at the airframe origin instead of the camera origin.

RTAB-Map reports the OAK camera position. PX4 deliberately receives that
unmodified measurement and applies ``EKF2_EV_POS_*`` internally. The map
planner cannot use PX4's correction, though: it works entirely in RTAB-Map's
``world``/VIO frames. This node applies the same rigid sensor offset only to
planner-facing ROS ``PoseStamped`` topics.

The configured offset follows PX4's convention: camera position relative to
the body origin in body FRD (+X forward, +Y right, +Z down). Incoming RTAB-Map
poses use ENU/FLU, so Y and Z are negated before rotating the offset into the
pose's world frame.
"""

from __future__ import annotations

import copy
import json
import math

from geometry_msgs.msg import PoseStamped
import rclpy
from px4_vio_bridge.vio_to_px4_odometry import (
    pose_rejection_reason,
    Quaternion,
    quaternion_to_matrix,
    transform_vector,
    Vector3,
)
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


def camera_to_body_position(
    camera_position_enu: Vector3,
    orientation_enu_flu: Quaternion,
    camera_position_frd: Vector3,
) -> Vector3:
    """Return the body origin in ENU from a camera pose and PX4 FRD offset.

    ``camera_position_frd`` is the vector body->camera. Therefore
    ``p_body = p_camera - R_world_body * r_body_camera``.
    """
    values = (*camera_position_enu, *orientation_enu_flu, *camera_position_frd)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("pose or camera offset contains a non-finite value")
    if math.sqrt(sum(value * value for value in orientation_enu_flu)) <= 1.0e-6:
        raise ValueError("quaternion has zero norm")

    # PX4 body FRD -> RTAB-Map body FLU.
    offset_flu = (
        camera_position_frd[0],
        -camera_position_frd[1],
        -camera_position_frd[2],
    )
    offset_enu = transform_vector(quaternion_to_matrix(orientation_enu_flu), offset_flu)
    return tuple(
        camera_position_enu[index] - offset_enu[index] for index in range(3)
    )  # type: ignore[return-value]


class CameraToBodyPose(Node):
    def __init__(self) -> None:
        super().__init__("camera_to_body_pose")
        self.declare_parameter("camera_pose_topic", "/rtabmap/pose")
        self.declare_parameter("camera_vio_pose_topic", "/rtabmap/vio_pose")
        self.declare_parameter("body_pose_topic", "/rtabmap/body_pose")
        self.declare_parameter("body_vio_pose_topic", "/rtabmap/body_vio_pose")
        self.declare_parameter("config_topic", "/rtabmap/body_pose/config")
        self.declare_parameter("camera_position_frd_x", 0.100)
        self.declare_parameter("camera_position_frd_y", -0.036)
        self.declare_parameter("camera_position_frd_z", 0.056)

        self.camera_pose_topic = str(self.get_parameter("camera_pose_topic").value)
        self.camera_vio_pose_topic = str(
            self.get_parameter("camera_vio_pose_topic").value
        )
        self.body_pose_topic = str(self.get_parameter("body_pose_topic").value)
        self.body_vio_pose_topic = str(
            self.get_parameter("body_vio_pose_topic").value
        )
        self.camera_position_frd: Vector3 = (
            float(self.get_parameter("camera_position_frd_x").value),
            float(self.get_parameter("camera_position_frd_y").value),
            float(self.get_parameter("camera_position_frd_z").value),
        )
        if not all(math.isfinite(value) for value in self.camera_position_frd):
            raise ValueError("camera_position_frd must be finite")
        if math.sqrt(sum(value * value for value in self.camera_position_frd)) > 1.0:
            raise ValueError("camera_position_frd magnitude exceeds 1 metre")
        if len({
            self.camera_pose_topic,
            self.camera_vio_pose_topic,
            self.body_pose_topic,
            self.body_vio_pose_topic,
        }) != 4:
            raise ValueError("camera and body pose topics must all be distinct")

        input_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        output_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        config_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.body_pose_pub = self.create_publisher(
            PoseStamped, self.body_pose_topic, output_qos
        )
        self.body_vio_pose_pub = self.create_publisher(
            PoseStamped, self.body_vio_pose_topic, output_qos
        )
        self.config_pub = self.create_publisher(
            String, str(self.get_parameter("config_topic").value), config_qos
        )
        self.camera_pose_sub = self.create_subscription(
            PoseStamped,
            self.camera_pose_topic,
            lambda message: self._publish_body_pose(
                message, self.body_pose_pub, preserve_invalid=False
            ),
            input_qos,
        )
        self.camera_vio_pose_sub = self.create_subscription(
            PoseStamped,
            self.camera_vio_pose_topic,
            lambda message: self._publish_body_pose(
                message, self.body_vio_pose_pub, preserve_invalid=True
            ),
            input_qos,
        )
        self.invalid_counts = {self.camera_pose_topic: 0, self.camera_vio_pose_topic: 0}

        config = String()
        config.data = json.dumps(
            {
                "camera_pose_topic": self.camera_pose_topic,
                "camera_vio_pose_topic": self.camera_vio_pose_topic,
                "body_pose_topic": self.body_pose_topic,
                "body_vio_pose_topic": self.body_vio_pose_topic,
                "camera_position_frd": list(self.camera_position_frd),
                "px4_visual_odometry_translation_applied": False,
            },
            separators=(",", ":"),
            sort_keys=True,
        )
        self.config_pub.publish(config)
        self.get_logger().warn(
            "Planner pose origin: camera->body FRD "
            f"({self.camera_position_frd[0]:+.3f}, "
            f"{self.camera_position_frd[1]:+.3f}, "
            f"{self.camera_position_frd[2]:+.3f}) m; "
            "PX4 visual odometry remains camera-origin"
        )

    def _publish_body_pose(self, message, publisher, preserve_invalid: bool) -> None:
        position: Vector3 = (
            float(message.pose.position.x),
            float(message.pose.position.y),
            float(message.pose.position.z),
        )
        orientation: Quaternion = (
            float(message.pose.orientation.w),
            float(message.pose.orientation.x),
            float(message.pose.orientation.y),
            float(message.pose.orientation.z),
        )
        reason = pose_rejection_reason(position, orientation)
        if reason is not None:
            topic = self.camera_vio_pose_topic if preserve_invalid else self.camera_pose_topic
            self.invalid_counts[topic] += 1
            count = self.invalid_counts[topic]
            if count == 1 or count % 100 == 0:
                self.get_logger().error(
                    f"Rejected camera pose on {topic} #{count}: {reason}"
                )
            # The continuous raw topic feeds existing reset/invalid-pose safety
            # checks. Preserve the exact invalid value so a reset sentinel
            # cannot be transformed into a plausible-looking body position.
            if preserve_invalid:
                publisher.publish(message)
            return

        body_position = camera_to_body_position(
            position, orientation, self.camera_position_frd
        )
        output = copy.deepcopy(message)
        output.pose.position.x = body_position[0]
        output.pose.position.y = body_position[1]
        output.pose.position.z = body_position[2]
        publisher.publish(output)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraToBodyPose()
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
