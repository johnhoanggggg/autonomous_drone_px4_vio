"""Exercise the actual component with synthetic images and calibrated TF.

The camera's optical Z axis must become body X in SensorData. An identity
camera model (the original bug) fails this test even if odometry initializes.
No OAK, PX4, or running ROS graph is needed.
"""
import os
import signal
import subprocess
import time

import numpy as np
import pytest

rclpy = pytest.importorskip("rclpy")
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from composition_interfaces.srv import LoadNode
from depthai_ros_msgs_v3.msg import TrackedFeature, TrackedFeatures
from geometry_msgs.msg import TransformStamped
from rtabmap_msgs.msg import SensorData
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage


@pytest.mark.parametrize("input_reliability", [ReliabilityPolicy.RELIABLE, ReliabilityPolicy.BEST_EFFORT])
def test_camera_model_uses_body_to_optical_tf(tmp_path, input_reliability):
    domain = 160 + os.getpid() % 50
    context = Context()
    rclpy.init(context=context, domain_id=domain)
    node = Node("tracked_geometry_test", context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    log = (tmp_path / "container.log").open("w")
    process = subprocess.Popen(
        ["ros2", "run", "rclcpp_components", "component_container",
         "--ros-args", "-r", "__node:=tracked_test_container"],
        env={**os.environ, "ROS_DOMAIN_ID": str(domain)},
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
    )
    received = []
    sub = node.create_subscription(SensorData, "/odom_sensor_data/features",
                                   received.append, qos_profile_sensor_data)
    publishers = [
        node.create_publisher(kind, topic, QoSProfile(depth=5, reliability=input_reliability))
        for kind, topic in (
            (Image, "/oak/left/image_rect"),
            (Image, "/oak/stereo/image_raw"),
            (CameraInfo, "/oak/left/camera_info"),
            (TrackedFeatures, "/oak/left_rect_feature_tracker/tracked_features"),
        )
    ]
    tf_pub = node.create_publisher(TFMessage, "/tf_static", QoSProfile(
        depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    try:
        client = node.create_client(LoadNode, "/tracked_test_container/_container/load_node")
        assert client.wait_for_service(timeout_sec=15), "component container did not start"
        request = LoadNode.Request()
        request.package_name = "px4_vio_bridge"
        request.plugin_name = "px4_vio_bridge::OakTrackedOdometry"
        request.node_name = "tracked_under_test"
        request.parameters = [Parameter(name, value=value).to_parameter_msg()
                              for name, value in {
                                  "frame_id": "test_body",
                                  "qos": int(input_reliability),
                                  "wait_for_transform": 0.01,
                                  "publish_tf": False,
                                  "publish_null_when_lost": False,
                                  "Vis/MinInliers": "5",
                                  "OdomF2M/BundleAdjustment": "0",
                              }.items()]
        future = client.call_async(request)
        executor.spin_until_future_complete(future, timeout_sec=15)
        assert future.done() and future.result().success
        discovery_deadline = time.monotonic() + 5
        while time.monotonic() < discovery_deadline:
            endpoints = node.get_subscriptions_info_by_topic("/oak/stereo/image_raw")
            if endpoints:
                break
            executor.spin_once(timeout_sec=0.05)
        assert endpoints and endpoints[0].qos_profile.reliability == input_reliability

        image = Image(height=120, width=160, encoding="mono8", step=160)
        image.data = np.random.default_rng(123).integers(
            0, 255, (120, 160), dtype=np.uint8).tobytes()
        depth = Image(height=120, width=160, encoding="16UC1", step=320)
        depth.data = np.full((120, 160), 2000, dtype=np.uint16).tobytes()
        info = CameraInfo(height=120, width=160)
        info.k = [100., 0., 80., 0., 100., 60., 0., 0., 1.]
        info.r = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
        # OAK's left CameraInfo carries a stereo translation. RGB-D must use
        # the left optical frame without that right-camera projection offset.
        info.p = [100., 0., 80., -7.5, 0., 100., 60., 0., 0., 0., 1., 0.]
        tracks = TrackedFeatures()
        for y in range(20, 101, 20):
            for x in range(20, 141, 20):
                feature = TrackedFeature()
                feature.position.x, feature.position.y = float(x), float(y)
                tracks.features.append(feature)

        def pump(seconds, stop_on_result=False):
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                stamp = node.get_clock().now().to_msg()
                for publisher, message in zip(publishers, (image, depth, info, tracks)):
                    message.header.stamp = stamp
                    message.header.frame_id = "test_optical"
                    publisher.publish(message)
                for _ in range(5):
                    executor.spin_once(timeout_sec=0.01)
                if stop_on_result and received:
                    return

        # Missing calibration must reject the tuple, not silently use identity.
        pump(1.5)
        assert not received, "uncalibrated images reached the odometry core"
        transform = TransformStamped()
        transform.header.frame_id = "test_body"
        transform.child_frame_id = "test_optical"
        transform.transform.translation.x = 0.1
        transform.transform.translation.y = -0.02
        transform.transform.translation.z = 0.03
        transform.transform.rotation.x = -0.5
        transform.transform.rotation.y = 0.5
        transform.transform.rotation.z = -0.5
        transform.transform.rotation.w = 0.5
        tf_pub.publish(TFMessage(transforms=[transform]))
        pump(5, stop_on_result=True)
        assert received, (tmp_path / "container.log").read_text()[-6000:]
        actual = received[-1].local_transform[0]
        assert actual.translation.x == pytest.approx(0.1)
        assert actual.translation.y == pytest.approx(-0.02)
        assert actual.translation.z == pytest.approx(0.03)
        # Rotation maps optical (right, down, forward) to body (forward, left, up).
        q = actual.rotation
        z_axis = (2 * (q.x*q.z + q.w*q.y),
                  2 * (q.y*q.z - q.w*q.x),
                  1 - 2 * (q.x*q.x + q.y*q.y))
        assert z_axis == pytest.approx((1., 0., 0.), abs=1e-6)
        assert received[-1].header.frame_id == "test_body"
        assert received[-1].left_camera_info[0].p[3] == 0.0
        assert received[-1].left_camera_info[0].p[0] == 100.0
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=5)
        log.close()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown(context=context)
