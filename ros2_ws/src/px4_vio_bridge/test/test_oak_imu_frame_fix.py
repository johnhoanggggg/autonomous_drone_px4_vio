"""The OAK IMU frame fix turns the driver's samples 180 deg about IMU x.

At rest the raw OAK-D-Lite accelerometer read +9.76 on IMU +y, which the
driver's oak_imu_frame TF maps to camera DOWN. The fixed sample must point the
reaction vector UP through that same TF (IMU -y), with the header unchanged.
No OAK, PX4, or running ROS graph is needed.
"""
import os
import signal
import subprocess
import time

import pytest

rclpy = pytest.importorskip("rclpy")
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from composition_interfaces.srv import LoadNode
from sensor_msgs.msg import Imu


def test_imu_is_turned_180_deg_about_x_and_header_kept(tmp_path):
    domain = 110 + os.getpid() % 40
    context = Context()
    rclpy.init(context=context, domain_id=domain)
    node = Node("imu_fix_test", context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    log = (tmp_path / "container.log").open("w")
    process = subprocess.Popen(
        ["ros2", "run", "rclcpp_components", "component_container",
         "--ros-args", "-r", "__node:=imu_fix_container"],
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
        env={**os.environ, "ROS_DOMAIN_ID": str(domain)})
    try:
        client = node.create_client(LoadNode, "/imu_fix_container/_container/load_node")
        assert client.wait_for_service(timeout_sec=15), "component container did not start"
        request = LoadNode.Request()
        request.package_name = "px4_vio_bridge"
        request.plugin_name = "px4_vio_bridge::OakImuFrameFix"
        request.remap_rules = ["imu_in:=/imu_fix_test/in", "imu_out:=/imu_fix_test/out"]
        future = client.call_async(request)
        executor.spin_until_future_complete(future, timeout_sec=15)
        assert future.result() is not None and future.result().success
        received = []
        node.create_subscription(Imu, "/imu_fix_test/out", received.append, qos_profile_sensor_data)
        publisher = node.create_publisher(Imu, "/imu_fix_test/in", qos_profile_sensor_data)
        sample = Imu()
        sample.header.frame_id = "oak_imu_frame"
        sample.header.stamp.sec = 7
        sample.linear_acceleration.x, sample.linear_acceleration.y, sample.linear_acceleration.z = (
            0.27, 9.76, -0.12)
        sample.angular_velocity.x, sample.angular_velocity.y, sample.angular_velocity.z = (
            0.1, 0.2, 0.3)
        deadline = time.monotonic() + 10
        while not received and time.monotonic() < deadline:
            publisher.publish(sample)
            executor.spin_once(timeout_sec=0.1)
        assert received, (tmp_path / "container.log").read_text()
        out = received[0]
        assert out.header.frame_id == "oak_imu_frame" and out.header.stamp.sec == 7
        assert (out.linear_acceleration.x, out.linear_acceleration.y, out.linear_acceleration.z) == \
            pytest.approx((0.27, -9.76, 0.12))
        assert (out.angular_velocity.x, out.angular_velocity.y, out.angular_velocity.z) == \
            pytest.approx((0.1, -0.2, -0.3))
        assert out.orientation_covariance[0] == -1.0
    finally:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
        log.close()
        node.destroy_node()
        rclpy.shutdown(context=context)
