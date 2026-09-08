"""Replay must never inject previously estimated poses into a fresh estimator."""
import importlib.util
from pathlib import Path

import yaml
import pytest


SCRIPT = Path(__file__).resolve().parents[4] / "scripts/ros_rtabmap_bag.py"
spec = importlib.util.spec_from_file_location("ros_rtabmap_bag", SCRIPT)
bag = importlib.util.module_from_spec(spec)
spec.loader.exec_module(bag)


def test_replay_only_uses_raw_inputs_and_static_calibration():
    command = bag.play_command("/tmp/path with spaces", 0.5)
    assert command[3] == "/tmp/path with spaces"
    topics = command[command.index("--topics") + 1:]
    assert set(topics) == set(bag.INPUT_TOPICS)
    assert not set(topics) & set(bag.BASELINE_TOPICS)
    assert "/tf" not in topics
    assert "--clock" in command


def test_record_preserves_baseline_for_comparison():
    command = bag.record_command("/tmp/baseline")
    topics = command[command.index("--topics") + 1:]
    assert set(topics) == set(bag.INPUT_TOPICS + bag.BASELINE_TOPICS)
    assert not any(topic.startswith("/fmu/") for topic in topics)


def test_missing_calibration_is_not_replayable(tmp_path):
    metadata = {"duration": {"nanoseconds": 1_000_000_000},
                "topics_with_message_count": [
                    {"topic_metadata": {"name": topic}, "message_count": 10}
                    for topic in bag.INPUT_TOPICS if topic != "/tf_static"]}
    (tmp_path / "metadata.yaml").write_text(yaml.safe_dump(
        {"rosbag2_bagfile_information": metadata}))
    report = bag.inspect_bag(tmp_path)
    assert not report["all_inputs_recorded"]
    assert report["missing_inputs"] == ["/tf_static"]
    metadata["topics_with_message_count"].append({
        "topic_metadata": {"name": "/tf_static"}, "message_count": 1})
    (tmp_path / "metadata.yaml").write_text(yaml.safe_dump(
        {"rosbag2_bagfile_information": metadata}))
    assert bag.inspect_bag(tmp_path)["all_inputs_recorded"]


def test_report_reads_ros_bag_timestamps_without_datetime_conversion(tmp_path, monkeypatch):
    rosbag = pytest.importorskip("rosbag2_py")
    from nav_msgs.msg import Odometry
    from rclpy.serialization import serialize_message

    monkeypatch.syspath_prepend(str(SCRIPT.parent))
    path = tmp_path / "poses"
    writer = rosbag.SequentialWriter()
    writer.open(rosbag.StorageOptions(uri=str(path), storage_id="mcap"),
                rosbag.ConverterOptions("", ""))
    topic = "/rtabmap/vio_odometry"
    writer.create_topic(rosbag.TopicMetadata(
        id=0, name=topic, type="nav_msgs/msg/Odometry", serialization_format="cdr"))
    for i in range(301):
        msg = Odometry()
        msg.pose.pose.orientation.w = 1.0
        msg.pose.pose.position.x = 0.0 if i <= 100 else 0.3
        stamp = 1_000_000_000 + i * 100_000_000
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
        writer.write(topic, serialize_message(msg), stamp)
    writer.close()
    result = bag.report_bag(path, 10, 0.05)["raw_vio_pose_the_one_px4_uses"]
    assert result["recording_duration_s"] == 30.0
    assert result["hold_comparison_valid"]
    assert result["return_to_origin_error_mm"] == 300.0
