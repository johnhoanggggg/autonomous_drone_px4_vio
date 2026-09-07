"""Replay must never inject previously estimated poses into a fresh estimator."""
import importlib.util
from pathlib import Path

import yaml


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
