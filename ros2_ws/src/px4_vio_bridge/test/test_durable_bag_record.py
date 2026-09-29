"""durable_bag_record must leave a finalized, readable bag however the flight ends.

The flight launches stop the recorder with SIGINT when the adapter exits (K/L or
auto land -> DONE); a closed terminal sends SIGHUP. Both must produce a non-empty
metadata.yaml and the "safe to unplug" line. Uses a real `ros2 bag record`.
"""
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import pytest

SCRIPT = Path(__file__).resolve().parents[1] / 'scripts' / 'durable_bag_record'


@pytest.mark.parametrize('stop_signal', [signal.SIGINT, signal.SIGHUP])
def test_bag_is_finalized_and_reported(tmp_path, stop_signal):
    domain = str(120 + os.getpid() % 40)
    env = {**os.environ, 'ROS_DOMAIN_ID': domain}
    bag = tmp_path / 'bag'
    talker = subprocess.Popen(
        ['ros2', 'topic', 'pub', '-r', '20', '/durable_test', 'std_msgs/msg/String', '{data: x}'],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, env=env, start_new_session=True)
    recorder = subprocess.Popen(
        [sys.executable, str(SCRIPT), '--sync-period', '0.5', '--',
         'ros2', 'bag', 'record', '--storage', 'mcap', '--disable-keyboard-controls',
         '--topics', '/durable_test', '--output', str(bag)],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, env=env)
    try:
        deadline = time.monotonic() + 20
        mcap = []
        while time.monotonic() < deadline and not (mcap and mcap[0].stat().st_size > 2000):
            time.sleep(0.5)
            mcap = list(bag.glob('*.mcap')) if bag.exists() else []
        time.sleep(2.0)
        recorder.send_signal(stop_signal)
        out, err = recorder.communicate(timeout=60)
    finally:
        os.killpg(talker.pid, signal.SIGINT)
        talker.wait(timeout=10)
        if recorder.poll() is None:
            recorder.kill()
    metadata = bag / 'metadata.yaml'
    assert metadata.is_file() and metadata.stat().st_size > 0, err
    assert 'BAG SAVED to disk' in out and 'safe to unplug' in out, (out, err)
    assert 'message_count' in metadata.read_text()
    info = subprocess.run(['ros2', 'bag', 'info', str(bag)], capture_output=True, text=True, env=env)
    assert '/durable_test' in info.stdout, info.stderr
