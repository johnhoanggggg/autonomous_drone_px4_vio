"""No hardware: verify local recorder topic selection."""
import importlib.util
import pytest
from pathlib import Path

SCRIPT = Path(__file__).resolve().parents[4] / 'scripts' / 'ros_parity_local_carry.py'
spec = importlib.util.spec_from_file_location('local_carry', SCRIPT)
carry = importlib.util.module_from_spec(spec)
spec.loader.exec_module(carry)


def test_default_stays_outputs_only():
    cmd = carry.record_command(Path('/tmp/example'))
    topics = cmd[cmd.index('--topics') + 1:]
    assert '/rtabmap/vio_odom_info_lite' in topics
    assert '/tf_static' in topics
    assert not any(t.startswith('/oak/') for t in topics)


def test_full_inputs_include_every_replay_input_once():
    cmd = carry.record_command(Path('/tmp/example'), True)
    topics = cmd[cmd.index('--topics') + 1:]
    assert set(['/oak/left/image_rect', '/oak/stereo/image_raw', '/oak/left/camera_info',
                '/oak/left_rect_feature_tracker/tracked_features', '/oak/imu/data', '/tf_static']) <= set(topics)
    assert len(topics) == len(set(topics))
    assert '--qos-profile-overrides-path' in cmd
    assert Path(cmd[cmd.index('--qos-profile-overrides-path') + 1]).is_file()
    assert not any(t.startswith('/fmu/') for t in topics)
    assert '/rtabmap/vio_odom_info' not in topics


def test_archive_verified_and_source_preserved(tmp_path):
    source = tmp_path/'ram'
    (source/'bag').mkdir(parents=True)
    (source/'bag/data.mcap').write_bytes(b'example capture' * 100)
    (source/'run.json').write_text('{"complete": true}')
    dest = tmp_path/'saved'
    assert carry.archive_capture(source, dest) == dest
    assert (dest/'bag/data.mcap').read_bytes() == (source/'bag/data.mcap').read_bytes()
    assert (source/'run.json').exists()
    assert not (tmp_path/'saved.partial').exists()


@pytest.mark.parametrize('name', ['saved', 'saved.partial'])
def test_archive_refuses_existing_destination(tmp_path, name):
    source = tmp_path/'ram'
    source.mkdir()
    (tmp_path/name).mkdir()
    with pytest.raises(FileExistsError):
        carry.archive_capture(source, tmp_path/'saved')
    assert source.exists()


def test_archive_corruption_preserves_source(tmp_path, monkeypatch):
    source = tmp_path/'ram'
    source.mkdir()
    (source/'data').write_bytes(b'original')
    copy = carry.shutil.copytree
    def corrupt(src, dst):
        copy(src, dst)
        (dst/'data').write_bytes(b'corrupted')
    monkeypatch.setattr(carry.shutil, 'copytree', corrupt)
    with pytest.raises(RuntimeError, match='checksum mismatch'):
        carry.archive_capture(source, tmp_path/'saved')
    assert (source/'data').read_bytes() == b'original'
    assert not (tmp_path/'saved').exists()
    assert (tmp_path/'saved.partial').exists()
