import os
import threading
import time

import numpy as np
import pytest
from nav_msgs.msg import OccupancyGrid

from px4_vio_bridge import slam_map_store as store


class FakeSlam:
    """Mimics RTABMapSLAM::run(): saves when elapsed > period, then resets."""

    def __init__(self, path, write_s=0.05):
        self.path = path
        self.period = None
        self.periodic = False
        self.write_s = write_s
        self.saves = 0
        self.last = time.monotonic()
        self.stop = False
        self.thread = threading.Thread(target=self.loop, daemon=True)
        self.thread.start()

    def setSaveDatabasePeriod(self, period):
        self.period = period

    def setSaveDatabasePeriodically(self, enabled):
        self.periodic = enabled

    def loop(self):
        while not self.stop:
            if self.periodic and self.period is not None and \
                    time.monotonic() - self.last > self.period:
                with open(self.path, 'ab') as handle:
                    handle.write(b'x' * 1000)
                time.sleep(self.write_s)
                self.saves += 1
                self.last = time.monotonic()
            time.sleep(0.005)


def test_arming_edges():
    edges = store.ArmingEdges()
    assert edges.update(1) is None  # first sample is not an edge
    assert edges.update(1) is None
    assert edges.update(2) == 'arm'
    assert edges.update(2) is None
    assert edges.update(1) == 'disarm'


def test_prepare_refuses_existing_database(tmp_path):
    db = tmp_path / 'map.db'
    assert 'new map' in store.prepare_database(str(db))
    db.write_bytes(b'old')
    with pytest.raises(FileExistsError):
        store.prepare_database(str(db))


def test_prepare_reuse_copies_and_never_touches_source(tmp_path):
    source = tmp_path / 'old.db'
    source.write_bytes(b'saved map')
    session = tmp_path / 'sessions' / 'new.db'
    store.prepare_database(str(session), str(source))
    assert session.read_bytes() == b'saved map'
    with pytest.raises(ValueError):
        store.prepare_database(str(source), str(source))
    with pytest.raises(FileNotFoundError):
        store.prepare_database(str(tmp_path / 'other.db'), str(tmp_path / 'missing.db'))


def test_trigger_saves_exactly_once_per_request(tmp_path):
    db = tmp_path / 'map.db'
    db.write_bytes(b'')
    slam = FakeSlam(str(db))
    reports = []
    trigger = store.MapSaveTrigger(slam, str(db), reports.append, settle_s=0.1)
    try:
        time.sleep(0.3)
        assert slam.saves == 0  # parked: the idle period never fires
        assert trigger.save_once('arm')
        time.sleep(0.8)  # longer than TRIGGER_PERIOD_S: must not save again
        assert slam.saves == 1
        assert slam.period == store.IDLE_PERIOD_S
        assert reports[-1]['state'] == 'saved' and reports[-1]['reason'] == 'arm'
    finally:
        slam.stop = True
        trigger.stop()


def test_trigger_reports_failure_when_nothing_is_written(tmp_path):
    db = tmp_path / 'map.db'
    db.write_bytes(b'')

    class DeadSlam:
        def setSaveDatabasePeriod(self, period):
            self.period = period

        def setSaveDatabasePeriodically(self, enabled):
            pass

    slam = DeadSlam()
    reports = []
    trigger = store.MapSaveTrigger(slam, str(db), reports.append, start_timeout_s=0.2)
    assert not trigger.save_once('disarm')
    assert reports[-1]['state'] == 'failed'
    assert slam.period == store.IDLE_PERIOD_S
    trigger.stop()


def test_hooks_bracket_the_save(tmp_path):
    db = tmp_path / 'map.db'
    db.write_bytes(b'')
    slam = FakeSlam(str(db))
    reports = []
    trigger = store.MapSaveTrigger(
        slam, str(db), reports.append, settle_s=0.05,
        before_save=lambda reason: {'seen': reason},
        after_save=lambda reason, before: {'relinked': before['seen'] == reason})
    try:
        assert trigger.save_once('manual')
        assert reports[-1]['relinked'] is True
    finally:
        slam.stop = True
        trigger.stop()


def grid_msg(data, origin, res=0.03, frame='world'):
    msg = OccupancyGrid()
    msg.header.frame_id = frame
    data = np.asarray(data, dtype=np.int8)
    msg.info.resolution = res
    msg.info.height, msg.info.width = data.shape
    msg.info.origin.position.x, msg.info.origin.position.y = origin
    msg.data = data.reshape(-1).tolist()
    return msg


def test_sidecar_round_trip(tmp_path):
    db = str(tmp_path / 'map.db')
    msg = grid_msg([[0, 100], [-1, 0]], (0.30, -0.60))
    store.save_grid_sidecar(db, msg)
    prior = store.load_grid_sidecar(db)
    assert prior['origin'] == pytest.approx((0.30, -0.60))
    assert prior['data'].tolist() == [[0, 100], [-1, 0]]
    assert store.load_grid_sidecar(str(tmp_path / 'none.db')) is None


def test_merge_live_wins_and_prior_fills_unknown():
    res = 0.03
    # Prior: 2x3 at origin (0, 0); live: 2x2 shifted one cell right.
    prior = {'data': np.array([[100, 100, 100], [0, 0, 0]], dtype=np.int8),
             'origin': (0.0, 0.0), 'resolution': res, 'frame_id': 'world',
             'path': 'p'}
    live = grid_msg([[0, -1], [-1, 100]], (res, 0.0))
    out = store.merge_prior_grid(prior, live)
    merged = np.asarray(out.data, dtype=np.int8).reshape(out.info.height, out.info.width)
    assert out.info.origin.position.x == pytest.approx(0.0)
    # Live free cell overrides the prior wall; live unknown keeps the prior.
    assert merged.tolist() == [[100, 0, 100], [0, 0, 100]]


def test_merge_extends_to_union_and_rejects_mismatch():
    prior = {'data': np.array([[100]], dtype=np.int8), 'origin': (-0.06, 0.0),
             'resolution': 0.03, 'frame_id': 'world', 'path': 'p'}
    out = store.merge_prior_grid(prior, grid_msg([[0]], (0.0, 0.0)))
    assert (out.info.width, out.info.height) == (3, 1)
    assert list(out.data) == [100, -1, 0]
    with pytest.raises(ValueError):
        store.merge_prior_grid(dict(prior, resolution=0.05), grid_msg([[0]], (0.0, 0.0)))
    with pytest.raises(ValueError):
        store.merge_prior_grid(prior, grid_msg([[0]], (0.0, 0.0), frame='map'))


def test_correction_helpers():
    assert store.correction_is_identity(np.eye(4))
    moved = np.eye(4)
    moved[0, 3] = 0.02
    assert not store.correction_is_identity(moved)
    yawed = np.eye(4)
    yawed[:2, :2] = [[0.0, -1.0], [1.0, 0.0]]
    shift, yaw = store.correction_delta(np.eye(4), yawed)
    assert shift == pytest.approx(0.0) and yaw == pytest.approx(90.0)
    assert os.path.basename(store.grid_sidecar_path('/a/b.db')) == 'b.db.grid.npz'


def test_prior_alignment_accepts_matching_walls_and_rejects_shifted():
    res = 0.03
    data = np.zeros((40, 40), dtype=np.int8)
    data[10, 5:35] = 100  # one wall
    prior = {'data': data, 'origin': (0.0, 0.0), 'resolution': res,
             'frame_id': 'world', 'path': 'p'}
    same = grid_msg(data, (0.0, 0.0))
    score, n = store.prior_alignment(prior, same)
    assert n == 30 and score == pytest.approx(1.0)
    # The same wall seen 15 cm (5 cells) off: outside the 2-cell search.
    shifted = grid_msg(data, (0.0, 5 * res))
    score, n = store.prior_alignment(prior, shifted)
    assert n > 0 and score == pytest.approx(0.0)
    # Live walls outside the prior's observed area say nothing.
    far = grid_msg(data, (10.0, 10.0))
    assert store.prior_alignment(prior, far) == (0.0, 0)
