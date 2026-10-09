"""Save and reuse the DepthAI RTABMapSLAM database.

DepthAI 3.5.0 gives no safe way to save from another thread:
RTABMapSLAM::saveDatabase() closes and re-opens RTAB-Map while the node thread
may be inside process(). Its periodic save, however, runs in the node's own
loop. MapSaveTrigger drives that loop: the period is parked at IDLE_PERIOD_S
(never fires), dropped to TRIGGER_PERIOD_S for one save, and parked again as
soon as the database file shows the write.

Every save is RTAB-Map close() + init(). init() on an existing database starts
a NEW map id that is not linked to the old one until a loop closure, so a
save is not free: see the bench numbers where this is wired in.
"""

import copy
import json
import math
import os
import shutil
import threading
import time

import numpy as np

IDLE_PERIOD_S = 1.0e9
TRIGGER_PERIOD_S = 0.5

ARMING_STATE_DISARMED = 1
ARMING_STATE_ARMED = 2


def file_signature(path):
    try:
        st = os.stat(path)
    except FileNotFoundError:
        return None
    return (st.st_mtime_ns, st.st_size)


def fsync_file(path):
    """Push a finished database to the SD card; the Pi loses power at pack swap."""
    with open(path, 'rb') as handle:
        os.fsync(handle.fileno())
    directory = os.open(os.path.dirname(os.path.abspath(path)) or '.', os.O_RDONLY)
    try:
        os.fsync(directory)
    finally:
        os.close(directory)


def prepare_database(path, load_from=''):
    """Make `path` the session database, optionally seeded from `load_from`.

    RTAB-Map loads whatever .db already sits at the path it is given and keeps
    mapping into it, so an existing file is never reused by accident: reuse is
    only through `load_from`, which is copied and never modified.
    Returns a short description for the log.
    """
    if not path.endswith('.db'):
        raise ValueError(f'SLAM database path must end in .db: {path}')
    directory = os.path.dirname(os.path.abspath(path))
    os.makedirs(directory, exist_ok=True)
    if load_from:
        source = os.path.abspath(os.path.expanduser(load_from))
        if not os.path.isfile(source):
            raise FileNotFoundError(f'map to reuse not found: {source}')
        if os.path.abspath(path) == source:
            raise ValueError(
                'reuse copies the saved map into a new session file; '
                'pass a different --slam-database-path')
        if os.path.exists(path):
            raise FileExistsError(f'session database already exists: {path}')
        shutil.copy2(source, path)
        fsync_file(path)
        return f'reusing {source} (copied to {path})'
    if os.path.exists(path):
        raise FileExistsError(
            f'{path} already exists and RTAB-Map would load it. Delete it, '
            'choose a new path, or reuse it with --slam-load-database')
    return f'new map at {path}'


class ArmingEdges:
    """Turn PX4 VehicleStatus.arming_state samples into arm/disarm events."""

    def __init__(self):
        self.state = None

    def update(self, arming_state):
        previous, self.state = self.state, int(arming_state)
        if previous is None or previous == self.state:
            return None
        if self.state == ARMING_STATE_ARMED:
            return 'arm'
        if previous == ARMING_STATE_ARMED:
            return 'disarm'
        return None


class MapSaveTrigger:
    """Run one RTABMapSLAM save in the SLAM node's own thread per request.

    `slam` needs setSaveDatabasePeriodically/setSaveDatabasePeriod; `report`
    is called with a dict for every state change. `before_save(reason)` and
    `after_save(reason, before)`, if given, bracket each save; the latter
    returns extra fields for the 'saved' report. Requests made while a
    save is running are coalesced into one follow-up save.
    """

    def __init__(self, slam, path, report, before_save=None, after_save=None,
                 clock=time.monotonic,
                 sleep=time.sleep, start_timeout_s=30.0, settle_s=0.5, poll_s=0.02):
        self.slam = slam
        self.path = path
        self.report = report
        self.before_save = before_save
        self.after_save = after_save
        self.clock = clock
        self.sleep = sleep
        self.start_timeout_s = start_timeout_s
        self.settle_s = settle_s
        self.poll_s = poll_s
        self._lock = threading.Lock()
        self._pending = []
        self._wake = threading.Event()
        self._stop = False
        self.saves = 0
        slam.setSaveDatabasePeriod(IDLE_PERIOD_S)
        slam.setSaveDatabasePeriodically(True)
        self._thread = threading.Thread(target=self._worker, daemon=True)
        self._thread.start()

    def request(self, reason):
        with self._lock:
            if reason not in self._pending:
                self._pending.append(reason)
        self.report({'state': 'requested', 'reason': reason, 'path': self.path})
        self._wake.set()

    def stop(self):
        self._stop = True
        self._wake.set()

    def _worker(self):
        while not self._stop:
            self._wake.wait()
            self._wake.clear()
            while not self._stop:
                with self._lock:
                    if not self._pending:
                        break
                    reasons, self._pending = self._pending, []
                self.save_once('+'.join(reasons))

    def save_once(self, reason):
        context = self.before_save(reason) if self.before_save is not None else None
        before = file_signature(self.path)
        start = self.clock()
        self.slam.setSaveDatabasePeriod(TRIGGER_PERIOD_S)
        try:
            # Wait for the node loop to start writing. RTAB-Map writes the
            # database only in close(), so any change is this save.
            while file_signature(self.path) == before:
                if self.clock() - start > self.start_timeout_s:
                    self.report({'state': 'failed', 'reason': reason, 'path': self.path,
                                 'error': f'no write within {self.start_timeout_s:.0f} s'})
                    return False
                self.sleep(self.poll_s)
        finally:
            # Park the period before the loop can come round again.
            self.slam.setSaveDatabasePeriod(IDLE_PERIOD_S)
        started = self.clock() - start
        # close() is still writing; wait for the file to stop changing.
        last = file_signature(self.path)
        stable_since = self.clock()
        while self.clock() - stable_since < self.settle_s:
            self.sleep(self.poll_s)
            current = file_signature(self.path)
            if current != last:
                last, stable_since = current, self.clock()
        try:
            fsync_file(self.path)
        except OSError as exc:
            self.report({'state': 'failed', 'reason': reason, 'path': self.path,
                         'error': f'fsync: {exc}'})
            return False
        self.saves += 1
        status = {
            'state': 'saved', 'reason': reason, 'path': self.path,
            'bytes': last[1] if last else 0,
            'start_delay_s': round(started, 3),
            'seconds': round(self.clock() - start, 3),
            'count': self.saves,
        }
        if self.after_save is not None:
            try:
                status.update(self.after_save(reason, context) or {})
            except Exception as exc:  # the database itself is already safe
                status['after_save_error'] = str(exc)
        self.report(status)
        return True


# --- Prior grid -------------------------------------------------------------
#
# The planner's /rtabmap/grid is assembled from DepthAI's in-memory cache of
# local grids, which only ever holds nodes processed in THIS process. A loaded
# database therefore relocalizes the drone but contributes no obstacles. The
# grid the planner last saw is stored next to the database instead, and merged
# back under the live grid once the reused map has linked.

def grid_sidecar_path(db_path):
    return db_path + '.grid.npz'


def save_grid_sidecar(db_path, grid_msg, extra=None):
    """Write the published OccupancyGrid next to the database, atomically."""
    target = grid_sidecar_path(db_path)
    tmp = target + '.tmp'
    data = np.asarray(grid_msg.data, dtype=np.int8).reshape(
        grid_msg.info.height, grid_msg.info.width)
    with open(tmp, 'wb') as handle:
        np.savez_compressed(
            handle, data=data,
            origin=np.array([grid_msg.info.origin.position.x,
                             grid_msg.info.origin.position.y]),
            resolution=np.array(grid_msg.info.resolution),
            frame_id=np.array(grid_msg.header.frame_id),
            meta=np.array(json.dumps(extra or {}, sort_keys=True)))
        handle.flush()
        os.fsync(handle.fileno())
    os.replace(tmp, target)
    fsync_file(target)
    return target


def load_grid_sidecar(db_path):
    path = grid_sidecar_path(db_path)
    if not os.path.isfile(path):
        return None
    with np.load(path) as z:
        return {
            'data': z['data'].astype(np.int8),
            'origin': (float(z['origin'][0]), float(z['origin'][1])),
            'resolution': float(z['resolution']),
            'frame_id': str(z['frame_id']),
            'path': path,
        }


def merge_prior_grid(prior, live_msg):
    """Return live_msg with the prior filling the cells live has not observed.

    Live knowledge always wins, so furniture that moved is corrected wherever
    this session has looked. Both grids must share frame and resolution; the
    output covers the union of their extents.
    """
    res = float(live_msg.info.resolution)
    if abs(prior['resolution'] - res) > 1e-9:
        raise ValueError(f'prior grid resolution {prior["resolution"]} != live {res}')
    if prior['frame_id'] != live_msg.header.frame_id:
        raise ValueError(f'prior grid frame {prior["frame_id"]} != live '
                         f'{live_msg.header.frame_id}')
    live = np.asarray(live_msg.data, dtype=np.int8).reshape(
        live_msg.info.height, live_msg.info.width)
    lx, ly = live_msg.info.origin.position.x, live_msg.info.origin.position.y
    px, py = prior['origin']
    # Both origins sit on the same RTAB-Map cell lattice; round, do not floor.
    lc, lr = round(lx / res), round(ly / res)
    pc, pr = round(px / res), round(py / res)
    c0, r0 = min(lc, pc), min(lr, pr)
    c1 = max(lc + live.shape[1], pc + prior['data'].shape[1])
    r1 = max(lr + live.shape[0], pr + prior['data'].shape[0])
    out = np.full((r1 - r0, c1 - c0), -1, dtype=np.int8)
    ph, pw = prior['data'].shape
    out[pr - r0:pr - r0 + ph, pc - c0:pc - c0 + pw] = prior['data']
    view = out[lr - r0:lr - r0 + live.shape[0], lc - c0:lc - c0 + live.shape[1]]
    known = live >= 0
    view[known] = live[known]
    msg = copy.deepcopy(live_msg)
    msg.info.width = int(out.shape[1])
    msg.info.height = int(out.shape[0])
    msg.info.origin.position.x = c0 * res
    msg.info.origin.position.y = r0 * res
    msg.data = out.reshape(-1).tolist()
    return msg


def correction_is_identity(matrix, tol=1e-6):
    """RTAB-Map's map correction is exactly identity until a loop closure."""
    return bool(np.allclose(np.asarray(matrix, dtype=float), np.eye(4), atol=tol))


def correction_delta(a, b):
    """Translation (m) and yaw (deg) between two 4x4 map corrections."""
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    d = np.linalg.inv(a) @ b
    yaw = math.degrees(math.atan2(d[1, 0], d[0, 0]))
    return float(np.linalg.norm(d[:3, 3])), abs(yaw)


def prior_alignment(prior, live_msg, occupied=50, search_cells=2):
    """Score how well the live walls land on the prior's walls.

    RTAB-Map's link puts this session in the frame of whichever saved map piece
    it recognised, which is not necessarily the piece the prior grid was saved
    in (a session restarted by a save may never have linked back). So the
    prior is only merged once the grids themselves agree. Returns the fraction
    of live occupied cells inside the prior's observed area that have a prior
    occupied cell within `search_cells`, and how many such live cells there were.
    """
    res = float(live_msg.info.resolution)
    if abs(prior['resolution'] - res) > 1e-9:
        return 0.0, 0
    live = np.asarray(live_msg.data, dtype=np.int8).reshape(
        live_msg.info.height, live_msg.info.width)
    data = prior['data']
    ph, pw = data.shape
    prior_occ = data > occupied
    # Dilate the prior's walls by search_cells (Chebyshev) without scipy.
    pad = np.pad(prior_occ, search_cells)
    near = np.zeros_like(prior_occ)
    for dr in range(2 * search_cells + 1):
        for dc in range(2 * search_cells + 1):
            near |= pad[dr:dr + ph, dc:dc + pw]
    rows, cols = np.nonzero(live > occupied)
    dc = round(live_msg.info.origin.position.x / res) - round(prior['origin'][0] / res)
    dr = round(live_msg.info.origin.position.y / res) - round(prior['origin'][1] / res)
    pr, pc = rows + dr, cols + dc
    inside = (pr >= 0) & (pr < ph) & (pc >= 0) & (pc < pw)
    pr, pc = pr[inside], pc[inside]
    observed = data[pr, pc] >= 0
    pr, pc = pr[observed], pc[observed]
    if pr.size == 0:
        return 0.0, 0
    return float(near[pr, pc].mean()), int(pr.size)
