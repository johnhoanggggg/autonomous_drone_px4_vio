"""Decide when to reset a lost DepthAI RTABMapVIO, and only on the ground.

RTABMapVIO runs rtabmap Odometry with Odom/ResetCountdown=0: once tracking is
lost it never re-initialises, and every later pose is the null transform
(the bridge's "RTABMap reset sentinel"). A loss in the first frames after
start, or while the drone is handled, therefore lasts the whole run.

rtabmap's own ResetCountdown is not used: in flight it re-initialises a fresh
map at the last pose, whose first frame "succeeds" with zero motion, so a real
loss alternates a frozen pose with the sentinel and defeats the hover node's
persistent-sentinel landing check. Resetting only while disarmed keeps the
in-flight behaviour (sentinel -> land) unchanged.
"""

import threading

from px4_vio_bridge.slam_map_store import ARMING_STATE_ARMED


def is_lost_pose(translation, quaternion, position_epsilon=1.0e-6,
                 orientation_epsilon=1.0e-3):
    """True for RTABMapVIO's null-transform output.

    Matches vio_to_px4_odometry.pose_rejection_reason: zero translation and
    an all-(+/-)0.5 quaternion (the zero rotation block, normalised).
    """
    if any(abs(v) > position_epsilon for v in translation):
        return False
    return min(
        max(abs(v - 0.5) for v in quaternion),
        max(abs(v + 0.5) for v in quaternion),
    ) <= orientation_epsilon


class VioResetPolicy:
    """Request a reset once the VIO has been lost for `lost_s` while disarmed.

    Arming state comes from PX4 VehicleStatus. With no status yet (bench, no
    flight controller) resets are allowed; once ARMED is seen they stop until a
    later status says otherwise, so a dropped PX4 link mid-flight never enables
    them. `cooldown_s` gives a fresh VIO time to initialise before the next try.
    """

    def __init__(self, lost_s=0.5, cooldown_s=2.0):
        self.lost_s = lost_s
        self.cooldown_s = cooldown_s
        self._lock = threading.Lock()
        self._armed = False
        self._seen_pose = False
        self._lost_since = None
        self._last_reset = None
        self.resets = 0

    def set_arming_state(self, arming_state):
        with self._lock:
            self._armed = int(arming_state) == ARMING_STATE_ARMED

    @property
    def armed(self):
        with self._lock:
            return self._armed

    def observe(self, lost, now):
        with self._lock:
            # The first output proves RTABMapVIO initialised its odometry;
            # reset() before that dereferences a null pointer.
            self._seen_pose = True
            if not lost:
                self._lost_since = None
            elif self._lost_since is None:
                self._lost_since = now

    def lost_for(self, now):
        with self._lock:
            return 0.0 if self._lost_since is None else now - self._lost_since

    def should_reset(self, now):
        with self._lock:
            if not self._seen_pose or self._armed or self._lost_since is None:
                return False
            if now - self._lost_since < self.lost_s:
                return False
            return self._last_reset is None or now - self._last_reset >= self.cooldown_s

    def mark_reset(self, now):
        with self._lock:
            self._last_reset = now
            self._lost_since = None
            self.resets += 1
