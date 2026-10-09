from px4_vio_bridge.slam_map_store import ARMING_STATE_ARMED, ARMING_STATE_DISARMED
from px4_vio_bridge.vio_reset import VioResetPolicy, is_lost_pose

SENTINEL_Q = (0.5, 0.5, 0.5, 0.5)


def test_sentinel_is_lost_either_sign():
    assert is_lost_pose((0.0, 0.0, 0.0), SENTINEL_Q)
    assert is_lost_pose((0.0, 0.0, 0.0), (-0.5, -0.5, -0.5, -0.5))


def test_real_pose_is_not_lost():
    # A stationary VIO is noisy, never bit-exact zero.
    assert not is_lost_pose((1e-4, 0.0, 0.0), SENTINEL_Q)
    assert not is_lost_pose((0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0))


def lost_run(policy, start, end, step=0.05):
    t = start
    while t <= end + 1e-9:
        policy.observe(True, t)
        t += step


def test_no_reset_before_first_output():
    policy = VioResetPolicy(lost_s=0.5)
    assert not policy.should_reset(10.0)


def test_resets_after_continuous_loss_while_disarmed():
    policy = VioResetPolicy(lost_s=0.5)
    policy.set_arming_state(ARMING_STATE_DISARMED)
    lost_run(policy, 0.0, 0.4)
    assert not policy.should_reset(0.4)
    lost_run(policy, 0.45, 0.5)
    assert policy.should_reset(0.5)


def test_good_pose_restarts_the_loss_timer():
    policy = VioResetPolicy(lost_s=0.5)
    lost_run(policy, 0.0, 0.4)
    policy.observe(False, 0.45)
    lost_run(policy, 0.5, 0.9)
    assert not policy.should_reset(0.9)
    assert policy.should_reset(1.0)


def test_never_resets_while_armed():
    policy = VioResetPolicy(lost_s=0.5)
    policy.set_arming_state(ARMING_STATE_ARMED)
    lost_run(policy, 0.0, 5.0)
    assert not policy.should_reset(5.0)
    policy.set_arming_state(ARMING_STATE_DISARMED)
    assert policy.should_reset(5.0)


def test_no_status_allows_bench_reset():
    policy = VioResetPolicy(lost_s=0.5)
    lost_run(policy, 0.0, 0.6)
    assert policy.should_reset(0.6)


def test_cooldown_between_resets():
    policy = VioResetPolicy(lost_s=0.5, cooldown_s=2.0)
    lost_run(policy, 0.0, 0.6)
    policy.mark_reset(0.6)
    assert policy.resets == 1
    lost_run(policy, 0.65, 2.0)
    assert not policy.should_reset(2.0)
    lost_run(policy, 2.05, 2.6)
    assert policy.should_reset(2.6)
