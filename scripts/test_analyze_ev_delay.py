import numpy as np
import pytest

from analyze_ev_delay import score_delays


@pytest.mark.parametrize('delay', [0.075, 0.225, 0.350])
def test_recovers_known_delay_with_irregular_vio_sampling(delay):
    rng = np.random.default_rng(7)
    gt = np.arange(0, 20, .005)
    et = np.cumsum(rng.uniform(.06, .09, 260))
    def angle(t):
        return .08*np.sin(2.3*t)+.04*np.sin(4.1*t)+.05*np.sin(.8*t)
    result = score_delays(et, angle(et-delay), gt, angle(gt), start=2, end=18)
    assert abs(result['delay_ms']-delay*1000) <= 5
    assert result['correlation'] > .999


def test_excludes_reset_and_gyro_dropout():
    gt = np.arange(0, 20, .005)
    gt = gt[(gt < 8) | (gt > 8.3)]
    et = np.arange(.05, 20, .075)
    def angle(t):
        return .1*np.sin(2.1*t)+.05*np.sin(4.2*t)
    ey = angle(et-.225)+(et >= 10)*1.2
    result = score_delays(et, ey, gt, angle(gt), start=2, end=18,
                          excluded=[(9.95, 10.1)])
    assert abs(result['delay_ms']-225) <= 5
    assert result['correlation'] > .999


def test_rejects_stationary_and_duplicate_timestamps():
    t = np.arange(0, 10, .02)
    with pytest.raises(ValueError, match='excitation'):
        score_delays(t, t*0, t, t*0, start=2, end=8)
    with pytest.raises(ValueError, match='increase strictly'):
        score_delays([0, 1, 1], [0, 1, 1], t, t, start=2, end=8)
