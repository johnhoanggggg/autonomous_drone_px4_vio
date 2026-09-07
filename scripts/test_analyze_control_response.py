"""Run with .venv-mavlink/bin/python3 -m unittest discover -s scripts -p test_analyze_control_response.py."""
import unittest
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

from analyze_control_response import analyse, recent_indices


class ControlResponseTest(unittest.TestCase):
    def test_matching_excludes_future_and_stale_samples(self):
        indices, valid = recent_indices(np.array([100, 300], dtype=np.uint64),
                                       np.array([90, 100, 151, 300], dtype=np.uint64), 50)
        np.testing.assert_array_equal(indices, [0, 0, 0, 1])
        np.testing.assert_array_equal(valid, [False, True, False, True])

    def test_ulog_byte_flags_remain_boolean_masks_and_window_is_respected(self):
        stamp = np.array([1000000, 1100000, 1200000], dtype=np.uint64)
        lp = {"timestamp": stamp, "xy_valid": np.ones(3, dtype=np.uint8),
              "v_xy_valid": np.ones(3, dtype=np.uint8),
              **{name: np.zeros(3) for name in ("vx", "vy", "ax", "ay", "heading")}}
        lp["vx"] = np.array([0.1, 0.2, 0.4])
        sp = {"timestamp": stamp, "vx": np.array([np.nan, 0.1, 0.1]),
              "vy": np.zeros(3), "acceleration[0]": np.zeros(3),
              "acceleration[1]": np.zeros(3)}
        attitude = {"timestamp": stamp, "q[0]": np.ones(3),
                    **{f"q[{i}]": np.zeros(3) for i in range(1, 4)}}
        log = SimpleNamespace(start_timestamp=1000000, dropouts=[], initial_parameters={},
            data_list=[SimpleNamespace(name=name, data=data, multi_id=0) for name, data in
                       [("vehicle_local_position", lp), ("vehicle_local_position_setpoint", sp),
                        ("vehicle_attitude", attitude)]])
        with patch("analyze_control_response.ULog", return_value=log):
            columns, summary = analyse("fake.ulg", start=0.05, end=0.25)
        np.testing.assert_allclose(columns["time_s"], [0.1, 0.2])
        self.assertEqual(summary["samples"], 2)
        self.assertAlmostEqual(summary["peak_actual_speed_m_s"], 0.4)
        self.assertAlmostEqual(summary["demand_speed_at_actual_peak_m_s"], 0.1)


if __name__ == "__main__":
    unittest.main()
