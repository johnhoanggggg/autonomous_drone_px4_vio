"""Quaternion/frame regressions for the onboard control comparison."""
import unittest

import numpy as np

from analyze_attitude_response import euler_degrees, wrapped_error


class AttitudeResponseTest(unittest.TestCase):
    def test_cardinal_axes_and_nonunit_quaternions(self):
        a = np.radians(30) / 2
        for axis in range(3):
            q = np.zeros((4, 1))
            q[0] = np.cos(a) * 2
            q[axis + 1] = np.sin(a) * 2
            expected = np.zeros((3, 1))
            expected[axis] = 30
            np.testing.assert_allclose(euler_degrees(q), expected, atol=1e-10)

    def test_invalid_attitude_is_not_zero_and_errors_wrap(self):
        self.assertTrue(np.isnan(euler_degrees(np.zeros((4, 1)))).all())
        np.testing.assert_allclose(wrapped_error(np.array([-179, 179]),
                                                np.array([179, -179])), [2, -2])


if __name__ == "__main__":
    unittest.main()
