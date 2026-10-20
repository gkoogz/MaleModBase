import unittest
import numpy as np
from malemod_base.upper_profile import compress_upper_profile


class UpperProfileTests(unittest.TestCase):
    def test_monotone_columns_and_unchanged_lower_half(self):
        z = np.linspace(-2, 5, 701)
        p = np.column_stack([np.full(len(z), 3.), np.zeros(len(z)), z])
        q = compress_upper_profile(p, [0, 0, 0], [1, 0, 0], 2,
                                   [1, 3, 8], [3, 3, 3], 1, 8)
        np.testing.assert_array_equal(q[:, :2], p[:, :2])
        np.testing.assert_array_equal(q[z <= 0], p[z <= 0])
        self.assertTrue(np.all(np.diff(q[:, 2]) > 0))
        self.assertTrue(np.all(q[z > 0, 2] <= p[z > 0, 2]))

    def test_no_overshoot_and_locked_rows_are_exact(self):
        p = np.array([[3., 0, 2], [3, 1, 3], [0, 0, 5]])
        q = compress_upper_profile(p, [0, 0, 0], [1, 0, 0], 2,
                                   [1, 3, 8], [2, 2, 2], 1, 8)
        np.testing.assert_array_equal(q, p)
        q = compress_upper_profile(p, [0, 0, 0], [1, 0, 0], 2,
                                   [1, 3, 8], [3, 3, 3], 1, 8, [True, False, False])
        np.testing.assert_array_equal(q[[0, 2]], p[[0, 2]])
        self.assertLess(q[1, 2], p[1, 2])


if __name__ == '__main__':
    unittest.main()
