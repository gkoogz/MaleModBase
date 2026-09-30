import unittest
import numpy as np
from malemod_base.motion_binding import linear_chain_weights


class MotionBindingTests(unittest.TestCase):
    def test_affine_reproduction_and_partition(self):
        knots = np.array([0., .1, .4, .8, 1.])
        x = np.linspace(-.1, 1.1, 301)
        weights = linear_chain_weights(x, knots)
        np.testing.assert_allclose(weights.sum(1), 1)
        np.testing.assert_allclose(weights @ knots, np.clip(x, 0, 1), atol=1e-15)
        self.assertTrue((weights >= 0).all())
        self.assertTrue((np.count_nonzero(weights, axis=1) <= 2).all())

    def test_reject_invalid_knots(self):
        for knots in ([0, 0], [1, 0], [0, np.nan], [0]):
            with self.assertRaises(ValueError):
                linear_chain_weights([.5], knots)
