import unittest
import numpy as np
from malemod_base.surface_anchor import measure_anchor


class SurfaceAnchorTest(unittest.TestCase):
    def setUp(self):
        # Two sheets: the nearest anterior sheet must win, independently of
        # face winding, vertex density, or a farther surface on the same ray.
        self.p = np.array([[0., -2, -2], [0, 2, -2], [0, 0, 2],
                           [-3, -2, -2], [-3, 2, -2], [-3, 0, 2]])
        self.f = np.array([[3, 4, 5], [2, 1, 0]])

    def test_original_donors_follow_affine_body_pose(self):
        a = measure_anchor(self.p, self.f, [4, 0, 0], [-2, 0, 0], 5)
        self.assertEqual(a.triangle, 1)
        self.assertEqual(a.donors, (2, 1, 0))
        np.testing.assert_allclose(a.evaluate(self.p), [0, 0, 0])
        self.assertEqual(a.distance, 4)
        transform = np.array([[0., -1, .2], [1, 0, 0], [0, .1, 1.3]])
        translated = self.p @ transform.T + [2, 3, 5]
        np.testing.assert_allclose(a.evaluate(translated), [2, 3, 5])

    def test_scale_and_rigid_coordinate_covariance(self):
        r = np.array([[0., -1, 0], [0, 0, 1], [-1, 0, 0]])
        for scale in [.001, 1, 1000]:
            offset = np.array([12., -4, 3])*scale
            p = self.p @ r.T*scale + offset
            a = measure_anchor(p, self.f, r @ np.array([4, 0, 0])*scale+offset,
                               r @ np.array([-1, 0, 0]), 5*scale)
            np.testing.assert_allclose(a.evaluate(p), offset, atol=1e-10)
            self.assertAlmostEqual(a.distance/scale, 4)

    def test_remote_missing_and_degenerate_surfaces_rejected(self):
        for origin, direction, distance in [([4, 0, 0], [-1, 0, 0], 3),
                                            ([4, 0, 0], [1, 0, 0], 10),
                                            ([4, 0, 0], [0, 0, 0], 10),
                                            ([4, 9, 0], [-1, 0, 0], 10)]:
            with self.assertRaises(ValueError):
                measure_anchor(self.p, self.f, origin, direction, distance)
        with self.assertRaises(ValueError):
            measure_anchor(np.zeros((3, 3)), np.array([[0, 1, 2]]),
                           [4, 0, 0], [-1, 0, 0], 10)


if __name__ == '__main__':
    unittest.main()
