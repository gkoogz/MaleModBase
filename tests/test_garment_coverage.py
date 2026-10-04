import unittest
import numpy as np
from malemod_base.garment_coverage import ray_depth, measure_side_coverage


class CoverageTests(unittest.TestCase):
    def setUp(self):
        self.v = np.array([[0., 0., 0.], [0., 1., 0.], [0., 0., 1.]])
        self.f = np.array([[0, 1, 2]], dtype=np.uint32)
        self.empty_v = np.empty((0, 3))
        self.empty_f = np.empty((0, 3), dtype=np.uint32)

    def measure(self, cloth, body=None, **kwargs):
        return measure_side_coverage(self.v, self.f,
            self.empty_v if body is None else body,
            self.empty_f if body is None else self.f,
            cloth, self.empty_f if not len(cloth) else self.f, **kwargs)

    def test_frontmost_depth_and_winding(self):
        v = np.concatenate((self.v + [2, 0, 0], self.v + [3, 0, 0]))
        f = np.array([[0, 1, 2], [3, 4, 5]])
        query = np.array([[0., .25, .25], [0, 2, 2], [0, 0, 0]])
        depth = ray_depth(v, f, query)
        self.assertEqual(depth[0], 3)
        self.assertTrue(np.isneginf(depth[1]))
        self.assertEqual(depth[2], 3)
        np.testing.assert_array_equal(depth, ray_depth(v, f[:, ::-1], query))
        v[:, 0] = 2 + v[:, 1] * 4 + v[:, 2] * 6
        self.assertAlmostEqual(ray_depth(v, f, query)[0], 4.5)

    def test_cloth_depth_is_not_visibility_mask(self):
        self.assertEqual(self.measure(self.v + [1, 0, 0])['coveredFraction'], 1)
        self.assertEqual(self.measure(self.v - [1, 0, 0])['coveredFraction'], 0)
        self.assertEqual(self.measure(self.empty_v)['coveredFraction'], 0)

    def test_native_body_occlusion_removed(self):
        result = self.measure(self.empty_v, body=self.v + [1, 0, 0])
        self.assertIsNone(result['coveredFraction'])
        self.assertEqual(result['visibleArea'], 0)

    def test_self_occlusion_and_side(self):
        v = np.concatenate((self.v, self.v + [1, 0, 0]))
        f = np.array([[0, 1, 2], [3, 4, 5]])
        result = measure_side_coverage(v, f, self.empty_v, self.empty_f,
                                      self.empty_v, self.empty_f)
        np.testing.assert_array_equal(result['visible'], [False, True])
        result = measure_side_coverage(-v, f[:, ::-1], self.empty_v,
                    self.empty_f, self.empty_v, self.empty_f, side=-1)
        np.testing.assert_array_equal(result['visible'], [False, True])

    def test_invalid_inputs_rejected(self):
        with self.assertRaises(ValueError):
            ray_depth(self.v, np.array([[0., 1., 2.]]), self.v)
        with self.assertRaises(ValueError):
            ray_depth(self.v, np.array([[0, 1, 3]]), self.v)
        with self.assertRaises(ValueError):
            ray_depth(self.v * np.nan, self.f, self.v)
        with self.assertRaises(ValueError):
            self.measure(self.empty_v, side=0)


if __name__ == '__main__':
    unittest.main()
