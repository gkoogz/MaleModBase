import unittest
import numpy as np
from malemod_base.deformation_path import repair_path
from malemod_base.surface_bending import chord_report


class DeformationPathTests(unittest.TestCase):
    def test_valid_endpoints_can_collapse_between_frames(self):
        p = np.array([[0., 0, 0], [1, 0, 0], [0, 1, 0]])
        # Half-turn endpoints have equal positive triangle area. Their straight
        # interpolation collapses at the midpoint and cannot be serialized.
        report = chord_report(p, p, -p, [[0, 1, 2]])
        self.assertFalse(report['safe'])
        self.assertEqual(report['collapsed_faces'], [0])
        self.assertAlmostEqual(report['fractions'][0], .5, places=4)
        self.assertTrue(chord_report(p, p, p+[0, 0, 1], [[0, 1, 2]])['safe'])

    def test_shared_edge_and_fixed_surrounding_rows(self):
        p = np.array([[0., 0, 0], [1, 0, 0], [0, 1, 0], [1, 1, 0], [.5, 0, 0]])
        f = np.array([[0, 4, 2], [4, 3, 2], [4, 1, 3]])
        target = p.copy()
        target[[1, 3], 2] = .6
        target[4, 2] = .3
        result = repair_path(p, target, f, [False, True, False, True, True],
                             [(4, 0, 1, .5)], steps=4)
        self.assertTrue(result.complete)
        np.testing.assert_allclose(result.positions, target, atol=1e-7)
        for knot in result.knots:
            np.testing.assert_array_equal(knot[[0, 2]], p[[0, 2]])
            np.testing.assert_allclose(knot[4], (knot[0]+knot[1])/2, atol=1e-12)

    def test_partial_path_is_not_success(self):
        p = np.array([[0., 0, 0], [1, 0, 0], [0, 1, 0], [1, 1, 0]])
        target = p.copy(); target[3] = [-1, -1, 0]
        result = repair_path(p, target, [[0, 1, 2], [1, 3, 2]],
                             [False]*4, steps=10)
        self.assertFalse(result.complete)
        self.assertLess(result.fractions[-1], 1)
        self.assertFalse(result.report['new_fold_pairs'])
        self.assertFalse(result.report['collapsed_faces'])

    def test_sliver_translation_and_invalid_seam(self):
        p = np.array([[0., 0, 0], [1, 0, 0], [0, 1e-6, 0]])
        target = p + [0, 0, 2]
        result = repair_path(p, target, [[0, 1, 2]], [True]*3, steps=3)
        self.assertTrue(result.complete)
        np.testing.assert_allclose(result.positions, target, atol=1e-7)
        with self.assertRaisesRegex(ValueError, 'donor'):
            repair_path(p, target, [[0, 1, 2]], [True]*3, [(2, 0, 1, .5)])


if __name__ == '__main__':
    unittest.main()
