import unittest
import numpy as np
from malemod_base.surface_bending import adjacent_faces, bending_report


class SurfaceBendingTests(unittest.TestCase):
    def setUp(self):
        self.points = np.array([[0., 0., 0.], [1., 0., 0.],
                                [0., 1., 0.], [1., 1., 0.]])
        self.faces = np.array([[0, 1, 2], [1, 3, 2]])

    def test_large_rigid_rotation_is_not_a_fold(self):
        rotation = np.diag([1., -1., -1.])
        report = bending_report(self.points, self.points @ rotation, self.faces)
        self.assertEqual(report['new_fold_pairs'], [])
        self.assertEqual(report['collapsed_faces'], [])
        self.assertAlmostEqual(report['minimum_area_ratio'], 1.)

    def test_fold_detected_even_with_full_triangle_area(self):
        folded = self.points.copy()
        folded[3] = [0., 0., 0.]
        report = bending_report(self.points, folded, self.faces)
        self.assertEqual(len(report['new_fold_pairs']), 1)
        self.assertEqual(report['collapsed_faces'], [])
        self.assertAlmostEqual(report['minimum_area_ratio'], 1.)

    def test_collapsed_face_is_separate_from_bending(self):
        collapsed = self.points.copy()
        collapsed[3] = [.5, .5, 0.]
        report = bending_report(self.points, collapsed, self.faces)
        self.assertEqual(report['collapsed_faces'], [1])

    def test_nonmanifold_edge_is_rejected(self):
        with self.assertRaisesRegex(ValueError, 'Nonmanifold'):
            adjacent_faces(np.array([[0, 1, 2], [1, 0, 3], [0, 1, 4]]))


if __name__ == '__main__':
    unittest.main()
