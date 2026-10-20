import unittest
import numpy as np
from malemod_base.constrained_attachment_fit import fit_surface


class AttachmentFitTest(unittest.TestCase):
    def test_clearance_on_interpolated_seam_moves_original_donors(self):
        # Both rim donors lie on the unit section, but their visible chord
        # cuts through it. The sample must participate in the coupled solve.
        p = np.array([[-.8, .6, 0], [.8, .6, 0], [0, 2, 0], [0, .6, 0]])
        faces = [[0, 3, 2], [3, 1, 2]]
        q, report = fit_surface(p, faces, p, [False, False, True, False],
                                [3], [[0, 1, 0]], [1.],
                                seams=[[3, 0, 1, .5]], iterations=4000,
                                tolerance=1e-8)
        self.assertGreaterEqual(q[3, 1], 1.-1e-11)
        np.testing.assert_allclose(q[3], .5*(q[0]+q[1]), atol=1e-13)
        np.testing.assert_array_equal(q[2], p[2])
        self.assertGreater(np.linalg.norm(q[:2]-p[:2]), .1)
        self.assertLess(report['residual'], 1e-8)

    def test_joint_clearance_keeps_donors_and_surrounding_tissue_exact(self):
        p = np.array([[0., 0, 0], [1, 0, 0], [0, 1, 0],
                      [1, 1, 0], [.5, 0, 0]])
        faces = [[0, 4, 2], [4, 1, 2], [1, 3, 2]]
        fixed = np.array([True, False, True, True, False])
        for scale in [.01, 1., 100.]:
            q, report = fit_surface(
                p*scale, faces, p*scale, fixed, [1, 1],
                [[1, 0, 0], [0, 0, 1]], np.array([1.2, .4])*scale,
                iterations=4000, tolerance=1e-8*scale,
                seams=[[4, 0, 1, .5]])
            self.assertGreaterEqual(q[1, 0], 1.2*scale-1e-10)
            self.assertGreaterEqual(q[1, 2], .4*scale-1e-10)
            np.testing.assert_array_equal(q[fixed], (p*scale)[fixed])
            np.testing.assert_allclose(q[4], (q[0]+q[1])*.5, atol=1e-12)
            self.assertLess(report['residual'], 1e-8*scale)

    def test_all_prescribed_surface_is_exact(self):
        p = np.array([[0., 0, 0], [1, 0, 0], [0, 1, 0]])
        q, _ = fit_surface(p, [[0, 1, 2]], p, [True]*3)
        np.testing.assert_array_equal(p, q)

    def test_cyclic_donors_are_rejected(self):
        p = np.array([[0., 0, 0], [1, 0, 0], [0, 1, 0]])
        with self.assertRaisesRegex(ValueError, 'Cyclic'):
            fit_surface(p, [[0, 1, 2]], p, [False]*3,
                        seams=[[0, 1, 2, .5], [1, 0, 2, .5]])

    def test_exact_section_coordinates_leave_axial_direction_free(self):
        p = np.array([[0., 0, 0], [1, .8, 0], [0, 1, 0],
                      [1, 1, 0], [.5, .4, 0]])
        faces = [[0, 4, 2], [4, 1, 2], [1, 3, 2]]
        fixed = np.array([True, False, True, True, False])
        q, _ = fit_surface(p, faces, p, fixed, [1, 1],
                           [[1, 0, 0], [0, 0, 1]], [1.2, .4],
                           plane_equalities=[True, True],
                           seams=[[4, 0, 1, .5]], iterations=4000,
                           tolerance=1e-9)
        np.testing.assert_allclose(q[1, [0, 2]], [1.2, .4], atol=1e-12)
        self.assertGreater(abs(q[1, 1]-p[1, 1]), .01)
        np.testing.assert_allclose(q[4], .5*(q[0]+q[1]), atol=1e-12)
        np.testing.assert_array_equal(q[fixed], p[fixed])

    def test_preserved_differentials_leave_neutral_shape_exact(self):
        p = np.array([[0., 0, 0], [1, 0, .4], [0, 1, 0], [1, 1, .2]])
        q, _ = fit_surface(p, [[0, 1, 2], [1, 3, 2]], p,
                           [True, False, True, False],
                           differential_positions=p, differential_mask=np.ones(4),
                           tolerance=1e-10)
        np.testing.assert_allclose(q, p, atol=1e-10)


if __name__ == '__main__':
    unittest.main()
