import unittest
import numpy as np
from malemod_base.convex_support import PlanarSupportProjector
from malemod_base.constrained_attachment_fit import fit_surface


class ConvexSupportTest(unittest.TestCase):
    def test_clearance_and_envelope_keep_axial_coordinate(self):
        normals = [[1., 0, 0], [-1, 0, 0], [0, 0, 1], [0, 0, -1],
                   [2**-.5, 0, 2**-.5]]
        offsets = [-1., -1, -1, -1, .5]
        projector = PlanarSupportProjector([(normals, offsets)]*4)
        points = np.array([[3., 7, 4], [-3, -9, 0], [0, 2, 0], [.6, 8, .6]])
        q = projector.project(points)
        np.testing.assert_array_equal(q[:, 1], points[:, 1])
        self.assertGreaterEqual((q@np.array(normals).T-offsets).min(), -1e-12)
        np.testing.assert_array_equal(q[-1], points[-1])
        np.testing.assert_allclose(projector.project(q), q, atol=1e-12)
        np.testing.assert_allclose(q[0, [0, 2]], [1, 1], atol=1e-12)

    def test_infeasible_envelope_is_rejected(self):
        with self.assertRaisesRegex(ValueError, 'conflicts'):
            PlanarSupportProjector([([[1., 0, 0], [-1, 0, 0],
                                     [0, 0, 1], [0, 0, -1]], [2, 0, -1, -1])])

    def test_surface_fit_cannot_escape_measured_envelope(self):
        p = np.array([[0., 0, 0], [8, 1, 7], [0, 2, 0]])
        normals = [[1., 0, 0], [-1, 0, 0], [0, 0, 1], [0, 0, -1]]
        q, _ = fit_surface(p, [[0, 1, 2]], p, [True, False, True],
                           [1]*4, normals, [0., -1, 0, -1],
                           tolerance=1e-8, iterations=4000, screen=10.)
        self.assertGreaterEqual((q[1]@np.array(normals).T-[0., -1, 0, -1]).min(), -1e-12)
        np.testing.assert_array_equal(q[[0, 2]], p[[0, 2]])


if __name__ == '__main__':
    unittest.main()
