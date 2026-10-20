import unittest
import numpy as np
from malemod_base.convex_support import PlanarHalfspaceProjector


class HalfspaceSupportTest(unittest.TestCase):
    def test_open_fillet_keeps_rays_and_axial_coordinate(self):
        normals = np.array([[1., 0, 0], [0, 1, 0], [1, 1, 0]])
        normals[2] /= np.sqrt(2)
        offsets = [0, 0, np.sqrt(2)]
        projector = PlanarHalfspaceProjector([(normals, offsets)]*3)
        points = [[-1., 0, 7], [3, -2, -4], [2, 3, .5]]
        expected = [[.5, 1.5, 7], [3, 0, -4], [2, 3, .5]]
        np.testing.assert_allclose(projector.project(points), expected, atol=1e-12)

    def test_rotated_scaled_and_translated_support(self):
        normals = np.array([[1., 0, 0], [0, 1, 0], [1, 1, 0]])
        normals[2] /= np.sqrt(2)
        rotation, _ = np.linalg.qr(np.array([[1., 2, 3], [4, -2, 1], [3, 1, -1]]))
        shift = np.array([4., -7, 2])
        for scale in [.01, 1., 100.]:
            n = normals@rotation.T
            offsets = np.array([0, 0, np.sqrt(2)])*scale+n@shift
            projector = PlanarHalfspaceProjector([(n, offsets)])
            point = np.array([[-1., 0, 7]])@rotation.T*scale+shift
            expected = np.array([[.5, 1.5, 7]])@rotation.T*scale+shift
            np.testing.assert_allclose(projector.project(point), expected, atol=1e-10)

    def test_empty_intersection_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "feasible"):
            PlanarHalfspaceProjector([([[1., 0, 0], [-1, 0, 0], [0, 1, 0]], [1, 1, 0])])


if __name__ == "__main__":
    unittest.main()
