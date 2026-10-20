import unittest
import numpy as np
from malemod_base.swept_collar import (Section, measure_section, triangle_overlap,
                                     radial_ramp, circumscribed_opening)


class SweptCollarTest(unittest.TestCase):
    def test_measured_non_circular_section_retains_edge_donors(self):
        phi = np.linspace(0, 2*np.pi, 32, endpoint=False)
        ring = np.column_stack([np.zeros(32), 2*np.cos(phi), 3*np.sin(phi)])
        points = np.concatenate([ring+[-1, 0, 0], ring+[1, 0, 0]])
        faces = []
        for i in range(32):
            j = (i+1)%32
            faces.extend([[i, j, 32+i], [j, 32+j, 32+i]])
        s = measure_section(points, faces, [0, 0, 0], [1, 0, 0])
        np.testing.assert_allclose(s.radii([0, np.pi/2, np.pi]), [2, 3, 2], atol=1e-9)
        reconstructed = points[s.donors[:, 0]]*(1-s.weights[:, None])+points[s.donors[:, 1]]*s.weights[:, None]
        np.testing.assert_allclose(s.project(reconstructed), s.polygon, atol=1e-10)

    def test_triangle_intersection_without_inside_vertices(self):
        polygon = np.array([[-1., -1], [1, -1], [1, 1], [-1, 1]])
        points = np.array([[-2., 0], [2, 0], [0, 3], [-3, -3], [3, -3], [0, 4], [4, 4], [5, 4], [4, 5]])
        np.testing.assert_array_equal(triangle_overlap(points, np.arange(9).reshape(3, 3), polygon), [True, True, False])

    def test_large_opening_keeps_rings_distinct_and_exterior_exact(self):
        r = np.linspace(2, 14, 2401)
        q = radial_ramp(r, 2, 10, 2)
        self.assertGreater(np.diff(q).min(), 0)
        self.assertEqual(q[0], 10)
        np.testing.assert_array_equal(q[r >= 12], r[r >= 12])
        # The radial derivative reaches the identity smoothly at the ramp edge.
        slope = np.diff(q)/np.diff(r)
        self.assertLess(abs(slope[np.searchsorted(r, 12)-2]-1), .002)
        np.testing.assert_array_equal(radial_ramp(r, 2, 2, 2), r)

    def test_unit_covariance(self):
        r = np.linspace(3, 13, 101)
        expected = radial_ramp(r, 3, 8, 2)
        for scale in [.01, 100]:
            np.testing.assert_allclose(radial_ramp(r*scale, 3*scale, 8*scale, 2*scale)/scale, expected, rtol=1e-10)

    def test_inferior_allowance_does_not_reverse_coarse_donor_edges(self):
        # A pointed underside exposes nonconvex support offsets even when a
        # circular fixture looks acceptable. Test both observed rim resolutions.
        from scipy.spatial import ConvexHull
        phi = np.linspace(0, 2*np.pi, 64, endpoint=False)
        points = np.vstack([np.column_stack([np.cos(phi), np.sin(phi)]), [0, -1.6]])
        polygon = points[ConvexHull(points).vertices]
        for count in [9, 14]:
            for phase in np.linspace(0, 2*np.pi/count, 13):
                angle = phase+np.arange(count)*2*np.pi/count
                corners = np.column_stack([np.cos(angle), np.sin(angle)])
                for scale in [.01, 1., 100.]:
                    section = Section(np.zeros(3), np.array([1., 0, 0]),
                                      np.array([0., 1, 0]), np.array([0., 0, 1]),
                                      polygon*scale, None, None)
                    _, rim = circumscribed_opening(section, corners*scale, .15*scale)
                    edges = np.roll(rim, -1, axis=0)-rim
                    following = np.roll(edges, -1, axis=0)
                    turns = edges[:, 0]*following[:, 1]-edges[:, 1]*following[:, 0]
                    self.assertGreater(turns.min(), 0)
                    outward = np.column_stack([edges[:, 1], -edges[:, 0]])
                    # Every point of the measured section remains inside every
                    # original-edge donor segment's outward supporting plane.
                    excess = section.polygon@outward.T-np.sum(rim*outward, axis=1)
                    self.assertLessEqual(excess.max(), 1e-10*scale*scale)
                    self.assertLess(np.ptp(rim[:, 0])/scale, 2.3)
                    self.assertAlmostEqual(rim[:, 1].max()/scale,
                                           polygon[:, 1].max(), places=10)

    def test_sparse_corner_distribution_cannot_make_collar_oversized(self):
        phi = np.linspace(0, 2*np.pi, 128, endpoint=False)
        polygon = np.column_stack([np.cos(phi), np.sin(phi)])
        section = Section(np.zeros(3), np.array([1., 0, 0]),
                          np.array([0., 1, 0]), np.array([0., 0, 1]),
                          polygon, None, None)
        angles = np.array([0., .2, .4, 2.5, 4.5])
        corners = np.column_stack([np.cos(angles), np.sin(angles)])
        _, rim = circumscribed_opening(section, corners, .15)
        self.assertAlmostEqual(np.ptp(rim[:, 0]), 2.03)
        with self.assertRaisesRegex(ValueError, 'exceeds measured width'):
            circumscribed_opening(section, corners, .15, max_lateral_ratio=1.01)


if __name__ == '__main__':
    unittest.main()
