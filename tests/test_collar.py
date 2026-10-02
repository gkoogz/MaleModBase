import csv
from pathlib import Path
import unittest

import numpy as np

from malemod_base.collar import CollarFrame, CollarPlan, recruitment, radial_targets

ROOT = Path(__file__).resolve().parents[1]


def frame(radius=3, scale=1):
    return CollarFrame((9 * scale, 0, 84.3 * scale), (1, 0, 0), (0, 0, 1), radius * scale, 30 * scale, scale)


def patch():
    # An independent planar fixture, not proprietary target geometry.
    points = np.array([(x, y, 84.3) for x in np.linspace(0, 30, 8) for y in np.linspace(-14, 14, 8)])
    faces = []
    for i in range(7):
        for j in range(7):
            a = i * 8 + j
            faces.extend([(a, a + 8, a + 1), (a + 8, a + 9, a + 1)])
    # A finer seam vertex constrained to an original body edge.
    points = np.vstack([points, points[27] * .3 + points[28] * .7])
    faces.extend([(27, 35, 64), (64, 35, 28)])
    return points, np.array(faces, dtype=np.int32), [(64, 27, 28, .7)]


class CollarTests(unittest.TestCase):
    def test_yawed_root_preserves_source_lateral_support_and_dense_weld(self):
        points,faces,seams=patch()
        axis=np.array([.8,.3,-.5196152422706632]);axis/=np.linalg.norm(axis)
        up=np.cross(axis,[0,1,0]);up/=np.linalg.norm(up)
        f=CollarFrame((9,0,84.3),tuple(axis),tuple(up),6,30,1)
        plan=CollarPlan(points,faces,seams,f)
        delta=np.zeros_like(points);delta[:,1]=.7;delta[:,2]=1.2
        result=plan.solve_displacement(delta)
        self.assertTrue(np.isfinite(result).all())
        np.testing.assert_allclose(result[64],result[27]*.3+result[28]*.7,atol=1e-12)
        self.assertTrue(np.all((plan.mask>=0)&(plan.mask<=1)))

    def test_metric_matches_native_active_source_fixtures(self):
        path = ROOT / 'tests/data/wolverine-collar-metric.csv'
        with path.open() as handle:
            rows = list(csv.DictReader(handle))
        self.assertEqual(len(rows), 1024)
        for row in rows:
            x = {key: float(value) for key, value in row.items()}
            f = CollarFrame((9, 0, float(np.float32(84.3))), (x['axis_x'], 0, x['axis_z']),
                            (x['up_x'], 0, x['up_z']), x['radius'], x['length'], 1)
            mask = recruitment([[x['x'], x['y'], x['z']]], f)[0]
            screen = max(x['area'], .005) * (2.5 + 2 * (1 - mask)**4)
            self.assertAlmostEqual(mask, x['mask'], delta=2e-6)
            self.assertAlmostEqual(screen, x['screen'], delta=3e-5)

    def test_recruitment_grows_outward_in_upper_pelvis_with_radius(self):
        points = np.array([(x, y, 90) for x in np.linspace(5, 15, 15) for y in np.linspace(-22, 22, 40)])
        support = [recruitment(points, frame(radius)) for radius in [2.9, 4.5, 6, 7.62]]
        counts = [np.count_nonzero(w > 1e-4) for w in support]
        self.assertEqual(counts, sorted(counts))
        self.assertGreater(counts[-1], counts[0])
        self.assertTrue(np.all((support[-1] >= 0) & (support[-1] <= 1)))
        # Ventral/inter-thigh guards intentionally prevent an unbounded sphere.
        self.assertEqual(recruitment([[0, 0, 40]], frame(7.62))[0], 0)

    def test_sparse_solve_matches_independent_dense_elimination_and_weld(self):
        points, faces, seams = patch()
        plan = CollarPlan(points, faces, seams, frame(6))
        targets = points.copy(); targets[:, 2] += plan.mask * 2
        output = plan.solve(targets)
        P = plan.projection.toarray(); K = plan.metric.toarray()
        rhs = P.T @ (targets * plan.screen[:, None])
        fixed = plan.before[plan.masters[plan.fixed]]
        reduced = rhs[plan.free] - K[np.ix_(plan.free, plan.fixed)] @ fixed
        dense = np.linalg.solve(K[np.ix_(plan.free, plan.free)], reduced)
        masters = plan.before[plan.masters].copy(); masters[plan.free] = dense
        np.testing.assert_allclose(output, P @ masters, atol=1e-10, rtol=1e-10)
        np.testing.assert_allclose(output[64], output[27] * .3 + output[28] * .7, atol=1e-12)
        np.testing.assert_array_equal(output[plan.masters[plan.fixed]], points[plan.masters[plan.fixed]])
        # Inputs and another plan are independent; cached solves don't overwrite rest positions.
        second = CollarPlan(points, faces, seams, frame(6))
        np.testing.assert_array_equal(second.before, plan.before)
        np.testing.assert_allclose(second.solve(targets), output, atol=1e-12)

    def test_measured_scale_preserves_solution_and_targets(self):
        points, faces, seams = patch()
        guides = np.broadcast_to(np.array([9., 0., 84.3]), points.shape)
        masks = recruitment(points, frame(6))
        targets = radial_targets(points, frame(6), masks, guides, np.full(len(points), 1.))
        scaled_targets = radial_targets(points * .01, frame(6, .01), masks, guides * .01, np.full(len(points), .01))
        np.testing.assert_allclose(scaled_targets, targets * .01, atol=1e-12)
        original = CollarPlan(points, faces, seams, frame(6)).solve(targets)
        scaled = CollarPlan(points * .01, faces, seams, frame(6, .01)).solve(scaled_targets)
        np.testing.assert_allclose(scaled, original * .01, atol=1e-10)

    def test_rejects_unmeasured_scale_invalid_seams_and_changed_topology(self):
        points, faces, seams = patch()
        with self.assertRaises(ValueError):
            CollarPlan(points, faces, seams, frame(6, 0))
        for invalid in [[(64, 27, 64, .7)], [(64, 27, 28, 2)], [(64, 27, 28, .7), (27, 64, 26, .5)]]:
            with self.subTest(seams=invalid), self.assertRaises(ValueError):
                CollarPlan(points, faces, invalid, frame(6))
        with self.assertRaises(ValueError):
            CollarPlan(points, faces, seams, frame()).solve(points[:-1])

    def test_authored_source_seam_donors_are_preserved(self):
        with np.load(ROOT / 'assets/wolverine-reference/geometry.npz') as bank:
            triples = bank['unified_collar_data__ucSeamVertices'].reshape(-1, 3)
            weights = bank['unified_collar_data__ucSeamWeights']
            self.assertEqual(len(triples), 34)
            self.assertEqual(len(set(triples[:, 0])), 34)
            self.assertTrue(np.all((weights >= 0) & (weights <= 1)))
            self.assertFalse(np.isin(triples[:, 1:], triples[:, 0]).any())


if __name__ == '__main__':
    unittest.main()
