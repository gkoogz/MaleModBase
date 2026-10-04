"""Independent finite-thickness seam fixtures, separate from actual characters."""
import math
from pathlib import Path
import sys
import unittest
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from malemod_base.garment_geometry_audit import measure_authored_joints


class AuthoredJointTests(unittest.TestCase):
    def fixture(self):
        vertices = np.array([[0., 0., 0.], [0., 0., .002], [1., 0., 0.], [1., 0., .002]])
        names = [f"hem-{side}-{end}" for side in (0, 1) for end in ("top", "bottom")] + [f"strap-{side}-bottom" for side in (0, 1)]
        joints = [dict(name=name, a=dict(vertices=[1, 3], weights=[.25, .75]), b=dict(vertices=[0, 2], weights=[.25, .75]), restOffset=[0., 0., .002], offsetProvenance="authored-thickness-contract") for name in names]
        return vertices, joints

    def test_authored_thickness_is_preserved(self):
        vertices, joints = self.fixture()
        result = measure_authored_joints(vertices, joints, 1., .003)
        self.assertTrue(result["passed"])
        self.assertEqual(result["maximumResidualNormalized"], 0.)
        self.assertEqual(result["joints"][0]["rawGapNormalized"], .002)

    def test_gap_mutation_is_rejected(self):
        vertices, joints = self.fixture()
        vertices[[1, 3], 2] += .001
        result = measure_authored_joints(vertices, joints, 1., .003)
        self.assertFalse(result["passed"])
        self.assertGreater(result["maximumResidualNormalized"], result["toleranceNormalized"])

    def test_scale_rotation_and_translation(self):
        vertices, joints = self.fixture()
        c, s = math.cos(.72), math.sin(.72)
        rotation = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
        vertices = (vertices @ rotation.T) * 153.7 + [49.6, -2.7, 18.2]
        for joint in joints:
            joint["restOffset"] = (np.asarray(joint["restOffset"]) @ rotation.T * 153.7).tolist()
        self.assertLess(measure_authored_joints(vertices, joints, 153.7, .003)["maximumResidualNormalized"], 1e-15)

    def test_fit_gap_cannot_authorize_itself(self):
        vertices, joints = self.fixture()
        joints[0]["offsetProvenance"] = "captured-fit"
        with self.assertRaises(ValueError):
            measure_authored_joints(vertices, joints, 1., .003)

    def test_missing_endpoints_remain_uncertified(self):
        vertices, joints = self.fixture()
        self.assertFalse(measure_authored_joints(vertices, joints[:-1], 1., .003)["certified"])
        self.assertFalse(measure_authored_joints(vertices, [], 1., .003)["certified"])

    def test_duplicate_or_noninteger_attachment_rejected(self):
        vertices,joints=self.fixture()
        with self.assertRaises(ValueError):
            measure_authored_joints(vertices,joints+[joints[0]],1.,.003)
        joints[0]['a']['vertices']=[1.1,3]
        with self.assertRaises(ValueError):
            measure_authored_joints(vertices,joints,1.,.003)


if __name__ == "__main__":
    unittest.main()
