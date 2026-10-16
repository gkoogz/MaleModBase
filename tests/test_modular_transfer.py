"""Semantic rigid ownership follows the authored distal component, not its rim."""
from pathlib import Path
import re
import unittest
import numpy as np
from tools.build_modular_transfer import axial_shape_weights


class DistalOwnership(unittest.TestCase):
    def test_complete_component_and_bounded_transition(self):
        root=Path(__file__).resolve().parents[1]
        arrays=np.load(root/'assets/wolverine-reference/geometry.npz')
        header=(root/'legacy/wolverine/src/runtime/r14_asset.h').read_text()
        start=int(re.search(r'\br14NewStart\s*=\s*(\d+)',header)[1])
        count=len(arrays['r14_asset__r14Base'])//3
        weights=axial_shape_weights(root)
        direct=arrays['neck_render_data__nrDirect']
        retained=(direct>=start)&(direct<count)
        self.assertGreater(int(retained.sum()),100)
        np.testing.assert_array_equal(weights[retained],np.ones(retained.sum()))
        self.assertTrue(np.isfinite(weights).all())
        self.assertGreater(int((weights==0).sum()),100)
        self.assertGreater(int(((weights>0)&(weights<1)).sum()),10)
        self.assertGreaterEqual(weights.min(),0)
        self.assertLessEqual(weights.max(),1)


if __name__=='__main__':unittest.main()
