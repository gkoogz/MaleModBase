import unittest
import numpy as np
from malemod_base.motion_binding import linear_chain_weights, reference_fields, reference_cage_weights


class MotionBindingTests(unittest.TestCase):
    def test_cage_keeps_seam_fixed_and_bounds_overlapping_fields(self):
        fields=np.array([[1,1,.5],[1,0,.5],[0,1,.5],[0,1,.5]])
        weights=reference_cage_weights(fields,[0,0,-4,4],2,[0,5,5,5],5)
        np.testing.assert_array_equal(weights[0],np.zeros(10))
        np.testing.assert_allclose(weights[1:].sum(1),1)
        self.assertEqual(weights[2,8],1)
        self.assertEqual(weights[3,9],1)
        scaled=reference_cage_weights(fields,[0,0,-.04,.04],.02,[0,.05,.05,.05],.05)
        np.testing.assert_allclose(weights,scaled)

    def test_reference_transfer_preserves_direct_and_signed_donors(self):
        bank={
            'physics_weights__phys_shaft_weight':np.array([.2,.8]),
            'physics_weights__phys_scrotum_weight':np.array([.8,.2]),
            'physics_weights__phys_flex_coordinate':np.array([0.,1.]),
            'r14_asset__r14Offsets':np.array([0,1,2]),
            'r14_asset__r14Sources':np.array([0,1]),
            'r14_asset__r14Weight':np.array([1.,1.]),
            'rounded_render_data__rsFineSource':np.array([0,1,1]),
            'rounded_render_data__rsFineBary':np.array([.25,.25,.5]),
            'neck_render_data__nrDirect':np.array([0,65535,2]),
            'neck_render_data__nrRows':np.array([0,0,2,2]),
            'neck_render_data__nrSources':np.array([0,1]),
            'neck_render_data__nrWeights':np.array([-1.,2.])}
        before={k:v.copy() for k,v in bank.items()}
        np.testing.assert_allclose(reference_fields(bank),[[.2,.8,0],[1,0,1],[.65,.35,.75]])
        for key in bank:np.testing.assert_array_equal(bank[key],before[key])

    def test_affine_reproduction_and_partition(self):
        knots = np.array([0., .1, .4, .8, 1.])
        x = np.linspace(-.1, 1.1, 301)
        weights = linear_chain_weights(x, knots)
        np.testing.assert_allclose(weights.sum(1), 1)
        np.testing.assert_allclose(weights @ knots, np.clip(x, 0, 1), atol=1e-15)
        self.assertTrue((weights >= 0).all())
        self.assertTrue((np.count_nonzero(weights, axis=1) <= 2).all())

    def test_reject_invalid_knots(self):
        for knots in ([0, 0], [1, 0], [0, np.nan], [0]):
            with self.assertRaises(ValueError):
                linear_chain_weights([.5], knots)
