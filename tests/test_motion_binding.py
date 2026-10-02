import unittest
import numpy as np
from malemod_base.motion_binding import linear_chain_weights, reference_fields, reference_cage_weights


class MotionBindingTests(unittest.TestCase):
    def test_protected_head_is_rigid_under_independent_bone_motion(self):
        from malemod_base.motion_binding import protected_cage_weights
        fields = np.array([[1, 0, t] for t in np.linspace(.78, 1, 100)])
        w = protected_cage_weights(fields, np.zeros(100), 2, np.full(100, 10.), 5)
        rng = np.random.default_rng(16)
        p = rng.normal(size=(100, 3))
        transforms = np.tile(np.eye(4), (10, 1, 1))
        for i in range(10):
            q, _ = np.linalg.qr(rng.normal(size=(3, 3)))
            transforms[i, :3, :3] = q
            transforms[i, :3, 3] = rng.normal(size=3)
        moved = np.einsum('nb,bij,nj->ni', w, transforms[:, :3], np.c_[p, np.ones(100)])
        np.testing.assert_allclose(np.linalg.norm(np.diff(moved, axis=0), axis=1),
                                   np.linalg.norm(np.diff(p, axis=0), axis=1), atol=1e-12)
        np.testing.assert_array_equal(w[:, 7], np.ones(100))
        self.assertEqual(np.count_nonzero(w), 100)

    def test_protected_binding_keeps_web_seam_and_coordinate_scale(self):
        from malemod_base.motion_binding import protected_cage_weights
        fields = np.array([[1, 0, .99], [0, 1, .99], [1, 0, .5], [1, 0, .7799999], [1, 0, .78]])
        w = protected_cage_weights(fields, [0, -4, 0, 0, 0], 2, [0, 10, 10, 10, 10], 5)
        np.testing.assert_array_equal(w[0], np.zeros(10))
        self.assertEqual(w[1, 8], 1)
        np.testing.assert_allclose(w[2:].sum(1), 1)
        self.assertLess(np.linalg.norm(w[3]-w[4]), 2e-6)
        scaled = protected_cage_weights(fields, [0, -.04, 0, 0, 0], .02, [0, .1, .1, .1, .1], .05)
        np.testing.assert_allclose(w, scaled)

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

    def test_mechanical_guide_is_continuous_and_rigid_frame_invariant(self):
        from malemod_base.motion_binding import sample_mechanical_guide
        p=np.column_stack([np.arange(12)*2.,np.zeros(12),-np.arange(12)**2*.05])
        p[:2,2]=0
        a,ta=sample_mechanical_guide(p,np.arange(1,11)/11-1e-7)
        b,tb=sample_mechanical_guide(p,np.arange(1,11)/11+1e-7)
        self.assertLess(np.linalg.norm(a-b,axis=1).max(),1e-5)
        self.assertLess(np.linalg.norm(ta-tb,axis=1).max(),1e-5)
        r=np.array([[0,-1,0],[1,0,0],[0,0,1]])
        c,d=sample_mechanical_guide(p,np.linspace(0,1,73))
        ct,dt=sample_mechanical_guide(p@r.T+10,np.linspace(0,1,73))
        np.testing.assert_allclose(ct,c@r.T+10,atol=1e-12)
        np.testing.assert_allclose(dt,d@r.T,atol=1e-12)
