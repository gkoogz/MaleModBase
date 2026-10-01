import unittest
from pathlib import Path
import numpy as np
from malemod_base.authored_shape import AuthoredShape
from malemod_base.rest_frame import SourceRestFrame
from malemod_base.root_profile import SourceRootProfile
from malemod_base.collar import smoother
from test_authored_shape import fixture_preferences
ROOT=Path(__file__).resolve().parents[1]


def fixture_inputs():
    with np.load(ROOT/'assets/wolverine-reference/geometry.npz',allow_pickle=False) as bank:
        shape=AuthoredShape(bank);rest=SourceRestFrame(bank)
    for p in fixture_preferences():
        stage=shape.evaluate(p);points=np.asarray(stage.coarse,dtype=np.float32)
        for mode in (0,1,2):
            frame=rest.evaluate(points,angle_degrees=stage.mapped_shape[4],overall=stage.mapped_shape[0],
                width=stage.mapped_shape[2],physics_state=mode,previous_length=24,
                pelvic_ramp_blend=float(smoother((stage.growth-.15)/1)))
            yield points,frame,np.float32(stage.growth)


class RootProfileTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        with np.load(ROOT/'assets/wolverine-reference/geometry.npz',allow_pickle=False) as bank:
            cls.profile=SourceRootProfile(bank)
    def test_original_source_every_vertex_all_profiles_and_modes(self):
        expected=np.load(ROOT/'tests/data/wolverine-root-profile.npy',allow_pickle=False)
        inputs=list(fixture_inputs());self.assertEqual(expected.shape,(len(inputs),2388,3))
        for i,(points,frame,growth) in enumerate(inputs):
            result=self.profile.evaluate(points,frame,collar_growth=growth)
            np.testing.assert_allclose(result.positions,expected[i],rtol=3e-6,atol=3e-5)
            excluded=(frame.flex>.60)|(self.profile.shaft<.08)|(self.profile.ball>=.50)|(self.profile.suspension>=1)
            np.testing.assert_array_equal(result.positions[excluded],points[excluded])
    def test_instances_and_caller_positions_remain_independent(self):
        points,frame,growth=next(fixture_inputs());saved=points.copy()
        a=self.profile.evaluate(points,frame,collar_growth=growth);snapshot=a.positions.copy()
        self.profile.evaluate(points,frame,collar_growth=1.5)
        np.testing.assert_array_equal(points,saved);np.testing.assert_array_equal(a.positions,snapshot)
    def test_rejects_invalid_prepared_input(self):
        points,frame,growth=next(fixture_inputs())
        for value in (-1,float('nan'),2):
            with self.assertRaises(ValueError):self.profile.evaluate(points,frame,collar_growth=value)
        with self.assertRaises(ValueError):self.profile.evaluate(points[:-1],frame,collar_growth=growth)
