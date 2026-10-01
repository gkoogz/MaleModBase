import csv,unittest
from pathlib import Path
import numpy as np
from malemod_base.authored_shape import AuthoredShape
from malemod_base.rest_frame import SourceRestFrame
from malemod_base.collar import smoother
from test_authored_shape import fixture_preferences
ROOT=Path(__file__).resolve().parents[1]

def cases():
    return [(p,mode,prior) for p in fixture_preferences() for mode in (0,1,2) for prior in (12.,24.,48.)]

class RestFrameTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        with np.load(ROOT/'assets/wolverine-reference/geometry.npz',allow_pickle=False) as bank:
            cls.shape=AuthoredShape(bank);cls.frame=SourceRestFrame(bank)
    def evaluate(self,p,mode,prior):
        stage=self.shape.evaluate(p)
        return self.frame.evaluate(stage.coarse,angle_degrees=stage.mapped_shape[4],
            overall=stage.mapped_shape[0],width=stage.mapped_shape[2],physics_state=mode,
            previous_length=prior,pelvic_ramp_blend=float(smoother((stage.growth-.15)/1.)))
    def test_original_measurement_endpoints_modes_and_prior_lengths(self):
        results=[self.evaluate(*case) for case in cases()]
        with (ROOT/'tests/data/wolverine-rest-frame.csv').open() as handle:rows=list(csv.DictReader(handle))
        self.assertEqual(len(rows),len(results)*18)
        for row in rows:
            output=results[int(row['case'])];i=int(row['index']);v=int(row['vertex'])
            np.testing.assert_allclose(output.centers[i],[float(row[x]) for x in ('x','y','z')],rtol=3e-6,atol=1e-4)
            self.assertAlmostEqual(output.body_radius,float(row['radius']),delta=3e-5)
            self.assertAlmostEqual(output.rest_length,float(row['length']),delta=1e-4)
            self.assertEqual(output.uses_previous_length,bool(int(row['prior_used'])))
            self.assertAlmostEqual(output.flex[v],float(row['flex']),delta=1e-5)
            self.assertAlmostEqual(output.root_follow[v],float(row['root_follow']),delta=3e-5)
    def test_requires_measured_prior_and_valid_source_geometry(self):
        stage=self.shape.evaluate()
        values=dict(angle_degrees=30,overall=1.2,width=1.59,physics_state=1,previous_length=24,pelvic_ramp_blend=0)
        for key,value in [('previous_length',0),('physics_state',3),('pelvic_ramp_blend',2),('width',float('nan'))]:
            with self.assertRaises(ValueError):self.frame.evaluate(stage.coarse,**dict(values,**{key:value}))
        with self.assertRaises(ValueError):self.frame.evaluate(stage.coarse[:-1],**values)
    def test_outputs_do_not_mutate_reference_or_another_evaluation(self):
        p=fixture_preferences()[0];a=self.evaluate(p,0,24);saved=a.centers.copy()
        self.evaluate(fixture_preferences()[-1],2,48)
        np.testing.assert_array_equal(a.centers,saved)
        np.testing.assert_array_equal(self.evaluate(p,0,24).centers,saved)

if __name__=='__main__':unittest.main()
