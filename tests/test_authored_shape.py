import csv
from pathlib import Path
import unittest
import numpy as np
from malemod_base.authored_shape import AuthoredShape, collar_growth, precursor_collar
from malemod_base.controls import defaults, SHAPE

ROOT = Path(__file__).resolve().parents[1]


def fixture_preferences():
    cases = [defaults()]
    for key in [x[0] for x in SHAPE] + ['hang']:
        for endpoint in [0 if key == 'length' else 1, 100]:
            p = defaults(); p[key] = endpoint; cases.append(p)
    for overall, width, angle, length in [(100,100,1,0),(100,100,100,100),(1,1,100,0)]:
        p=defaults();p.update(overall=overall,width=width,angle=angle,length=length);cases.append(p)
    return cases


class AuthoredShapeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        with np.load(ROOT/'assets/wolverine-reference/geometry.npz', allow_pickle=False) as bank:
            cls.shape = AuthoredShape(bank)

    def test_original_cpp_stage_for_endpoints_and_combined_extremes(self):
        with (ROOT/'tests/data/wolverine-authored-shape.csv').open(encoding='utf-8') as handle:
            expected=list(csv.DictReader(handle))
        results=[self.shape.evaluate(p) for p in fixture_preferences()]
        self.assertEqual(len(expected),len(results)*65)
        for row in expected:
            stage=results[int(row['case'])];i=int(row['index'])
            np.testing.assert_allclose(stage.coarse[i],[float(row[x]) for x in ['x','y','z']],atol=3e-5,rtol=1e-6)
            self.assertAlmostEqual(stage.growth,float(row['growth']),delta=3e-7)
            self.assertAlmostEqual(stage.hang_displacement[i,2],float(row['hang_z']),delta=2e-6)

    def test_morph_reference_is_distinct_from_displayed_neutral(self):
        np.testing.assert_allclose(self.shape.overall_width(1.5,1.15),self.shape.targets[1,1])
        self.assertAlmostEqual(collar_growth(1.2,1.59),.106086956521739)
        self.assertEqual(collar_growth(2.5,2.),1.5)

    def test_early_stage_exposes_missing_final_solver_and_does_not_apply_hang_early(self):
        p=defaults();a=self.shape.evaluate(p);p['hang']=100;b=self.shape.evaluate(p)
        np.testing.assert_array_equal(a.coarse,b.coarse)
        self.assertIn('final UnifiedCollar',a.omitted_stages)
        self.assertGreater(np.max(np.abs(b.hang_displacement)),0)
        # No mutable per-character globals or changed reference table.
        np.testing.assert_array_equal(self.shape.evaluate().coarse,a.coarse)

    def test_ramp_keeps_exterior_and_rejects_uncalibrated_invalid_inputs(self):
        points=np.array([[9.,0,84.3],[9,20,84.3]])
        output=precursor_collar(points,[0,100],1.5,False)
        np.testing.assert_array_equal(output[1],points[1])
        self.assertGreater(output[0,0],points[0,0])
        with self.assertRaises(ValueError):precursor_collar(points,[-1,0],1.5,True)
