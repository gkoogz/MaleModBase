import math
import unittest
from dataclasses import astuple
from pathlib import Path
import csv
import numpy as np
from malemod_base.controls import defaults, PHYSICS
from malemod_base.physics_controls import evaluate, mode_value, suspension_limits


def preference(**changes):
    values = defaults()
    values.update(changes)
    return dict(format='malemod.controls', version=1, values=values)


class PhysicsControlTests(unittest.TestCase):
    def test_original_float32_code_fixture(self):
        path=Path(__file__).parent/'data/physics-controls.csv'
        with path.open(encoding='utf-8',newline='') as source:
            rows=list(csv.DictReader(source))
        self.assertEqual(len(rows),600)
        for row in rows:
            values={key:float(row[key]) for key,*_ in PHYSICS}
            result=evaluate(preference(**values),mode=float(row['mode']),rest_length=float(row['rest_length']))
            actual=[*astuple(result),*suspension_limits(3,5)]
            expected=[float(row['source_'+str(i)]) for i in range(22)]
            np.testing.assert_allclose(actual,expected,rtol=3e-6,atol=2e-8)

    def test_neutral_source_material_values_and_modes(self):
        p = evaluate(preference(), mode=2, rest_length=24)
        self.assertAlmostEqual(p.shaft_mass, 1.825)
        self.assertAlmostEqual(p.lobe_mass, 1.925)
        self.assertAlmostEqual(p.shaft_drag, 1.8)
        self.assertAlmostEqual(p.lobe_drag, 2.25)
        self.assertAlmostEqual(p.shaft_motion_response, 1.208)
        self.assertAlmostEqual(p.lobe_motion_response, 1.298)
        self.assertAlmostEqual(p.shaft_bend_compliance, .0015)
        self.assertAlmostEqual(p.root_mass, 2.376)
        self.assertEqual([mode_value(i, 5, 42, 110) for i in range(3)], [5, 42, 110])

    def test_every_physics_slider_changes_its_material_law(self):
        for key, *_ in PHYSICS:
            low = evaluate(preference(**{key: 1}), mode=2, rest_length=24)
            high = evaluate(preference(**{key: 100}), mode=2, rest_length=24)
            self.assertNotEqual(low, high, key)
        a = evaluate(preference(shaft_stiffness=1), mode=2, rest_length=24)
        b = evaluate(preference(shaft_stiffness=100), mode=2, rest_length=24)
        self.assertGreater(a.shaft_bend_compliance, b.shaft_bend_compliance)
        a = evaluate(preference(scrotum_stiffness=1), mode=2, rest_length=24)
        b = evaluate(preference(scrotum_stiffness=100), mode=2, rest_length=24)
        self.assertGreater(a.suspension_compliance, b.suspension_compliance)
        self.assertLess(a.lobe_torsion, b.lobe_torsion)

    def test_independent_channels_and_geometry_inputs(self):
        a = evaluate(preference(), mode=.75, rest_length=24)
        b = evaluate(preference(scrotum_weight=100), mode=.75, rest_length=24)
        self.assertEqual(a.shaft_mass, b.shaft_mass)
        self.assertEqual(a.root_damping, b.root_damping)
        self.assertNotEqual(a.lobe_mass, b.lobe_mass)
        self.assertEqual(suspension_limits(1, 2), (2.45, 2.45*1.12+.3))
        self.assertEqual(suspension_limits(4, 3), (4, 4*1.12+.45))
        with self.assertRaises(ValueError): evaluate(preference(), mode=3, rest_length=24)
        with self.assertRaises(ValueError): evaluate(preference(), mode=2, rest_length=0)
        with self.assertRaises(ValueError): suspension_limits(1, float('nan'))
