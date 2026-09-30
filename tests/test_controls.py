import math
from pathlib import Path
import re
import unittest
from malemod_base import controls as c


class ControlsTest(unittest.TestCase):
    def test_constants_match_authoritative_source(self):
        root = Path(__file__).resolve().parents[1]/'legacy/wolverine/src/runtime'
        source = (root/'d3d9_proxy.cpp').read_text()
        def array(name):
            text = re.search(r'\b'+name+r'\[\d+\]=\{([^}]+)\}', source)[1]
            return [float(x.strip().removesuffix('f')) for x in text.split(',')]
        self.assertEqual([x[3] for x in c.SHAPE], array('neutralShape'))
        self.assertEqual([x[3] for x in c.PHYSICS], array('neutralPhysics'))
        lows = array('coherentShapeLow')
        # Length explicitly supersedes its old low morph in MapLength100.
        self.assertEqual([x[2] for i, x in enumerate(c.SHAPE) if i != 1],
                         [x for i, x in enumerate(lows) if i != 1])
        rows = re.findall(r'\{"[^"]+",([^}]+)\}',
                          re.search(r'physSpecs\[8\]=\{(.+?)\};', source)[1])
        self.assertEqual([(x[2], x[4]) for x in c.PHYSICS],
                         [tuple(map(float, row.split(',')[:2])) for row in rows])

    def test_source_knots_and_order(self):
        self.assertEqual(len(c.ORDER), 18)
        for key, _, low, neutral, high in c.SHAPE + c.PHYSICS:
            self.assertAlmostEqual(c.mapped(key, c.limits(key)[0]), low)
            self.assertAlmostEqual(c.mapped(key, 50), neutral)
            self.assertAlmostEqual(c.mapped(key, 100), high)
        self.assertAlmostEqual(c.mapped('glans', 0), .82)
        self.assertAlmostEqual(c.mapped('glans', 50), .9706456)
        self.assertAlmostEqual(c.mapped('glans', 100), 1.56)
        self.assertEqual(c.defaults()['state'], 2)

    def test_monotone_finite_complete_catalog(self):
        for spec in c.catalog()['controls']:
            values = spec['mapping']
            self.assertEqual(len(values), spec['maximum']-spec['minimum']+1)
            self.assertTrue(all(math.isfinite(v) for v in values))
            self.assertTrue(all(b >= a for a, b in zip(values, values[1:])))

    def test_preferences_roundtrip_and_rejection(self):
        doc = dict(format='malemod.controls', version=1, values=c.defaults())
        self.assertEqual(c.read_preferences(doc), c.defaults())
        for key, value in [('width', float('nan')), ('length', -1), ('state', 1.5),
                           ('overall', True), ('audio', 50), ('glans', 101)]:
            with self.assertRaises((ValueError, KeyError)):
                c.read_preferences(dict(doc, values={key: value}))
        with self.assertRaises(ValueError):
            c.read_preferences(dict(doc, version=2))


if __name__ == '__main__':
    unittest.main()
