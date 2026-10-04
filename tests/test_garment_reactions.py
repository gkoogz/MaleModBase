"""Authored force ownership stays bounded through actual source lineage."""
import unittest
from pathlib import Path
import numpy as np
from malemod_base.garment_reactions import bounded_reaction_bindings, reference_reaction_bindings, reaction_bindings_header

ROOT = Path(__file__).resolve().parents[1]


class GarmentReactions(unittest.TestCase):
    def test_actual_full_source_binding(self):
        bank = np.load(ROOT/'assets/wolverine-reference/geometry.npz')
        weights = reference_reaction_bindings(bank)
        self.assertEqual(weights.shape, (17528, 14))
        self.assertTrue(np.isfinite(weights).all())
        self.assertGreaterEqual(weights.min(), 0)
        self.assertLessEqual(weights.sum(1).max(), 1+1e-14)
        self.assertGreater(np.count_nonzero(weights[:, 2:12].sum(1)), 5000)
        self.assertGreater(np.count_nonzero(weights[:, 12:].sum(1)), 5000)
        # Body/collar vertices with no authored ownership remain static; no
        # nearest-station or bone guess recruits them into tissue mechanics.
        self.assertEqual(np.count_nonzero(weights.sum(1)==0), 54)
        text = reaction_bindings_header(weights)
        self.assertEqual(text.count('},'), len(weights))

    def test_signed_interpolation_does_not_amplify_load(self):
        values = np.zeros((3, 14));values[1, :2]=[-.2, .6];values[2, :2]=[1.5, .5]
        result = bounded_reaction_bindings(values)
        np.testing.assert_array_equal(result[0], 0)
        np.testing.assert_array_equal(result[1, :2], [0, .6])
        np.testing.assert_array_equal(result[2, :2], [.75, .25])
        for invalid in (np.nan, np.inf):
            values[0, 0]=invalid
            with self.assertRaises(ValueError):bounded_reaction_bindings(values)
        with self.assertRaises(ValueError):bounded_reaction_bindings(np.ones((2, 13)))


if __name__ == '__main__':
    unittest.main()
