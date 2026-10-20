import unittest
import numpy as np
from malemod_base.attachment_fields import attachment_strength, frame_follow, neighbour_support


class AttachmentFieldsTests(unittest.TestCase):
    def test_collar_controls_do_not_follow_shaft_dominant_material(self):
        # Captured saddle ownership versus the distant shaft fold limiter.
        weight = attachment_strength([.83, .346, .5], [.10, .692, .5])
        np.testing.assert_array_equal(weight, [1, 0, .5])

    def test_neighbour_support_keeps_hard_boundaries(self):
        f = np.array([[0, 1, 2], [1, 3, 2], [2, 3, 4]])
        active = np.array([1, 0, 0, 0, 0], bool)
        fixed = np.array([0, 1, 0, 0, 1], bool)
        np.testing.assert_array_equal(neighbour_support(f, active, fixed), [1, 0, 1, 1, 0])
        np.testing.assert_array_equal(active, [1, 0, 0, 0, 0])

    def test_frame_influence_is_continuous_and_bounded(self):
        w = frame_follow(np.linspace(-.1, .4, 101))
        self.assertTrue(np.all(np.diff(w) >= 0)); self.assertEqual(w[0], 0); self.assertEqual(w[-1], 1)

    def test_invalid_materials_and_topology_are_rejected(self):
        with self.assertRaises(ValueError): attachment_strength([np.nan], [0])
        with self.assertRaises(ValueError): neighbour_support([[0, 1, 3]], [1, 0], [0, 0])


if __name__ == '__main__': unittest.main()
