import unittest
from malemod_base.control_transport import size_factors,normalized_samples


class ControlTransportTests(unittest.TestCase):
    def test_fitted_neutral_is_identity_and_source_endpoint_ratios_are_retained(self):
        self.assertEqual(size_factors(),dict(axial=1.,radial=1.,head=1.,lobes=1.))
        p=size_factors(dict(overall=100,length=100,width=100,scrotum=100))
        self.assertAlmostEqual(p['axial'],(2.5/1.2)*(2.4/1.6))
        self.assertAlmostEqual(p['radial'],(2.5/1.2)*(2./1.59))
        self.assertAlmostEqual(p['lobes'],(2.5/1.2)*(2./1.53))

    def test_independent_controls_do_not_resize_unrelated_regions(self):
        for key,changed in [('length',{'axial'}),('width',{'radial'}),('glans',{'head'}),('scrotum',{'lobes'})]:
            values=size_factors({key:100})
            self.assertEqual({k for k,v in values.items() if v!=1.},changed)
        for bad in ({'shaft_weight':50},{'overall':True},{'length':float('nan')},{'scrotum':0}):
            with self.assertRaises(ValueError):size_factors(bad)

    def test_exported_samples_have_identity_midpoint_and_correct_zero_support(self):
        self.assertEqual(len(normalized_samples('length')),101)
        self.assertEqual(len(normalized_samples('overall')),100)
        self.assertEqual(normalized_samples('length')[50],1.)
        self.assertEqual(normalized_samples('overall')[49],1.)
