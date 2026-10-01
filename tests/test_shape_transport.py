from pathlib import Path
import unittest
import numpy as np
from malemod_base.shape_transport import ShapeTransport, AXES, mapped_knots, inverse_mapping, coordinates
from malemod_base.control_transport import size_factors


class ShapeTransportTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        with np.load(Path(__file__).resolve().parents[1]/'assets/wolverine-reference/geometry.npz') as bank:
            cls.transport=ShapeTransport(bank,np.zeros((10,3)),np.broadcast_to(np.eye(3),(10,3,3)))

    def test_neutral_is_exact_identity_and_all_combined_extremes_stay_positive(self):
        result=self.transport.evaluate()
        np.testing.assert_allclose(result[:,:3],0,atol=1e-12)
        np.testing.assert_allclose(result[:,3:6],1,atol=1e-12)
        lattice=self.transport.lattice()
        self.assertEqual(lattice.shape,(4,4,4,4,10,15))
        self.assertTrue(np.isfinite(lattice).all())
        self.assertGreater(lattice[...,3:6].min(),0)

    def test_maximum_fits_source_sections_better_than_ratio_scaling(self):
        prefs={key:100 for key in AXES}
        target=self.transport.shape.evaluate(prefs).coarse
        fitted=self.transport.evaluate(prefs)
        old=size_factors(prefs)
        new_error=old_error=0
        for i,mask in enumerate(self.transport.masks):
            a=self.transport.reference[mask];b=target[mask]
            prediction=(a*fitted[i,3:6])@fitted[i,6:].reshape(3,3).T+fitted[i,:3]
            naive=a*np.array([old['axial'],old['radial'],old['radial']] if i<8 else [old['lobes']]*3)
            new_error+=np.sum((prediction-b)**2)
            old_error+=np.sum((naive-b)**2)
            np.testing.assert_allclose(prediction.mean(0),b.mean(0),atol=1e-10)
        self.assertLess(new_error,old_error*.5)

    def test_lattice_coordinates_preserve_authored_breakpoints_default_and_endpoints(self):
        from malemod_base.controls import mapped,limits
        for key in AXES:
            knots=mapped_knots(key)
            for value in knots:
                self.assertAlmostEqual(mapped(key,inverse_mapping(key,value)),value,places=12)
            table=coordinates(key);lo,_=limits(key)
            self.assertEqual(table[0],0);self.assertEqual(table[-1],3)
            self.assertAlmostEqual(table[50-lo],float(np.flatnonzero(knots==mapped(key,50))[0]))
