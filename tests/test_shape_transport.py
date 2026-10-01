from pathlib import Path
import unittest
import numpy as np
from malemod_base.shape_transport import ShapeTransport, AXES, mapped_knots, inverse_mapping, coordinates
from malemod_base.control_transport import size_factors


class ShapeTransportTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        with np.load(Path(__file__).resolve().parents[1]/'assets/wolverine-reference/geometry.npz') as bank:
            from malemod_base.authored_shape import AuthoredShape
            shape=AuthoredShape(bank)
            origins=np.array([shape.base[(abs(shape.flex-t)<.075)&(shape.shaft>.5)].mean(0)
                              for t in np.linspace(0,1,8)]+
                             [shape.base[(shape.ball>.8)&(shape.base[:,1]*side>.25)].mean(0) for side in [-1,1]])
            cls.transport=ShapeTransport(bank,origins,np.broadcast_to(np.eye(3),(10,3,3)))

    def test_neutral_is_exact_identity_and_all_combined_extremes_stay_positive(self):
        result=self.transport.evaluate()
        np.testing.assert_allclose(result[:,:3],0,atol=1e-12)
        np.testing.assert_allclose(result[:,3:6],1,atol=1e-12)
        lattice=self.transport.lattice()
        self.assertEqual(lattice.shape,(4,4,4,4,10,15))
        self.assertTrue(np.isfinite(lattice).all())
        self.assertGreater(lattice[...,3:6].min(),0)

    def test_maximum_has_common_shaft_radius_and_coherent_crown_transform(self):
        prefs={key:100 for key in AXES}
        fitted=self.transport.evaluate(prefs)
        frame=self.transport.measure_frame(prefs)
        radius=frame.body_radius/self.transport.reference_frame.body_radius
        np.testing.assert_allclose(fitted[:8,3:6],radius,atol=1e-12)
        point=self.transport.reference[-1]
        transformed=[]
        for i in (6,7):
            origin=self.transport.origins[i]
            transformed.append((point-origin)*fitted[i,3:6]+origin+fitted[i,:3])
        np.testing.assert_allclose(*transformed,atol=1e-12)
        # Convex skin blending of two crown joints cannot kink or squash it:
        # both joints implement the same similarity transform on every point.
        np.testing.assert_allclose(.37*transformed[0]+.63*transformed[1],transformed[0],atol=1e-12)

    def test_lattice_coordinates_preserve_authored_breakpoints_default_and_endpoints(self):
        from malemod_base.controls import mapped,limits
        for key in AXES:
            knots=mapped_knots(key)
            for value in knots:
                self.assertAlmostEqual(mapped(key,inverse_mapping(key,value)),value,places=12)
            table=coordinates(key);lo,_=limits(key)
            self.assertEqual(table[0],0);self.assertEqual(table[-1],3)
            self.assertAlmostEqual(table[50-lo],float(np.flatnonzero(knots==mapped(key,50))[0]))
