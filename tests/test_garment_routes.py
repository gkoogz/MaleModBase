import unittest
import numpy as np
from malemod_base.garment_routes import classic_route_targets


class MeasuredRoutes(unittest.TestCase):
    def test_shared_steering_units_and_frame(self):
        contour = np.array([[-1,-.5,2],[1,-.5,2],[-1,.5,2],[1,.5,2]])
        args = (contour,[1,0,2],[.7,-.6,.8],[.2,-.4,.9],[0,1,0],[0,0,1])
        targets, proof = classic_route_targets(*args)
        self.assertAlmostEqual(targets[0,1], .12)
        self.assertAlmostEqual(targets[1,2], .728)
        self.assertAlmostEqual(targets[2,2], .8568)
        scale=100.;scaled,_=classic_route_targets(contour*scale,np.array(args[1])*scale,np.array(args[2])*scale,np.array(args[3])*scale,*args[4:])
        np.testing.assert_allclose(scaled,targets*scale,rtol=0,atol=1e-12)
        rotation=np.array([[0,-1,0],[1,0,0],[0,0,1]])
        rotated,_=classic_route_targets(contour@rotation.T,rotation@args[1],rotation@args[2],rotation@args[3],rotation@args[4],rotation@args[5])
        np.testing.assert_allclose(rotated,targets@rotation.T,rtol=0,atol=1e-12)
        self.assertEqual(proof['version'],2)
        with self.assertRaises(ValueError):classic_route_targets(*args[:4],[0,2,0],args[5])


if __name__=='__main__':
    unittest.main()
