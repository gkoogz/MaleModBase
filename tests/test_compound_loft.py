import unittest
import numpy as np
from malemod_base.modular_anatomy import compound_loft_orientation,loft_orientation


class CompoundLoftTest(unittest.TestCase):
    def fixture(self):
        # A narrow tube turns smoothly around a broad arc. Most of the turn
        # happens early: linearly mixing endpoint normals is not its exterior.
        t=.5*(1-np.cos(np.pi*np.arange(25)/24))
        angle=2.8*(1-np.exp(-15*t))/(1-np.exp(-15))
        phi=np.arange(64)*2*np.pi/64
        normals=np.array([np.c_[np.cos(phi)*np.cos(a),np.cos(phi)*np.sin(a),np.sin(phi)] for a in angle])
        center=np.c_[4*np.cos(angle),4*np.sin(angle),np.zeros(len(angle))]
        points=center[:,None,:]+.4*normals
        rings=np.arange(points.size//3).reshape(points.shape[:2])
        return points.reshape(-1,3),rings,normals[0],normals[-1]

    def test_large_normal_turn_is_not_a_fold(self):
        p,r,a,b=self.fixture()
        self.assertGreater(loft_orientation(p,r,a,b)['reversedFaces'],0)
        result=compound_loft_orientation(p,r,a,b)
        self.assertEqual(result['reversedFaces'],0)
        self.assertGreater(result['minimumAdjacentCosine'],0)

    def test_closed_rim_cannot_hide_an_interior_fold(self):
        p,r,a,b=self.fixture()
        p[r[10]]=p[np.roll(r[10],24)]
        self.assertGreater(compound_loft_orientation(p,r,a,b)['reversedFaces'],0)

    def test_wrong_boundary_orientation_is_rejected(self):
        p,r,a,b=self.fixture()
        self.assertGreater(compound_loft_orientation(p,r,-a,b)['reversedFaces'],0)

    def test_collapsed_row_is_rejected(self):
        p,r,a,b=self.fixture();p[r[7]]=p[r[6]]
        self.assertGreater(compound_loft_orientation(p,r,a,b)['collapsedFaces'],0)


if __name__=='__main__':unittest.main()
