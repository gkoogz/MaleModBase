import unittest
import numpy as np
from malemod_base.fly_panels import folded_fly

class FlyPanels(unittest.TestCase):
    def test_cut_lineage_and_hinges(self):
        p=np.array([[10,-10,0],[10,10,0],[10,-10,10],[10,10,10]],float)
        t=np.array([[0,1,2],[1,3,2]])
        r=folded_fly(p,t,front_axis=0,side_axis=1,height_axis=2,front_plane=0,lower=0,upper=10,half_width=5,angle=np.deg2rad(140))
        rest=np.einsum('ij,ijk->ik',r['weights'],p[r['donors']])
        self.assertTrue(np.allclose(r['weights'].sum(axis=1),1))
        self.assertTrue(np.all(r['weights']>=-1e-9))
        self.assertTrue(np.allclose(r['positions'][r['panels']==0],rest[r['panels']==0]))
        for side in (-1,1):
            panel=r['panels']==side;hinge=side*rest[:,2]*.5
            edge=panel & (np.abs(rest[:,1]-hinge)<1e-8)
            self.assertTrue(np.any(edge));self.assertTrue(np.allclose(r['positions'][edge],rest[edge]))
            self.assertTrue(np.all(r['positions'][panel,0]>=rest[panel,0]-1e-8))
        self.assertGreater(len(r['triangles']),len(t))
    def test_back_preserved_and_invalid_rejected(self):
        p=np.array([[-10,-2,8],[-10,2,8],[-10,0,10]],float)
        args=dict(front_axis=0,side_axis=1,height_axis=2,front_plane=0,lower=0,upper=10,half_width=5,angle=2)
        r=folded_fly(p,[[0,1,2]],**args)
        self.assertTrue(np.allclose(r['positions'],p));self.assertTrue(np.all(r['panels']==0))
        with self.assertRaises(ValueError):folded_fly(p,[[0,1,2]],**dict(args,upper=0))

if __name__=='__main__':unittest.main()
