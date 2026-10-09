import unittest
import numpy as np
from malemod_base.fly_panels import folded_fly,fold_rigid_attachment,fly_motion_bindings

class FlyPanels(unittest.TestCase):
    def test_motion_seam_aliases_and_belt_band(self):
        p=np.array([[10,5,10],[10,5,10],[10,0,10],[10,0,5],[10,0,10]],float)
        r=fly_motion_bindings(p,[1,1,1,-1,0],lower=0,upper=10,half_width=5,belt_lower=7,belt_upper=9)
        np.testing.assert_array_equal(r['fly'],[0,0,1,1,0])
        np.testing.assert_array_equal(r['belt'],[0,0,1,0,0])
        np.testing.assert_array_equal(r['hinges'][0],r['hinges'][1])
        with self.assertRaises(ValueError):fly_motion_bindings(p,[1]*5,lower=0,upper=10,half_width=5,belt_lower=9,belt_upper=7)
    def test_rigid_buckle_retains_distances_and_follows_flap_centroid(self):
        p=np.array([[10,-2,9],[11,2,9],[10,-2,11],[11,2,11]],float)
        for side in (-1,1):
            q=fold_rigid_attachment(p,front_axis=0,side_axis=1,height_axis=2,lower=0,upper=12,half_width=5,angle=2,side=side)
            np.testing.assert_allclose(np.linalg.norm(q[:,None]-q[None,:],axis=2),np.linalg.norm(p[:,None]-p[None,:],axis=2),atol=1e-12)
            center=p.mean(0);hinge=side*5*center[2]/12;offset=center[1]-hinge
            np.testing.assert_allclose(q.mean(0),[center[0]-side*offset*np.sin(2),hinge+offset*np.cos(2),center[2]])
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
