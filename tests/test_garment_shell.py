import unittest
import numpy as np
from malemod_base.garment_shell import thin_shell,clip_scalar_band


class ShellTests(unittest.TestCase):
    def test_two_cuts_preserve_corner_lineage(self):
        p=np.array([[0,0,0],[2,0,0],[0,2,0]],float)
        r=clip_scalar_band(p,[[0,1,2]],p[:,0]-.3,1.1-p[:,0])
        self.assertTrue(np.all(r['positions'][:,0]>=.3-1e-10))
        self.assertTrue(np.all(r['positions'][:,0]<=1.1+1e-10))
        self.assertTrue(np.allclose(r['weights'].sum(1),1))
        self.assertTrue(np.allclose(np.einsum('ij,ijk->ik',r['weights'],p[r['donors']]),r['positions']))

    def test_volume_and_source_aliases(self):
        # Two triangles have distinct UV corners at a coincident diagonal.
        p=np.array([[0,0,0],[2,0,0],[2,1,0],[0,0,0],[2,1,0],[0,1,0]],float)
        r=thin_shell(p,[[0,1,2],[3,4,5]],np.tile([0,0,1.],(6,1)),.12,offset=.03)
        self.assertEqual(r['boundary_edges'],4)
        self.assertEqual(len(r['triangles']),12)
        self.assertTrue(np.array_equal(r['source'][:12],list(range(6))*2))
        q=r['positions'];t=r['triangles']
        volume=np.sum(np.einsum('ij,ij->i',q[t[:,0]],np.cross(q[t[:,1]],q[t[:,2]])))/6
        self.assertAlmostEqual(abs(volume),2*.12)
        self.assertTrue(np.allclose(q[:6,2],.03))
        self.assertTrue(np.allclose(q[6:12,2],-.09))

    def test_normals_weld_but_edge_shading_is_separate(self):
        p=np.array([[0,0,0],[1,0,0],[0,1,0]],float)
        r=thin_shell(p,[[0,1,2]],np.tile([0,0,1.],(3,1)),.1)
        self.assertEqual(len(set(r['normal_layers'][6:])),3)
        for a,b in zip(r['positions'][6:],r['source'][6:]):
            self.assertTrue(np.allclose(a[:2],p[b,:2]))

    def test_refuse_bad_inputs(self):
        p=np.eye(3)
        for thickness in (0,-1,np.nan):
            with self.assertRaises(ValueError):thin_shell(p,[[0,1,2]],p,thickness)
        with self.assertRaises(ValueError):thin_shell(p,[[0,1,2]],np.zeros_like(p),.1)


if __name__=='__main__':unittest.main()
