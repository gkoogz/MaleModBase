import unittest
import numpy as np
from PIL import Image
from malemod_base.skin_match import match,sample,linear
from malemod_base.refinement import refine


class TestSkinRefinement(unittest.TestCase):
    def test_measured_seam_match_and_detail(self):
        p=np.array([[[160,120,80,70],[180,135,90,90]]],dtype=np.uint8)
        source=np.tile([160,120,80],(4,1));target=np.tile([145,110,115],(4,1))
        im,proof=match(Image.fromarray(p),source,target);a=np.asarray(im)
        np.testing.assert_array_equal(a[0,0,:3],target[0]);np.testing.assert_array_equal(a[:,:,3],p[:,:,3])
        self.assertTrue(np.all(a[0,1,:3]>a[0,0,:3]))
        np.testing.assert_array_equal(sample(im,[[0,0],[1,1]]),a[0,:,:3])

    def test_conforming_edges_and_source_lineage(self):
        p=np.array([[0,0,0],[1,0,0],[1,1,0],[0,1,0]],float);f=np.array([[0,1,2],[0,2,3]])
        q,g,lineage,parent=refine(p,f,[True,False],[(0,1)])
        np.testing.assert_array_equal(q,lineage@p);np.testing.assert_array_equal(q[:4],p)
        cross=np.cross(q[g[:,1]]-q[g[:,0]],q[g[:,2]]-q[g[:,0]])
        self.assertTrue(np.all(cross[:,2]>0));self.assertAlmostEqual(cross[:,2].sum()/2,1)
        edges={tuple(sorted(e)) for tri in g for e in [(tri[0],tri[1]),(tri[1],tri[2]),(tri[2],tri[0])]}
        self.assertNotIn((0,2),edges);self.assertIn((0,1),edges)

    def test_uv_aliases_refine_together(self):
        p=np.array([[0,0,0],[1,0,0],[1,1,0],[0,0,0],[1,1,0],[0,1,0]],float)
        q,f,l,parent=refine(p,[[0,1,2],[3,4,5]],[True,False],aliases=[0,1,2,0,2,3])
        self.assertEqual(sum(np.all(q==[.5,.5,0],axis=1)),2)
        donors=[tuple(l.getrow(i).indices) for i in np.flatnonzero(np.all(q==[.5,.5,0],axis=1))]
        self.assertEqual(set(donors),{(0,2),(3,4)})
