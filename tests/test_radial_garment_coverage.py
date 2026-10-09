import unittest
import numpy as np
from malemod_base.radial_garment_coverage import radial_coverage,radial_triangle_coverage

class Coverage(unittest.TestCase):
    def test_visibility_footprint_can_mask_skin_beyond_cloth_without_masking_cuts(self):
        a=np.deg2rad([170,190,170,190]);p=np.column_stack((np.cos(a)*5,np.sin(a)*5,[0,0,10,10]))
        t=[[0,1,2],[1,3,2]];q=np.array([[-6,0,5],[-6,0,.1],[-6,0,12],[6,0,5]])
        args=dict(radial_axes=(0,1),height_axis=2,cut_inset=.3)
        self.assertEqual(radial_coverage(p,t,q,check_depth=False,**args).tolist(),[True,False,False,False])
        self.assertFalse(np.any(radial_coverage(p,t,q,**args)))
        body=np.array([[-6,-.1,4],[-6,.1,4],[-6,0,5]])
        self.assertTrue(radial_triangle_coverage(p,t,body,[[0,1,2]],check_depth=False,**args)[0])
        self.assertFalse(radial_triangle_coverage(p,t,body,[[0,1,2]],**args)[0])
    def test_cut_height_radius_and_seam(self):
        a=np.deg2rad([170,190,170,190]);p=np.column_stack((np.cos(a)*5,np.sin(a)*5,[0,0,10,10]))
        q=np.array([[-4,0,5],[-6,0,5],[-4,0,12],[4,0,5]],float)
        self.assertEqual(radial_coverage(p,[[0,1,2],[1,3,2]],q,radial_axes=(0,1),height_axis=2).tolist(),[True,False,False,False])
    def test_cut_support_preserves_skin_without_treating_uv_alias_as_cut(self):
        a=np.deg2rad([170,190,170,190]);p=np.column_stack((np.cos(a)*5,np.sin(a)*5,[0,0,10,10]))
        p=np.concatenate((p,p[[1,2]]));t=[[0,1,2],[4,3,5]]
        q=[[-4,0,.1],[-4,0,5]]
        self.assertEqual(radial_coverage(p,t,q,radial_axes=(0,1),height_axis=2,cut_inset=.3).tolist(),[False,True])
    def test_face_crossing_concave_opening_keeps_original_skin(self):
        p=np.array([[5,0,0],[5,2,0],[5,2,4],[5,0,4],[5,2,2],[5,4,0],[5,4,2]],float)
        t=[[0,1,2],[0,2,3],[1,5,6],[1,6,4]]
        b=np.array([[4,1.2,3.8],[4,3.1,1.5],[4,.4,.5]],float)
        self.assertTrue(np.all(radial_coverage(p,t,b,radial_axes=(0,1),height_axis=2)))
        self.assertFalse(radial_triangle_coverage(p,t,b,[[0,1,2]],radial_axes=(0,1),height_axis=2,cut_inset=0.)[0])
if __name__=='__main__':unittest.main()
