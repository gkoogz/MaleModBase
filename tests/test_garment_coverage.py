import unittest
import numpy as np
from malemod_base.garment_coverage import radial_coverage

class Coverage(unittest.TestCase):
    def test_cut_height_radius_and_seam(self):
        a=np.deg2rad([170,190,170,190]);p=np.column_stack((np.cos(a)*5,np.sin(a)*5,[0,0,10,10]))
        q=np.array([[-4,0,5],[-6,0,5],[-4,0,12],[4,0,5]],float)
        self.assertEqual(radial_coverage(p,[[0,1,2],[1,3,2]],q,radial_axes=(0,1),height_axis=2).tolist(),[True,False,False,False])
if __name__=='__main__':unittest.main()
