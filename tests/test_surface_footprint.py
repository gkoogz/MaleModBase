import unittest
import numpy as np
from malemod_base.surface_footprint import ContactArea, transfer_chart


class ContactAreaTest(unittest.TestCase):
    def surface(self):
        # Same chart area per triangle; the left patch has a larger physical
        # metric. A chart-area or axial-length doubling gives the wrong answer.
        xy=np.array([[-6,-2],[0,-2],[4,-2],[-6,2],[0,2],[4,2]],float)
        p=np.c_[xy[:,0],xy[:,1],np.maximum(-xy[:,0],0)*2]
        f=np.array([[0,1,3],[1,4,3],[1,2,4],[2,5,4]])
        return p,f,xy

    def test_doubles_physical_contact_with_fixed_front(self):
        p,f,xy=self.surface();area=ContactArea(p,f,xy,np.ones(len(f),bool))
        footprint=np.array([[0,-1],[3,-1],[3,1],[0,1]],float)
        result,report=area.expand_rear(footprint,2,4)
        self.assertAlmostEqual(report['baselineArea'],6)
        self.assertAlmostEqual(report['measuredArea'],12,delta=.00012)
        self.assertAlmostEqual(report['rearExtension'],3/np.sqrt(5),delta=.0001)
        np.testing.assert_array_equal(result[1:3],footprint[1:3])
        self.assertLess(result[0,0],0)
        np.testing.assert_allclose(area.centroid(footprint),[1.5,0,0],atol=1e-12)
        np.testing.assert_allclose(area.centroid(result),[(1.5-1.5/np.sqrt(5))/2,0,1.5/np.sqrt(5)],atol=1e-4)

    def test_never_silently_clamps_or_counts_overlapping_surface(self):
        p,f,xy=self.surface();area=ContactArea(p,f,xy,np.ones(len(f),bool))
        footprint=np.array([[0,-1],[3,-1],[3,1],[0,1]],float)
        with self.assertRaisesRegex(ValueError,'Insufficient'):
            area.expand_rear(footprint,2,.1)
        with self.assertRaisesRegex(ValueError,'leaves measured'):
            area.area(footprint+[-20,0])
        duplicate=np.vstack([f,f[0]])
        with self.assertRaisesRegex(ValueError,'overlaps'):
            ContactArea(p,duplicate,xy,np.ones(len(duplicate),bool)).area(footprint+[-2,0])

    def test_edge_refinement_preserves_chart_donors_and_explicit_periodicity(self):
        p=np.array([[0,0,0],[2,0,0],[0,2,0]],float);f=np.array([[0,1,2]])
        xy=np.array([[0,3.1],[2,-3.1],[0,2.9]])
        query=np.array([[1,0,0],[.5,.5,0]])
        original=p.copy()
        ordinary=transfer_chart(query,p,f,xy)
        np.testing.assert_allclose(ordinary,[[1,0],[.5,1.5]])
        periodic=transfer_chart(query,p,f,xy,periodic_axes=(1,))
        self.assertAlmostEqual(abs(periodic[0,1]),np.pi)
        np.testing.assert_array_equal(p,original)
        with self.assertRaisesRegex(ValueError,'no exact chart donor'):
            transfer_chart([[1,0,1]],p,f,xy)

    def test_identity_area_request_is_exact(self):
        p,f,xy=self.surface();area=ContactArea(p,f,xy,np.ones(len(f),bool))
        footprint=np.array([[0,-1],[3,-1],[3,1],[0,1]],float)
        result,report=area.expand_rear(footprint,1,2)
        np.testing.assert_array_equal(result,footprint)
        self.assertEqual(report['rearExtension'],0)


if __name__=='__main__':unittest.main()
