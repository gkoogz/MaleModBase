import unittest
import numpy as np
from scipy.spatial import ConvexHull
from malemod_base.meridian_surface import cloth_surface, triangle_clearance


class MeridianSurfaceTests(unittest.TestCase):
    def test_32_ray_fitted_boundary_is_preserved(self):
        theta=np.arange(32)*2*np.pi/32
        starts=np.column_stack([3*np.cos(theta),3*np.sin(theta),np.full(32,3.)])
        fitted=starts.copy();fitted[:,2]+=.05*np.cos(theta)
        paths=[np.array([p,[0,0,0]]) for p in fitted]
        points,faces,proof=cloth_surface(paths,np.vstack([starts,starts[0]]),
            np.arange(32)/32,[0,0,0],[0,0,-1],[0,0,-1],[],rows=24,
            subdivisions=1,end_density=True,boundary_points=fitted)
        np.testing.assert_array_equal(points[:32],fitted)
        self.assertEqual(len(points),769)
        self.assertEqual(len(faces),1504)
        self.assertEqual(proof['boundaryVertices'],32)
        with self.assertRaisesRegex(ValueError,'every angular column'):
            cloth_surface(paths,np.vstack([starts,starts[0]]),np.arange(32)/32,
                [0,0,0],[0,0,-1],[0,0,-1],[],subdivisions=1,boundary_points=fitted[:-1])

    def test_loft_clears_a_proxy_that_intersects_initial_panels(self):
        theta=np.arange(12)*2*np.pi/12
        starts=np.column_stack([2*np.cos(theta),2*np.sin(theta),np.full(12,3.)])
        paths=[np.array([p,[0,0,0]]) for p in starts]
        proxy=np.array([[1.1,0,1],[0,1.1,1],[-1.1,0,1],[0,-1.1,1],[0,0,.1],[0,0,1.9]])
        hull=ConvexHull(proxy)
        points,faces,proof=cloth_surface(paths,np.vstack([starts,starts[0]]),np.arange(12)/12,
            [0,0,0],[0,0,-1],[0,0,-1],[(proxy,hull.simplices)],rows=20,subdivisions=2)
        scores,_=triangle_clearance(points,faces,hull.equations)
        self.assertGreaterEqual(scores.min(),-1e-7)
        self.assertGreater(proof['maximumClearanceAdjustment'],.1)
        self.assertLess(proof['maximumClearanceAdjustment'],1.)

    def test_rows_cluster_at_both_seam_and_tip(self):
        theta=np.arange(8)*2*np.pi/8
        starts=np.column_stack([3*np.cos(theta),3*np.sin(theta),np.full(8,3.)])
        _,_,proof=cloth_surface([np.array([p,[0,0,0]]) for p in starts],
            np.vstack([starts,starts[0]]),np.arange(8)/8,[0,0,0],[0,0,-1],
            [0,0,-1],[],rows=16,subdivisions=2,end_density=True)
        steps=np.diff(proof['rowParameters'])
        self.assertLess(steps[0],steps[7]/5)
        self.assertLess(steps[-1],steps[7]/5)
        self.assertEqual(proof['boundaryError'],0.)

    def test_triangle_check_detects_interior_cut_with_clear_corners(self):
        cube=np.array([[x,y,z] for x in [-1,1] for y in [-1,1] for z in [-1,1]])
        planes=ConvexHull(cube).equations
        points=np.array([[2.,0,0],[0,2.,0],[0,0,2.]])
        self.assertTrue(np.all((points@planes[:,:3].T+planes[:,3]).max(1)>0))
        scores,_=triangle_clearance(points,np.array([[0,1,2]]),planes)
        self.assertLess(scores[0],0)

    def test_loft_is_one_boundary_disk_and_keeps_seam(self):
        theta=np.arange(8)*2*np.pi/8
        starts=np.column_stack([3*np.cos(theta),3*np.sin(theta),np.full(8,3.)])
        paths=[np.array([p,[0,0,0]]) for p in starts]
        outline=np.vstack([starts,starts[0]])
        points,faces,proof=cloth_surface(paths,outline,np.arange(8)/8,[0,0,0],
            [0,0,-1],[0,0,-1],[],rows=8,subdivisions=2)
        np.testing.assert_allclose(points[:16:2],starts,atol=3e-15)
        self.assertEqual(proof['boundaryVertices'],16)
        self.assertGreater(proof['minimumTriangleArea'],0)
        self.assertEqual(len(points)-len(np.unique(np.sort(np.vstack([
            faces[:,[0,1]],faces[:,[1,2]],faces[:,[2,0]]]),axis=1),axis=0))+len(faces),1)


if __name__=='__main__':unittest.main()
