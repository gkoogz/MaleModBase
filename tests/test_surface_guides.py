import unittest
import numpy as np
from malemod_base.surface_guides import project_surface, bezier, smooth_clearance_curve, fixed_ribbon


class SurfaceGuideTests(unittest.TestCase):
    def test_triangle_interior_edge_corner_and_clearance(self):
        vertices = np.array([[0.,0.,0.], [2.,0.,0.], [0.,2.,0.]])
        points = [[.5,.5,4.], [2.,2.,1.], [-1.,-1.,3.]]
        projected, normals, ids, bary = project_surface(
            points, vertices, [[0,1,2]], np.tile([0.,0.,1.], (3,1)), .1)
        np.testing.assert_allclose(projected, [[.5,.5,.1], [1.,1.,.1], [0.,0.,.1]])
        np.testing.assert_allclose(bary.sum(1), 1.)
        self.assertTrue((bary>=0).all())
        np.testing.assert_allclose(bary@vertices+.1*normals, projected)
        np.testing.assert_array_equal(ids, [0,0,0])

    def test_curve_preserves_both_attachment_endpoints(self):
        controls=np.array([[1.,2.,3.],[4.,5.,6.],[7.,8.,9.],[10.,11.,12.]])
        curve=bezier(controls,81)
        np.testing.assert_array_equal(curve[[0,-1]],controls[[0,-1]])

    def test_smooth_hem_preserves_endpoints_and_clears_obstacle_without_kinks(self):
        t=np.linspace(0,1,81)
        guide=np.column_stack([2+.15*np.sin(t*30),np.zeros(len(t)),10*t])
        def lift(points):
            raised=points.copy();middle=(points[:,2]>4)&(points[:,2]<6)
            raised[middle,0]=np.maximum(raised[middle,0],3.)
            return raised
        def clearance(points):
            return np.maximum.reduce([points[:,0]-3.,4.-points[:,2],points[:,2]-6.])
        curve,proof=smooth_clearance_curve(guide,lift,[1.,0.,0.],clearance_values=clearance)
        np.testing.assert_array_equal(curve[[0,-1]],guide[[0,-1]])
        np.testing.assert_allclose(lift(curve),curve,atol=1e-5)
        np.testing.assert_allclose(curve,bezier(proof['controls'],161))
        self.assertLess(curve[:,0].max(),5.)
        self.assertTrue(proof['boundedFit'])

    def test_fixed_hem_ribbon_is_closed_and_keeps_its_width(self):
        curve=bezier([[0,0,0],[.5,0,2],[1,1,3],[0,1,5]],41)
        vertices,faces=fixed_ribbon(curve,.3,.04,[1.,0.,0.])
        rows=vertices.reshape(-1,4,3)
        np.testing.assert_allclose((rows[:,0]+rows[:,1])/2,curve)
        np.testing.assert_allclose(np.linalg.norm(rows[:,1]-rows[:,0],axis=1),.3)
        np.testing.assert_allclose(np.linalg.norm(rows[:,2]-rows[:,1],axis=1),.04)
        edges=np.sort(np.vstack([faces[:,[0,1]],faces[:,[1,2]],faces[:,[2,0]]]),axis=1)
        _,counts=np.unique(edges,axis=0,return_counts=True)
        self.assertTrue((counts==2).all())
        triangles=vertices[faces]
        self.assertTrue((np.linalg.norm(np.cross(triangles[:,1]-triangles[:,0],triangles[:,2]-triangles[:,0]),axis=1)>1e-9).all())


if __name__=='__main__':
    unittest.main()
