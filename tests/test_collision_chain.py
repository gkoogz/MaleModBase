import unittest
import numpy as np
from malemod_base.collision_chain import chain, closed_link, ovoid, edge_arc, circle, hemisphere_dome


class CollisionChainTests(unittest.TestCase):
    def closed(self,faces):
        edges=np.sort(np.vstack([faces[:,[0,1]],faces[:,[1,2]],faces[:,[2,0]]]),axis=1)
        _,count=np.unique(edges,axis=0,return_counts=True)
        self.assertTrue(np.all(count==2))

    def test_dome_keeps_exact_rim_and_apex_and_is_closed(self):
        center=np.array([3.,4.,5.]);apex=center+[0.,0.,3.]
        rim=circle(center,[0,0,1],2.)
        points,faces,proof=hemisphere_dome(rim,center,apex)
        np.testing.assert_array_equal(points[:64],rim)
        np.testing.assert_array_equal(points[-2],apex)
        local=points[:-2]-center
        np.testing.assert_allclose((local[:,:2]**2).sum(1)/4+local[:,2]**2/9,1.,atol=1e-12)
        self.assertEqual(np.count_nonzero(np.linalg.norm(points-apex,axis=1)<1e-10),1)
        self.closed(faces)
        triangles=points[faces]
        self.assertTrue((np.linalg.norm(np.cross(triangles[:,1]-triangles[:,0],triangles[:,2]-triangles[:,0]),axis=1)>1e-9).all())
        self.assertEqual(proof['axialScale'],1.5)

    def test_equal_radius_and_height_give_true_hemisphere(self):
        points,_,_=hemisphere_dome(circle([0,0,0],[0,0,1],2.),[0,0,0],[0,0,2.])
        np.testing.assert_allclose(np.linalg.norm(points[:-1],axis=1),2.,atol=1e-12)

    def test_joint_circles_and_seven_closed_links(self):
        centers=np.array([[.07*i*i,0,i] for i in range(8)])
        radii=np.linspace(.4,.2,8)
        rings,faces,normals=chain(centers,radii)
        for i in range(8):
            np.testing.assert_allclose(np.linalg.norm(rings[i]-centers[i],axis=1),radii[i],atol=1e-12)
            np.testing.assert_allclose((rings[i]-centers[i])@normals[i],0,atol=1e-12)
        for i in range(7):
            points,triangles=closed_link(rings[i],rings[i+1],centers[i],centers[i+1])
            self.closed(triangles)

    def test_ovoid_same_physics_origin_axes_and_shape(self):
        center=np.array([2,3,4]);radii=np.array([2,1,3]);axes=np.array([[0,1,0],[-1,0,0],[0,0,1]])
        p,f=ovoid(center,axes,radii)
        local=(p-center)@axes.T/(radii*1.03)
        z=local[:,2]
        np.testing.assert_allclose(np.sum(local[:,:2]**2,axis=1),(1-.13*z)**2*(1-z*z),atol=1e-12)
        self.closed(f)

    def test_eighty_degree_arc_stays_on_nonlevel_bottom_edge(self):
        t=np.arange(64)*2*np.pi/64
        edge=np.column_stack([2*np.cos(t),np.sin(t),.2*np.cos(2*t)])
        arc=edge_arc(edge,[1,0,0],[0,1,0])
        angles=np.degrees(np.arctan2(arc[:,1],arc[:,0]))
        np.testing.assert_allclose(angles,np.linspace(-40,40,81),atol=1e-10)
        self.assertGreater(np.ptp(arc[:,2]),.1)


if __name__=='__main__':unittest.main()
