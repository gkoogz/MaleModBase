import unittest
import numpy as np
from malemod_base.modular_anatomy import Mesh,clip
from malemod_base.modular_shading import corner_frames,unit,boundary_uv_field


class ModularShadingTest(unittest.TestCase):
    def test_boundary_uv_preserves_chart_and_affine_field(self):
        x,y=np.meshgrid(np.linspace(0,2,7),np.linspace(0,1,6))
        p=np.c_[x.ravel(),y.ravel(),np.zeros(x.size)];faces=[]
        for j in range(5):
            for i in range(6):
                a=j*7+i;faces.extend([[a,a+1,a+8],[a,a+8,a+7]])
        fixed=np.flatnonzero((x.ravel()==0)|(x.ravel()==2)|(y.ravel()==0)|(y.ravel()==1))
        expected=p[:,:2]@np.array([[.2,.1],[-.1,.3]])+[.4,.2]
        before=p.copy();field=boundary_uv_field(p,faces,fixed,expected[fixed])
        np.testing.assert_array_equal(field[fixed],expected[fixed])
        np.testing.assert_allclose(field,expected,atol=1e-8)
        np.testing.assert_array_equal(p,before)

    def test_cut_inherits_field_instead_of_recomputing_one_sided_normals(self):
        points=np.array([[0,0,0],[2,0,0],[0,2,0]],float)
        mesh=Mesh.source(points,np.array([[0,1,2]]),points[:,:2])
        mesh.normal_field=unit(np.array([[0,0,1],[.3,0,1],[0,.4,1]]))
        cut=clip(mesh,[1,0,0],.5)
        np.testing.assert_allclose(cut.normal_field,cut.donors@mesh.normal_field,atol=1e-14)
        cut2=clip(cut,[0,1,0],.25)
        np.testing.assert_allclose(cut2.normal_field,cut2.donors@mesh.normal_field,atol=1e-14)

    def test_uv_split_keeps_shared_normal_and_mirrored_tangent_orientation(self):
        p=np.array([[0,0,0],[1,0,0],[1,1,0],[0,1,0]],float)
        mesh=Mesh.source(p,np.array([[0,1,2],[0,2,3]]),p[:,:2])
        mesh.corner_uv[1,:,0]=1-mesh.corner_uv[1,:,0]
        before=mesh.corner_uv.copy();frames=corner_frames(mesh)
        np.testing.assert_array_equal(frames['normal'][0,0],frames['normal'][1,0])
        np.testing.assert_array_equal(frames['normal'][0,2],frames['normal'][1,1])
        np.testing.assert_array_equal(frames['sign'][0],1)
        np.testing.assert_array_equal(frames['sign'][1],-1)
        np.testing.assert_allclose(np.sum(frames['normal']*frames['tangent'],axis=2),0,atol=1e-14)
        np.testing.assert_array_equal(mesh.corner_uv,before)
        self.assertEqual(len(frames['singular_uv_faces']),0)

    def test_singular_uv_is_reported_not_called_textured_ready(self):
        p=np.array([[0,0,0],[1,0,0],[0,1,0]],float)
        mesh=Mesh.source(p,np.array([[0,1,2]]),np.zeros((3,2)))
        frames=corner_frames(mesh)
        np.testing.assert_array_equal(frames['singular_uv_faces'],[0])
        self.assertTrue(np.isfinite(frames['tangent']).all())


if __name__=='__main__':unittest.main()
