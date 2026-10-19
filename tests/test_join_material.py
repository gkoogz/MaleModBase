import unittest
import numpy as np
from malemod_base.join_material import support_weights, common_detail, pixel_triangles


class JoinMaterialTest(unittest.TestCase):
    def test_support_crosses_component_join_and_protects_far_surface(self):
        p=np.array([[x,y,0.] for x in range(7) for y in [0,1]])
        f=np.array([[2*x,2*x+2,2*x+1] for x in range(6)]+[[2*x+1,2*x+2,2*x+3] for x in range(6)])
        w=support_weights(p,f,[4,5,6,7],2.5,[12,13])
        np.testing.assert_equal(w[4:8],1)
        self.assertGreater(w[8],0);self.assertEqual(w[12],0)
        limited=support_weights(p,f,[4,5,6,7],10,[12,13])
        self.assertEqual(limited[12],0);self.assertEqual(limited[6],1)
        with self.assertRaises(ValueError):support_weights(p,f,[4,5,6,7],10,[6,13])

    def test_detail_has_no_component_or_uv_boundary(self):
        tile=np.random.default_rng(2).uniform(.1,.3,(24,24,3))
        p=np.array([[1.,2.,3.],[1.,2.,3.],[1.+1e-7,2.,3.]])
        n=np.tile([0.,0.,1.],(3,1))
        result=common_detail(tile,p,n,.3)
        np.testing.assert_array_equal(result[0],result[1])
        np.testing.assert_allclose(result[0],result[2],atol=1e-6)
        np.testing.assert_allclose(common_detail(np.ones_like(tile)*.2,p,n,.3),0,atol=1e-12)

    def test_raster_uses_gpu_texel_centers_and_barycentric_positions(self):
        uv=np.array([[.125,.125],[.875,.125],[.125,.875]])
        for y,x,b in pixel_triangles(uv,4,4):
            np.testing.assert_allclose(b.sum(1),1)
            np.testing.assert_allclose(b@uv,np.c_[(x+.5)/4,(y+.5)/4])


if __name__=='__main__':unittest.main()
