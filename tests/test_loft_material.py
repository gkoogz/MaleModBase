import unittest
import numpy as np
from malemod_base.loft_material import chart, bake_patch, periodic_samples, surface_density, retained_skin_tile, surface_uv_gradients, sampled_uv_directions


class LoftMaterialTest(unittest.TestCase):
    def test_quantized_tiny_ears_do_not_control_material_derivative(self):
        x,y=np.meshgrid(np.linspace(-.02,.02,21),np.linspace(-.02,.02,21))
        p=np.c_[x.ravel(),y.ravel(),np.zeros(x.size)]
        p=np.vstack([p,[[1e-9,0,0],[0,1e-9,0]]])
        uv=(.7+p[:,:2]*.6).astype('f2').astype(float)
        uv[-2:]+=[.00049,0]
        index=int(np.argmin(np.linalg.norm(p,axis=1)))
        result=sampled_uv_directions(p,uv,np.arange(len(p)),[index],[[.2,.1,0]],.001,.03,.03)
        np.testing.assert_allclose(result,[[.12,.06]],atol=.001)

    def test_retained_chart_continues_folds_across_rim(self):
        p=np.array([[0.,0,0],[2,0,0],[0,3,0]])
        uv=np.array([[.1,.2],[.5,.2],[.1,.5]])
        g=surface_uv_gradients(p,[[0,1,2]],uv)
        np.testing.assert_allclose(g,np.broadcast_to([[.2,0,0],[0,.1,0]],g.shape),atol=1e-12)
        s=np.linspace(0,1,64,endpoint=False);u=np.linspace(0,1,129)
        t=np.linspace(0,1,101)[:,None]
        wave=.2+.05*np.sin(2*np.pi*(u[None]*4+t*7))
        field=np.repeat(wave[:,:,None],3,axis=2)
        boundary=np.repeat((.2+.05*np.sin(2*np.pi*s*4))[:,None],3,axis=1)
        result=bake_patch(boundary,boundary,s,np.full((8,8,3),.2),129,101,[4,2],continuations=(field,field))
        # Detail moves along the meridian rather than becoming vertical stripes.
        self.assertLess(np.max(np.abs(result[2]-field[2])),.002)
        np.testing.assert_array_equal(result[0],periodic_samples(boundary,s,u))
        np.testing.assert_array_equal(result[:,0],result[:,-1])

    def test_skin_tile_follows_seeded_component_across_vertex_aliases(self):
        x,y=np.meshgrid(np.linspace(0,1,11),np.linspace(0,1,11))
        points=np.c_[x.ravel(),y.ravel(),np.zeros(x.size)]
        ids=np.arange(len(points)).reshape(11,11);faces=[]
        for row in range(10):
            for col in range(10):
                a,b,c,d=ids[row,col],ids[row,col+1],ids[row+1,col],ids[row+1,col+1]
                faces.extend([[a,b,c],[b,d,c]])
        faces=np.array(faces);uv=points[:,:2]*[.1,.2]+[.52,.1]
        # Split all corners into aliases, as native packed buffers do.
        a=points[faces].reshape(-1,3);ua=uv[faces].reshape(-1,2)
        b=a+[3,0,0];ub=(a[:,:2]*[.25,.4])+[.7,.5]
        p=np.vstack([a,b]);v=np.vstack([ua,ub]);f=np.arange(len(p)).reshape(-1,3)
        center,half,density,count=retained_skin_tile(p,f,v,np.zeros(len(f),bool),[0],1024,512)
        self.assertEqual(count,200)
        self.assertTrue(.52<center[0]/1023<.62)
        self.assertTrue(.1<center[1]/511<.3)
        self.assertAlmostEqual(density,np.sqrt(1023*.1*511*.2))
        self.assertTrue((half>=4).all())
        normal=np.zeros((512,1024,3),dtype='u1');normal[:]=[128,128,255]
        yy,xx=np.mgrid[60:140,575:625]
        normal[60:140,575:625,0]=np.where((xx+yy)%4<2,105,151)
        textured,_,_,_=retained_skin_tile(p,f,v,np.zeros(len(f),bool),[0],1024,512,normal)
        self.assertTrue(575<textured[0]<625 and 60<textured[1]<140)

    def test_density_tracks_surface_scale_and_ignores_collapsed_uv(self):
        p=np.array([[0.,0,0],[2,0,0],[0,2,0],[2,2,0]])
        f=np.array([[0,1,2],[1,3,2]]);uv=p[f,:2]*.03
        self.assertAlmostEqual(surface_density(p,f,uv),.03)
        self.assertAlmostEqual(surface_density(p*3,f,uv),.01)
        uv[1]=0
        self.assertAlmostEqual(surface_density(p,f,uv),.03)

    def test_closed_chart_has_no_cross_seam_triangles_or_degenerate_uv(self):
        theta=np.linspace(0,2*np.pi,24,endpoint=False)
        z=np.array([0,.02,.7,2,3.])
        p=np.stack(np.broadcast_arrays(np.cos(theta)[None],np.sin(theta)[None],z[:,None]),axis=-1).reshape(-1,3)
        rings=np.arange(len(p)).reshape(len(z),-1)
        faces=[]
        for a,b in zip(rings[:-1],rings[1:]):
            for i in range(len(a)):
                j=(i+1)%len(a);faces.extend([[a[i],a[j],b[j]],[a[i],b[j],b[i]]])
        before=p.copy()
        selected,uv,s,t,length,c=chart(p,rings,faces,[0,0,1,1])
        self.assertTrue(selected.all())
        self.assertLess(np.ptp(uv[:,:,0],axis=1).max(),.05)
        e1,e2=uv[:,1]-uv[:,0],uv[:,2]-uv[:,0]
        det=e1[:,0]*e2[:,1]-e1[:,1]*e2[:,0]
        self.assertTrue((det>0).all())
        np.testing.assert_allclose(t,z[:,None]/3+np.zeros((5,24)))
        np.testing.assert_array_equal(p,before)
        faces.append([rings[0,0],rings[0,1],rings[0,2]])
        selection,*_=chart(p,rings,faces,[0,0,1,1])
        self.assertFalse(selection[-1])

    def test_pigment_and_detail_match_both_boundaries_and_closed_seam(self):
        s=np.linspace(0,1,32,endpoint=False)
        a=np.c_[.2+.03*np.sin(s*2*np.pi),np.full(32,.1),np.full(32,.08)]
        b=a+[.2,.1,.1]
        tile=np.random.default_rng(4).uniform(.1,.3,(16,16,3))
        result=bake_patch(a,b,s,tile,129,65,[8,4])
        np.testing.assert_array_equal(result[0],periodic_samples(a,s,np.linspace(0,1,129)))
        np.testing.assert_array_equal(result[-1],periodic_samples(b,s,np.linspace(0,1,129)))
        np.testing.assert_array_equal(result[:,0],result[:,-1])
        self.assertTrue(np.isfinite(result).all())
        # Pigment spans the complete neck without a lower-color plateau.
        pigment=bake_patch(a,b,s,np.full((16,16,3),.2),129,65,[8,4])
        self.assertGreater((pigment[16].mean(0)-a.mean(0))[0],.015)
        self.assertLess((pigment[16].mean(0)-a.mean(0))[0],.03)
        np.testing.assert_allclose(pigment[32].mean(0),(a.mean(0)+b.mean(0))/2,atol=.001)

    def test_detail_tile_does_not_tilt_the_entire_generated_surface(self):
        s=np.linspace(0,1,16,endpoint=False)
        boundary=np.tile([0.,0.,1.],(16,1))
        tile=np.tile([.3,-.2,1.],(16,16,1))
        tile/=np.linalg.norm(tile,axis=2,keepdims=True)
        result=bake_patch(boundary,boundary,s,tile,65,33,[4,2],normal=True)
        np.testing.assert_allclose(result,np.broadcast_to([0.,0.,1.],result.shape),atol=1e-12)

    def test_normal_blend_is_unit_length_with_exact_boundary_vectors(self):
        s=np.linspace(0,1,16,endpoint=False)
        a=np.tile([0.,0.,1.],(16,1));b=np.tile([.6,0.,.8],(16,1))
        tile=np.tile([.2,.1,1.],(16,16,1));tile/=np.linalg.norm(tile,axis=2,keepdims=True)
        result=bake_patch(a,b,s,tile,65,33,[4,2],normal=True)
        np.testing.assert_allclose(np.linalg.norm(result,axis=2),1,atol=1e-12)
        np.testing.assert_array_equal(result[0],np.tile(a[0],(65,1)))
        np.testing.assert_array_equal(result[-1],np.tile(b[0],(65,1)))

    def test_lower_skin_detail_does_not_switch_at_a_narrow_cuff(self):
        s=np.linspace(0,1,16,endpoint=False)
        a=np.tile([.6,0.,.8],(16,1));b=np.tile([0.,0.,1.],(16,1))
        tile=np.tile([0.,0.,1.],(16,16,1))
        result=bake_patch(a,b,s,tile,65,401,[4,2],normal=True)
        # At 2.5% arclength the original join is still carried forward;
        # by 20% it has blended fully into the independent detail field.
        self.assertLess(np.linalg.norm(result[10,0]-a[0]),.02)
        np.testing.assert_allclose(result[80,0],tile[0,0],atol=1e-12)

    def test_fine_boundary_normals_are_not_extruded_into_long_streaks(self):
        s=np.linspace(0,1,128,endpoint=False)
        x=.25*np.cos(2*np.pi*48*s)
        a=np.c_[x,np.zeros(len(s)),np.sqrt(1-x*x)]
        b=np.tile([0.,0.,1.],(128,1));tile=np.tile([0.,0.,1.],(16,16,1))
        result=bake_patch(a,b,s,tile,513,401,[4,2],normal=True)
        self.assertLess(np.ptp(result[40,:,0]),.1)
        np.testing.assert_array_equal(result[0],periodic_samples(a,s,np.linspace(0,1,513)))

    def test_mounted_join_blends_broad_shading_without_extruding_pores(self):
        s=np.linspace(0,1,128,endpoint=False)
        x=.2*np.cos(2*np.pi*48*s)
        upper=np.c_[x,np.full(len(s),.5),np.sqrt(.75-x*x)]
        lower=np.tile([0.,0.,1.],(128,1));tile=np.tile([0.,0.,1.],(16,16,1))
        result=bake_patch(lower,upper,s,tile,513,401,[4,2],normal=True)
        np.testing.assert_array_equal(result[-1],periodic_samples(upper,s,np.linspace(0,1,513)))
        self.assertGreater(result[-11,:,1].mean(),.48)
        self.assertLess(np.ptp(result[-41,:,0]),.08)
        np.testing.assert_allclose(result[-81],np.tile([0.,0.,1.],(513,1)),atol=1e-12)


if __name__=='__main__': unittest.main()
