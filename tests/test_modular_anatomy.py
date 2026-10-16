import unittest
import numpy as np
from malemod_base.modular_anatomy import Mesh,clip,cap,directed_boundary,complete_shaft,radial_attachment,original_pelvis_ramp,trumpet_neck,vertex_normals,shaft_surface_normals,loft_orientation


def tube(angles=None):
    if angles is None:angles=np.linspace(0,2*np.pi,32,endpoint=False)
    count=len(angles)
    p=np.array([[x,np.cos(a),np.sin(a)] for x in [0.,1.,2.,3.] for a in angles])
    faces=[]
    for r in range(3):
        for j in range(count):
            a=r*count+j;b=r*count+(j+1)%count;c=a+count;d=b+count
            faces.extend([[a,b,c],[b,d,c]])
    m=Mesh.source(p,np.array(faces),p[:,[0,1]])
    for loop in directed_boundary(m):m=cap(m,loop)
    m.validate(True);return m


class ModularAnatomyTest(unittest.TestCase):
    def test_normals_do_not_depend_on_coplanar_triangle_subdivision(self):
        p=np.array([[0,0,0],[2,0,0],[0,3,0],[0,0,1]],float)
        f=np.array([[0,2,1],[0,1,3],[1,2,3],[2,0,3]])
        source=Mesh.source(p,f,p[:,:2])
        # Splitting one face changes its incident area at existing vertices;
        # it must not change the normals that control a new tangent patch.
        q=np.vstack([p,.03*p[0]+.94*p[2]+.03*p[1]])
        refined=Mesh.source(q,np.vstack([f[1:],[[0,2,4],[2,1,4],[1,0,4]]]),q[:,:2])
        np.testing.assert_allclose(vertex_normals(source),vertex_normals(refined)[:4],atol=1e-12)

    def test_near_coincident_section_knots_do_not_turn_attachment_directions(self):
        a=np.linspace(0,2*np.pi,32,endpoint=False)
        source=tube(np.sort(np.r_[a,a+1e-5]))
        root=np.array([-2.,0,0])
        shaft,rings,center,_=complete_shaft(clip(source,[1,0,0],1.5),root,refine=True)
        normals=shaft_surface_normals(shaft,rings,root,center)
        q=shaft.points[rings[-2]];n=normals[rings[-2]]
        derivative=np.linalg.norm(np.roll(n,-1,axis=0)-n,axis=1)/np.linalg.norm(np.roll(q,-1,axis=0)-q,axis=1)
        self.assertLess(derivative.max(),2.)
        np.testing.assert_allclose(n[:,0],0,atol=1e-10)

    def test_closed_shaft_without_scrotum_and_exact_retained_faces(self):
        source=tube();cut=clip(source,[1,0,0],1.5)
        shaft,rings,center,radial=complete_shaft(cut,[-2,0,0])
        self.assertEqual(shaft.validate(True)['boundaryEdges'],0)
        np.testing.assert_array_equal(shaft.points[:len(cut.points)],cut.points)
        np.testing.assert_array_equal(shaft.faces[:len(cut.faces)],cut.faces)
        np.testing.assert_array_equal(shaft.corner_uv[:len(cut.faces)],cut.corner_uv)
        np.testing.assert_allclose(cut.donors@source.points[:source.donors.shape[1]],cut.points,atol=1e-12)
        self.assertEqual(len(directed_boundary(shaft)),0)

    def test_rim_follows_shaft_affine_motion_without_feedback(self):
        shaft,*_=complete_shaft(clip(tube(),[1,0,0],1.5),[-2,0,0])
        rim=np.array([[-1,-.5,-2],[-1,.5,-2],[1,.5,-2],[1,-.5,-2]])
        source=Mesh.source(rim,np.array([[0,1,2],[0,2,3]]),rim[:,:2])
        original=shaft.points.copy();binding=radial_attachment(source,np.arange(4),shaft,[-2,0,0],[1.5,0,0])
        targets=binding.targets(shaft)
        np.testing.assert_array_equal(shaft.points,original)
        transform=np.array([[0,-2,0],[2,0,0],[0,0,.7]])
        shaft.points=shaft.points@transform.T+[2,3,4]
        np.testing.assert_allclose(binding.targets(shaft),targets@transform.T+[2,3,4],atol=1e-12)
        source.points*=5
        np.testing.assert_array_equal(shaft.points,original@transform.T+[2,3,4])

    def test_ramp_uses_virgin_pelvis_and_exact_triangle_donors(self):
        body=np.array([[0,-8,-8],[0,8,-8],[0,8,8],[0,-8,8]],float);faces=np.array([[0,1,2],[0,2,3]])
        old=body.copy();a=np.linspace(0,2*np.pi,32,endpoint=False)
        for radius in [.25,1.,3.]:
            radial=radius*np.c_[np.zeros(32),np.cos(a),np.sin(a)]
            ramp=original_pelvis_ramp(body,faces,[-.2,0,0],radial,[1,0,0])
            donor=np.einsum('ij,ijk->ik',ramp['outer_barycentric'],body[faces[ramp['outer_body_faces']]])
            np.testing.assert_allclose(ramp['points'][:32],donor,atol=1e-12)
            np.testing.assert_allclose(ramp['points'][-32:],np.array([-.2,0,0])+ramp['inner_station']*np.array([1,0,0])+radial,atol=1e-12)
            np.testing.assert_array_equal(body,old)

    def test_invalid_cuts_are_rejected(self):
        with self.assertRaises(ValueError):clip(tube(),[1,0,0],20)
        with self.assertRaises(ValueError):complete_shaft(tube(),[-2,0,0])

    def test_refined_shaft_preserves_distal_geometry_and_girth(self):
        distal=clip(tube(),[1,0,0],1.5)
        shaft,rings,_,radial=complete_shaft(distal,[-2,0,0],refine=True)
        self.assertEqual(shaft.validate(True)['boundaryEdges'],0)
        np.testing.assert_array_equal(shaft.points[:len(distal.points)],distal.points)
        np.testing.assert_allclose(np.ptp(radial[:,1:],axis=0),[2,2],atol=1e-12)
        np.testing.assert_allclose(radial[:,1:].min(0),[-1,-1],atol=1e-12)
        theta=np.sort(np.mod(np.arctan2(radial[:,2],radial[:,1]),2*np.pi))
        self.assertLess(np.diff(np.r_[theta,theta[0]+2*np.pi]).max(),np.deg2rad(3))
        self.assertGreater(len(rings[0]),len(directed_boundary(distal)[0]))

    def test_trumpet_preserves_sack_and_flattens_to_shaft(self):
        shaft,*_=complete_shaft(clip(tube(),[1,0,0],1.5),[-2,0,0])
        # A retained open sack with an upward boundary and outward winding.
        theta=np.linspace(0,2*np.pi,32,endpoint=False)
        p=np.array([[r*np.cos(a),r*np.sin(a),z] for r,z in [(.6,-3.),(.7,-2.)] for a in theta])
        f=[]
        for j in range(32):
            k=(j+1)%32;f.extend([[j,k,j+32],[k,k+32,j+32]])
        sack=Mesh.source(p,np.array(f),p[:,:2])
        bottom=min(directed_boundary(sack),key=lambda loop:sack.points[loop,2].mean())
        sack=cap(sack,bottom);loop=directed_boundary(sack)[0]
        if np.einsum('ij,ij->i',shaft.points[shaft.faces[:,0]],np.cross(shaft.points[shaft.faces[:,1]],shaft.points[shaft.faces[:,2]])).sum()<0:shaft=shaft.reversed()
        original=shaft.points.copy()
        narrow=radial_attachment(sack,loop,shaft,[-2,0,0],[1.5,0,0])
        binding=radial_attachment(sack,loop,shaft,[-2,0,0],[1.5,0,0],flare=1.3)
        self.assertGreater(np.ptp(binding.targets(shaft)[:,0]),np.ptp(narrow.targets(shaft)[:,0]))
        result,rim,contract=trumpet_neck(sack,binding,shaft)
        np.testing.assert_array_equal(result.points[:len(sack.points)],sack.points)
        np.testing.assert_array_equal(result.corner_uv[:len(sack.faces)],sack.corner_uv)
        np.testing.assert_array_equal(shaft.points,original)
        np.testing.assert_allclose(result.points[rim.vertices],rim.targets(shaft),atol=1e-12)
        np.testing.assert_allclose(np.sum(contract['end_derivative']*contract['shaft_normals'],axis=1),0,atol=1e-12)
        q=result.points[contract['rings']];last=q[-1]-q[-2]
        mismatch=abs(np.sum(last*contract['shaft_normals'],axis=1))/np.linalg.norm(last,axis=1)
        self.assertLess(mismatch.max(),np.sin(np.deg2rad(1)))
        self.assertEqual(len(directed_boundary(result)),1)
        self.assertEqual(int(contract['reversed_faces']),0)
        # Corrupt only an interior strip; the rim remains exactly welded.
        folded=result.points.copy();middle=contract['rings'][len(contract['rings'])//2]
        folded[middle]=folded[np.roll(middle,len(middle)//2)]
        audit=loft_orientation(folded,contract['rings'],contract['source_normals'],contract['shaft_normals'])
        self.assertGreater(audit['reversedFaces'],0)

    def test_side_wrap_reaches_both_sides_without_widening_shaft(self):
        shaft,*_=complete_shaft(clip(tube(),[1,0,0],1.5),[-2,0,0])
        a=np.linspace(0,2*np.pi,48,endpoint=False)
        p=np.c_[.8*np.cos(a),.6*np.sin(a),np.full(len(a),-2.)]
        source=Mesh.source(p,np.array([[0,j,j+1] for j in range(1,len(a)-1)]),p[:,:2])
        before=shaft.points.copy()
        for reach in [90,100,120]:
            binding=radial_attachment(source,np.arange(len(a)),shaft,[-2,0,0],[1.5,0,0],side_wrap_degrees=reach)
            end=binding.targets(shaft);theta=np.degrees(np.arctan2(end[:,1],-end[:,2]))
            self.assertAlmostEqual(theta.max(),reach,places=8)
            self.assertAlmostEqual(theta.min(),-reach,places=8)
            np.testing.assert_array_equal(shaft.points,before)


if __name__=='__main__':unittest.main()
