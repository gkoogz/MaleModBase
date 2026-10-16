import importlib.util
import unittest
import numpy as np
from scipy import sparse
from malemod_base.modular_anatomy import Mesh,clip,cap,directed_boundary,complete_shaft,radial_attachment,trumpet_neck,vertex_normals
from malemod_base.modular_union import visible_union,trumpet_union
from malemod_base.modular_pelvis import pelvic_union,ramp_orientation
from tests.test_modular_anatomy import tube
from malemod_base.modular_shading import corner_frames,unit


@unittest.skipUnless(importlib.util.find_spec('shapely'),'Install requirements-authoring.txt for exact cut tests')
class ModularUnionTest(unittest.TestCase):
    def test_pelvic_ramp_preserves_body_and_existing_outer_boundary(self):
        p=np.array([[0,-5,-5],[0,5,-5],[0,5,5],[0,-5,5]],float)
        body=Mesh.source(p,np.array([[0,1,2],[0,2,3]]),p[:,1:])
        root=np.array([-.2,0,0])
        shaft,rings,center,radial=complete_shaft(clip(tube(),[1,0,0],1.5),root)
        if np.einsum('ij,ij->i',shaft.points[shaft.faces[:,0]],np.cross(shaft.points[shaft.faces[:,1]],shaft.points[shaft.faces[:,2]])).sum()<0:shaft=shaft.reversed()
        body.donors=sparse.hstack([body.donors,sparse.csr_matrix((4,shaft.donors.shape[1]))],format='csr')
        shaft.donors=sparse.hstack([sparse.csr_matrix((len(shaft.points),4)),shaft.donors],format='csr')
        original=shaft.points.copy()
        result,contract,report=pelvic_union(body,shaft,root,center-root,radial)
        self.assertEqual(report['rampReversedFaces'],0)
        self.assertEqual(report['finalBoundaryLoops'],1)
        np.testing.assert_array_equal(shaft.points,original)
        loops=directed_boundary(result)
        np.testing.assert_array_equal(np.sort(result.points[loops[0]],axis=0),np.sort(p,axis=0))
        body_rows=np.flatnonzero(np.asarray(result.donors[:,:4].sum(1)).ravel()>1-1e-12)
        np.testing.assert_allclose(result.points[body_rows],result.donors[body_rows,:4]@p,atol=1e-12)
        for key,row in [('outer_normal',0),('inner_normal',-1)]:
            np.testing.assert_allclose(unit(result.normal_field[contract['rings'][row]]),contract[key],atol=1e-12)
        frames=corner_frames(result)
        np.testing.assert_allclose(frames['normal'],unit(result.normal_field[result.faces]),atol=1e-12)
        # Extra underside room changes neither the side width nor the top.
        _,plain,_=pelvic_union(body,shaft,root,center-root,radial,underside_margin=0)
        np.testing.assert_allclose(np.ptp(contract['footprint'][:,0]),np.ptp(plain['footprint'][:,0]),atol=1e-12)
        np.testing.assert_allclose(contract['footprint'][:,1].max(),plain['footprint'][:,1].max(),atol=1e-12)
        self.assertLess(contract['footprint'][:,1].min(),plain['footprint'][:,1].min())

    def test_weld_has_one_closed_surface_and_preserves_complete_shaft(self):
        shaft,rings,center,_=complete_shaft(clip(tube(),[1,0,0],1.5),[-2,0,0])
        if np.einsum('ij,ij->i',shaft.points[shaft.faces[:,0]],np.cross(shaft.points[shaft.faces[:,1]],shaft.points[shaft.faces[:,2]])).sum()<0:shaft=shaft.reversed()
        a=np.linspace(0,2*np.pi,32,endpoint=False)
        p=np.array([[r*np.cos(t),r*np.sin(t),z] for r,z in [(.6,-3.),(.7,-2.)] for t in a])
        f=[]
        for j in range(32):
            k=(j+1)%32;f.extend([[j,k,j+32],[k,k+32,j+32]])
        sack=Mesh.source(p,np.array(f),p[:,:2]);bottom=min(directed_boundary(sack),key=lambda loop:sack.points[loop,2].mean())
        sack=cap(sack,bottom)
        # Put the independent fixtures in one explicit provenance bank.
        original_columns=shaft.donors.shape[1]
        shaft.donors=sparse.hstack([shaft.donors,sparse.csr_matrix((len(shaft.points),len(p)))],format='csr')
        sack.donors=sparse.hstack([sparse.csr_matrix((len(sack.points),original_columns)),sack.donors],format='csr')
        loop=directed_boundary(sack)[0]
        binding=radial_attachment(sack,loop,shaft,[-2,0,0],center,flare=1.3,side_wrap_degrees=100)
        original=shaft.points.copy();original_uv=shaft.corner_uv.copy()
        union,_,_,_,report=trumpet_union(shaft,sack,binding,rings,np.array([-2.,0,0]),center,vertex_normals(sack))
        np.testing.assert_array_equal(shaft.points,original)
        np.testing.assert_array_equal(shaft.corner_uv,original_uv)
        self.assertEqual(union.validate(True)['boundaryEdges'],0)
        self.assertEqual(shaft.validate(True)['boundaryEdges'],0)
        self.assertEqual(report['refinementReversedFaces'],0)
        self.assertGreater(report['sharedSeamVertices'],len(loop))
        # Closed sphere topology, rather than two coincident closed objects.
        edge=np.unique(np.sort(np.concatenate([union.faces[:,[0,1]],union.faces[:,[1,2]],union.faces[:,[2,0]]]),axis=1),axis=0)
        self.assertEqual(len(union.points)-len(edge)+len(union.faces),2)

    def test_tilted_ramp_uses_fixed_body_frame_across_widths(self):
        p=np.array([[0,-5,-5],[0,5,-5],[0,5,5],[0,-5,5]],float)
        body=Mesh.source(p,np.array([[0,1,2],[0,2,3]]),p[:,1:])
        root=np.array([-.2,0,0])
        shaft,_,center,radial=complete_shaft(clip(tube(),[1,0,0],1.5),root)
        if np.einsum('ij,ij->i',shaft.points[shaft.faces[:,0]],np.cross(shaft.points[shaft.faces[:,1]],shaft.points[shaft.faces[:,2]])).sum()<0:shaft=shaft.reversed()
        body.donors=sparse.hstack([body.donors,sparse.csr_matrix((4,shaft.donors.shape[1]))],format='csr')
        shaft.donors=sparse.hstack([sparse.csr_matrix((len(shaft.points),4)),shaft.donors],format='csr')
        for angle,width in [(-25,1.),(25,.65),(25,1.),(25,1.35),(50,1.)]:
            with self.subTest(angle=angle,width=width):
                t=np.deg2rad(angle);rotation=np.array([[np.cos(t),0,-np.sin(t)],[0,1,0],[np.sin(t),0,np.cos(t)]])
                transform=rotation@np.diag([1,width,width])
                moved=Mesh((shaft.points-root)@transform.T+root,shaft.faces,shaft.corner_uv,shaft.donors,shaft.source_faces,shaft.generated)
                result,contract,report=pelvic_union(body,moved,root,rotation@np.array([1.,0,0]),radial@transform.T,mount_axis=[1.,0,0],mount_radial=radial*width)
                self.assertEqual(report['rampReversedFaces'],0)
                self.assertEqual(report['bodyDisplacement'],0)
                self.assertTrue(report['originalBoundaryEdgesPreserved'])
                broken=result.points.copy();middle=contract['rings'][len(contract['rings'])//2]
                broken[middle[3:6]]=broken[middle[3:6][::-1]]
                audit=ramp_orientation(broken,contract['rings'],contract['angles'])
                self.assertGreater(audit['reversedFaces'],0)


if __name__=='__main__':unittest.main()
