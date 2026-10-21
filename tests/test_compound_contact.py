import unittest
import numpy as np
from scipy import sparse
from malemod_base.modular_anatomy import Mesh,directed_boundary,vertex_normals,cap
from malemod_base.modular_union import charted_trumpet_union
from malemod_base.surface_footprint import ContactArea


class CompoundContactTest(unittest.TestCase):
    def test_recruit_cut_and_weld_preserves_source_boundaries_and_exact_donors(self):
        # A chart spans two differently sloped support regions. Its physical
        # area is deliberately not its flattened area.
        xy=np.array([[-4,-4],[0,-4],[4,-4],[-4,4],[0,4],[4,4]],float)
        p=np.c_[xy,np.maximum(-xy[:,0],0)*.4]
        f=np.array([[0,1,3],[1,4,3],[1,2,4],[2,5,4]])
        f=f[:,[0,2,1]]  # underside: the retained sac lies outside this surface
        target=Mesh.source(p,f,xy);target.normal_field=vertex_normals(target)
        theta=np.arange(32)*2*np.pi/32
        q=np.array([[r*np.cos(a)+.6,r*np.sin(a),z] for r,z in [(.35,-2),(.5,-1.5)] for a in theta])
        faces=[]
        for j in range(32):
            k=(j+1)%32;faces.extend([[j,k,j+32],[k,k+32,j+32]])
        source=Mesh.source(q,np.array(faces),q[:,:2])
        source=cap(source,min(directed_boundary(source),key=lambda loop:source.points[loop,2].mean()))
        source.normal_field=vertex_normals(source);loop=directed_boundary(source)[0]
        bank=target.donors.shape[1];source_bank=source.donors.shape[1]
        target.donors=sparse.hstack([target.donors,sparse.csr_matrix((len(p),source_bank))],format='csr')
        source.donors=sparse.hstack([sparse.csr_matrix((len(source.points),bank)),source.donors],format='csr')
        old_target=target.points.copy();old_source=source.points.copy()
        baseline=source.points[loop,:2].copy();baseline[:,0]=(baseline[:,0]-.6)*1.5+.6
        area=ContactArea(p,f,xy,np.ones(len(f),bool));footprint,measurement=area.expand_rear(baseline,2,2)
        result,neck,rim,info,report=charted_trumpet_union(target,source,loop,xy,footprint,np.ones(len(f),bool))
        self.assertAlmostEqual(measurement['areaRatio'],2,delta=2e-5)
        np.testing.assert_array_equal(source.points,old_source)
        np.testing.assert_array_equal(target.points,old_target)
        self.assertEqual(report['originalBoundaryLoops'],report['finalBoundaryLoops'])
        self.assertEqual(report['neckReversedFaces'],0)
        np.testing.assert_allclose(np.einsum('ij,ijk->ik',info['source_barycentric'],source.points[info['source_triangles']]),neck.points[info['rings'][0]],atol=1e-10)
        np.testing.assert_allclose(np.einsum('ij,ijk->ik',info['support_barycentric'],target.points[info['support_triangles']]),neck.points[info['rings'][-1]],atol=1e-10)
        _,moved,_,contract,bounded=charted_trumpet_union(target,source,loop,xy,footprint,np.ones(len(f),bool),source_shift=(np.array([1.,0,0]),3.))
        self.assertGreater(bounded['appliedSourceShift'],0)
        self.assertLessEqual(bounded['appliedSourceShift'],3)
        self.assertGreaterEqual(bounded['neckMinimumAdjacentCosine'],.5)
        start=np.einsum('ij,ijk->ik',contract['source_barycentric'],source.points[contract['source_triangles']])+contract['source_translation']
        np.testing.assert_allclose(start,moved.points[contract['rings'][0]],atol=1e-10)
        np.testing.assert_array_equal(source.points,old_source)


if __name__=='__main__':unittest.main()
