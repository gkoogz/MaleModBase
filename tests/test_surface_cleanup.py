import unittest
import numpy as np
from scipy import sparse
from malemod_base.modular_anatomy import Mesh
from malemod_base.surface_cleanup import collapse_cut_slivers


class CutCleanupTest(unittest.TestCase):
    def fixture(self):
        p=np.array([[1,0,0],[-1,0,0],[0,1,0],[0,-1,0],[0,0,1],[0,0,-1]],float)
        faces=np.array([[0,2,4],[2,1,4],[1,3,4],[3,0,4],
                        [2,0,5],[1,2,5],[3,1,5],[0,3,5]])
        original=Mesh.source(p,faces,p[:,:2])
        # One generated near-duplicate intersection, shared by both incident
        # triangles, with deliberately different UV aliases on the two sides.
        t=1e-8;q=p[0]*(1-t)+p[2]*t;p=np.vstack([p,q])
        f=[];uv=[];source=[]
        for fi,face in enumerate(faces):
            values=original.corner_uv[fi]+fi*3
            if 0 in face and 2 in face:
                for j in range(3):
                    a,b,c=face[j],face[(j+1)%3],face[(j+2)%3]
                    if {a,b}!={0,2}:continue
                    u=t if a==0 else 1-t
                    mid=values[j]*(1-u)+values[(j+1)%3]*u
                    f.extend([[a,6,c],[6,b,c]])
                    uv.extend([[values[j],mid,values[(j+2)%3]],
                               [mid,values[(j+1)%3],values[(j+2)%3]]])
                    source.extend([fi,fi]);break
            else:f.append(face);uv.append(values);source.append(fi)
        donor=sparse.vstack([sparse.eye(6),sparse.csr_matrix([[1-t,0,t,0,0,0]])],format='csr')
        return Mesh(p,np.array(f),np.array(uv),donor,np.array(source),np.array([False]*6+[True]))

    def test_removes_cut_sliver_but_keeps_samples_lineage_and_uv_aliases(self):
        mesh=self.fixture();clean,info=collapse_cut_slivers(mesh,relative_area=1e-8)
        self.assertEqual(len(clean.faces),8)
        self.assertEqual(clean.validate(True)['boundaryEdges'],0)
        np.testing.assert_array_equal(clean.points,mesh.points)
        np.testing.assert_array_equal(clean.donors.toarray(),mesh.donors.toarray())
        self.assertEqual(info['vertex_map'][6],0)
        for face,uv,parent in zip(clean.faces,clean.corner_uv,clean.source_faces):
            np.testing.assert_allclose(uv,clean.points[face,:2]+parent*3,atol=1e-12)

    def test_refuses_to_remove_protected_front_knot(self):
        with self.assertRaisesRegex(ValueError,'Cannot safely remove'):
            collapse_cut_slivers(self.fixture(),protected=[6],relative_area=1e-8)


if __name__=='__main__':unittest.main()
