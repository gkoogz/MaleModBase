import unittest
import numpy as np
from malemod_base.garment_coverage import project_covered_faces


class CoverageTests(unittest.TestCase):
    def test_retains_body_and_canonical_boundary(self):
        p=np.array([[0,0,0],[1,0,0],[1,1,0],[0,1,0]])
        faces=np.array([[0,1,2],[0,2,3]])
        donors=np.repeat(np.arange(5)[:,None],3,axis=1)
        weights=np.tile([1.,0,0],(5,1))
        target=np.array([[0,1,2],[0,2,3],[0,1,4]])
        result=project_covered_faces(p,faces,faces[1:],donors,weights,target)
        np.testing.assert_array_equal(result,[True,False,False])

    def test_coincident_uv_alias_and_subdivided_face(self):
        p=np.array([[0,0,0],[1,0,0],[0,1,0],[1,0,0]])
        donors=np.array([[0,0,0],[3,3,3],[2,2,2],[0,1,0]])
        weights=np.array([[1.,0,0],[1,0,0],[1,0,0],[.5,.5,0]])
        result=project_covered_faces(p,[[0,1,2]],np.empty((0,3),int),donors,weights,[[0,1,2],[0,3,2]])
        np.testing.assert_array_equal(result,[True,True])

    def test_rejects_invalid_weights(self):
        with self.assertRaises(ValueError):
            project_covered_faces([[0,0,0]],[[0,0,0]],[[0,0,0]],[[0,0,0]],[[.2,.2,.2]],[[0,0,0]])


if __name__=='__main__':unittest.main()
