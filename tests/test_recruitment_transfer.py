import unittest
import numpy as np
from malemod_base.recruitment_transfer import SourceBodyField,virtual_bind_translation
from malemod_base.collar import CollarFrame,CollarPlan

class RecruitmentTransferTests(unittest.TestCase):
    def test_source_field_constant_and_exact_donor(self):
        source=np.array([[9+i,0,84] for i in range(8)],float)
        field=SourceBodyField(source,source[:2])
        delta=np.tile([.2,.3,.4],(8,1))
        np.testing.assert_allclose(field.displacement(source+delta),delta[:2],atol=1e-14)
        delta[0]=[1,2,3]
        np.testing.assert_allclose(field.displacement(source+delta)[0],delta[0])

    def test_virtual_pivot_native_skin_equivalence(self):
        from scipy.spatial.transform import Rotation
        rng=np.random.default_rng(700)
        for _ in range(100):
            bind,material,current,vertex=rng.normal(size=(4,3));r=Rotation.random(random_state=rng).as_matrix()
            delta=virtual_bind_translation(bind,material,current,r)
            np.testing.assert_allclose(r@(vertex-bind)+bind+delta,r@(vertex-material)+current,atol=1e-14)

    def test_displacement_zero_preserves_rest_and_slave(self):
        p=np.array([[9,-2,84],[9,2,84],[12,-2,84],[12,2,84],[9,0,84]],float)
        faces=np.array([[0,2,4],[4,2,3],[4,3,1]])
        plan=CollarPlan(p,faces,[(4,0,1,.5)],CollarFrame((9,0,84),(1,0,0),(0,0,1),3,24,1),(2,3))
        np.testing.assert_array_equal(plan.solve_displacement(np.zeros_like(p)),np.zeros_like(p))
        delta=np.zeros_like(p);delta[:2,2]=1
        out=plan.solve_displacement(delta)
        np.testing.assert_allclose(out[4],(out[0]+out[1])*.5,atol=1e-14)
        np.testing.assert_array_equal(out[[2,3]],delta[[2,3]])
