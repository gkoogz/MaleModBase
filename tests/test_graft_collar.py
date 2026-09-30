import unittest
import numpy as np
from test_graft import fixture
from malemod_base.graft import fit_graft
from malemod_base.graft_collar import GraftCollar
from malemod_base.collar import CollarFrame
from malemod_base.surface_limit import correction_limit


class GraftCollarTests(unittest.TestCase):
    def domain(self):
        g=fit_graft(*fixture(),np.array([[1,0,0],[0,1,0]]),np.zeros(3),1.)
        p=g.points[:,[2,0,1]]+[9,0,84.3]
        domain=GraftCollar(p,g.faces,g.body_seam,g.module_seam,g.body_edge_donors,
                           np.arange(4),1e-7)
        return g,domain

    def test_original_edge_weld_and_protected_part_boundary_survive_expansion(self):
        g,d=self.domain();f=CollarFrame((9,0,84.3),(1,0,0),(0,0,1),6.,4.,1.)
        target=d.points.copy();target[:,1]*=1.7;target[:,2]+=1
        out=d.solve(target,f)
        np.testing.assert_array_equal(out[:4],d.points[:4])
        np.testing.assert_array_equal(out[g.body_seam],out[g.module_seam])
        donors=g.body_edge_donors;a=donors[:,0].astype(int);b=donors[:,1].astype(int);w=donors[:,2,None]
        np.testing.assert_allclose(out[g.body_seam],out[a]*(1-w)+out[b]*w,atol=1e-12)
        self.assertGreater(np.max(np.linalg.norm(out-d.points,axis=1)),.01)

    def test_factorization_is_reused_and_actual_guides_are_required(self):
        _,d=self.domain();f=CollarFrame((9,0,84.3),(1,0,0),(0,0,1),6.,4.,1.)
        p=d.prepare(f);self.assertIs(p,d.prepare(f))
        self.assertIsNot(p,d.prepare(CollarFrame((9,0,84.3),(1,0,0),(0,0,1),7.,4.,1.)))
        with self.assertRaises(ValueError):d.solve_guided(f,[[0,0,0]],[0])

    def test_rejects_broken_original_edge_correspondence(self):
        g,d=self.domain();donors=g.body_edge_donors.copy();donors[0,2]=.5
        with self.assertRaises(ValueError):
            GraftCollar(d.points,g.faces,g.body_seam,g.module_seam,donors,np.arange(4),1e-7)

    def test_continuous_area_bound_and_linear_seam_constraints(self):
        p=np.array([[0,0,0],[1,0,0],[0,1,0]],float);d=np.zeros_like(p);d[2,1]=-3
        fraction=correction_limit(p,d,np.array([[0,1,2]]),.025,0.)
        self.assertAlmostEqual(fraction,.975/3)
        for t in np.linspace(0,fraction,100):
            q=p+t*d;self.assertGreaterEqual(np.cross(q[1]-q[0],q[2]-q[0])[2],.025-1e-12)
        g,domain=self.domain();f=CollarFrame((9,0,84.3),(1,0,0),(0,0,1),6.,4.,1.)
        target=domain.points.copy();target[:,1]*=-4
        out,alpha=domain.solve_checked(target,f,.025)
        self.assertGreaterEqual(alpha,0);self.assertLessEqual(alpha,1)
        np.testing.assert_array_equal(out[g.body_seam],out[g.module_seam])
        np.testing.assert_array_equal(out[:4],domain.points[:4])
