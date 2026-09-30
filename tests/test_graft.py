import unittest
import numpy as np
from malemod_base.graft import (fit_graft, topology_ids, boundary_loops,
                               extend_seam_field, preserve_orientation, limit_influences)


def fixture():
    # Planar square with an inner square aperture; cone with six boundary edges.
    body=np.array([[-2,-2,0],[2,-2,0],[2,2,0],[-2,2,0],[-1,-1,0],[1,-1,0],[1,1,0],[-1,1,0]],float)
    faces=np.array([[0,1,5],[0,5,4],[1,2,6],[1,6,5],[2,3,7],[2,7,6],[3,0,4],[3,4,7],[4,5,6],[4,6,7]])
    angle=np.arange(6)*2*np.pi/6+.1
    module=np.vstack([np.c_[.8*np.cos(angle),.8*np.sin(angle),np.full(6,.2)],[0,0,2]])
    mf=np.array([[i,(i+1)%6,6] for i in range(6)])
    remove=np.arange(10)>=8
    return body,faces,module,mf,remove


class GraftTests(unittest.TestCase):
    def fit(self, values=None):
        return fit_graft(*(values or fixture()),np.array([[1,0,0],[0,1,0]]),np.zeros(3),1.)

    def test_weld_lineage_and_original_body_preservation(self):
        b,f,m,mf,remove=fixture();g=self.fit()
        np.testing.assert_array_equal(g.points[:len(b)],b)
        np.testing.assert_array_equal(g.points[g.body_seam],g.points[g.module_seam])
        np.testing.assert_allclose(np.asarray(g.body_lineage.sum(1)).ravel(),1)
        np.testing.assert_allclose(np.asarray(g.module_lineage.sum(1)).ravel(),1)
        donors=g.body_edge_donors;a=donors[:,0].astype(int);c=donors[:,1].astype(int);u=donors[:,2,None]
        np.testing.assert_allclose(g.points[g.body_seam],b[a]*(1-u)+b[c]*u)
        _,ids=topology_ids(g.points);loops=boundary_loops(ids[g.faces]);self.assertEqual(len(loops),1);self.assertEqual(len(loops[0]),4)
        area=np.linalg.norm(np.cross(g.points[g.faces[:,1]]-g.points[g.faces[:,0]],g.points[g.faces[:,2]]-g.points[g.faces[:,0]]),axis=1)
        self.assertGreater(area.min(),1e-5)

    def test_skin_seam_under_different_bone_transforms(self):
        g=self.fit();n=g.body_count
        bw=np.zeros((n,2));bw[:,0]=np.clip((g.points[:n,0]+2)/4,0,1);bw[:,1]=1-bw[:,0]
        mw=extend_seam_field(g.points[n:],g.faces[len(g.body_face_lineage):]-n,g.module_seam-n,bw[g.body_seam],np.array([1.,0.]),1.)
        w=np.vstack([bw,mw]);np.testing.assert_allclose(w.sum(1),1,atol=1e-12)
        self.assertGreaterEqual(w.min(),-1e-12)
        # Independent affine poses: exact seam weights keep the weld closed.
        t0=np.array([[1,0,0],[0,0,-1],[0,1,0]]);t1=np.diag([.7,1.2,.9])
        posed=(g.points@t0.T+[2,3,1])*w[:,0,None]+(g.points@t1.T+[-1,0,4])*w[:,1,None]
        np.testing.assert_allclose(posed[g.body_seam],posed[g.module_seam],atol=1e-12)

    def test_uv_alias_identity_retained(self):
        b,f,m,mf,remove=fixture();m=np.vstack([m,m[0]]);mf=mf.copy();mf[-1,1]=len(m)-1
        g=self.fit((b,f,m,mf,remove))
        np.testing.assert_array_equal(g.points[g.body_count],g.points[g.body_count+len(m)-1])
        self.assertEqual(g.module_lineage.shape[1],len(m))

    def test_rejects_opening_into_existing_boundary_and_nonmanifold(self):
        data=list(fixture());data[4]=np.array([True]*9+[False])
        with self.assertRaises(ValueError):self.fit(tuple(data))

    def test_orientation_repair_preserves_locked_seam_and_rejects_impossible_budget(self):
        p=np.array([[0,0,0],[1,0,0],[0,1,0],[.2,.2,0]],float)
        f=np.array([[0,1,3],[1,2,3],[2,0,3]])
        q=p.copy();q[3]=[-.1,.2,0]
        repaired,report=preserve_orientation(p,q,f,np.array([0,1,2]),.5)
        np.testing.assert_array_equal(repaired[:3],q[:3])
        self.assertGreater(report['minimumAreaRatio'],.01)
        self.assertLess(report['maximumMove'],.5)
        with self.assertRaises(ValueError):preserve_orientation(p,q,f,np.arange(4),.5)
        with self.assertRaises(ValueError):preserve_orientation(p,q,f,np.array([0,1,2]),.001)

    def test_influence_limit_preserves_quantized_sums_and_tie_order(self):
        w=np.array([[.2,.2,.2,.2,.1],[.6,.3,0,0,0]])
        result,discarded=limit_influences(w,4,.11)
        np.testing.assert_allclose(result.sum(1),w.sum(1))
        np.testing.assert_array_equal(result[1],w[1])
        np.testing.assert_allclose(discarded,[.1,0])
        with self.assertRaises(ValueError):limit_influences(w,4,.05)
        data=list(fixture());data[3]=np.vstack([data[3],data[3][0]])
        with self.assertRaises(ValueError):self.fit(tuple(data))

if __name__=='__main__':unittest.main()
