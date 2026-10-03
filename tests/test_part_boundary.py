import unittest
import numpy as np
from malemod_base.part_boundary import weld_parts
from malemod_base.graft import topology_ids, boundary_loops


def cylinder(count, direction, phase=0):
    angle=np.arange(count)*2*np.pi/count+phase
    ring=np.c_[np.cos(angle),np.sin(angle),np.zeros(count)]
    points=np.vstack([ring,ring+[0,0,direction]])
    faces=[]
    for i in range(count):
        j=(i+1)%count
        faces.extend([[i,j,count+i],[j,count+j,count+i]])
    return points,np.array(faces)


class PartBoundaryTests(unittest.TestCase):
    def make(self):
        meshes=[cylinder(8,-1),cylinder(5,-1,.05),cylinder(11,1,.03),cylinder(7,1,.05)]
        loops=[]
        for p,f in meshes:
            k,a=topology_ids(p)
            loops.append(min(boundary_loops(a[f]),key=lambda l:abs(p[k[l],2].mean())))
        return meshes,weld_parts(meshes,loops,[[1,0,0],[0,1,0]],[0,0,0],.8,.3)

    def test_four_sides_and_lods_share_edges_positions_and_skin(self):
        meshes,weld=self.make()
        rng=np.random.default_rng(18)
        canonical=np.c_[.4+.1*weld.canonical[:,0],.6-.1*weld.canonical[:,0]]
        transforms=rng.normal(size=(2,3,3));translations=rng.normal(size=(2,3))
        posed=[]
        for (p,f),part in zip(meshes,weld.parts):
            np.testing.assert_array_equal(part.points[part.seam],weld.canonical)
            # Lineage sums and face ancestry survive refinement.
            np.testing.assert_allclose(np.asarray(part.lineage.sum(1)).ravel(),1)
            self.assertLess(part.face_lineage.max(),len(f))
            donors=part.edge_donors;a=donors[:,0].astype(int);b=donors[:,1].astype(int);u=donors[:,2,None]
            np.testing.assert_allclose((part.lineage@p)[part.seam],p[a]*(1-u)+p[b]*u)
            skin=part.bind_field(np.tile([.5,.5],(len(p),1)),canonical)
            q=(np.einsum('bij,nj->nbi',transforms,part.points)+translations)*skin[:,:,None]
            posed.append(q.sum(1)[part.seam])
            np.testing.assert_array_equal(part.points[part.protected],(part.lineage@p)[part.protected])
        for q in posed[1:]:np.testing.assert_array_equal(q,posed[0])
        expanded=weld.canonical*[1.5,1.2,1]+[0,0,.3]
        outputs=weld.publish(expanded,[p.points for p in weld.parts])
        for q,part in zip(outputs,weld.parts):np.testing.assert_array_equal(q[part.seam],expanded)

    def test_uv_aliases_remain_distinct(self):
        meshes,weld=self.make();part=weld.parts[0]
        uv=np.c_[np.arange(len(meshes[0][0])),np.ones(len(meshes[0][0]))]
        actual=part.field(uv)
        np.testing.assert_array_equal(actual[:len(uv)],uv)
        with self.assertRaises(ValueError):part.field(uv[:-1])

    def test_incomplete_publish_and_invalid_correspondence_rejected(self):
        meshes,weld=self.make()
        with self.assertRaises(ValueError):weld.publish(weld.canonical,[weld.parts[0].points])
        with self.assertRaises(ValueError):weld.publish(weld.canonical*np.nan,[p.points for p in weld.parts])
        with self.assertRaises(ValueError):weld_parts(meshes,[np.arange(3)]*4,[[1,0,0],[0,1,0]],[0,0,0],1,.3)


if __name__=='__main__':unittest.main()
