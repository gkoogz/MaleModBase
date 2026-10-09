import unittest
import numpy as np
from scipy.spatial import ConvexHull
from malemod_base.collision_chain import enclosing_tip
from malemod_base.taut_guides import (union_taut_paths, lift_embedded_anchors,
    parallel_taut_paths, meridian_taut_paths, ordered_meridian_anchors,
    adaptive_meridian_paths, bend_density)


class TautGuideTests(unittest.TestCase):
    def test_bulged_tip_is_inside_polygon_margin(self):
        surface=np.array([[0.,0.,0.],[1.,1.6,0.],[2.,.4,0.],[2.1,0.,0.]])
        tip,radius,receipt=enclosing_tip(surface,[0,0,0],2.,[2,0,0],[1,0,0])
        self.assertGreater(tip[0],2.1)
        self.assertGreater(radius,.4)
        fraction=surface[:,0]/tip[0]
        self.assertTrue((np.abs(surface[:,1]) <= (2*(1-fraction)+radius*fraction)*np.cos(np.pi/64)).all())
        self.assertLessEqual(receipt['maxRadialExcess'],0.)

    def test_taut_string_round_box_has_expected_length_and_convex_turns(self):
        p=np.array([[x,y,z] for x in [-1.,1.] for y in [-1.,1.] for z in [-1.,1.]])
        mesh=(p,ConvexHull(p).simplices)
        paths,receipts=union_taut_paths([[-3.,0.,0.]],np.array([3.,0.,0.]),np.array([0.,0.,1.]),[mesh])
        self.assertAlmostEqual(receipts[0]['length'],2*np.sqrt(5)+2)
        path=paths[0];edge=np.diff(path,axis=0)
        turns=np.cross(edge[:-1],edge[1:])[:,2]
        self.assertTrue((turns>=-1e-8).all() or (turns<=1e-8).all())
        for a,b in zip(path[:-1],path[1:]):
            samples=a+np.linspace(.001,.999,101)[:,None]*(b-a)
            self.assertFalse(((np.abs(samples)<1-1e-8).all(1)).any())

    def test_embedded_anchor_lifts_to_outer_surface(self):
        p=np.array([[x,y,z] for x in [-1.,1.] for y in [-1.,1.] for z in [-1.,1.]])
        mesh=(p,ConvexHull(p).simplices)
        lifted=lift_embedded_anchors([[0.,0.,0.],[2.,0.,0.]],[mesh],[1.,0.,0.])
        self.assertGreater(lifted[0,0],1.)
        np.testing.assert_array_equal(lifted[1],[2.,0.,0.])

    def test_parallel_lanes_keep_lateral_coordinate_and_separate_ends(self):
        p=np.array([[x,y,z] for x in [-1.,1.] for y in [-1.,1.] for z in [-1.,1.]])
        mesh=(p,ConvexHull(p).simplices)
        starts=np.array([[-3.,y,0.] for y in [-.75,0.,.75]])
        paths,_,targets,proof=parallel_taut_paths(starts,[3.,0.,0.],[0.,1.,0.],[mesh])
        for start,path,target in zip(starts,paths,targets):
            np.testing.assert_allclose(path[:,1],start[1],atol=1e-12)
            np.testing.assert_array_equal(path[-1],target)
        self.assertLess(proof['laneDrift'],1e-12)
        self.assertAlmostEqual(proof['minimumLaneSeparation'],.75)

    def test_meridians_wrap_in_distinct_outward_halfplanes_and_share_pole(self):
        p=np.array([[x,y,z] for x in [-1.,1.] for y in [-1.,1.] for z in [-1.,1.]])
        mesh=(p,ConvexHull(p).simplices)
        angles=np.arange(12)*2*np.pi/12
        starts=np.column_stack([3*np.cos(angles),3*np.sin(angles),np.full(12,3.)])
        pole=np.array([0.,0.,-2.])
        paths,_,proof=meridian_taut_paths(starts,pole,[0.,0.,-1.],[mesh])
        self.assertGreater(proof['minimumLongitudeSeparation'],.5)
        for start,path in zip(starts,paths):
            direction=start.copy();direction[2]=0;direction/=np.linalg.norm(direction)
            np.testing.assert_array_equal(path[0],start)
            np.testing.assert_array_equal(path[-1],pole)
            self.assertTrue(((path[:-1]-pole)@direction>0).all())
            np.testing.assert_allclose((path-pole)@np.cross([0,0,-1],direction),0,atol=1e-10)

    def test_reversed_neighbor_angles_are_repaired_without_reindexing(self):
        angles=np.array([0.,.8,1.6,1.5,2.4,3.2,4.,4.8,5.6])
        starts=np.column_stack([3*np.cos(angles),3*np.sin(angles),np.ones(len(angles))])
        adjusted,proof=ordered_meridian_anchors(starts,[0,0,0],[0,0,1],[])
        theta=np.unwrap(np.arctan2(adjusted[:,1],adjusted[:,0]))
        self.assertTrue((np.diff(theta)>0).all())
        np.testing.assert_allclose(adjusted[:,2],starts[:,2])
        self.assertLess(proof['maxAnchorAdjustment'],.2)

    def test_density_recomputes_when_wrapping_geometry_changes(self):
        angle=np.linspace(0,2*np.pi,65)
        outline=np.column_stack([4*np.cos(angle),2*np.sin(angle),np.full(len(angle),3.)])
        pole=np.array([0.,0.,-2.])
        free=adaptive_meridian_paths(outline,pole,[0.,0.,-1.],[],count=12,candidates=36)
        np.testing.assert_allclose(free[-1]['originFractions'],np.arange(12)/12,atol=1e-7)
        self.assertAlmostEqual(bend_density(np.array([[0.,0.,0.],[1.,0.,0.],[2.,0.,0.]])),0.)
        p=np.array([[x,y,z] for x in [-1.,1.] for y in [-1.,1.] for z in [-1.,1.]])
        wrapped=adaptive_meridian_paths(outline,pole,[0.,0.,-1.],[(p,ConvexHull(p).simplices)],count=12,candidates=36)
        self.assertGreater(wrapped[-1]['densityRatio'],1.05)
        self.assertGreater(np.max(np.abs(np.array(wrapped[-1]['originFractions'])-np.arange(12)/12)),.001)
        self.assertEqual(len(wrapped[0]),12)
        for path in wrapped[0]:np.testing.assert_array_equal(path[-1],pole)

    def test_nearly_coincident_proxy_vertices_do_not_reset_turn_constraints(self):
        p=np.array([[x,y,z] for x in [-1.,1.] for y in [-1.,1.] for z in [-1.,1.]])
        faces=ConvexHull(p).simplices
        paths,receipts=union_taut_paths([[-3.,0.,0.]],np.array([3.,0.,0.]),np.array([0.,0.,1.]),[(p,faces),(p+1e-12,faces)])
        self.assertAlmostEqual(receipts[0]['length'],2*np.sqrt(5)+2)
        edges=np.diff(paths[0],axis=0)
        self.assertTrue((np.linalg.norm(edges,axis=1)>1e-8).all())
        turns=np.cross(edges[:-1],edges[1:])[:,2]
        self.assertTrue((turns>=-1e-8).all() or (turns<=1e-8).all())


if __name__=='__main__':unittest.main()
