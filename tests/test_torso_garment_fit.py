import unittest
import numpy as np
from malemod_base.torso_garment_fit import refit_between_bodies,refit_radially,clear_projected_faces,refine_triangles,expand_projected_sections,smooth_tubular_chart

class FitTest(unittest.TestCase):
 def test_section_offset_retains_fold_depth_and_side_join(self):
  p=np.array([[10,-1,0],[10,1,0],[10,0,2],[11,-1,0],[0,0,1]],float)
  q=expand_projected_sections(p,[[0,1,2]],[[14,0,1]],.2,spacing=4,offset_width=2)
  np.testing.assert_array_equal(q[:,1:],p[:,1:])
  np.testing.assert_array_equal(q[4],p[4])
  self.assertAlmostEqual(q[3,0]-q[0,0],1,places=3)
  self.assertGreaterEqual(np.dot([.25,.25,.5],q[:3,0]),14.2-1e-5)
  with self.assertRaises(ValueError):expand_projected_sections(p,[[0,1,2]],[],.2,offset_width=0)
 def test_outward_envelope_is_independent_of_winding(self):
  p=np.array([[10,-1,0],[10,1,0],[10,0,2]],float)
  args=dict(supports=[[14,0,1]],clearance=.2,offset_width=2,outward_cosine=.75)
  q=expand_projected_sections(p,[[0,1,2]],**args)
  reversed_q=expand_projected_sections(p,[[2,1,0]],**args)
  np.testing.assert_allclose(q,reversed_q)
  with self.assertRaises(ValueError):expand_projected_sections(p,[[0,1,2]],**dict(args,outward_cosine=2))
 def test_smoothed_expansion_keeps_reference_folds_and_cut_height(self):
  b=np.array([[2,-2,0],[2,2,0],[2,2,3],[2,-2,3]],float);t=np.array([[0,1,2],[0,2,3]])
  p=np.array([[2.4,-.5,1],[2.8,0,2],[2.4,.5,1],[2.4,-.5,1]],float)
  q,_=refit_between_bodies(p,b,t,b,t,.2,garment_triangles=[[0,1,2]],smoothing_passes=8)
  np.testing.assert_allclose(q,p,atol=1e-12)
  with self.assertRaises(ValueError):refit_between_bodies(p,b,t,b,t,.2,smoothing_passes=8)
 def test_reference_identity_retains_folds_cuts_and_aliases(self):
  b=np.array([[2,-2,0],[2,2,0],[2,2,3],[2,-2,3]],float);t=np.array([[0,1,2],[0,2,3]])
  p=np.array([[2.4,0,1],[2.8,0,1],[2.4,0,1]],float)
  q,_=refit_between_bodies(p,b,t,b,t,.2)
  np.testing.assert_allclose(q,p,atol=1e-12)
 def test_body_displacement_keeps_fold_depth_and_target_bindings(self):
  b=np.array([[2,-2,0],[2,2,0],[2,2,3],[2,-2,3]],float);t=np.array([[0,1,2],[0,2,3]])
  target=b.copy();target[:,0]=3
  p=np.array([[2.4,0,1],[2.8,0,1],[2.4,0,1]],float)
  q,bindings=refit_between_bodies(p,b,t,target,t,.2)
  np.testing.assert_allclose(q,p+[1,0,0],atol=1e-12)
  for point,(face,w,gap) in zip(q,bindings):
   self.assertAlmostEqual(np.linalg.norm(point-np.asarray(w)@target[t[face]]),gap)
 def test_body_displacement_enforces_clearance_without_erasing_deeper_folds(self):
  b=np.array([[2,-2,0],[2,2,0],[2,2,3],[2,-2,3]],float);t=np.array([[0,1,2],[0,2,3]])
  target=b.copy();target[:,0]=3
  q,_=refit_between_bodies([[2.1,0,1],[2.8,0,1]],b,t,target,t,.2)
  np.testing.assert_allclose(q[:,0],[3.2,3.8])
 def test_chart_relaxation_keeps_cuts_and_aliases(self):
  angle=np.array([-.4,0,.4]*3,float);height=np.repeat([0.,1.,2.],3);height[4]=1.8
  p=np.column_stack((2*np.cos(angle),2*np.sin(angle),height))
  faces=[]
  for row in range(2):
   for col in range(2):
    a=row*3+col;faces.extend(([a,a+1,a+3],[a+1,a+4,a+3]))
  q=smooth_tubular_chart(p,faces)
  np.testing.assert_allclose(q[[0,1,2,3,5,6,7,8]],p[[0,1,2,3,5,6,7,8]])
  self.assertAlmostEqual(q[4,2],1.,places=6)
 def test_section_expansion_preserves_fold_depth_and_aliases(self):
  p=np.array([[2,-1,0],[2,1,0],[2,0,2],[2,-1,0],[2.1,-1,0]],float)
  q=expand_projected_sections(p,[[0,1,2]],[[3,0,1]],.2)
  np.testing.assert_array_equal(q[0],q[3]);np.testing.assert_array_equal(q[:,1:],p[:,1:])
  self.assertAlmostEqual(q[4,0]/q[0,0],2.1/2)
  self.assertGreaterEqual(np.dot([.25,.25,.5],q[:3,0]),3.2-1e-5)
 def test_clearance_moves_uv_aliases_together(self):
  p=np.array([[2,-1,0],[2,1,0],[2,0,2],[2,-1,0]],float)
  q=clear_projected_faces(p,[[0,1,2]],[[3,0,1]],.2,aliases=[0,1,2,0])
  np.testing.assert_array_equal(q[0],q[3])
 def test_refinement_preserves_aliases_donors_boundary_and_winding(self):
  p=np.array([[2,0,0],[2,2,0],[2,0,2],[2,0,0]],float)
  q,t,d=refine_triangles(p,[[0,1,2],[3,2,1]])
  np.testing.assert_array_equal(q[:4],p)
  np.testing.assert_array_equal(q,(p[d[:,0]]+p[d[:,1]])*.5)
  self.assertEqual(len(t),8)
  self.assertGreater(len(q),6) # positional alias 3 is deliberately retained
  self.assertTrue(np.all(np.cross(q[t[:4,1]]-q[t[:4,0]],q[t[:4,2]]-q[t[:4,0]])[:,0]>0))
 def test_face_bridge_over_convex_chest(self):
  source=np.array([[2,-1,0],[2,1,0],[2,0,2]],float)
  out=clear_projected_faces(source,[[0,1,2]],[[3,0,1]],.2)
  self.assertGreaterEqual(np.dot([.25,.25,.5],out[:,0]),3.2-1e-6)
  np.testing.assert_array_equal(out[:,1:],source[:,1:])
 def test_enlarged_front_ease_aliases_and_binding(self):
  b=np.array([[2,-2,0],[2,2,0],[2,2,3],[2,-2,3]],float)
  t=[[0,1,2],[0,2,3]]
  source=[[1,0,1],[1,0,1],[3,0,1]]
  out,bindings=refit_radially(source,b,t,.2)
  np.testing.assert_allclose(out,[[2.2,0,1],[2.2,0,1],[3,0,1]])
  for p,(face,w,gap) in zip(out,bindings):
   q=np.array(w)@b[np.array(t[face])]
   self.assertAlmostEqual(np.linalg.norm(p-q),gap)
 def test_missing_surface_and_bad_values_reject(self):
  with self.assertRaises(ValueError):refit_radially([[1,0,4]],[[2,-2,0],[2,2,0],[2,2,3]],[[0,1,2]],.2)
  with self.assertRaises(ValueError):refit_radially([[float('nan'),0,1]],[[2,-2,0],[2,2,0],[2,2,3]],[[0,1,2]],.2)
 def test_bounded_open_patch(self):
  b=[[2,.1,0],[2,2,0],[2,2,3]]
  out,bindings=refit_radially([[1,0,1]],b,[[0,1,2]],.2,fallback_distance=2)
  self.assertTrue(np.isfinite(out).all());self.assertAlmostEqual(sum(bindings[0][1]),1.)
  with self.assertRaises(ValueError):refit_radially([[1,0,1]],b,[[0,1,2]],.2,fallback_distance=.1)
if __name__=='__main__':unittest.main()
