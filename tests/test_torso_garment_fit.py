import unittest
import numpy as np
from malemod_base.torso_garment_fit import refit_between_bodies,refit_radially,clear_projected_faces,refine_triangles,expand_projected_sections,smooth_tubular_chart
from malemod_base.torso_garment_fit import curved_boundary_midpoints,_bounded_smooth_field,wrap_body_surface

class FitTest(unittest.TestCase):
 def test_surface_wrap_uses_local_contour_and_finite_donors(self):
  body=np.array([[2,-2,0],[2,2,0],[3,0,2]],float)
  p=np.array([[1.8,-.5,.5],[1.8,.5,.5],[2.4,0,1.5],[1.8,-.5,.5]])
  q,bindings=wrap_body_surface(p,[[0,1,2],[3,1,2]],body,[[0,1,2]],.2,smoothing_passes=0)
  np.testing.assert_allclose(q[0],q[3],atol=1e-12)
  normal=np.cross(body[1]-body[0],body[2]-body[0]);normal/=np.linalg.norm(normal)
  for point,(face,weights,gap) in zip(q,bindings):
   self.assertTrue(np.all(np.asarray(weights)>=-1e-12));self.assertAlmostEqual(sum(weights),1.)
   contact=np.asarray(weights)@body
   np.testing.assert_allclose(point,contact+normal*.2,atol=1e-12)
  self.assertGreater(q[2,0]-q[0,0],.4)
  r,_=wrap_body_surface(p,[[0,1,2],[3,1,2]],body,[[0,1,2]],.2,smoothing_passes=10)
  np.testing.assert_allclose(r[0],r[3],atol=1e-12)
  self.assertTrue(np.isfinite(r).all())
 def test_surface_wrap_rejects_invalid_inputs(self):
  p=np.array([[1,0,0],[1,1,0],[1,0,1]],float);tri=[[0,1,2]]
  for clearance,steps in [(0,10),(.2,-1),(.2,1.5),(float('nan'),10)]:
   with self.assertRaises(ValueError):wrap_body_surface(p,tri,p,tri,clearance,steps)
  with self.assertRaises(ValueError):wrap_body_surface(p,tri,p,[[0,0,1]],.2)
  with self.assertRaises(ValueError):wrap_body_surface(p,[[0,1,5]],p,tri,.2)
 def test_smooth_field_bounds_peaks_and_certifies_all_clearances(self):
  a=np.array([[.1,.9,0,0],[0,.3,.7,0],[0,0,.05,.95],[.5,.5,0,0]])
  b=np.array([4.,3.,1.,4.])
  field=_bounded_smooth_field(a,b,(2,2),64.)
  self.assertGreaterEqual(field.min(),0.)
  self.assertLessEqual(field.max(),4.*1.05+1.1e-6)
  self.assertTrue(np.all(a@field>=b-1e-5))
  # Reordering source faces must not create a new raised pocket.
  np.testing.assert_allclose(field,_bounded_smooth_field(a[::-1],b[::-1],(2,2),64.),atol=1e-4)
 def test_bounded_fit_preserves_aliases_folds_side_join_and_support(self):
  p=np.array([[10,-1,0],[10,1,0],[10,0,2],[11,-1,0],[10,-1,0],[0,0,1]],float)
  args=dict(offset_width=2,lateral_spacing=2,field_regularization=64)
  q=expand_projected_sections(p,[[0,1,2]],[[14,0,1]],.2,**args)
  np.testing.assert_array_equal(q[:,1:],p[:,1:]);np.testing.assert_array_equal(q[0],q[4]);np.testing.assert_array_equal(q[5],p[5])
  self.assertAlmostEqual(q[3,0]-q[0,0],1.,places=3)
  self.assertGreaterEqual(np.dot([.25,.25,.5],q[:3,0]),14.2-1e-5)
  self.assertLess(np.max(q[:,0]-p[:,0]),4.42)
  for value in (0,-1,float('nan')):
   with self.assertRaises(ValueError):expand_projected_sections(p,[[0,1,2]],[],.2,offset_width=2,field_regularization=value)
  np.testing.assert_array_equal(_bounded_smooth_field([[1,0]],[-1],(2,),64),[0,0])
 def test_lateral_field_does_not_inflate_remote_cut_to_center_peak(self):
  p=np.array([[2,-1,0],[2,1,0],[2,0,2],[2,20,0],[2,22,0],[2,21,2]],float)
  t=[[0,1,2],[3,4,5]]
  q=expand_projected_sections(p,t,[[4,0,1]],.2,offset_width=2,lateral_spacing=2)
  self.assertGreaterEqual(np.dot([.25,.25,.5],q[:3,0]),4.2-1e-5)
  self.assertLess(q[5,0],2.05)
  np.testing.assert_array_equal(q[:,1:],p[:,1:])
 def test_cut_rounding_keeps_original_corners_and_limits_new_offsets(self):
  p=np.array([[1,0,0],[0,1,0],[-1,0,0],[0,-1,0],[0,0,0]],float)
  t=np.array([[0,1,4],[1,2,4],[2,3,4],[3,0,4]])
  q,faces,lineage=refine_triangles(p,t)
  rounded=curved_boundary_midpoints(p,t,q,lineage,np.ones(len(p),bool),.1)
  np.testing.assert_array_equal(rounded[:len(p)],p)
  self.assertLessEqual(np.linalg.norm(rounded-q,axis=1).max(),.1+1e-12)
  edge=np.where(np.all(lineage==[0,1],axis=1))[0][0]
  self.assertGreater(np.linalg.norm(rounded[edge]),np.linalg.norm(q[edge]))
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
