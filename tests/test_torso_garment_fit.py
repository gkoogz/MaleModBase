import unittest
import numpy as np
from malemod_base.torso_garment_fit import refit_radially,clear_projected_faces

class FitTest(unittest.TestCase):
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
