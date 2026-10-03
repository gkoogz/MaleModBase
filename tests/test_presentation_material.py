import unittest
import numpy as np
from PIL import Image
from malemod_base.material_atlas import atlas_uv,compose_tiles,bake_repeat
from malemod_base.presentation_binding import presentation_bindings


class TestPresentationMaterial(unittest.TestCase):
    def test_atlas_preserves_tiles_and_islands(self):
        p=np.arange(4*4*4,dtype=np.uint8).reshape(4,4,4);a=Image.fromarray(p);b=Image.fromarray(255-p)
        output=np.asarray(compose_tiles([a,b]));np.testing.assert_array_equal(output[:,:4],p);np.testing.assert_array_equal(output[:,4:],255-p)
        uv=np.array([[.25,.75],[1,0]])
        np.testing.assert_array_equal(atlas_uv(uv,1),[[.625,.75],[1,0]])
        np.testing.assert_array_equal(np.asarray(bake_repeat(a,1)),p)
        expected=np.array([[[int(round(p[i:i+2,j:j+2,c].mean())) for c in range(4)] for j in [0,2,0,2]] for i in [0,2,0,2]],dtype=np.uint8)
        np.testing.assert_array_equal(np.asarray(bake_repeat(a,2)),expected)
        with self.assertRaises(ValueError):atlas_uv([[1.1,0]],0)

    def test_material_binding_rigid_cap_and_lobes(self):
        f=np.array([[1,0,.83],[1,0,1],[0,1,.2],[0,1,.2],[1,0,.2]])
        p=np.array([[0,0,0],[1,0,0],[0,-2,0],[0,2,0],[0,0,0]])
        frame,amount=presentation_bindings(f,p,[[0,-2,0],[0,2,0]],np.array([1,1,1,1,0]))
        np.testing.assert_array_equal(frame,[12,12,13,14,0]);np.testing.assert_array_equal(amount,[1,1,1,1,0])
        with self.assertRaises(ValueError):presentation_bindings(f,p,[[0,-2,0],[0,2,0]],np.ones(5)*2)
