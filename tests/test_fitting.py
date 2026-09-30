import json
from pathlib import Path
import sys
import tempfile
import unittest
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from fit_mesh import fit, transform
from export_geometry import dimension, arrays

class FittingTests(unittest.TestCase):
    def profile(self):
        return {'contractVersion':1,'scale':2,'basis':[[1,0,0],[0,1,0],[0,0,1]],'translation':[10,20,30]}

    def test_scale_translation_and_normals(self):
        with tempfile.TemporaryDirectory() as temp:
            source=Path(temp)/'source.obj';output=Path(temp)/'result.obj'
            source.write_text('v 1 2 3\nvn 0 0 1\nvt .2 .3\nf 1/1 2/2 3/3\n')
            fit(source,self.profile(),output)
            self.assertEqual(output.read_text().splitlines(),['v 12 24 36','vn 0 0 1','vt .2 .3','f 1/1 2/2 3/3'])

    def test_reflection_preserves_front_faces_and_uv_indices(self):
        p=self.profile();p['basis'][0][0]=-1
        with tempfile.TemporaryDirectory() as temp:
            source=Path(temp)/'source.obj';output=Path(temp)/'result.obj'
            source.write_text('v 1 2 3\nf 1/4 2/5 3/6\n')
            fit(source,p,output)
            self.assertIn('f 1/4 3/6 2/5',output.read_text())

    def test_invalid_profiles(self):
        for key,value in [('scale',0),('scale',float('nan')),('basis',[[2,0,0],[0,1,0],[0,0,1]]),('translation',[1,2])]:
            p=self.profile();p[key]=value
            with self.assertRaises(ValueError): transform(p)

    def test_source_cannot_be_overwritten(self):
        with tempfile.TemporaryDirectory() as temp:
            source=Path(temp)/'source.obj';source.write_text('v 0 0 0\n')
            with self.assertRaises(ValueError): fit(source,self.profile(),source)

    def test_dimension_parser_rejects_code(self):
        self.assertEqual(dimension('COUNT*3',{'COUNT':4}),12)
        with self.assertRaises(ValueError): dimension('__import__("os")',{})

    def test_literal_array_decoding_and_symbolic_omission(self):
        with tempfile.TemporaryDirectory() as temp:
            path=Path(temp)/'data.h'
            path.write_text('static const unsigned COUNT=2;\nstatic const float x[COUNT*3]={1.f,2.f,3.f,4.f,5.f,6.f};\nstatic const unsigned char y[2]={0xff,0x10};\nstatic const unsigned z[1]={missingSymbol};')
            found={name:data for name,data,_ in arrays(path)}
            np.testing.assert_array_equal(found['x'],[1,2,3,4,5,6])
            np.testing.assert_array_equal(found['y'],[255,16])
            self.assertNotIn('z',found)

if __name__=='__main__': unittest.main()
