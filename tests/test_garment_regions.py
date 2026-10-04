"""Exact authored semantic lineage proof; no game access or install."""
import sys, unittest
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from malemod_base.garment_regions import reference_anatomy_semantics,anatomy_regions,anatomy_regions_header,full_source_root_boundary

class SemanticLineage(unittest.TestCase):
    def test_full_native_lineage(self):
        bank=np.load(ROOT/'assets/wolverine-reference/geometry.npz')
        fields=reference_anatomy_semantics(bank)
        groups=anatomy_regions(fields)
        self.assertEqual(fields.shape,(17528,4))
        self.assertEqual({k:len(v) for k,v in groups.items()},dict(shaft=5448,glans=2993,leftLobe=3851,rightLobe=3771))
        retained=bank['neck_render_data__nrDirect']
        authored_glans=(retained>=1721+34*96)&(retained<10554)
        self.assertTrue(np.all(np.isin(np.flatnonzero(authored_glans),groups['glans'])))
        ids=np.concatenate(list(groups.values()))
        self.assertEqual(len(np.unique(ids)),len(ids))
        self.assertLess(ids.max(),17528)
        # Both lobes retain their original material side. Do not classify a
        # live rotated/distorted surface with engine axes or invented bones.
        base=bank['r14_asset__r14Base'].reshape(-1,3)
        for name,sign in [('leftLobe',-1),('rightLobe',1)]:
            direct=retained[groups[name]]
            direct=direct[direct<1721]
            self.assertTrue(np.all(base[direct,1]*sign>=-1e-7))
        header=anatomy_regions_header(groups)
        self.assertIn('jockstrap_region_glans',header)
        with self.assertRaises(ValueError):anatomy_regions(fields[:3])
        invalid=fields.copy();invalid[0,0]=np.nan
        with self.assertRaises(ValueError):anatomy_regions(invalid)

    def test_complete_original_root_boundary(self):
        bank=np.load(ROOT/'assets/wolverine-reference/geometry.npz')
        points=bank['derived__final_reference_positions']
        faces=bank['neck_render_data__nrIndices'].reshape(-1,3)
        opening=full_source_root_boundary(points,faces)
        self.assertEqual(len(opening),54)
        self.assertEqual(len(np.unique(opening)),54)
        self.assertLess(opening.max(),17528)
        np.testing.assert_array_equal(full_source_root_boundary(points+np.array([1000.,-500.,250.]),faces),opening)
        np.testing.assert_array_equal(full_source_root_boundary(points*100.,faces),opening)
        with self.assertRaises(ValueError):full_source_root_boundary(points,faces[:-10])
        bad=faces.copy();bad[0,0]=17528
        with self.assertRaises(ValueError):full_source_root_boundary(points,bad)

if __name__=='__main__':unittest.main()
