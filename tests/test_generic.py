import json
from pathlib import Path
import struct
import sys
import tempfile
import unittest
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from malemod_base.generic import build
from malemod_base.glb import write_glb
from malemod_base.contracts import validate_preferences,validate_character,classify_feature

class GenericTests(unittest.TestCase):
    def profile(self):return json.loads((ROOT/'profiles/default-preferences.json').read_text())
    def test_topology_and_binding(self):
        asset=build(self.profile());mesh=asset['mesh'];p=np.array(mesh['positions']);n=np.array(mesh['normals']);f=np.array(mesh['indices'])
        self.assertTrue(np.isfinite(p).all());self.assertGreaterEqual(int(f.min()),0);self.assertLess(int(f.max()),len(p))
        np.testing.assert_allclose(np.array(mesh['weights']).sum(axis=1),1)
        self.assertLess(int(np.array(mesh['joints']).max()),len(asset['skeleton']))
        cross=np.cross(p[f[:,1]]-p[f[:,0]],p[f[:,2]]-p[f[:,0]])
        self.assertTrue((np.einsum('ij,ij->i',cross,n[f].mean(axis=1))>0).all())
        for i,j in enumerate(asset['skeleton']):self.assertLess(j['parent'],i)
        names={j['id'] for j in asset['skeleton']}
        self.assertTrue(all(s['joint'] in names for s in asset['sockets']))
    def test_controls_change_geometry_without_topology_drift(self):
        baseline=build(self.profile());shape=np.array(baseline['mesh']['positions'])
        for channel in baseline['controls']:
            p=self.profile();p['body'][channel]=1;new=build(p)
            self.assertEqual(new['mesh']['indices'],baseline['mesh']['indices'])
            self.assertGreater(float(np.linalg.norm(np.array(new['mesh']['positions'])-shape)),.01)
    def test_glb_bind_transforms(self):
        asset=build(self.profile())
        with tempfile.TemporaryDirectory() as temp:
            path=Path(temp)/'body.glb';write_glb(path,asset['mesh'],asset['skeleton']);data=path.read_bytes()
        magic,version,length=struct.unpack_from('<III',data);self.assertEqual((magic,version,length),(0x46546c67,2,len(data)))
        size,kind=struct.unpack_from('<II',data,12);self.assertEqual(kind,0x4e4f534a);gltf=json.loads(data[20:20+size]);binary=data[28+size:]
        accessor=gltf['accessors'][gltf['skins'][0]['inverseBindMatrices']];view=gltf['bufferViews'][accessor['bufferView']]
        matrices=np.frombuffer(binary,dtype='<f4',count=len(asset['skeleton'])*16,offset=view['byteOffset']).reshape(-1,4,4).transpose(0,2,1)
        basis=np.array([[1,0,0],[0,0,1],[0,-1,0]])
        for i,j in enumerate(asset['skeleton']):
            point=np.r_[basis@np.array(j['position']),1];np.testing.assert_allclose(matrices[i]@point,[0,0,0,1],atol=1e-6)
        self.assertEqual(len(gltf['skins'][0]['joints']),25)
    def test_unknown_controls_and_uncalibrated_characters_rejected(self):
        p=self.profile();p['body']['magic_face']=.5
        with self.assertRaises(ValueError):validate_preferences(p)
        with self.assertRaises(ValueError):validate_character({'contractVersion':1,'metersPerUnit':None})
        p={'contractVersion':1,'metersPerUnit':.01,'jointMapping':{'pelvis':'observedHip'},'observedNativeJoints':['observedHip'],'capabilities':{'pose_sampling':'verified'}}
        validate_character(p,['pelvis'])
        with self.assertRaises(ValueError):validate_character(p,['head'])
    def test_feature_routing(self):
        self.assertEqual(classify_feature('garment'),'base');self.assertEqual(classify_feature('native_joint_mapping'),'adapter')

if __name__=='__main__':unittest.main()
