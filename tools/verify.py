"""Verify import provenance and decoded geometry against its source tables."""
import hashlib
import json
import subprocess
import sys
from pathlib import Path
import numpy as np
from export_geometry import arrays
from extract_clinical import generate, FILES
from extract_physics import outputs as physics_outputs
from extract_collar import outputs as collar_outputs
from export_controls import outputs as control_outputs
from extract_authored_shape import outputs as authored_outputs
from extract_physics_controls import outputs as physics_control_outputs
from extract_rest_frame import outputs as rest_frame_outputs
from extract_root_profile import outputs as root_profile_outputs

ROOT = Path(__file__).resolve().parents[1]

def main():
    boundary=json.loads((ROOT/'provenance/part-boundary-presentation.json').read_text())
    for item in boundary['files']:
        if hashlib.sha256((ROOT/item['path']).read_bytes()).hexdigest()!=item['sha256']:
            raise ValueError('Part boundary/presentation provenance differs: '+item['path'])
    subprocess.run([sys.executable,str(ROOT/'tools/extract_motion_filter.py')],check=True)
    motion=json.loads((ROOT/'provenance/motion-filter.json').read_text())
    for item in motion['files']:
        if hashlib.sha256((ROOT/item['path']).read_bytes()).hexdigest()!=item['sha256']:
            raise ValueError('Motion/calibration provenance differs: '+item['path'])
    graft_runtime=json.loads((ROOT/'provenance/graft-runtime.json').read_text())
    if graft_runtime['sourceCommit']!=json.loads((ROOT/'provenance/wolverine.json').read_text())['commit']:
        raise ValueError('C++ graft source revision differs')
    for item in graft_runtime['files']:
        if hashlib.sha256((ROOT/item['path']).read_bytes()).hexdigest()!=item['sha256']:
            raise ValueError('C++ graft implementation/provenance differs: '+item['path'])
    overall=json.loads((ROOT/'provenance/overall-recruitment.json').read_text())
    for item in overall['files']:
        if hashlib.sha256((ROOT/item['path']).read_bytes()).hexdigest()!=item['sha256']:
            raise ValueError('Overall recruitment provenance differs: '+item['path'])
    rigid=json.loads((ROOT/'provenance/rigid-cluster.json').read_text())
    for item in rigid['files']:
        if hashlib.sha256((ROOT/item['path']).read_bytes()).hexdigest()!=item['sha256']:
            raise ValueError('Rigid-cluster implementation/provenance differs: '+item['path'])
    secondary=json.loads((ROOT/'provenance/secondary-rig.json').read_text())
    if secondary['sourceCommit']!=json.loads((ROOT/'provenance/wolverine.json').read_text())['commit']:
        raise ValueError('Secondary rig source commit differs')
    for item in secondary['sources']+secondary['files']:
        if hashlib.sha256((ROOT/item['path']).read_bytes()).hexdigest()!=item['sha256']:
            raise ValueError('Secondary-rig source/implementation differs: '+item['path'])
    subprocess.run([sys.executable,str(ROOT/'tools/extract_ovoid_support.py'),'--check'],check=True)
    subprocess.run([sys.executable,str(ROOT/'tools/extract_surface_runtime.py'),'--verify-provenance'],check=True)
    for path,content in root_profile_outputs().items():
        if (ROOT/path).read_bytes()!=content.encode():
            raise ValueError('Root-profile extraction/provenance differs: '+path)
    for path,content in rest_frame_outputs().items():
        if (ROOT/path).read_bytes()!=content.encode():
            raise ValueError('Rest frame extraction/provenance differs: '+path)
    for path, content in physics_control_outputs().items():
        if (ROOT/path).read_bytes() != content.encode():
            raise ValueError('Active physics control extraction/provenance differs: '+path)
    for path, content in authored_outputs().items():
        if (ROOT/path).read_bytes() != content.encode():
            raise ValueError('Authored shape extraction/provenance differs: '+path)
    for path, content in control_outputs().items():
        if (ROOT/path).read_bytes() != content.encode():
            raise ValueError('Control catalog/provenance differs: '+path)
    report = json.loads((ROOT/'provenance/wolverine.json').read_text())
    count = 0
    for item in report['files']:
        path = ROOT/item['path']
        if len(path.read_bytes()) != item['bytes'] or hashlib.sha256(path.read_bytes()).hexdigest() != item['sha256']:
            raise ValueError(f'Imported file changed: {path}')
        count += 1
    for item in report['materials']:
        if item['status'] == 'copied' and hashlib.sha256((ROOT/item['path']).read_bytes()).hexdigest() != item['expectedSHA256']:
            raise ValueError(f'Material changed: {item["name"]}')
    if (ROOT/'include/malemod/detail/surface_limit.h').read_bytes() != (ROOT/'legacy/wolverine/src/runtime/surface_limit.h').read_bytes():
        raise ValueError('Portable limiter differs from imported numerical implementation')
    support=json.loads((ROOT/'provenance/radial-clinical-support.json').read_text())
    for item in support['files']:
        if hashlib.sha256((ROOT/item['path']).read_bytes()).hexdigest()!=item['sha256']:
            raise ValueError('Radial/clinical support provenance differs: '+item['path'])
    garments=json.loads((ROOT/'provenance/garments.json').read_text())
    for path,expected in garments['fileHashes'].items():
        if hashlib.sha256((ROOT/path).read_bytes()).hexdigest()!=expected:
            raise ValueError('Shared garment provenance differs: '+path)
    clinical=json.loads((ROOT/'provenance/clinical.json').read_text())
    for item in clinical['files']:
        if hashlib.sha256((ROOT/item['source']).read_bytes()).hexdigest()!=item['sourceSHA256']:
            raise ValueError('Clinical source provenance differs')
        if hashlib.sha256((ROOT/item['path']).read_bytes()).hexdigest()!=item['sha256']:
            raise ValueError('Clinical generated provenance differs')
    for name in FILES:
        if (ROOT/'include/malemod/clinical'/name).read_bytes()!=generate(name).encode():
            raise ValueError(f'Clinical extraction recipe differs: {name}')
    if hashlib.sha256((ROOT/clinical['bakeAsset']).read_bytes()).hexdigest()!=clinical['bakeSHA256']:
        raise ValueError('Clinical baked source differs')
    physics=json.loads((ROOT/'provenance/physics-kernels.json').read_text())
    if hashlib.sha256((ROOT/physics['source']).read_bytes()).hexdigest()!=physics['sourceSHA256']:
        raise ValueError('Physics source provenance differs')
    for item in physics['files']:
        if hashlib.sha256((ROOT/item['path']).read_bytes()).hexdigest()!=item['sha256']:
            raise ValueError('Physics generated provenance differs')
    for path,text in physics_outputs().items():
        if (ROOT/path).read_bytes()!=text.encode():
            raise ValueError('Physics extraction recipe differs: '+path)
    collar=json.loads((ROOT/'provenance/collar.json').read_text())
    if hashlib.sha256((ROOT/collar['source']).read_bytes()).hexdigest()!=collar['sourceSHA256']:
        raise ValueError('Collar source provenance differs')
    for item in collar['files']+collar['relatedFiles']:
        if hashlib.sha256((ROOT/item['path']).read_bytes()).hexdigest()!=item['sha256']:
            raise ValueError('Collar generated/related provenance differs: '+item['path'])
    for path,text in collar_outputs().items():
        if (ROOT/path).read_bytes()!=text.encode():
            raise ValueError('Collar extraction recipe differs: '+path)
    generic=ROOT/'assets/generic-male'
    graft=json.loads((ROOT/'provenance/rest-graft.json').read_text())
    for item in graft['files']:
        if hashlib.sha256((ROOT/item['path']).read_bytes()).hexdigest()!=item['sha256']:
            raise ValueError('Rest-graft implementation provenance differs: '+item['path'])
    if hashlib.sha256((ROOT/graft['sourceBank']).read_bytes()).hexdigest()!=graft['sourceBankSHA256']:
        raise ValueError('Rest-graft source bank differs')
    motion=json.loads((ROOT/'provenance/motion-binding.json').read_text())
    for item in motion['files']:
        if hashlib.sha256((ROOT/item['path']).read_bytes()).hexdigest()!=item['sha256']:
            raise ValueError('Motion binding provenance differs: '+item['path'])
    generic_manifest=json.loads((generic/'manifest.json').read_text())
    for name,digest in generic_manifest['files'].items():
        if hashlib.sha256((generic/name).read_bytes()).hexdigest()!=digest:
            raise ValueError('Generic reference hash differs: '+name)
    asset = ROOT/'assets/wolverine-reference'
    manifest = json.loads((asset/'manifest.json').read_text())
    with np.load(asset/'geometry.npz',allow_pickle=False) as bank:
        for path in sorted((ROOT/'legacy/wolverine/src/runtime').glob('*.h')):
            for name,data,_ in arrays(path):
                np.testing.assert_array_equal(bank[path.stem+'__'+name],data)
        for item in manifest['arrays']:
            if hashlib.sha256(bank[item['key']].tobytes()).hexdigest() != item['sha256']:
                raise ValueError(f'Array mismatch: {item["key"]}')
    for item in manifest['meshes']:
        data = (asset/item['file']).read_bytes()
        if hashlib.sha256(data).hexdigest() != item['sha256']: raise ValueError('Mesh hash mismatch')
        vertices = sum(line.startswith(b'v ') for line in data.splitlines())
        faces = sum(line.startswith(b'f ') for line in data.splitlines())
        if (vertices,faces) != (item['vertices'],item['triangles']): raise ValueError('Mesh counts mismatch')
    print(f'PASS: {count} imported files, materials, {len(manifest["arrays"])} source arrays, exported meshes, generic reference and graft/collar/physics/clinical provenance.')

if __name__ == '__main__': main()
