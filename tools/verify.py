"""Verify import provenance and decoded geometry against its source tables."""
import hashlib
import json
from pathlib import Path
import numpy as np
from export_geometry import arrays
from extract_clinical import generate, FILES

ROOT = Path(__file__).resolve().parents[1]

def main():
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
    print(f'PASS: {count} imported files, materials, {len(manifest["arrays"])} source arrays and all exported mesh hashes/counts.')

if __name__ == '__main__': main()
