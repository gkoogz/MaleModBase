"""Apply a validated character-local fit to an OBJ reference surface."""
import argparse
import json
from pathlib import Path
import numpy as np

def transform(profile):
    if profile.get('contractVersion') != 1: raise ValueError('Unsupported profile contract')
    basis = np.asarray(profile['basis'],dtype=float)
    translation = np.asarray(profile['translation'],dtype=float)
    scale = float(profile['scale'])
    if basis.shape != (3,3) or translation.shape != (3,): raise ValueError('Invalid transform dimensions')
    if not np.isfinite(basis).all() or not np.isfinite(translation).all() or not np.isfinite(scale) or scale <= 0:
        raise ValueError('Transform must be finite with positive scale')
    if not np.allclose(basis.T@basis,np.eye(3),atol=1e-7): raise ValueError('Basis must be orthonormal')
    return basis,translation,scale

def fit(source, profile, output):
    basis,translation,scale = transform(profile)
    if source.resolve() == output.resolve(): raise ValueError('Output must differ from source')
    lines = []
    for line in source.read_text().splitlines():
        fields = line.split()
        if fields and fields[0] in {'v','vn'}:
            p = np.asarray([float(x) for x in fields[1:4]])
            p = basis@p
            if fields[0] == 'v': p = p*scale+translation
            line = fields[0]+' '+' '.join(format(float(x),'.9g') for x in p)
        elif fields and fields[0] == 'f' and np.linalg.det(basis) < 0:
            line = 'f '+' '.join([fields[1]]+fields[:1:-1])
        lines.append(line)
    output.parent.mkdir(parents=True,exist_ok=True)
    output.write_text('\n'.join(lines)+'\n')

def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('source',type=Path);p.add_argument('profile',type=Path);p.add_argument('output',type=Path)
    args = p.parse_args()
    fit(args.source,json.loads(args.profile.read_text()),args.output)
    print(f'Wrote {args.output}; geometry fitting only, skin retargeting remains unresolved.')

if __name__ == '__main__': main()
