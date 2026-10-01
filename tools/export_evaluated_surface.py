"""Export an authoritative oracle surface, preserving its original vertex IDs."""
import argparse
from pathlib import Path
import numpy as np
from audit_surface_oracle import read_packed, ROOT


def export(prefix, destination):
    positions, packed = read_packed(str(prefix) + '.final')
    indices = np.fromfile(str(prefix) + '.final-indices', dtype='<u2').reshape(-1, 3)
    with np.load(ROOT / 'assets/wolverine-reference/geometry.npz') as bank:
        expected = bank['neck_render_data__nrIndices'].reshape(-1, 3)
    if not np.array_equal(indices, expected):
        raise ValueError('Evaluated surface no longer has the original topology')
    if not np.isfinite(positions).all():
        raise ValueError('Non-finite evaluated vertex')
    destination.parent.mkdir(parents=True, exist_ok=True)
    # Verified in the original UpdateNeckRender's UV decoder.
    uv = packed[:, 28:32].copy().view('<f2').reshape(-1, 2)
    with destination.open('w', newline='\n') as out:
        out.write('# Evaluated original-source surface; source vertex order retained.\n')
        out.write('# Source units. Reference oracle output, not observed gameplay.\n')
        for p in positions:
            out.write('v %.9g %.9g %.9g\n' % tuple(p))
        for p in uv:
            out.write('vt %.9g %.9g\n' % tuple(p))
        for face in indices + 1:
            out.write('f ' + ' '.join('%d/%d' % (i, i) for i in face) + '\n')
    print(str(destination.resolve()))
    return positions, indices


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('prefix', type=Path)
    parser.add_argument('destination', type=Path)
    args = parser.parse_args()
    export(args.prefix, args.destination)
