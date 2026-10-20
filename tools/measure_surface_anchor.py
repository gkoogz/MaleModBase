"""Measure an anchor on an unmodified, calibrated body NPZ (points, faces).

Game export/parsing belongs in the adapter. Generated body meshes and per-game
binding results stay outside Base; this tool records their input hash/donors.
"""
import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import sys
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from malemod_base.surface_anchor import measure_anchor


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--surface', type=Path, required=True)
    parser.add_argument('--origin', type=float, nargs=3, required=True)
    parser.add_argument('--inward', type=float, nargs=3, required=True)
    parser.add_argument('--max-distance', type=float, required=True)
    parser.add_argument('--units', required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    with np.load(args.surface, allow_pickle=False) as mesh:
        points, faces = mesh['points'], mesh['faces']
        binding = measure_anchor(points, faces, args.origin, args.inward,
                                 args.max_distance)
        record = {
            'contractVersion': 1,
            'feature': 'surface.rest-anchor',
            'surfaceSHA256': hashlib.sha256(args.surface.read_bytes()).hexdigest(),
            'pointCount': len(points), 'triangleCount': len(faces),
            'units': args.units, 'origin': args.origin, 'inward': args.inward,
            'maxDistance': args.max_distance,
            'binding': asdict(binding),
            'position': binding.evaluate(points).tolist(),
            'verification': {'sourceMeasurement': True, 'native': False,
                             'visualAttachmentGate': False},
        }
    args.output.write_text(json.dumps(record, indent=2, allow_nan=False)+'\n',
                           encoding='utf-8')
    print(json.dumps({'position': record['position'], 'output': str(args.output)}))


if __name__ == '__main__':
    main()
