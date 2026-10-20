"""Repair unsafe interpolation intervals without changing reviewed endpoints."""
import argparse
import hashlib
import json
import sys
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from malemod_base.deformation_path import repair_path
from malemod_base.surface_bending import chord_report


def run(domain_path, part_paths, output):
    d = np.load(domain_path); f = d['faces']; parts = [np.load(p) for p in part_paths]
    ref = parts[0]['before']; points = [ref.copy()]
    for part in parts:
        if not np.array_equal(part['before'], ref):
            raise ValueError('Trajectory references differ')
        for knot in part['knots']:
            if not np.array_equal(points[-1], knot):
                points.append(knot.copy())
    fixed = np.zeros(len(ref), bool); fixed[d['protected']] = True
    fixed |= d['roles'][:, 0] >= .05
    original_endpoint = points[-1].copy()
    i, insertions = 0, 0
    output = Path(output); output.parent.mkdir(parents=True, exist_ok=True)
    while i+1 < len(points):
        check = chord_report(ref, points[i], points[i+1], f)
        if check['safe']:
            i += 1
            continue
        if insertions >= 180:
            raise RuntimeError('Trajectory repair exceeded its authoring bound')
        t = np.clip(np.median(check['fractions']), .15, .85)
        target = points[i]+t*(points[i+1]-points[i])
        free = np.zeros(len(ref), bool); free[f[np.array(check['faces'], int)].ravel()] = True
        for _ in range(2):
            free[f[free[f].any(1)].ravel()] = True
        for slave, a, b, _ in d['seams']:
            if free[int(slave)]:
                free[[int(a), int(b)]] = True
        free[fixed] = False
        result = repair_path(ref, target, f, free, d['seams'], start=points[i],
                             steps=1, max_iterations=200)
        for retry in range(4):
            if result.complete:
                break
            if retry % 2:
                free[f[free[f].any(1)].ravel()] = True
                for slave, a, b, _ in d['seams']:
                    if free[int(slave)]:
                        free[[int(a), int(b)]] = True
                free[fixed] = False
            seed = points[i+1] if retry % 2 == 0 else points[i]
            result = repair_path(ref, target, f, free, d['seams'], start=seed,
                                 steps=1, max_iterations=200)
        if not result.complete:
            raise RuntimeError('Cannot repair interpolation interval '+str(i)+': '+result.message)
        if np.max(abs(result.positions-points[i])) < 1e-10:
            raise RuntimeError('Trajectory repair made no progress')
        points.insert(i+1, result.positions); insertions += 1
        print(f'insert {insertions}: interval {i}, repair rows {free.sum()}', flush=True)
        np.savez(output.with_suffix('.checkpoint.npz'), before=ref, after=points[-1],
                 knots=np.array(points), verifiedPrefix=i, complete=False)
    if not np.array_equal(points[-1], original_endpoint):
        raise RuntimeError('Trajectory changed its reviewed endpoint')
    # Remove a knot only when its replacement chord passes the same continuous
    # test. This keeps runtime cost bounded without cutting unsafe corners.
    i = 1
    while i+1 < len(points):
        if chord_report(ref, points[i-1], points[i+1], f)['safe']:
            points.pop(i)
        else:
            i += 1
    np.savez(output, before=ref, after=points[-1], knots=np.array(points),
             fractions=np.linspace(0, 1, len(points)), complete=True)
    report = dict(complete=True, continuousTriangleGate=True, knots=len(points), insertions=insertions,
                  inputs=[dict(path=str(Path(p).resolve()), sha256=hashlib.sha256(Path(p).read_bytes()).hexdigest()) for p in [domain_path, *part_paths]],
                  nativeObserved=False, visualPassed=False)
    output.with_suffix('.json').write_text(json.dumps(report, indent=2))
    print(json.dumps(report), flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--domain', type=Path, required=True)
    parser.add_argument('--part', type=Path, action='append', required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(); run(args.domain, args.part, args.output)
