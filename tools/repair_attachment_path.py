"""Author an explicit captured-mesh target; never install or launch a game."""
import argparse
import hashlib
import json
import sys
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from malemod_base.deformation_path import repair_path
from malemod_base.surface_bending import bending_report


def run(domain_path, target_path, output, start_path=None, steps=40):
    domain = np.load(domain_path)
    target = np.load(target_path)
    reference, requested = target['before'], target['after']
    previous = reference if start_path is None else np.load(start_path)['after']
    faces = domain['faces']
    report = bending_report(reference, requested, faces)
    bad = np.unique(np.r_[np.asarray(report['new_fold_pairs'], dtype=int).ravel(),
                             np.asarray(report['collapsed_faces'], dtype=int)])
    editable = np.zeros(len(reference), bool)
    editable[faces[bad].ravel()] = True
    for _ in range(2):
        editable[faces[editable[faces].any(1)].ravel()] = True
    if 'weight' in target:
        editable &= target['weight'] > 1e-5
    if 'roles' in domain and 'bodyCount' in domain:
        editable &= (domain['unique'] < int(domain['bodyCount'])) | (domain['roles'][:, 0] < .05)
    for slave, a, b, _ in domain['seams']:
        if editable[int(slave)]:
            editable[[int(a), int(b)]] = True
    editable[domain['protected']] = False
    output = Path(output); output.parent.mkdir(parents=True, exist_ok=True)
    inputs = {name: dict(path=str(Path(path).resolve()), sha256=hashlib.sha256(Path(path).read_bytes()).hexdigest())
              for name, path in [('domain', domain_path), ('target', target_path), ('start', start_path)] if path is not None}
    print(json.dumps(dict(inputs=inputs, repairRows=int(editable.sum()))), flush=True)
    knots, fractions = [previous.copy()], [0.]
    def progress(fraction, positions, message):
        knots.append(positions.copy()); fractions.append(fraction)
        np.savez(output.with_suffix('.checkpoint.npz'), before=reference, after=positions,
                 knots=np.array(knots), fractions=np.array(fractions), complete=False)
        print(f'accepted {fraction:.3f}: {message}', flush=True)
    result = repair_path(reference, requested, faces, editable, domain['seams'],
                         start=previous, steps=steps, max_iterations=200, progress=progress)
    np.savez(output, before=reference, after=result.positions, knots=np.array(result.knots),
             fractions=np.array(result.fractions), complete=result.complete, free=editable)
    report = dict(inputs=inputs, complete=result.complete, fraction=result.fractions[-1],
                  message=result.message, **result.report, nativeObserved=False, visualPassed=False)
    output.with_suffix('.json').write_text(json.dumps(report, indent=2))
    print(json.dumps({k: report[k] for k in ['complete', 'fraction', 'message']}), flush=True)
    return result.complete


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ['domain', 'target', 'output']:
        parser.add_argument('--'+name, type=Path, required=True)
    parser.add_argument('--start', type=Path)
    parser.add_argument('--steps', type=int, default=40)
    args = parser.parse_args()
    raise SystemExit(0 if run(args.domain, args.target, args.output, args.start, args.steps) else 2)
