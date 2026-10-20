"""Compare settled solver traces; remove common root rotation from motion metrics."""
import argparse
import json
from pathlib import Path
import numpy as np


def measure(path):
    rows = [json.loads(line) for line in path.read_text().splitlines()]
    rod = np.array([r['rod'] for r in rows])
    lobes = np.array([r['lobes'] for r in rows])
    anchors = np.array([r['anchors'] for r in rows])
    axis = rod[:, 1] - rod[:, 0]
    axis /= np.linalg.norm(axis, axis=1)[:, None]
    lateral = np.tile([0., 1., 0.], (len(rows), 1))
    lateral -= axis * np.sum(lateral * axis, axis=1)[:, None]
    lateral /= np.linalg.norm(lateral, axis=1)[:, None]
    basis = np.stack([axis, lateral, np.cross(axis, lateral)], axis=1)
    local_rod = np.einsum('nij,nkj->nki', basis, rod - rod[:, :1])
    relative = np.einsum('nij,nkj->nki', basis, lobes - anchors)
    spans = np.diff(rod, axis=1)
    spans /= np.linalg.norm(spans, axis=2)[:, :, None]
    bends = np.degrees(np.arccos(np.clip(np.sum(spans[:, :-1] * spans[:, 1:], axis=2), -1, 1)))
    assert all(r['protected'] == 0 for r in rows)
    assert np.isfinite(rod).all() and np.isfinite(relative).all()
    return dict(
        rootRelativeTipRange=float(np.linalg.norm(np.ptp(local_rod[20:80, -1], axis=0))),
        characterLocalTipRange=float(np.linalg.norm(np.ptp(rod[20:80, -1], axis=0))),
        anchorRelativeLobeRange=np.linalg.norm(np.ptp(relative[20:80], axis=0), axis=1).tolist(),
        anchorRelativeLateralRange=np.ptp(relative[20:80, :, 1], axis=0).tolist(),
        peakInterspanBendDegrees=float(bends[20:80].max()),
        finalRelativeLobeSpeed=float(np.linalg.norm(np.diff(relative[-10:], axis=0), axis=2).max() / .05),
        units='Uncalibrated source coordinates; not clinical measurements')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    args = parser.parse_args()
    result = {'settleSeconds': 20, 'driveSeconds': 4, 'releaseSeconds': 4, 'comparisons': {}}
    for size in (50, 100):
        pair = {name: measure(args.directory / f'{name}-{size}.jsonl') for name in ('reference', 'candidate')}
        pair['lateralGain'] = (np.array(pair['candidate']['anchorRelativeLateralRange']) /
                               pair['reference']['anchorRelativeLateralRange']).tolist()
        assert min(pair['lateralGain']) > 1.5, 'Independent lobe response did not improve'
        # Do not equate increased excursion with improved flexibility; retain
        # all metrics, including a decrease in root-relative tip movement.
        result['comparisons'][str(size)] = pair
    result['clock'] = {}
    for prefix in ('', 'reference-'):
        a, b = [json.loads((args.directory / f'{prefix}clock-{dt}.json').read_text()) for dt in (50, 150)]
        error = float(np.linalg.norm(np.array(a['rod']) - b['rod'], axis=1).max())
        result['clock'][prefix or 'candidate'] = error
    assert result['clock']['candidate'] < .0001
    assert result['clock']['reference-'] > .01
    result['passed'] = True
    (args.directory / 'relative-response-proof.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
