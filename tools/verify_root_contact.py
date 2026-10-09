"""Audit the opt-in candidate independently of historical deployed receipts."""
import argparse
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def verify(root, kernel=None):
    report = json.loads((root / 'provenance/root-contact.json').read_text())
    if report['wireVersion'] != 7 or report['defaultEnabled']:
        raise ValueError('Unexpected candidate protocol or default activation')
    if any(report[k] for k in ('observedCandidateGameplay', 'fullAttachmentGate',
                               'nativeAdoptionAccepted', 'installed')):
        raise ValueError('Development receipt must not claim native acceptance')
    for item in report['sourceFiles']:
        path = (root / item['path']).resolve()
        if not path.is_relative_to(root.resolve()) or digest(path) != item['sha256']:
            raise ValueError('Candidate source differs: ' + item['path'])
    closure = json.loads((root / 'tools/data/surface-closure.json').read_text())
    for item in closure['sources']:
        if digest(root / item['path']) != item['sha256']:
            raise ValueError('Original source differs: ' + item['path'])
    if kernel is not None and digest(kernel) != report['processKernelSHA256']:
        raise ValueError('Process kernel differs from candidate receipt')
    return len(report['sourceFiles']), len(closure['sources'])

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--kernel', type=Path)
    args = parser.parse_args()
    sources, originals = verify(ROOT, args.kernel)
    print('PASS', sources, 'candidate source hashes and', originals,
          'immutable original hashes; native acceptance remains false')
