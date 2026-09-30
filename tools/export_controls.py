"""Export authoritative slider tables; adapters translate, not maintain copies."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from malemod_base.controls import catalog


def outputs():
    paths = ['legacy/wolverine/src/runtime/d3d9_proxy.cpp',
             'legacy/wolverine/src/runtime/morph_targets_faired.h',
             'legacy/wolverine/src/runtime/compliant_dynamics.h']
    implementations = ['malemod_base/controls.py', 'tests/test_controls.py']
    def hashes(files):
        return [dict(path=p, sha256=hashlib.sha256((ROOT/p).read_bytes()).hexdigest()) for p in files]
    contract = catalog()
    provenance = dict(format='malemod.controls-provenance', version=1,
        sourceCommit=json.loads((ROOT/'provenance/wolverine.json').read_text())['commit'],
        sourceFiles=hashes(paths), files=hashes(implementations),
        mappingScope='Source UI v5 rest controls; animation modulation excluded',
        omissions=['throb animation', 'idle audio', 'sequence controls',
                   'complete authored physics runtime', 'native engine bindings'],
        physicalCalibration='No SI unit inference; adapters calibrate explicitly')
    return {'modules/live-controls.json': json.dumps(contract, indent=2)+'\n',
            'provenance/controls.json': json.dumps(provenance, indent=2)+'\n'}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true')
    args = parser.parse_args()
    for path, content in outputs().items():
        if args.check:
            if (ROOT/path).read_bytes() != content.encode():
                raise SystemExit('Control export differs: '+path)
        else:
            (ROOT/path).write_bytes(content.encode())
    print('Control catalog and provenance match.' if args.check else 'Exported controls and provenance.')
