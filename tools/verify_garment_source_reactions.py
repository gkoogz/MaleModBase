"""Verify actual source CDF consumption, with exact binary and source receipts.

No game launch or installed-file changes. The CLI must be built from the same
checkout using the process-isolated Win32 source recipe; wire5 is rejected.
"""
import argparse
import hashlib
import json
import struct
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def git(*args):
    return subprocess.check_output(['git', '-C', str(ROOT), *args], text=True).strip()


def verify(executable, library, output, legacy_executable=None):
    executable, library, output = [Path(p).resolve() for p in (executable, library, output)]
    if any(not p.is_relative_to(ROOT / 'build') for p in (executable, library, output)):
        raise ValueError('Use owned build binaries and report directory')
    if not executable.is_file() or not library.is_file():
        raise ValueError('The actual source CLI and library must exist')
    output.mkdir(parents=True, exist_ok=True)
    proof = []
    for state in (0, 1, 2):
        records = {}
        for mode in (0, 2, 3, 4, 5, 6):
            path = output / f'state{state}-mode{mode}.bin'
            run = subprocess.run([str(executable), str(mode), str(state), str(path)],
                                 capture_output=True, text=True, check=True)
            data = path.read_bytes()
            if len(data) < 8 or struct.unpack_from('<I', data, 0)[0] != 6:
                raise ValueError('The executed source binary did not emit wire6')
            records[mode] = dict(file=path.name, sha256=digest(path), bytes=len(data),
                                 sourceGuides=run.stdout.strip())
        if records[0]['sha256'] != records[2]['sha256']:
            raise ValueError('Disabled nonzero garment input changed source bytes')
        if records[3]['sha256'] != records[4]['sha256']:
            raise ValueError('Repeated CDF publication replayed a source impulse')
        if records[0]['sha256'] == records[3]['sha256']:
            raise ValueError('Physical CDF impulse failed to change the actual source')
        if records[5]['sha256'] != records[3]['sha256']:
            raise ValueError('A zero-source-substep publication lost or replayed its impulse')
        if records[6]['sha256'] != records[3]['sha256']:
            raise ValueError('Pending CDF increments did not accumulate before source substep')
        case = dict(state=state, records=records, disabledExactParity=True,
                    cdfRetryExactParity=True, physicalImpulseChangesSource=True,
                    pendingNoSubstepExactParity=True, pendingCoalescingExactParity=True)
        if legacy_executable:
            legacy_executable = Path(legacy_executable).resolve()
            if not legacy_executable.is_relative_to(ROOT/'build') or not legacy_executable.is_file():
                raise ValueError('Use an owned immutable legacy source binary')
            path = output/f'legacy-state{state}-mode0.bin'
            subprocess.run([str(legacy_executable), '0', str(state), str(path)],
                           capture_output=True, text=True, check=True)
            legacy_data = path.read_bytes()
            if struct.unpack_from('<I', legacy_data)[0] != 5:
                raise ValueError('Reference binary is not the prior wire5 source')
            if legacy_data[4:] != (output/records[0]['file']).read_bytes()[4:]:
                raise ValueError('Disabled numerical payload differs from prior source')
            case['legacyDisabled'] = dict(file=path.name, sha256=digest(path), bytes=len(legacy_data),
                                          wire=5, numericalPayloadExact=True)
        proof.append(case)
    paths = ['src/surface/runtime.cpp', 'tools/extract_surface_runtime.py',
             'tests/garment_support_runtime_cli.cpp', 'include/malemod/surface/wire.hpp',
             'include/malemod/surface/runtime.hpp', 'include/malemod/surface/garment_impulse.hpp',
             'include/malemod/garments/reaction_support.hpp',
             'legacy/wolverine/src/runtime/compliant_dynamics.h',
             'provenance/wolverine.json', 'provenance/source-surface.json',
             'tools/verify_garment_source_reactions.py']
    source_paths = ['CMakeLists.txt', 'include', 'src', 'malemod_base', 'legacy',
                    'tools', 'provenance', 'assets', 'modules', 'profiles']
    receipt = dict(schema=2, wire=6, sourceProcessIsolated=True, installedGameUnchanged=True,
                   sourceCommit=git('rev-parse', 'HEAD'),
                   sourceDirty=git('status', '--porcelain', '--untracked-files=all', '--', *source_paths),
                   workspaceDirty=git('status', '--porcelain', '--untracked-files=all'),
                   executable=dict(path=str(executable.relative_to(ROOT)), sha256=digest(executable)),
                   library=dict(path=str(library.relative_to(ROOT)), sha256=digest(library)),
                   sourceHashes={p:digest(ROOT/p) for p in paths}, cases=proof)
    if legacy_executable:
        receipt['legacyExecutable'] = dict(path=str(legacy_executable.relative_to(ROOT)),
                                           sha256=digest(legacy_executable))
    (output/'proof.json').write_text(json.dumps(receipt, indent=2)+'\n', encoding='utf-8')
    print(output/'proof.json')
    return receipt


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('executable', 'library', 'output'):
        parser.add_argument('--'+name, type=Path, required=True)
    parser.add_argument('--legacy-executable', type=Path)
    args = parser.parse_args()
    verify(args.executable, args.library, args.output, args.legacy_executable)
