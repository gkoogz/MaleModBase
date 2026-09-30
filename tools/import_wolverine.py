"""Import an immutable, hash-audited upstream source snapshot."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[1]
URL = 'https://github.com/gkoogz/XMenOriginsWolverineMaleMod'

def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def git(source, *args):
    return subprocess.check_output(['git', '-C', str(source), *args], text=True).strip()

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    args = parser.parse_args()
    source = args.source.resolve()
    if git(source, 'status', '--porcelain', '--untracked-files=no'):
        raise SystemExit('Tracked source has local changes; commit or preserve them before import.')
    destination = ROOT / 'legacy/wolverine'
    if destination.exists():
        raise SystemExit('Snapshot already exists; use a new checkout for a new source import.')
    paths = subprocess.check_output(['git','-C',str(source),'ls-files','-z']).decode().split('\0')
    records, omissions = [], []
    for relative in filter(None, paths):
        src = source / relative
        if src.is_symlink() or not src.is_file():
            raise SystemExit(f'Unsupported source entry: {relative}')
        if src.suffix.lower() in {'.exp', '.lib', '.res', '.dll', '.exe'}:
            omissions.append({'path': relative, 'reason': 'compiled artifact', 'sha256': sha(src)})
            continue
        target_relative = 'SOURCE-AGENTS.txt' if relative == 'AGENTS.md' else relative
        dest = destination / target_relative
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(src, dest)
        records.append({'sourcePath':relative, 'path':dest.relative_to(ROOT).as_posix(),
                        'bytes':src.stat().st_size, 'sha256':sha(src)})
    manifest = json.loads((source / 'manifest.json').read_text())
    materials = []
    for name, expected in manifest['payload'].items():
        if Path(name).suffix.lower() not in {'.dds', '.png'}:
            continue
        src = source / 'payload' / name
        item = {'name':name, 'expectedSHA256':expected.lower(), 'status':'not-available'}
        if src.is_file():
            if sha(src) != expected.lower():
                raise SystemExit(f'Material hash mismatch: {name}')
            dest = ROOT / 'assets/materials/wolverine' / name
            dest.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(src, dest)
            item.update(status='copied',path=dest.relative_to(ROOT).as_posix(),bytes=dest.stat().st_size)
        materials.append(item)
    dependencies = []
    for path in destination.rglob('*'):
        if path.suffix.lower() not in {'.h','.cpp','.cs','.cmd'}:
            continue
        text = path.read_text(errors='replace')
        for include in re.findall(r'^\s*#include\s+"([^"]+)"',text,re.M):
            if not (path.parent / include).exists() and not (destination/'third-party'/include).exists():
                dependencies.append({'file':path.relative_to(destination).as_posix(),'include':include})
    report = {'contractVersion':1,'repository':URL,'commit':git(source,'rev-parse','HEAD'),
              'releaseTag':'v2.0.0-beta.1','releaseCommit':git(source,'rev-parse','v2.0.0-beta.1^{commit}'),
              'sourceWasClean':True,'files':records,'omissions':omissions,'materials':materials,
              'missingQuotedIncludes':dependencies,'upstreamRootLicense':'not-present'}
    shared = ROOT/'include/malemod/detail/surface_limit.h'
    shared.parent.mkdir(parents=True,exist_ok=True)
    shutil.copyfile(source/'src/runtime/surface_limit.h',shared)
    out = ROOT / 'provenance/wolverine.json'
    out.parent.mkdir(parents=True,exist_ok=True)
    out.write_text(json.dumps(report,indent=2)+'\n')
    print(f'Imported {len(records)} files; {len(omissions)} compiled artifacts omitted; '
          f'{sum(m["status"] == "copied" for m in materials)} verified material maps.')

if __name__ == '__main__':
    main()
