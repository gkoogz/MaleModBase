"""Extract the original angular root-profile functions and verify provenance."""
import argparse,hashlib,json
from pathlib import Path
from extract_authored_shape import function
ROOT=Path(__file__).resolve().parents[1]


def source_content():
    text=(ROOT/'legacy/wolverine/src/runtime/d3d9_proxy.cpp').read_text(encoding='utf-8')
    return '// Exact source functions; see tools/extract_root_profile.py.\n'+'\n\n'.join(
        function(text,name) for name in ['Smooth01','Smoother01','SampleRestShaftFrame','LogicalShaftOwner','RegularizeSharedRootProfile'])+'\n'


def outputs():
    content=source_content();path='tests/data/wolverine-root-profile.inc'
    files=['legacy/wolverine/src/runtime/d3d9_proxy.cpp','malemod_base/root_profile.py',
        'tests/root_profile_oracle.cpp','tools/build_root_profile_fixture.py','tests/test_root_profile.py',
        'tests/data/wolverine-root-profile.npy']
    record=dict(format='malemod.root-profile-provenance',version=1,
        sourceCommit=json.loads((ROOT/'provenance/wolverine.json').read_text())['commit'],
        files=[dict(path=p,sha256=hashlib.sha256((ROOT/p).read_bytes()).hexdigest()) for p in files],
        oracle=path,oracleSHA256=hashlib.sha256(content.encode()).hexdigest(),
        scope='Angular root-profile stage on identical caller-supplied float32 source geometry and measured rest frame',
        omissions=['preceding fairing','logical/glans render construction','egg rest fitting',
            'final UnifiedCollar solve','complete coupled dynamics','native output'])
    return {path:content,'provenance/root-profile.json':json.dumps(record,indent=2)+'\n'}


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--check',action='store_true');parser.add_argument('--source-only',action='store_true')
    args=parser.parse_args()
    items={'tests/data/wolverine-root-profile.inc':source_content()} if args.source_only else outputs()
    for path,content in items.items():
        if args.check:
            if (ROOT/path).read_bytes()!=content.encode():raise SystemExit('Root-profile extraction differs: '+path)
        else:(ROOT/path).write_bytes(content.encode())
