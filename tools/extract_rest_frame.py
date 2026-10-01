"""Extract source rest measurement functions for an SDK-free oracle."""
import argparse,hashlib,json
from pathlib import Path
from extract_authored_shape import function
ROOT=Path(__file__).resolve().parents[1]


def outputs():
    source='legacy/wolverine/src/runtime/d3d9_proxy.cpp'
    text=(ROOT/source).read_text(encoding='utf-8')
    names=['RestShaftDirection','ClosestRestShaftFlex','BuildShaftRestFrame','SampleRestShaftFrame']
    content='// Exact source functions; see tools/extract_rest_frame.py.\n'
    content+='\n\n'.join(function(text,name) for name in names)+'\n'
    path='tests/data/wolverine-rest-frame.inc'
    files=[source,'malemod_base/rest_frame.py','tests/rest_frame_oracle.cpp',
           'tools/build_rest_frame_fixture.py','tests/test_rest_frame.py','tests/data/wolverine-rest-frame.csv']
    record=dict(format='malemod.rest-frame-provenance',version=1,
        sourceCommit=json.loads((ROOT/'provenance/wolverine.json').read_text())['commit'],
        files=[dict(path=p,sha256=hashlib.sha256((ROOT/p).read_bytes()).hexdigest()) for p in files],
        oracle=path,oracleSHA256=hashlib.sha256(content.encode()).hexdigest(),
        scope='Rest measurement of caller-supplied source geometry; fixture input is the separately verified early stage',
        omissions=['preceding fairing/root-profile preparation','logical/glans render stages',
                   'complete coupled dynamics','native visible output'])
    return {path:content,'provenance/rest-frame.json':json.dumps(record,indent=2)+'\n'}


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--check',action='store_true')
    args=parser.parse_args()
    for p,content in outputs().items():
        if args.check:
            if (ROOT/p).read_bytes()!=content.encode():raise SystemExit('Rest frame extraction differs: '+p)
        else:(ROOT/p).write_bytes(content.encode())
