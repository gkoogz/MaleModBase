"""Extract a narrow ORIGINAL-source C++ oracle for the early authored stage."""
import argparse
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def function(text, name):
    import re
    match = re.search(r'static [^\n;]+\b' + name + r'\([^;]*?\)\s*\{', text)
    if match is None:
        raise ValueError('Source function not found: ' + name)
    start = match.start(); brace = text.index('{', match.start()); depth = 1; end = brace + 1
    while depth:
        depth += (text[end] == '{') - (text[end] == '}'); end += 1
    return text[start:end]


def outputs():
    paths = ['legacy/wolverine/src/runtime/d3d9_proxy.cpp',
             'legacy/wolverine/src/runtime/prepared_shape.h',
             'legacy/wolverine/src/runtime/morph_targets_faired.h',
             'legacy/wolverine/src/runtime/physics_weights.h',
             'legacy/wolverine/src/runtime/collar_fairing.h',
             'legacy/wolverine/src/runtime/pelvic_root_binding.h',
             'legacy/wolverine/src/runtime/suspension_weights.h']
    source = (ROOT/paths[0]).read_text(encoding='utf-8')
    prepared = (ROOT/paths[1]).read_text(encoding='utf-8')
    # The complete loop is copied verbatim. Do not relabel it as the complete
    # prepared shape: all functions after FairUnifiedCollar remain omitted.
    loop = prepared[prepared.index('  for(UINT i=0;i<graftCount;i++){'):
                    prepared.index('  FairUnifiedCollar(')]
    names = ['Smoother01', 'ShaftRoot', 'OverallShapeScale', 'ShaftWidthScale',
             'BallShapeScale', 'PelvisCollarGrowth', 'ApplyPelvisCollar',
             'ApplyShaftPoseAngle', 'FlareAttachment', 'HangOffset', 'SampleOverallWidthVertex']
    text = '// Original source oracle; extraction recipe is in tools/extract_authored_shape.py.\n'
    text += '\n\n'.join(function(source, name) for name in names)
    text += '\nstatic void BuildEarlyAuthoredStage(){\n  float collarGrowth=PelvisCollarGrowth();\n' + loop + '}\n'
    generated = 'tests/data/wolverine-authored-shape.inc'
    def hashes(files):
        return [dict(path=p, sha256=hashlib.sha256((ROOT/p).read_bytes()).hexdigest()) for p in files]
    provenance = dict(format='malemod.authored-shape-provenance', version=1,
        sourceCommit=json.loads((ROOT/'provenance/wolverine.json').read_text())['commit'],
        sourceFiles=hashes(paths), oracle=generated,
        oracleSHA256=hashlib.sha256(text.encode()).hexdigest(),
        files=hashes(['malemod_base/authored_shape.py', 'tools/build_shape_fixture.py', 'tests/authored_shape_oracle.cpp',
                     'tests/test_authored_shape.py', 'tests/data/wolverine-authored-shape.csv']),
        scope='Early 2388-point authored stage and separate hang offset, source units',
        omissions=['remaining prepared-shape fairing/regularization', 'glans render refinement',
                   'logical rest section/egg fitting', 'final UnifiedCollar integration',
                   'full posed physics/contact', 'native buffer/graph output'])
    return {generated: text, 'provenance/authored-shape.json': json.dumps(provenance, indent=2)+'\n'}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__); parser.add_argument('--check', action='store_true')
    args = parser.parse_args()
    for p, text in outputs().items():
        if args.check:
            if (ROOT/p).read_bytes() != text.encode():
                raise SystemExit('Authored shape extraction differs: ' + p)
        else:
            (ROOT/p).write_bytes(text.encode())
