"""Preserve active source scalar laws for an SDK-free numerical oracle."""
import argparse
import hashlib
import json
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[1]


def outputs():
    proxy_path = 'legacy/wolverine/src/runtime/d3d9_proxy.cpp'
    dynamics_path = 'legacy/wolverine/src/runtime/compliant_dynamics.h'
    proxy = (ROOT/proxy_path).read_text(encoding='utf-8')
    dynamics = (ROOT/dynamics_path).read_text(encoding='utf-8')

    def line(text, prefix):
        found = [s.strip() for s in text.splitlines() if s.strip().startswith(prefix)]
        if len(found) != 1:
            raise ValueError('Ambiguous source expression: ' + prefix)
        return found[0]

    def function(text, prefix):
        start = text.index(prefix)
        brace = text.index('{', start)
        depth = 1
        end = brace+1
        while depth:
            depth += (text[end] == '{') - (text[end] == '}')
            end += 1
        return text[start:end]

    mode = function(proxy, 'static float ModeValue(')
    suspension = function(dynamics, 'static PDSuspensionData PDPrepareSuspension(')
    response = line(dynamics, 'float response=i<pdBody0?')
    code = '// Exact scalar source statements. Test oracle only; not a second runtime.\n'
    code += mode+'\n'+suspension+'\n'
    code += 'static std::array<float,22> SourceMaterialProfile(){\n'
    code += line(dynamics, 'float shaftMass=')+'\n'
    code += line(dynamics, 'float shaftDrag=')+'\n'
    code += 'float shaftResponse,shaftGravity,bodyResponse,bodyGravity;\n'
    code += 'for(int i=0;i<2;i++){const int pdBody0=1;'+response+'\n'
    code += 'if(i==0){shaftResponse=response;shaftGravity=gravity;}else{bodyResponse=response;bodyGravity=gravity;}}\n'
    for source, prefix in [(dynamics, 'float stiffness=max(0.f,min(1.f,physUI[0]/100.f));'),
                           (dynamics, 'float ratio=.10f+.40f*'), (dynamics, 'float torsion=20.f*'),
                           (proxy, 'float mass=(1.f+physValues[1]*'), (proxy, 'float k=ModeValue(38.f,'),
                           (proxy, 'float droop=ModeValue(.015f,'), (proxy, 'float limit=ModeValue(.42f,')]:
        code += line(source, prefix)+'\n'
    code += '''auto suspension=PDPrepareSuspension(0);
return {shaftMass,bodyMass,shaftDrag,bodyDrag,shaftResponse,bodyResponse,
shaftGravity,bodyGravity,bendCompliance,ratio,suspension.compliance,
suspension.shearCompliance,suspension.ratio,torsion,mass,k,damping,droop,
drive,limit,suspension.rest,suspension.hard};}
'''
    paths = ['malemod_base/physics_controls.py', 'tests/test_physics_controls.py',
             'tests/physics_controls_oracle.cpp', 'tests/data/physics-controls.csv',
             'tools/extract_physics_controls.py', 'tools/build_physics_controls_fixture.py']
    provenance = dict(format='malemod.physics-controls-provenance', version=1,
        sourceCommit='dc64bdc44e75fd5521f066cdb2975277e9c34302',
        sourceFiles=[dict(path=p, sha256=hashlib.sha256((ROOT/p).read_bytes()).hexdigest())
                     for p in [proxy_path, dynamics_path]],
        files=[dict(path=p, sha256=hashlib.sha256((ROOT/p).read_bytes()).hexdigest())
               for p in paths if (ROOT/p).exists()],
        oracleIncludeSHA256=hashlib.sha256(code.encode()).hexdigest(),
        scope='Active raw UI and mapped-value scalar laws; no complete solver or native runtime parity',
        sourceUnits='Caller measured source rest space; no SI inference',
        deferred=['clinical modulation', 'contact geometry', 'rest-guide generation', 'native output'])
    return {'tests/data/wolverine-physics-controls.inc': code,
            'provenance/physics-controls.json': json.dumps(provenance, indent=2)+'\n'}


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--check', action='store_true')
    args=parser.parse_args()
    for path, value in outputs().items():
        if args.check:
            if (ROOT/path).read_bytes()!=value.encode():
                raise ValueError('Physics control provenance changed: '+path)
        else:
            (ROOT/path).write_bytes(value.encode())


if __name__=='__main__':main()
