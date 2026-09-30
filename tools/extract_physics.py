"""Extract Wolverine XPBD distance and dissipative-bend kernels with explicit state."""
import argparse
import hashlib
import json
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
SOURCE=ROOT/'legacy/wolverine/src/runtime/compliant_dynamics.h'

def function(text,name):
    start=text.index('static ',text.rfind('\n',0,text.index(name+'(')))
    brace=text.index('{',start);depth=1;end=brace+1
    while depth:
        depth+=(text[end]=='{')-(text[end]=='}');end+=1
    return text[start:end]

def outputs():
    source=SOURCE.read_text();names=['PDDistance','PDPrepareBend','PDBendPrepared']
    selected=[function(source,name) for name in names]
    bend='struct PDBendData {float alpha,gamma,factor;V3 oldValue;};'
    reference=bend+'\n'+'\n'.join(selected)+'\n'
    portable=bend+'\n'+'\n'.join(selected)+'\n'
    portable=portable.replace('static void PDDistance(','inline void SolveDistance(State& state,')
    portable=portable.replace('static PDBendData PDPrepareBend(int i,float compliance,float dt)', 'inline PDBendData PrepareBend(const State& state,int i,float compliance,float dt,float bounce01)')
    portable=portable.replace('static void PDBendPrepared(','inline void SolveBendPrepared(State& state,')
    portable=portable.replace('physUI[2]/100.f','bounce01')
    for old,new in [('pdOldPosition','state.oldPosition'),('pdPosition','state.position'),('pdInvMass','state.invMass')]:portable=portable.replace(old,new)
    portable='#pragma once\n// Generated from Wolverine; see provenance/physics-kernels.json.\n#include "state.hpp"\nnamespace malemod::physics {\n'+portable+'}\n'
    return {'include/malemod/physics/xpbd_kernels.hpp':portable,'tests/data/wolverine-xpbd.inc':reference}

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--check',action='store_true');args=p.parse_args();records=[]
    for path,text in outputs().items():
        dest=ROOT/path
        if args.check:
            if not dest.exists() or dest.read_bytes()!=text.encode():raise SystemExit('Kernel extraction differs: '+path)
        else:dest.parent.mkdir(parents=True,exist_ok=True);dest.write_text(text,newline='\n')
        records.append({'path':path,'sha256':hashlib.sha256(text.encode()).hexdigest()})
    report={'contractVersion':1,'source':SOURCE.relative_to(ROOT).as_posix(),'sourceSHA256':hashlib.sha256(SOURCE.read_bytes()).hexdigest(),'files':records,
            'scope':'Distance and Kelvin-Voigt bending kernels; full authored suspension/contact solver remains separate.',
            'changes':['explicit instance state','explicit normalized bounce input','standard C++ dependencies']}
    text=json.dumps(report,indent=2)+'\n';dest=ROOT/'provenance/physics-kernels.json'
    if args.check:
        if dest.read_bytes()!=text.encode():raise SystemExit('Kernel provenance differs')
    else:dest.write_text(text,newline='\n')
    print('PASS: XPBD kernel extraction recipe.' if args.check else 'Extracted distance and dissipative bending kernels.')
if __name__=='__main__':main()
