"""Extract the source tapered-ellipsoid support for float-only backends."""
import hashlib,json,re
from pathlib import Path
from extract_physics import function
ROOT=Path(__file__).resolve().parents[1]
def outputs():
    source=ROOT/'legacy/wolverine/src/runtime/pouch_contact.h'
    body=function(source.read_text(),'CPSupportLocal')
    reference=body+'\n'
    body=body.replace('static V3 CPSupportLocal','inline V3 OvoidSupport')
    body=body.replace('copysignf(r.z,v)','sign')
    body=body.replace('if(h<1e-8f)return {0,0,sign};','float sign=r.z;if(v<0.f)sign=-r.z;\n if(h<1e-8f)return {0,0,sign};')
    body=body.replace('double','float').replace('sqrt(', 'sqrtf(')
    body=body.replace('.999999999999','.999999')
    body=re.sub(r'float\(([^()]*)\)',r'(\1)',body)
    body=body.replace('max(1e-24,','max(1e-24f,').replace('max(0.,','max(0.f,')
    body=re.sub(r'(?<![\w.])(\d*\.\d+|\d+\.)(?![\w.])',lambda m:m[0]+'f',body)
    text='#pragma once\n// Source-derived float support; see provenance/ovoid-support.json.\n#include "state.hpp"\nnamespace malemod::physics {\n'+body+'\n}\n'
    path=ROOT/'include/malemod/physics/ovoid_support.hpp'

    report=dict(source=str(source.relative_to(ROOT)),sourceSHA256=hashlib.sha256(source.read_bytes()).hexdigest(),
     file=str(path.relative_to(ROOT)),sha256=hashlib.sha256(text.encode()).hexdigest(),
     changes=['float Newton arithmetic for backends without double','finite pole clamp 0.999999'],
     fullCoupledPhysicsParity=False)
    return {str(path.relative_to(ROOT)):text,'tests/data/wolverine-ovoid-support.inc':reference,'provenance/ovoid-support.json':json.dumps(report,indent=2)+'\n'}

if __name__=='__main__':
    import sys
    for path,text in outputs().items():
        if '--check' in sys.argv:
            if (ROOT/path).read_bytes()!=text.encode():raise ValueError('Ovoid extraction differs: '+path)
        else:(ROOT/path).write_text(text,newline='\n')
    print('Verified source ovoid support' if '--check' in sys.argv else 'Extracted source ovoid support')
