"""Extract the active source motion filter without its clock/diagnostic APIs."""
import argparse,hashlib,json,re
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
SOURCE=ROOT/'legacy/wolverine/src/runtime/d3d9_proxy.cpp'
HEADER=ROOT/'include/malemod/motion_filter.hpp'
sha=lambda data:hashlib.sha256(data).hexdigest()


def generate():
    raw=SOURCE.read_bytes();text=raw.decode('utf-8')
    start=text.index('static void TrackCharacterMotionMatrices(')
    end=text.index('static void CaptureCharacterMotion(',start)
    function=text[start:end].strip()
    normal=text[text.index('static void Normalize3('):text.index('static float SoftDeadzone(')].strip()
    declarations=re.search(r'^static bool motionTracked,.*$',text,re.M).group(0)+'\n'+re.search(r'^static float motionPelvisMatrix\[12\].*$',text,re.M).group(0)+'\nstatic bool motionCollisionBonesReady;'
    names=list(dict.fromkeys(re.findall(r'\bmotion\w+',declarations)))
    state=declarations.replace('static ','').replace('DWORD','std::uint32_t').replace('LONG','std::int32_t')
    body=function[function.index('{')+1:function.rfind('}')]
    body=re.sub(r'^  if\(InterlockedCompareExchange.*\n','',body,flags=re.M)
    body=body.replace('DWORD now=GetTickCount();','')
    expected='LONG sample=InterlockedIncrement(&motionSamples);'
    if expected not in body:raise ValueError('Source sample publication changed')
    body=body[:body.index(expected)]+'++motionSamples;\n'
    for name in names:body=re.sub(r'\b'+name+r'\b','state_.'+name,body)
    header='''// Generated from the immutable active Wolverine filter. Do not hand tune.
#pragma once
#include <array>
#include <cstdint>
#include <cmath>
#include <cstring>
#include <algorithm>
#include <stdexcept>
namespace malemod::motion {
class Tracker {
 public:
 using Matrix=std::array<float,12>; // row-major affine skin delta, model space
 struct State {
'''+state+'''
 };
 void Reset(){state_=State{};}
 const State& Read()const noexcept{return state_;}
 // Force-only input never advertises unavailable thigh skin matrices.
 void TrackPoseOnly(std::uint32_t now,const Matrix& pelvis){
  const Matrix unavailable{};Track(now,pelvis,unavailable,unavailable);state_.motionCollisionBonesReady=false;
 }
 void Track(std::uint32_t now,const Matrix& pelvis,const Matrix& left,const Matrix& right){
  for(const auto* matrix:{&pelvis,&left,&right})for(float x:*matrix)if(!std::isfinite(x))throw std::invalid_argument("Non-finite motion matrix");
  const auto* bone=pelvis.data();const auto* leftThigh=left.data();const auto* rightThigh=right.data();
  using std::min;using std::max;
'''+body+''' }
 private:
 State state_{};
'''+normal+'''
};
}
'''
    oracle='''// Original function body with deterministic SDK diagnostic/clock stubs.
namespace original_motion {
using DWORD=std::uint32_t;using LONG=std::int32_t;
static DWORD oracleNow=0;
static DWORD GetTickCount(){return oracleNow;}
static LONG motionBoneLogged=0;
static LONG InterlockedCompareExchange(LONG* value,LONG set,LONG expected){auto old=*value;if(old==expected)*value=set;return old;}
static LONG InterlockedIncrement(LONG* value){return ++*value;}
static void Log(const char*,...){}
using std::min;using std::max;
'''+declarations+'\n'+normal+'\n'+function+'\n'+'''inline malemod::motion::Tracker::State Read(){malemod::motion::Tracker::State state{};
'''+''.join(f'std::memcpy(&state.{name},&{name},sizeof({name}));\n' for name in names)+'''return state;}
inline void Reset(){oracleNow=0;motionBoneLogged=0;
'''+''.join(f'std::memset(&{name},0,sizeof({name}));\n' for name in names)+''' }
inline bool Equal(const malemod::motion::Tracker::State& a,const malemod::motion::Tracker::State& b){return
'''+ '&&\n'.join(f'!std::memcmp(&a.{name},&b.{name},sizeof(a.{name}))' for name in names)+''';}
}
'''
    return raw,function,header.encode(),oracle.encode()


def run(write=False):
    raw,function,header,oracle=generate()
    if write:HEADER.write_bytes(header)
    elif not HEADER.exists() or HEADER.read_bytes()!=header:raise ValueError('Motion header differs from source extraction')
    directory=ROOT/'build/motion-filter';directory.mkdir(parents=True,exist_ok=True)
    (directory/'original-reference.inc').write_bytes(oracle)
    result=dict(contractVersion=1,sourceCommit=json.loads((ROOT/'provenance/wolverine.json').read_text())['commit'],
        sourcePath=SOURCE.relative_to(ROOT).as_posix(),sourceSHA256=sha(raw),sourceFunctionSHA256=sha(function.encode()),
        headerSHA256=sha(header),recipeSHA256=sha(Path(__file__).read_bytes()),oracleSHA256=sha(oracle),
        changes=['Clock supplied as uint32 milliseconds','Diagnostics removed','Per-instance source state','Nonfinite inputs rejected before state changes'],
        numericalExpressionsUnchanged=True,actorLocalToWorldExcluded=True,installed=False,observedGameplay=False)
    if write:(ROOT/'provenance/motion-filter.json').write_text(json.dumps(result,indent=2)+'\n')
    else:
        existing=json.loads((ROOT/'provenance/motion-filter.json').read_text())
        if any(existing.get(k)!=v for k,v in result.items()):raise ValueError('Motion provenance differs')
    print('Source motion header and original reference verified')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--write',action='store_true');run(p.parse_args().write)
