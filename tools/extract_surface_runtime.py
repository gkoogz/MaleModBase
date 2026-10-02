"""Generate the SDK-free source surface kernel from checked declaration spans.

The immutable snapshot is a provenance input, not a directly compiled library.
Numerical expressions and topology remain source-derived; session isolation,
serial scheduling, inactive sequence projection and half conversion are explicit.
"""
import argparse,json,re
from pathlib import Path
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--output',type=Path)
parser.add_argument('--check',action='store_true')
parser.add_argument('--verify-provenance',action='store_true')
parser.add_argument('--process-isolated',action='store_true',help='One character per process; restore source parallel geometry')
parser.add_argument('--serial-geometry',action='store_true',help='Diagnostic only: isolate global storage from source scheduling')
args=parser.parse_args()
if args.serial_geometry and not args.process_isolated:parser.error('--serial-geometry requires --process-isolated')
root=Path(__file__).resolve().parents[1]
runtime=root/'legacy/wolverine/src/runtime'
data=json.loads((root/'tools/data/surface-closure.json').read_text())
import hashlib
for item in data['sources']:
 if hashlib.sha256((root/item['path']).read_bytes()).hexdigest()!=item['sha256']:raise ValueError('Surface source changed: '+item['path'])
for d in data['declarations']:
 d['source']=(root/d['file']).read_bytes()[d['start']:d['end']].decode()
# Original textual include order, never alphabetical header order.
order={};rank=0;seen=set()
def visit(path):
 global rank
 if path in seen:return
 seen.add(path);last=0
 raw=path.read_bytes()
 for m in re.finditer(rb'^\s*#include\s+"([^"]+)"',raw,re.M):
  order[(path.relative_to(root).as_posix(),last,m.start())]=rank;rank+=1
  target=path.parent/m.group(1).decode()
  if target.exists() and target.resolve().is_relative_to(runtime):visit(target.resolve())
  last=m.end()
 order[(path.relative_to(root).as_posix(),last,len(raw)+1)]=rank;rank+=1
visit(runtime/'d3d9_proxy.cpp')
def position(d):
 for (file,a,b),v in order.items():
  if file==d['file'] and a<=d['start']<b:return (v,d['start'])
 return (100000,d['start'])
decl=[]
skip={'Log','PerfScope','PerfClock','GeometryFor','GeometryScheduler','teachingFluid','ApplyControlMapping'}
for d in data['declarations']:
 if d['kind'] not in ['CursorKind.VAR_DECL','CursorKind.FUNCTION_DECL','CursorKind.STRUCT_DECL','CursorKind.CLASS_TEMPLATE','CursorKind.TYPE_ALIAS_DECL']:continue
 if d['parent'] and d['parent'] not in ['UnifiedCollar','teaching','translation-unit']:continue
 if d['parent']=='teaching':continue
 if d['name'] in skip or 'teaching' in Path(d['file']).name or 'fluid' in Path(d['file']).name:continue
 decl.append(d)
# Same-start variables and template instantiations share source declarations.
groups={}
for d in decl:
 key=(d['file'],d['start'])
 if key not in groups or d['end']>groups[key]['end']:groups[key]=d
decl=list(groups.values())
# Exclude declarations nested inside another selected definition.
decl=[d for d in decl if not any(p['file']==d['file'] and p['start']<d['start'] and p['end']>=d['end'] for p in decl)]
# Preserve the original forward declarations, which precede their definitions.
names={d['name'] for d in decl if d['kind']=='CursorKind.FUNCTION_DECL'}
for file in sorted({d['file'] for d in decl}):
 raw=(root/file).read_bytes()
 for m in re.finditer(rb'^static [^\r\n{};]+\([^\r\n{};]*\);',raw,re.M):
  if any(re.search(rb'\b'+re.escape(name.encode())+rb'\s*\(',m.group()) for name in names):
   decl.append(dict(file=file,start=m.start(),end=m.end(),name='',kind='forward',source=m.group().decode(),parent=''))
prelude='''// Generated full numerical kernel; see tools/extract_surface_runtime.py.
#include <cstdint>
#include <cstring>
#include <cmath>
#include <vector>
#include <algorithm>
#include <xmmintrin.h>
#include <Eigen/SparseCholesky>
#include <Eigen/Geometry>
#ifndef _MSC_VER
#define __forceinline inline
#define _finite std::isfinite
#endif
namespace malemod::surface::source {
using UINT=unsigned; using LONG=long;
using std::min;using std::max;
using D3DXFLOAT16=uint16_t;
static void D3DXFloat16To32Array(float* dst,const uint16_t* src,unsigned n){
 for(unsigned i=0;i<n;i++){uint32_t h=src[i],sign=(h&0x8000)<<16,e=(h>>10)&31,m=h&1023,bits;
  if(e==0){if(m==0)bits=sign;else{int exp=-14;while(!(m&1024)){m<<=1;--exp;}bits=sign|((exp+127)<<23)|((m&1023)<<13);}}
  else bits=sign|((e==31?255:e+112)<<23)|(m<<13);
  memcpy(dst+i,&bits,4);
 }
}
static void D3DXFloat32To16Array(uint16_t* dst,const float* src,unsigned n){
 for(unsigned i=0;i<n;i++){uint32_t bits;memcpy(&bits,src+i,4);unsigned sign=(bits>>16)&0x8000,m=bits&0x7fffff;int e=((bits>>23)&255)-112;
  if(e>=31)dst[i]=uint16_t(sign|0x7c00|(m?0x200:0));
  else if(e<=0){if(e<-10)dst[i]=uint16_t(sign);else{m|=0x800000;unsigned shift=14-e,rounded=(m+((1u<<(shift-1))-1)+((m>>shift)&1))>>shift;dst[i]=uint16_t(sign|rounded);}}
  else{m+=0xfff+((m>>13)&1);if(m&0x800000){m=0;++e;}dst[i]=uint16_t(sign|(e<<10)|(m>>13));}
 }
}
static LONG InterlockedExchange(volatile LONG* p,LONG value){LONG old=*p;*p=value;return old;}
static void Log(const char*,...){}
struct PerfScope { explicit PerfScope(int){} };
static double PerfClock(){return 0;}
template<class F>static void GeometryFor(unsigned n,const F& f){for(unsigned i=0;i<n;i++)f(i);}
namespace teaching { struct DisabledTimeline {bool active=false;double time=0;struct Sample{float firm=0;};Sample Get()const{return {};}}; }
'''
parts=[prelude]
if args.process_isolated:
 # Process isolation lets independent geometry workers see the one character's
 # shared state, exactly as in the source. Constraint/reduction order is unchanged.
 geometry=(runtime/'geometry_pass.h').read_text()
 parallel=geometry[geometry.index('static Concurrency::Scheduler* GeometryScheduler()'):geometry.index('// Prepared arithmetic')]
 if 'constexpr unsigned batches=16' not in parallel or 'template<class Function>' not in parallel:raise ValueError('Source bounded geometry dispatcher changed')
 prelude=prelude.replace('#include <Eigen/Geometry>','#include <Eigen/Geometry>\n#include <ppl.h>')
 prelude=prelude.replace('template<class F>static void GeometryFor(unsigned n,const F& f){for(unsigned i=0;i<n;i++)f(i);}',parallel)
 if args.serial_geometry:
  prelude=prelude.replace(parallel,'template<class F>static void GeometryFor(unsigned n,const F& f){for(unsigned i=0;i<n;i++)f(i);}')
 prelude=prelude.replace('#include <ppl.h>','#include <ppl.h>\n#include <chrono>')
 prelude=prelude.replace('struct PerfScope { explicit PerfScope(int){} };', '''static double surfaceGeometryMilliseconds[16]{};
struct PerfScope {
 int id;std::chrono::steady_clock::time_point begin;
 explicit PerfScope(int value):id(value),begin(std::chrono::steady_clock::now()){}
 ~PerfScope(){surfaceGeometryMilliseconds[id]+=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();}
};''')
 parts=[prelude]
for d in sorted(decl,key=position):
 s=d['source'].replace('\r\n','\n').replace('\r','')
 if d['kind']=='CursorKind.FUNCTION_DECL':
  raw=(root/d['file']).read_bytes();start=raw.rfind(b'\n',0,d['start'])+1;prefix=raw[start:d['start']].decode()
  if prefix.startswith('template<'):s=prefix+s
 if d['name']=='teachingTimeline':s='static teaching::DisabledTimeline teachingTimeline'
 if d['name']=='LiveRootDirection':s=re.sub(r'\s*if\(teachingTimeline.active\)yaw\+=teachingFluid.MainLateralYaw\(teachingTimeline.time\);','',s)
 if d['name']=='PDInput':
  storage='static ' if args.process_isolated else 'static thread_local '
  parts.append(storage+'bool surfaceCollisionEnabled=false;\n'+storage+'float surfaceThighRadii[2]{7.2f,7.2f},surfacePelvisRadius=6.4f,surfaceTargetThighRadii[2]{},surfaceTargetPelvisRadius=0;\n'+storage+'V3 surfacePelvis[2]{{3.f,0.f,70.f},{5.4f,0.f,86.f}},surfaceOldPelvis[2]{},surfaceTargetPelvis[2]{};')
  s=s.replace('float gait,side;', 'float gait,side; V3 surfacePelvis[2];float surfaceThighRadii[2],surfacePelvisRadius;')
 if d['name']=='Build' and d['parent']=='UnifiedCollar':
  # Observe the actual cache rebuild; do not duplicate its invalidation policy
  # in an adapter or modify the source numerical expressions.
  storage='static ' if args.process_isolated else 'static thread_local '
  parts.append('namespace UnifiedCollar {\n'+storage+'float surfaceMetricFrame[11]{};\n'+storage+'uint32_t surfaceMetricGeneration=0;\n}')
  at=s.find('{')+1
  s=s[:at]+'''\n const float observed[11]={root.x,root.y,root.z,axis.x,axis.y,axis.z,up.x,up.y,up.z,radius,length};
 memcpy(surfaceMetricFrame,observed,sizeof(observed));++surfaceMetricGeneration;
'''+s[at:]
 if d['name']=='PDReadInput':
  s=s.replace('return x;', 'for(unsigned i=0;i<2;i++){x.surfacePelvis[i]=surfaceCollisionEnabled?surfaceTargetPelvis[i]:(i?V3{5.4f,0.f,86.f}:V3{3.f,0.f,70.f});x.surfaceThighRadii[i]=surfaceCollisionEnabled?surfaceTargetThighRadii[i]:7.2f;}x.surfacePelvisRadius=surfaceCollisionEnabled?surfaceTargetPelvisRadius:6.4f;return x;')
 if d['name']=='PDSetInput':
  at=s.rfind('}')
  s=s[:at]+'for(unsigned i=0;i<2;i++){surfaceOldPelvis[i]=surfacePelvis[i];surfacePelvis[i]=a.surfacePelvis[i]+(b.surfacePelvis[i]-a.surfacePelvis[i])*t;surfaceThighRadii[i]=a.surfaceThighRadii[i]+(b.surfaceThighRadii[i]-a.surfaceThighRadii[i])*t;}surfacePelvisRadius=a.surfacePelvisRadius+(b.surfacePelvisRadius-a.surfacePelvisRadius)*t;\n'+s[at:]
 if d['name']=='UpdateCompliantDynamics':
  s=s.replace('memcpy(pdThigh,target.thigh,sizeof(pdThigh));', 'memcpy(pdThigh,target.thigh,sizeof(pdThigh));memcpy(surfacePelvis,target.surfacePelvis,sizeof(surfacePelvis));')
 if d['name']=='StepConstraintSolver':
  # Optional measured collision envelope. Keep the original expressions in the
  # reference branch so its strict floating-point replay is unchanged.
  substitutions={
   'pdOldThigh[j*2+1],7.2f,dt)':
    'pdOldThigh[j*2+1],surfaceCollisionEnabled?surfaceThighRadii[j]:7.2f,dt)',
   'PDBodyCapsule(pelvis[s],s,{3.f,0.f,70.f},{5.4f,0.f,86.f},{3.f,0.f,70.f},{5.4f,0.f,86.f},6.4f,dt)':
    'PDBodyCapsule(pelvis[s],s,surfaceCollisionEnabled?surfacePelvis[0]:V3{3.f,0.f,70.f},surfaceCollisionEnabled?surfacePelvis[1]:V3{5.4f,0.f,86.f},surfaceCollisionEnabled?surfaceOldPelvis[0]:V3{3.f,0.f,70.f},surfaceCollisionEnabled?surfaceOldPelvis[1]:V3{5.4f,0.f,86.f},surfaceCollisionEnabled?surfacePelvisRadius:6.4f,dt)',
   'Length(pdPosition[i]-q)-7.2f-logicalShaftBodyRadius*.85f':
    'Length(pdPosition[i]-q)-(surfaceCollisionEnabled?surfaceThighRadii[j]:7.2f)-logicalShaftBodyRadius*.85f',
  }
  for before,after in substitutions.items():
   if s.count(before)!=1:raise ValueError('Source character collision span changed: '+before)
   s=s.replace(before,after)
 # Mutable numerical state belongs to a session's worker thread. Immutable
 # source tables remain shared. Function-local caches require the same rule.
 if not args.process_isolated and d['kind']=='CursorKind.VAR_DECL' and not re.match(r'static\s+(?:const|constexpr)\b',s):
  s=s.replace('static ','static thread_local ',1)
 if not args.process_isolated and d['kind']=='CursorKind.FUNCTION_DECL':
  brace=s.find('{');s=s[:brace+1]+re.sub(r'\bstatic\s+(?!const\b|constexpr\b)', 'static thread_local ',s[brace+1:])
 if d['kind'] not in ['CursorKind.FUNCTION_DECL','forward']:s+=';'
 if d['parent']=='UnifiedCollar':s='namespace UnifiedCollar {\n'+s+'\n}'
 parts.append(s)
output=args.output or root/'build/surface-runtime/surface-kernel.inc'
output.parent.mkdir(parents=True,exist_ok=True)
content='\n'.join(parts)+'\n} // namespace malemod::surface::source\n'
if args.verify_provenance:
 if args.process_isolated:raise ValueError('Process variant uses its own measured replay provenance')
 report=json.loads((root/'provenance/source-surface.json').read_text())
 if report['sourceCommit']!=data['sourceCommit']:raise ValueError('Surface source revision differs')
 for item in report['files']:
  if hashlib.sha256((root/item['path']).read_bytes()).hexdigest()!=item['sha256']:raise ValueError('Surface implementation provenance differs: '+item['path'])
 expected=report['generatedKernelSHA256']
 if hashlib.sha256(content.encode()).hexdigest()!=expected:raise ValueError('Generated surface provenance differs')
elif args.check:
 if output.read_bytes()!=content.encode():
  previous=output.read_bytes();actual=content.encode()
  at=next((i for i,(a,b) in enumerate(zip(previous,actual)) if a!=b),min(len(previous),len(actual)))
  raise ValueError('Generated surface kernel differs at byte '+str(at)+': '+repr(actual[max(0,at-60):at+120]))
else:output.write_text(content,encoding='utf-8',newline='\n')
print('Emitted',len(decl),'declarations',sum(map(len,parts)),'bytes')
