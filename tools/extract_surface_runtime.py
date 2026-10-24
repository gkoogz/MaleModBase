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
#include <malemod/surface/pelvic_frame.hpp>
#include <malemod/surface/scrotal_compliance.hpp>
#include <malemod/surface/garment_support.hpp>
#include <malemod/surface/garment_impulse.hpp>
#include <malemod/clinical/teaching_sequence.h>
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
static thread_local float surfaceClinicalLateralYaw=0;
static thread_local float surfaceAttachmentAnchor[3]={9.0f,0.f,84.3f};
static thread_local bool surfaceTargetOwnsCollar=false;
namespace teaching {
using ::malemod::clinical::teaching::WeightedMainPulse;
using ::malemod::clinical::teaching::PulseEnvelopeAt;
using ::malemod::clinical::teaching::Ease;
struct ProjectedTimeline {bool active=false;double time=0;::malemod::clinical::teaching::Sample Get()const{::malemod::clinical::teaching::Timeline t;t.active=active;t.time=time;return t.Get();}};
}
'''
parts=[prelude]
if args.process_isolated:
 # Process isolation lets independent geometry workers see the one character's
 # shared state, exactly as in the source. Constraint/reduction order is unchanged.
 geometry=(runtime/'geometry_pass.h').read_text()
 parallel=geometry[geometry.index('static Concurrency::Scheduler* GeometryScheduler()'):geometry.index('// Prepared arithmetic')]
 if 'constexpr unsigned batches=16' not in parallel or 'template<class Function>' not in parallel:raise ValueError('Source bounded geometry dispatcher changed')
 prelude=prelude.replace('#include <Eigen/Geometry>','#include <Eigen/Geometry>\n#include <ppl.h>')
 prelude=prelude.replace('static thread_local float surfaceClinicalLateralYaw','static float surfaceClinicalLateralYaw')
 prelude=prelude.replace('static thread_local float surfaceAttachmentAnchor','static float surfaceAttachmentAnchor')
 prelude=prelude.replace('static thread_local bool surfaceTargetOwnsCollar','static bool surfaceTargetOwnsCollar')
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
 if d['name'] in ('RestBallAnchor','BallAnchor','PDMaterialTarget','PDSuspensionMassFiniteDifference','PDSuspensionMass','PDVentralHeight'):
  s=s.replace('.12f','::malemod::surface::scrotal::MechanicalPartition::suspensionCoordinate')
 if d['name']=='ResetCompliantDynamics':
  storage='static ' if args.process_isolated else 'static thread_local '
  parts.append('#ifdef MALEMOD_COMPLIANT_SCROTAL_SKIN\n'+storage+'::malemod::surface::scrotal::Envelope surfaceScrotalSkin[2];\n'+storage+'float surfaceSkinFraction=1,surfaceSkinMaximum=0,surfaceSkinProtectedError=0;\n#endif')
  s=s.replace('pdReady=false;','''pdReady=false;
#ifdef MALEMOD_COMPLIANT_SCROTAL_SKIN
 for(auto& skin:surfaceScrotalSkin)skin.Reset();
#endif
''')
 if d['name']=='PDSuspensionShear':
  old='alpha=compliance/(dt*dt);'
  if s.count(old)!=1:raise ValueError('Suspension effective mass span changed')
  s=s.replace(old,'''alpha=compliance/(dt*dt);
#ifdef MALEMOD_COMPLIANT_SCROTAL_SKIN
  compliance=::malemod::surface::scrotal::ShearCompliance(w,physUI[4]);
  alpha=compliance/(dt*dt);
#endif''')
 if d['name']=='StepConstraintSolver':
  marker='PDBendData bendData[shaftNodeCount];'
  if s.count(marker)!=1:raise ValueError('Solid rod compliance entry changed')
  s=s.replace(marker,'if(surfaceTargetOwnsCollar)bendCompliance*=::malemod::surface::SolidRodComplianceScale(physicsState);\n '+marker)
 if d['name']=='PDPrepareBend':
  old='float ratio=.10f+.40f*(1.f-max(0.f,min(1.f,physUI[2]/100.f)));'
  if s.count(old)!=1:raise ValueError('Rod damping span changed')
  s=s.replace(old,'''#ifdef MALEMOD_COMPLIANT_SCROTAL_SKIN
 float ratio=.05f+.25f*(1.f-max(0.f,min(1.f,physUI[2]/100.f)));
#else
 '''+old+'''
#endif''')
 if d['name']=='StepRootSuspension':
  old='float k=ModeValue(38.f,22.f,10.f),damping=2.f*sqrtf(k*mass)*ModeValue(.60f,.72f,.86f);'
  if s.count(old)!=1:raise ValueError('Root suspension damping span changed')
  s=s.replace(old,'''#ifdef MALEMOD_COMPLIANT_SCROTAL_SKIN
  // The root previously ignored the bounce control and remained almost
  // critically damped in the flexible state. Retain stiffness, mass and stops.
  float ratio=.20f+.60f*(1.f-max(0.f,min(1.f,physUI[2]/100.f)));
  float k=ModeValue(38.f,22.f,10.f),damping=2.f*sqrtf(k*mass)*ratio;
#else
  '''+old+'''
#endif''')
 if d['name']=='PDSuspensionShear':
  old='float gamma=2.1f*sqrtf(compliance/max(w,1e-8f))/dt;'
  if s.count(old)!=1:raise ValueError('Scrotal shear source span changed')
  s=s.replace(old,'''#ifdef MALEMOD_COMPLIANT_SCROTAL_SKIN
  float gamma=::malemod::surface::scrotal::Response(physUI[4],physUI[6]).shearDamping*sqrtf(compliance/max(w,1e-8f))/dt;
#else
  '''+old+'''
#endif''')
 if d['name']=='ApplyPouchSurface':
  marker=' for(unsigned i=0;i<r14Count;i++)r14Positions[i]=rsPositions[i];'
  if s.count(marker)!=1:raise ValueError('Pouch publication span changed')
  s=s.replace(marker,'''#ifdef MALEMOD_COMPLIANT_SCROTAL_SKIN
 // Immutable donor ownership separates external skin from the hidden rod.
 // Fine vertices inherit the same donor field, never a moving spatial test.
 static float skinMask[rsCount],skinRight[rsCount];static bool skinReady=false;
 if(!skinReady){
  for(unsigned i=0;i<r14Count;i++){
   float ball=0,shaft=0;
   for(unsigned j=r14Offsets[i];j<r14Offsets[i+1];j++){
    unsigned id=r14Sources[j];float weight=r14Weight[j];
    ball+=weight*phys_scrotum_weight[id];shaft+=weight*max(phys_shaft_weight[id],phys_attachment_weight[id]);
   }
   skinMask[i]=::malemod::surface::scrotal::SkinOwnership(ball,shaft);
   skinRight[i]=Smoother01(.5f+r14Base[3*i+1]/1.4f);
  }
  for(unsigned k=0;k<rsFineCount;k++)for(unsigned j=0;j<3;j++){
   unsigned id=rsFineSource[k*3+j];float w=rsFineBary[k*3+j];
   skinMask[r14Count+k]+=w*skinMask[id];skinRight[r14Count+k]+=w*skinRight[id];
  }
  skinReady=true;
 }
 static V3 skinBefore[rsCount],skinDelta[rsCount];memcpy(skinBefore,rsPositions,sizeof(skinBefore));
 auto left=surfaceScrotalSkin[0].Displacement(),right=surfaceScrotalSkin[1].Displacement();
 for(unsigned i=0;i<rsCount;i++){
  float t=skinRight[i],w=skinMask[i];
  rsPositions[i]=rsPositions[i]+V3{left[0]*(1-t)+right[0]*t,left[1]*(1-t)+right[1]*t,left[2]*(1-t)+right[2]*t}*w;
 }
 CPKeepSkinOutside();
 for(unsigned i=0;i<rsCount;i++)skinDelta[i]=skinMask[i]>0?rsPositions[i]-skinBefore[i]:V3{};
 // Prevent fold/collapse against the pre-secondary-motion surface. No seam,
 // shaft-owned or body vertex receives an independent skin displacement.
 float skinFraction=1.f;
 for(unsigned k=0;k<rsIndexCount;k+=3){unsigned a=rsIndices[k],b=rsIndices[k+1],c=rsIndices[k+2];
  skinFraction=min(skinFraction,RSSafeFraction(skinBefore[a],skinBefore[b],skinBefore[c],skinDelta[a],skinDelta[b],skinDelta[c],.5f));
 }
 surfaceSkinFraction=skinFraction;surfaceSkinMaximum=surfaceSkinProtectedError=0;
 for(unsigned i=0;i<rsCount;i++){
  rsPositions[i]=skinBefore[i]+skinDelta[i]*skinFraction;
  float change=Length(rsPositions[i]-skinBefore[i]);surfaceSkinMaximum=max(surfaceSkinMaximum,change);
  if(skinMask[i]==0)surfaceSkinProtectedError=max(surfaceSkinProtectedError,change);
 }
#endif
'''+marker)
 if d['name']=='ShaftRoot':
  if s!='static V3 ShaftRoot(){return {9.0f,0.f,84.3f};}':raise ValueError('Source attachment root span changed')
  s='static V3 ShaftRoot(){return {surfaceAttachmentAnchor[0],surfaceAttachmentAnchor[1],surfaceAttachmentAnchor[2]};}'
 if d['name']=='ApplyShaftPoseAngle':
  old_follow='if(bw<.05f&&membership>.02f)active=1.f+(pelvicAngleFollow[i]-1.f)*preparedPelvicRampBlend;'
  if s.count(old_follow)!=1:raise ValueError('Source pelvis angle follow changed')
  s=s.replace(old_follow,old_follow+'\n  if(surfaceTargetOwnsCollar)active+=(1.f-active)*SurfaceDorsalRelease(i);')
  s='''static float SurfaceDorsalRelease(UINT i){
 const float z=morph_base[i*3+2]-84.3f,y=morph_base[i*3+1],r=sqrtf(y*y+z*z);
 return Smoother01((neutralShape[4]-sliderValues[4])/45.f)*Smoother01(((r>1e-6f?z/r:0)+.3f)/.8f)*(1-Smoother01(phys_scrotum_weight[i]/.2f));
}
'''+s
  at='if(active<=.001f)return;'
  if s.count(at)!=1:raise ValueError('Source angle ownership span changed')
  # Rebase the authored material BEFORE rotation; the same root then anchors
  # the rest guide, constraints and source collar frame. This is opt-in and
  # bypassed arithmetically at the original source anchor.
  rebase='''const auto anchor=ShaftRoot();
  if(anchor.x!=9.0f||anchor.y!=0.f||anchor.z!=84.3f){
   value[0]+=(anchor.x-9.0f)*active;
   value[1]+=anchor.y*active;
   value[2]+=(anchor.z-84.3f)*active;
  }
  '''
  s=s.replace(at,rebase+at)
 if d['name']=='BuildShaftRestFrame':
  if s.count('center.y=0.f;')!=1:raise ValueError('Source guide lateral anchor span changed')
  s=s.replace('center.y=0.f;','center.y=ShaftRoot().y;')
  marker='  for(UINT i=0;i<graftCount;i++){\n    graftRestFlex[i]=ClosestRestShaftFlex(graftDeformedPositions[i]);'
  if s.count(marker)!=1:raise ValueError('Source rest material station entry changed')
  s=s.replace(marker,'''  if(physicsState==0&&surfaceTargetOwnsCollar&&sliderUI[1]<50.f){
    const float blend=SurfaceShortMaterialBlend();const V3 root=ShaftRoot(),span=shaftRestCenters[shaftRestSampleCount-1]-root;
    for(int i=1;i<shaftRestSampleCount;i++)shaftRestCenters[i]=shaftRestCenters[i]*(1.f-blend)+(root+span*(float(i)/(shaftRestSampleCount-1)))*blend;
  }
'''+marker)
  s='static float SurfaceShortMaterialBlend(){return surfaceTargetOwnsCollar?Smoother01((50.f-sliderUI[1])/50.f):0.f;}\n'+s
 if d['name']=='UpdateR14':
  start=s.index('  if(sliderUI[1]<50.f){')
  end=s.index('  // Fair only the proximal ventral transition.',start)
  original=s[start:end]
  corrected=original+'''  if(surfaceTargetOwnsCollar&&sliderUI[1]<50.f){
    // The independent head is an isotropic material body. Length may shorten
    // the shaft, but must not flatten the crown through an anisotropic donor
    // cage. Reuse its authored glans controls, exact topology and root seam.
    static V3 material[r14Count];
    for(UINT i=r14NewStart;i<r14Count;i++)material[i]=base[i]+V3{r14LowerDelta[3*i],r14LowerDelta[3*i+1],r14LowerDelta[3*i+2]}*lowerFactor;
    V3 axis{},center{};SampleShaftChain(.90f,center,axis);
    ::malemod::surface::RestoreCrownMaterial(r14Positions,material,r14Count,r14NewStart,r14RingCount,r14SegmentCount,r14CrownRing,axis,SurfaceShortMaterialBlend());
  }
'''
  s=s[:start]+corrected+s[end:]
 if d['name']=='ApplyConstraintCurve':
  marker='t=t*(1.f-distal)+graftRestFlex[i]*distal;'
  if s.count(marker)!=1:raise ValueError('Source material station span changed')
  s=s.replace(marker,marker+'\n  const float materialBlend=SurfaceShortMaterialBlend();if(materialBlend>0.f){t+=(graftRestFlex[i]-t)*materialBlend;distal+=(1.f-distal)*materialBlend;}')
 if d['name']=='CaptureLogicalShaftSurface':
  marker='logicalShaftRestRadial[i]=offset-tangent*Dot(offset,tangent);'
  if s.count(marker)!=1:raise ValueError('Source material residual span changed')
  s=s.replace(marker,'logicalShaftRestRadial[i]=offset-tangent*(Dot(offset,tangent)*(1.f-SurfaceShortMaterialBlend()));')
 if d['name']=='FlareAttachment':
  if s.count('value[1]*=scale;')!=1:raise ValueError('Source radial origin span changed')
  s=s.replace('value[1]*=scale;','if(root.y==0.f)value[1]*=scale;else value[1]=root.y+(value[1]-root.y)*scale;')
 if d['name']=='BuildPreparedShape':
  shortening='value[axis]+=delta*(s==3?pouchOwner:1.f);'
  if s.count(shortening)!=1:raise ValueError('Source length composition span changed')
  s=s.replace(shortening,'if(surfaceTargetOwnsCollar&&s==1&&v<spec.def)delta*=OverallShapeScale();\n        '+shortening)
  parts.append('static V3 surfaceMorphSnapshot[graftCount],surfaceFairSnapshot[graftCount];')
  s=s.replace('  FairUnifiedCollar(controlled,graftFirstVertex,collarGrowth);','  memcpy(surfaceMorphSnapshot,graftDeformedPositions,sizeof(surfaceMorphSnapshot));\n  FairUnifiedCollar(controlled,graftFirstVertex,collarGrowth);\n  memcpy(surfaceFairSnapshot,graftDeformedPositions,sizeof(surfaceFairSnapshot));')
 if d['kind']=='CursorKind.FUNCTION_DECL':
  raw=(root/d['file']).read_bytes();start=raw.rfind(b'\n',0,d['start'])+1;prefix=raw[start:d['start']].decode()
  if prefix.startswith('template<'):s=prefix+s
 if d['name']=='teachingTimeline':s='static teaching::ProjectedTimeline teachingTimeline'

 if d['name']=='PDInput':
  storage='static ' if args.process_isolated else 'static thread_local '
  parts.append(storage+'bool surfaceGarmentEnabled=false,surfaceGarmentContactReaction=false,surfaceGarmentPendingReady=false;\n'+storage+'V3 surfaceGarmentShaft{},surfaceGarmentLobes[2]{},surfaceGarmentPendingRod[12]{},surfaceGarmentPendingLobes[2]{},surfaceGarmentPendingAngular[2]{};\n'+storage+'::malemod::surface::GarmentImpulseCursor surfaceGarmentCursor;')
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
  old='write[ucSeamVertices[k*3]]=write[ucSeamVertices[k*3+1]]||write[ucSeamVertices[k*3+2]];'
  if s.count(old)!=1:raise ValueError('Original-edge seam publication source span changed')
  # The eliminated slave is corrected even when both donors are fixed. Always
  # publish its aliases, otherwise an earlier body pass can leave a visible gap.
  s=s.replace(old,'write[ucSeamVertices[k*3]]=true;')
 if d['name']=='Apply' and d['parent']=='UnifiedCollar':
  if s.count('if(!valid)return;')!=1:raise ValueError('Source collar metric/solve boundary changed')
  # Keep the measured metric/cache available to the target, but avoid a
  # second solve against the reference character's body and seam.
  old='Topology();Read(body);V3 root,axis;SampleShaftChain(0,root,axis);V3 up=Unit(Cross(axis,{0,1,0}));'
  new='Topology();Read(body);const auto pelvic=StablePelvicRecruitmentFrame();V3 root{pelvic.root.x,pelvic.root.y,pelvic.root.z},axis{pelvic.axis.x,pelvic.axis.y,pelvic.axis.z},up{pelvic.up.x,pelvic.up.y,pelvic.up.z};'
  if s.count(old)!=1:raise ValueError('Observed collar frame source span changed')
  # Intentional shared algorithm revision: pelvic support no longer rotates
  # with RestAngle/live swing. The immutable source remains unchanged.
  s=s.replace(old,new)
  s='''static ::malemod::collar::RecruitmentFrame StablePelvicRecruitmentFrame(){
 const auto root=ShaftRoot();
 const bool calibrated=root.x!=9.0f||root.y!=0.f||root.z!=84.3f;
 const float angle=(calibrated?sliderValues[4]:neutralShape[4])*3.1415926535f/180.f;
 return ::malemod::collar::StableRecruitmentFrame({root.x,root.y,root.z},{cosf(angle),0.f,-sinf(angle)},{0,1,0});
}
'''+s
 if d['name']=='PDReadInput':
  s=s.replace('return x;', 'for(unsigned i=0;i<2;i++){x.surfacePelvis[i]=surfaceCollisionEnabled?surfaceTargetPelvis[i]:(i?V3{5.4f,0.f,86.f}:V3{3.f,0.f,70.f});x.surfaceThighRadii[i]=surfaceCollisionEnabled?surfaceTargetThighRadii[i]:7.2f;}x.surfacePelvisRadius=surfaceCollisionEnabled?surfaceTargetPelvisRadius:6.4f;return x;')
 if d['name']=='PDSetInput':
  at=s.rfind('}')
  s=s[:at]+'for(unsigned i=0;i<2;i++){surfaceOldPelvis[i]=surfacePelvis[i];surfacePelvis[i]=a.surfacePelvis[i]+(b.surfacePelvis[i]-a.surfacePelvis[i])*t;surfaceThighRadii[i]=a.surfaceThighRadii[i]+(b.surfaceThighRadii[i]-a.surfaceThighRadii[i])*t;}surfacePelvisRadius=a.surfacePelvisRadius+(b.surfacePelvisRadius-a.surfacePelvisRadius)*t;\n'+s[at:]
 if d['name']=='UpdateCompliantDynamics':
  # Wire-valid requests reach .15 seconds. The reference .10/24 cap silently
  # lost up to one third of a slow native frame's actual simulated time.
  s=s.replace('constraintAccumulator+=min(dt,.10f);const float fixed=1.f/240.f;', '''#ifdef MALEMOD_COMPLIANT_SCROTAL_SKIN
 constraintAccumulator+=min(dt,.15f);const float fixed=1.f/240.f;
#else
 constraintAccumulator+=min(dt,.10f);const float fixed=1.f/240.f;
#endif''')
  s=s.replace('int steps=min(24,int((constraintAccumulator+1e-7f)/fixed));', '''#ifdef MALEMOD_COMPLIANT_SCROTAL_SKIN
 int steps=min(36,int((constraintAccumulator+1e-7f)/fixed));
#else
 int steps=min(24,int((constraintAccumulator+1e-7f)/fixed));
#endif''')
  s=s.replace('memcpy(pdThigh,target.thigh,sizeof(pdThigh));', 'memcpy(pdThigh,target.thigh,sizeof(pdThigh));memcpy(surfacePelvis,target.surfacePelvis,sizeof(surfacePelvis));')
 if d['name']=='StepConstraintSolver':
  old='V3 torque=Cross(up,CPDesiredUp(s))*20000.f+up*(twist*torsion);cpOmega[s]=(cpOmega[s]+torque*dt)*expf(-120.f*dt);PDRotateBody(s,cpOmega[s]*dt);'
  if s.count(old)!=1:raise ValueError('Scrotal angular source span changed')
  s=s.replace(old,'''#ifdef MALEMOD_COMPLIANT_SCROTAL_SKIN
  auto response=::malemod::surface::scrotal::Response(physUI[4],physUI[6]);
  V3 torque=Cross(up,CPDesiredUp(s))*response.angularStiffness+up*(twist*torsion);
  cpOmega[s]=(cpOmega[s]+torque*dt)*(1.f/(1.f+response.angularDamping*dt+response.angularStiffness*dt*dt));
  PDRotateBody(s,cpOmega[s]*dt);
#else
  '''+old+'''
#endif''')
  at=s.rfind('}')
  s=s[:at]+'''#ifdef MALEMOD_COMPLIANT_SCROTAL_SKIN
 static_assert(shaftNodeCount==::malemod::surface::scrotal::MechanicalPartition::shaftNodes,"Continuous rod node contract");
 for(unsigned side=0;side<2;side++){
  V3 center=CPCenter(side),r=CPRadii(side);
  surfaceScrotalSkin[side].Step(dt,{center.x,center.y,center.z},min(r.x,min(r.y,r.z)),physUI[4],physUI[6]);
 }
#endif
'''+s[at:]
  old='V3 acceleration{0.f,side*86.f*response,gait*86.f*response-gravity};float drag=i<pdBody0?shaftDrag:bodyDrag;'
  new='''V3 acceleration{0.f,side*86.f*response,gait*86.f*response-gravity};
  if(surfaceGarmentEnabled&&!surfaceGarmentContactReaction){auto a=i<pdBody0?surfaceGarmentShaft:surfaceGarmentLobes[i-pdBody0];auto bounded=::malemod::surface::BoundGarmentAcceleration({a.x,a.y,a.z},gravity);acceleration=acceleration+V3{bounded.x,bounded.y,bounded.z};}
  float drag=i<pdBody0?shaftDrag:bodyDrag;'''
  if s.count(old)!=1:raise ValueError('Observed source acceleration integration span changed')
  s=s.replace(old,new)
  at='PDRecoverPouchVentral();'
  pending='''if(surfaceGarmentPendingReady){
  for(int i=2;i<shaftNodeCount;i++)pdVelocity[i]=pdVelocity[i]+surfaceGarmentPendingRod[i]*pdInvMass[i];
  for(int i=0;i<2;i++){pdVelocity[pdBody0+i]=pdVelocity[pdBody0+i]+surfaceGarmentPendingLobes[i]*pdInvMass[pdBody0+i];cpOmega[i]=cpOmega[i]+surfaceGarmentPendingAngular[i]*pdInvInertia[i];}
  for(auto& p:surfaceGarmentPendingRod)p={};for(auto& p:surfaceGarmentPendingLobes)p={};for(auto& p:surfaceGarmentPendingAngular)p={};surfaceGarmentPendingReady=false;
 }
 '''
  if s.count(at)!=1:raise ValueError('Observed source generalized integration span changed')
  s=s.replace(at,pending+at)
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
# Exact clinical control projection from the hash-checked reference. Geometry
# and physics expressions remain source-owned; clock/input ownership is external.
proxy=(runtime/'d3d9_proxy.cpp').read_text()
def function(name):
 start=proxy.index('static float '+name+'(') if name!='ApplyControlMapping' else proxy.index('static void ApplyControlMapping(){')
 brace=proxy.index('{',start);end=brace+1;depth=1
 while depth:
  depth+=(proxy[end]=='{')-(proxy[end]=='}');end+=1
 return proxy[start:end]
storage='static ' if args.process_isolated else 'static thread_local '
extra='''static const float coherentShapeLow[7]={.85f,1.0f,.95f,1.0f,-80.f,-2.f,-3.f};
static const float neutralPhysics[8]={78.f,86.f,12.f,62.f,28.f,94.f,18.f,72.f};
struct ClinicalPhysSpec{float lo,hi;};
static const ClinicalPhysSpec physSpecs[8]={{-100,400},{0,100},{0,100},{10,200},{0,100},{0,100},{0,100},{10,200}};
'''+storage+'float effectiveShapeUI[7]{},effectiveGlansUI=50,throbSizePulse=0,throbTwitchPulse=0,throbAngleSizePulse=0;\n'+storage+'teaching::ClinicalFluidProjection teachingFluid;\n'
fluid=(runtime/'teaching_volume.h').read_text()
def member(signature):
 start=fluid.index(signature);brace=fluid.index('{',start);end=brace+1;depth=1
 while depth:
  depth+=(fluid[end]=='{')-(fluid[end]=='}');end+=1
 return fluid[start:end]
parts[0]+='\nnamespace teaching { struct ClinicalFluidProjection {float angleGain[4]={1,1,1,1},lateralGain[4]{};struct {float lateralWobbleDegrees=0;} settings;'+member('static float EventTime(')+member('float MainLateralYaw(')+'}; }\n'
parts.insert(1,extra)
for name in ['UnmapControl100','ThrobEnvelope','AdvanceTwitchClock','ApplyControlMapping']:parts.append(function(name))
collar=(runtime/'unified_collar_solver.h').read_text()
a=collar.index('  V3 p=Point(before,i),q=p-root;',collar.index('target=before;'))
b=collar.index('  target.row(i)+=',a)
body=collar[a:b].replace('V3 p=Point(before,i),q=p-root;','V3 q=p-root;').replace('if(radial<1e-8f)continue;','if(radial<1e-8f)return {};').replace('(float)mask[i]','(float)mask')
parts.append('namespace UnifiedCollar { static V3 RadialTargetDelta(V3 p,V3 root,V3 axis,V3 up,float radius,float length,double mask){float growth=Smoother01((radius-2.9f)/4.72f);\n'+body+'return delta;} }')
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
