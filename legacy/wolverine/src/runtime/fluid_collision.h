#pragma once
static bool FluidWorkActive();
// Exact skinned-body collision plus read-only PhysX 2.8.1 static-scene queries.
// No PhysX SDK binary or header is redistributed; this file declares only the
// documented query ABI already provided by the game.
#include "fluid_gameplay_bones.h"
#include <cfloat>

struct FluidTri {unsigned a,b,c;unsigned char section;unsigned source;};
struct FluidAabb {V3 lo,hi;};
struct FluidBvhNode {FluidAabb box;int left=-1,right=-1;unsigned first=0,count=0;};
static std::vector<V3> fluidCollisionVertices;
static std::vector<FluidTri> fluidCollisionTriangles;
static std::vector<unsigned> fluidTriangleOrder;
static std::vector<FluidBvhNode> fluidBvh;
static float fluidBodyPalette[3][75*12]{};
static UINT fluidBodyPaletteCount[3]{};
static LONG fluidBodyPaletteFrame[3]={-2,-2,-2};
static float fluidCollisionLocal[16]{},fluidCollisionView[16]{};
static V3 fluidCameraWorld{};static DWORD fluidCameraTick;static bool fluidCollisionWorld;
static V3 fluidCollisionEmitter{};static bool fluidCollisionEmitterValid;
static LONG fluidCollisionFrame=-2;static bool fluidBvhTopologyReady;static unsigned fluidTopologyMask=0;

static V3 FluidMatrixPoint(const float* m,V3 p){return {p.x*m[0]+p.y*m[4]+p.z*m[8]+m[12],p.x*m[1]+p.y*m[5]+p.z*m[9]+m[13],p.x*m[2]+p.y*m[6]+p.z*m[10]+m[14]};}
static V3 FluidMatrixVector(const float* m,V3 p){return {p.x*m[0]+p.y*m[4]+p.z*m[8],p.x*m[1]+p.y*m[5]+p.z*m[9],p.x*m[2]+p.y*m[6]+p.z*m[10]};}
#include "fluid_world_origin.h"
struct FluidPixelCameraLayout {IDirect3DPixelShader9* shader;UINT reg;bool valid;};
static FluidPixelCameraLayout fluidPixelCameraLayouts[256]{};static UINT fluidPixelCameraLayoutCount;
static FluidPixelCameraLayout* FluidPixelCameraLayoutFor(IDirect3DDevice9* d){
 IDirect3DPixelShader9* shader=nullptr;if(FAILED(d->GetPixelShader(&shader))||!shader)return nullptr;for(UINT i=0;i<fluidPixelCameraLayoutCount;i++)if(fluidPixelCameraLayouts[i].shader==shader){shader->Release();return &fluidPixelCameraLayouts[i];}if(fluidPixelCameraLayoutCount>=256){shader->Release();return nullptr;}auto& layout=fluidPixelCameraLayouts[fluidPixelCameraLayoutCount++];layout.shader=shader;UINT bytes=0;if(FAILED(shader->GetFunction(nullptr,&bytes))||!bytes)return &layout;std::vector<DWORD> code((bytes+3)/4);if(FAILED(shader->GetFunction(code.data(),&bytes)))return &layout;ID3DXConstantTable* table=nullptr;if(FAILED(D3DXGetShaderConstantTable(code.data(),&table))||!table)return &layout;const char* names[]={"CameraWorldPos"};D3DXHANDLE h=nullptr;for(const char* name:names)if((h=table->GetConstantByName(nullptr,name)))break;D3DXCONSTANT_DESC desc{};UINT one=1;if(h&&SUCCEEDED(table->GetConstantDesc(h,&desc,&one))&&desc.RegisterSet==D3DXRS_FLOAT4&&desc.RegisterCount>=1){layout.reg=desc.RegisterIndex;layout.valid=true;Log("pixel camera origin found: c%u",layout.reg);}table->Release();return &layout;
}
static void CaptureFluidCamera(IDirect3DDevice9* d){
 if(fluidWorldCameraFrame==renderFrameSerial)return;
 float c[4];ShaderLayout* l=GetShaderLayout(d);if(l&&l->cameraValid&&SUCCEEDED(d->GetVertexShaderConstantF(l->cameraRegister,c,1))&&_finite(c[0])&&_finite(c[1])&&_finite(c[2])){fluidCameraWorld={c[0],c[1],c[2]};fluidCameraTick=GetTickCount();fluidWorldCameraFrame=renderFrameSerial;return;}
 FluidPixelCameraLayout* p=FluidPixelCameraLayoutFor(d);if(p&&p->valid&&SUCCEEDED(d->GetPixelShaderConstantF(p->reg,c,1))&&_finite(c[0])&&_finite(c[1])&&_finite(c[2])){fluidCameraWorld={c[0],c[1],c[2]};fluidCameraTick=GetTickCount();fluidWorldCameraFrame=renderFrameSerial;}
}
static void CaptureFluidBodySection(IDirect3DDevice9* d,int section){
 if(!FluidWorkActive())return;
 if(section<0||section>2)return;ShaderLayout* l=GetShaderLayout(d);if(!l||!l->valid)return;UINT n=min(75u,l->boneCount/3u);
 if(!n||FAILED(d->GetVertexShaderConstantF(l->boneRegister,fluidBodyPalette[section],n*3)))return;
 fluidBodyPaletteCount[section]=n;fluidBodyPaletteFrame[section]=renderFrameSerial;
 d->GetVertexShaderConstantF(l->localRegister,fluidCollisionLocal,4);if(l->viewValid)d->GetVertexShaderConstantF(l->viewRegister,fluidCollisionView,4);
 CaptureFluidCamera(d);
}
static V3 FluidSkinVertex(const unsigned char* p,const unsigned char* boneOverride,const float* palette,UINT paletteCount){
 V3 q;memcpy(&q,p,12);V3 out{};float total=0;
 for(int i=0;i<4;i++){unsigned b=boneOverride?boneOverride[i]:p[20+i];float w=p[24+i]/255.f;if(!p[24+i]||b>=paletteCount)continue;out=out+TransformPoint(palette+b*12,q)*w;total+=w;}
 return total>.0001f?out/total:q;
}
static FluidAabb FluidEmptyBox(){return {{FLT_MAX,FLT_MAX,FLT_MAX},{-FLT_MAX,-FLT_MAX,-FLT_MAX}};}
static void FluidGrow(FluidAabb& b,V3 p){b.lo.x=min(b.lo.x,p.x);b.lo.y=min(b.lo.y,p.y);b.lo.z=min(b.lo.z,p.z);b.hi.x=max(b.hi.x,p.x);b.hi.y=max(b.hi.y,p.y);b.hi.z=max(b.hi.z,p.z);}
static FluidAabb FluidMerge(FluidAabb a,FluidAabb b){FluidGrow(a,b.lo);FluidGrow(a,b.hi);return a;}
static int FluidBuildBvh(unsigned first,unsigned count){
 int at=(int)fluidBvh.size();fluidBvh.push_back({});fluidBvh[at].first=first;fluidBvh[at].count=count;FluidAabb centroid=FluidEmptyBox();
 for(unsigned i=first;i<first+count;i++){auto&t=fluidCollisionTriangles[fluidTriangleOrder[i]];FluidGrow(centroid,(fluidCollisionVertices[t.a]+fluidCollisionVertices[t.b]+fluidCollisionVertices[t.c])/3.f);}
 if(count<=8)return at;V3 span=centroid.hi-centroid.lo;int axis=span.y>span.x?1:0;if((axis==0?span.x:span.y)<span.z)axis=2;unsigned mid=first+count/2;
 std::nth_element(fluidTriangleOrder.begin()+first,fluidTriangleOrder.begin()+mid,fluidTriangleOrder.begin()+first+count,[&](unsigned x,unsigned y){auto&a=fluidCollisionTriangles[x];auto&b=fluidCollisionTriangles[y];V3 ca=(fluidCollisionVertices[a.a]+fluidCollisionVertices[a.b]+fluidCollisionVertices[a.c])/3.f,cb=(fluidCollisionVertices[b.a]+fluidCollisionVertices[b.b]+fluidCollisionVertices[b.c])/3.f;return axis==0?ca.x<cb.x:axis==1?ca.y<cb.y:ca.z<cb.z;});
 int left=FluidBuildBvh(first,mid-first),right=FluidBuildBvh(mid,first+count-mid);fluidBvh[at].left=left;fluidBvh[at].right=right;fluidBvh[at].count=0;return at;
}
static FluidAabb FluidRefitBvh(int index){auto& n=fluidBvh[index];FluidAabb b=FluidEmptyBox();if(n.count){for(unsigned i=n.first;i<n.first+n.count;i++){auto&t=fluidCollisionTriangles[fluidTriangleOrder[i]];FluidGrow(b,fluidCollisionVertices[t.a]);FluidGrow(b,fluidCollisionVertices[t.b]);FluidGrow(b,fluidCollisionVertices[t.c]);}}else b=FluidMerge(FluidRefitBvh(n.left),FluidRefitBvh(n.right));n.box=b;return b;}
static bool FluidSegmentBox(V3 a,V3 b,const FluidAabb& box,float radius){V3 d=b-a;float lo=0,hi=1;for(int k=0;k<3;k++){float p=k==0?a.x:k==1?a.y:a.z,v=k==0?d.x:k==1?d.y:d.z,mn=(k==0?box.lo.x:k==1?box.lo.y:box.lo.z)-radius,mx=(k==0?box.hi.x:k==1?box.hi.y:box.hi.z)+radius;if(fabsf(v)<1e-7f){if(p<mn||p>mx)return false;}else{float x=(mn-p)/v,y=(mx-p)/v;if(x>y)std::swap(x,y);lo=max(lo,x);hi=min(hi,y);if(lo>hi)return false;}}return true;}
static bool FluidSegmentTriangle(V3 from,V3 to,V3 a,V3 b,V3 c,float& fraction,float& u,float& v,V3& normal){V3 d=to-from,e1=b-a,e2=c-a,p=Cross(d,e2);float det=Dot(e1,p);if(fabsf(det)<1e-7f)return false;float inv=1.f/det;V3 s=from-a;u=Dot(s,p)*inv;if(u<0||u>1)return false;V3 q=Cross(s,e1);v=Dot(d,q)*inv;if(v<0||u+v>1)return false;fraction=Dot(e2,q)*inv;if(fraction<0||fraction>1)return false;normal=Unit(Cross(e1,e2));if(Dot(normal,d)>0)normal=normal*-1.f;return true;}
static bool FluidBodySweep(V3 from,V3 to,float radius,volumeFluid::FluidImpact& hit,float& best,bool skipEmitter=true){
 if(fluidBvh.empty())return false;bool found=false;int stack[96],top=0;stack[top++]=0;
 while(top){int ni=stack[--top];auto&n=fluidBvh[ni];if(!FluidSegmentBox(from,to,n.box,radius))continue;if(!n.count){if(top+2<96){stack[top++]=n.left;stack[top++]=n.right;}continue;}for(unsigned i=n.first;i<n.first+n.count;i++){unsigned ti=fluidTriangleOrder[i];auto&t=fluidCollisionTriangles[ti];float f,u,v;V3 normal;if(FluidSegmentTriangle(from,to,fluidCollisionVertices[t.a],fluidCollisionVertices[t.b],fluidCollisionVertices[t.c],f,u,v,normal)&&f<best){V3 impact=from+(to-from)*f;
    // The outlet is part of the same watertight graft as the target body. A
    // short clearance volume prevents a newly emitted sample from immediately
    // ray-hitting the lip or adjacent shaft skin; normal body collision resumes
    // as soon as it has actually left the outlet neighborhood.
    if(skipEmitter&&fluidCollisionEmitterValid&&Length(impact-fluidCollisionEmitter)<5.25f)continue;
    best=f;hit.p=impact;hit.n=normal;hit.normalSign=Dot(normal,Cross(fluidCollisionVertices[t.b]-fluidCollisionVertices[t.a],fluidCollisionVertices[t.c]-fluidCollisionVertices[t.a]))<0?-1.f:1.f;hit.kind=volumeFluid::FLUID_IMPACT_BODY;hit.section=t.section;hit.triangle=ti;hit.sourceTriangle=t.source;hit.baryU=u;hit.baryV=v;hit.shape=nullptr;found=true;}}}
 return found;
}

// PhysX 2.8.1 x86 ABI, verified against the SDK headers with MSVC's class
// layout report.  NxScene::raycastClosestShape is slot 112 (slot 114 is an
// overlap query with a different signature), and isWritable is slot 139.
// Query only static shapes while the scene is writable, then cache the actual
// hit plane in small world-space cells.  This keeps the fluid loop cheap while
// retaining the level collision geometry's real height and normal.
struct FluidNxRay {V3 origin,direction;};
struct FluidNxRaycastHit {void* shape;V3 impact,normal;UINT face,internalFace;float distance,u,v;unsigned short material;unsigned short pad;UINT flags;};
static_assert(sizeof(FluidNxRaycastHit)==56,"PhysX 2.8.1 NxRaycastHit ABI mismatch");
typedef void* (__cdecl *FluidNxGetSdkFn)();
typedef UINT (__thiscall *FluidNxSceneCountFn)(void*);
typedef void* (__thiscall *FluidNxGetSceneFn)(void*,UINT);
typedef bool (__thiscall *FluidNxWritableFn)(void*);
typedef void* (__thiscall *FluidNxRaycastFn)(void*,const FluidNxRay&,int,FluidNxRaycastHit&,UINT,float,UINT,const void*,void**);
struct FluidGroundCell {int x,y;V3 point,normal;DWORD touched;int layer=0;};
static std::vector<FluidGroundCell> fluidGroundCells;static void* fluidNxSdk;static DWORD fluidNxRetryTick;static LONG fluidNxFrame=-2;static UINT fluidNxQueries;static bool fluidGroundHitLogged,fluidNxReadyLogged;
static float fluidPhysicsScale=0;static unsigned fluidScaleReference=0,fluidScaleMatches[2]{};
static LONG fluidScaleFrame=-2;static void* fluidPhysicsScene=nullptr;static void* fluidScaleScenes[2]{};
static V3 fluidScaleFirstPoint[2]{};
static bool (*fluidPhysicsRayTest)(V3,V3,float,V3&,V3&)=nullptr;
static bool fluidPhysicsRayAvailable=true;
static void FluidQueryFrame(){if(fluidNxFrame!=renderFrameSerial){fluidNxFrame=renderFrameSerial;fluidNxQueries=0;}}
static void* FluidPhysicsSdk(){
 if(fluidNxSdk)return fluidNxSdk;DWORD now=GetTickCount();if(now-fluidNxRetryTick<1000u)return nullptr;fluidNxRetryTick=now;HMODULE loader=GetModuleHandleA("PhysXLoader.dll");if(!loader)return nullptr;auto getSdk=(FluidNxGetSdkFn)GetProcAddress(loader,"NxGetPhysicsSDK");if(!getSdk)return nullptr;fluidNxSdk=getSdk();if(fluidNxSdk&&!fluidNxReadyLogged){fluidNxReadyLogged=true;Log("PhysX world collision connected: raycast slot 112, writable slot 139");}return fluidNxSdk;
}
static bool FluidPhysicsRayRaw(V3 origin,V3 direction,float distance,V3& point,V3& normal,void** resultScene=nullptr){
 fluidPhysicsRayAvailable=false;
 if(fluidPhysicsRayTest){fluidPhysicsRayAvailable=true;return fluidPhysicsRayTest(origin,direction,distance,point,normal);}
 void* sdk=FluidPhysicsSdk();if(!sdk||distance<=.001f)return false;void** sdkVt=*(void***)sdk;auto count=(FluidNxSceneCountFn)sdkVt[6];auto sceneAt=(FluidNxGetSceneFn)sdkVt[7];UINT scenes=count(sdk);bool found=false;float nearest=distance;FluidNxRay ray{origin,Unit(direction)};
 for(UINT i=0;i<scenes&&i<16;i++){void* scene=sceneAt(sdk,i);if(!scene||(fluidPhysicsScene&&scene!=fluidPhysicsScene))continue;void** vt=*(void***)scene;auto writable=(FluidNxWritableFn)vt[139];if(!writable(scene))continue;fluidPhysicsRayAvailable=true;FluidNxRaycastHit h{};auto cast=(FluidNxRaycastFn)vt[112];void* shape=cast(scene,ray,1,h,0xffffffffu,nearest,2u|4u|16u|64u,nullptr,nullptr);if(shape&&h.distance>=0&&h.distance<nearest&&_finite(h.impact.x)&&_finite(h.impact.y)&&_finite(h.impact.z)&&_finite(h.normal.x)&&_finite(h.normal.y)&&_finite(h.normal.z)&&Length(h.normal)>.5f){nearest=h.distance;point=h.impact;normal=Unit(h.normal);if(resultScene)*resultScene=scene;found=true;}}
 return found;
}
static bool FluidVerifyPhysicsScale(){
 if(fluidPhysicsScale>0)return true;
 if(fluidScaleFrame==renderFrameSerial||fluidWorldReferences.size()<2)return false;
 fluidScaleFrame=renderFrameSerial;FluidQueryFrame();const float candidates[2]={.02f,1.f};
 // Match two distinct rendered surfaces to actual collision surfaces. No
 // guessed world-unit conversion or hits from a leftover menu physics scene.
 for(unsigned attempt=0;attempt<2;attempt++){
  const auto& ref=fluidWorldReferences[fluidScaleReference%fluidWorldReferences.size()];++fluidScaleReference;
  for(unsigned c=0;c<2;c++){
   V3 p,n;void* scene=nullptr;float scale=candidates[c];++fluidNxQueries;
   if(!FluidPhysicsRayRaw((ref.p+ref.n*2.f)*scale,ref.n*-1.f,4.f*scale,p,n,&scene))continue;
   if(Length(p/scale-ref.p)>.35f||fabsf(Dot(n,ref.n))<.9f)continue;
   if(fluidScaleMatches[c]&&fluidScaleScenes[c]!=scene)fluidScaleMatches[c]=0;
   fluidScaleScenes[c]=scene;
   if(fluidScaleMatches[c]&&Length(ref.p-fluidScaleFirstPoint[c])<2.f)continue;
   if(!fluidScaleMatches[c])fluidScaleFirstPoint[c]=ref.p;
   if(++fluidScaleMatches[c]>=2){fluidPhysicsScale=scale;fluidPhysicsScene=scene;Log("fluid physics scale verified against BSP surfaces: scale=%.5f scene=%p refs=%u",scale,scene,(unsigned)fluidWorldReferences.size());return true;}
  }
 }return false;
}
static bool FluidPhysicsRay(V3 origin,V3 direction,float distance,V3& point,V3& normal){
 if(!FluidVerifyPhysicsScale())return false;
 V3 physicsPoint;if(!FluidPhysicsRayRaw(origin*fluidPhysicsScale,direction,distance*fluidPhysicsScale,physicsPoint,normal))return false;
 point=physicsPoint/fluidPhysicsScale;return true;
}
static FluidGroundCell* FluidGroundCellAt(V3 p,bool probe){
 const float cellSize=6.f;int x=(int)floorf(p.x/cellSize),y=(int)floorf(p.y/cellSize),layer=(int)floorf(p.z/32.f);DWORD now=GetTickCount();
 for(auto& c:fluidGroundCells)if(c.x==x&&c.y==y&&c.layer==layer&&DWORD(now-c.touched)<250u)return &c;
 if(!probe||fluidNxQueries>=96)return nullptr;++fluidNxQueries;V3 hit,normal;V3 origin{p.x,p.y,p.z+2.f};
 if(!FluidPhysicsRay(origin,{0,0,-1},600.f,hit,normal)||normal.z<.20f)return nullptr;
 FluidGroundCell value{x,y,hit,normal,now,layer};
 for(auto& c:fluidGroundCells)if(c.x==x&&c.y==y&&c.layer==layer){c=value;return &c;}
 if(fluidGroundCells.size()>=1024){auto oldest=std::min_element(fluidGroundCells.begin(),fluidGroundCells.end(),[](const FluidGroundCell&a,const FluidGroundCell&b){return a.touched<b.touched;});*oldest=value;return &*oldest;}
 fluidGroundCells.push_back(value);return &fluidGroundCells.back();
}
static bool FluidGroundSweep(V3 from,V3 to,float radius,volumeFluid::FluidImpact& hit,float& best,bool skipEmitter=true){
 if(!fluidCollisionWorld||TankCameraSceneActive()||to.z>=from.z)return false;
 FluidQueryFrame();FluidGroundCell* cell=FluidGroundCellAt((from+to)*.5f,true);if(!cell)return false;
 float clearance=radius*.72f+.015f,d0=Dot(from-cell->point,cell->normal)-clearance,d1=Dot(to-cell->point,cell->normal)-clearance;
 if(d0 < -clearance||d1>0||d0-d1<1e-6f)return false;
 float f=max(0.f,d0/(d0-d1));if(f>=best||fluidNxQueries>=128)return false;
 // Cached planes are only broad-phase hints. Confirm at the actual contact
 // location so a neighboring step or the edge of a ledge cannot make a splat.
 V3 center=from+(to-from)*f,point,normal;++fluidNxQueries;
 if(!FluidPhysicsRay(center+V3{0,0,2.f},{0,0,-1},clearance+4.f,point,normal)||normal.z<.2f)return false;
 d0=Dot(from-point,normal)-clearance;d1=Dot(to-point,normal)-clearance;
 if(d0 < -clearance||d1>0||d0-d1<1e-6f)return false;
 f=max(0.f,d0/(d0-d1));if(f>=best)return false;
 best=f;hit.p=point;hit.n=normal;hit.kind=volumeFluid::FLUID_IMPACT_WORLD;hit.triangle=0;hit.baryU=hit.baryV=0;hit.shape=nullptr;
 if(!fluidGroundHitLogged){fluidGroundHitLogged=true;Log("verified fluid world contact: p=(%.2f %.2f %.2f) scale=%.5f",point.x,point.y,point.z,fluidPhysicsScale);}return true;
}
static bool FluidCollisionSweep(V3 from,V3 to,float radius,volumeFluid::FluidImpact& hit){
 volumeFluid::collisionDeferred=false;
 float best=1.0001f;bool body=FluidBodySweep(from,to,radius,hit,best);bool world=FluidGroundSweep(from,to,radius,hit,best);
 // Floor cache is a fast path, not a restriction to horizontal receivers.
 V3 delta=to-from;float distance=Length(delta);FluidQueryFrame();
 if(!world&&fluidCollisionWorld&&!TankCameraSceneActive()&&distance>.001f){
  if(fluidNxQueries>=128||!FluidVerifyPhysicsScale()){volumeFluid::collisionDeferred=!body;return body;}
  V3 p,n;++fluidNxQueries;if(FluidPhysicsRay(from,delta/distance,distance,p,n)){
   float f=Length(p-from)/distance;if(f<best){hit={};hit.p=p;hit.n=Dot(n,delta)>0?n*-1.f:n;hit.kind=volumeFluid::FLUID_IMPACT_WORLD;world=true;}
  }
  if(!fluidPhysicsRayAvailable)volumeFluid::collisionDeferred=!body;
 }
 return body||world;
}

static void FluidAppendSection(const unsigned char* packed,UINT vertexCount,const unsigned short* indices,UINT triangleCount,const unsigned char* gameplayBones,const float* palette,UINT paletteCount,unsigned char section,bool world,bool buildTopology){
 unsigned base=(unsigned)fluidCollisionVertices.size();fluidCollisionVertices.resize(base+vertexCount);
 // Section inputs are immutable here. Batch independent skinning into fixed
 // output slots; topology, BVH refit and collision queries stay on the caller.
 GeometryFor(vertexCount,[&](unsigned i){const unsigned char* p=packed+i*32;const unsigned char* bones=gameplayBones?gameplayBones+i*4:nullptr;V3 v=FluidSkinVertex(p,bones,palette,paletteCount);if(world)v=FluidMatrixPoint(fluidCollisionLocal,v)+fluidCameraWorld;fluidCollisionVertices[base+i]=v;});
 if(buildTopology)for(UINT i=0;i<triangleCount;i++)fluidCollisionTriangles.push_back({base+indices[i*3],base+indices[i*3+1],base+indices[i*3+2],section,i});
}
static void PrepareFluidCollision(){PerfScope perf(9);
 bool title=TankCameraSceneActive();fluidCollisionWorld=!title&&fluidWorldCameraFrame==renderFrameSerial;
 if(fluidCollisionFrame==renderFrameSerial)return;fluidCollisionFrame=renderFrameSerial;fluidCollisionVertices.clear();
 unsigned mask=0;for(int i=0;i<3;i++)if(fluidBodyPaletteCount[i])mask|=1u<<i;
 bool rebuild=!fluidBvhTopologyReady||mask!=fluidTopologyMask;if(rebuild){fluidCollisionTriangles.clear();fluidBvhTopologyReady=false;fluidTopologyMask=mask;}
 for(int s=0;s<2;s++){if(fluidBodyPaletteCount[s]==0)continue;const unsigned char* packed=s?menuRetargetBodyDynamic1:menuRetargetBodyDynamic0;UINT vc=s?menuRetargetBodyVertexCount1:menuRetargetBodyVertexCount0;const unsigned short* ix=s?menuRetargetBodyIndices1:menuRetargetBodyIndices0;UINT tc=s?menuRetargetBodyTriangleCount1:menuRetargetBodyTriangleCount0;const unsigned char* bones=title?nullptr:(s?fluidGameplayBones1:fluidGameplayBones0);FluidAppendSection(packed,vc,ix,tc,bones,fluidBodyPalette[s],fluidBodyPaletteCount[s],(unsigned char)s,fluidCollisionWorld,rebuild);}
 if(fluidBodyPaletteCount[2]){const unsigned char* packed=title?menuTankPacked:nrPacked;FluidAppendSection(packed,nrCount,nrIndices,nrIndexCount/3,nullptr,fluidBodyPalette[2],fluidBodyPaletteCount[2],2,fluidCollisionWorld,rebuild);}
 if(fluidCollisionTriangles.empty()){volumeFluid::collisionSweep=nullptr;return;}
 if(!fluidBvhTopologyReady||fluidTriangleOrder.size()!=fluidCollisionTriangles.size()){fluidTriangleOrder.resize(fluidCollisionTriangles.size());for(unsigned i=0;i<fluidTriangleOrder.size();i++)fluidTriangleOrder[i]=i;fluidBvh.clear();FluidBuildBvh(0,(unsigned)fluidTriangleOrder.size());fluidBvhTopologyReady=true;Log("exact fluid collision BVH: %u skinned vertices, %u triangles, %u nodes",(unsigned)fluidCollisionVertices.size(),(unsigned)fluidCollisionTriangles.size(),(unsigned)fluidBvh.size());}
 FluidRefitBvh(0);volumeFluid::collisionSweep=FluidCollisionSweep;
}
static bool FluidResolveBodyAnchor(unsigned triangle,float u,float v,V3& p,V3& n){if(triangle>=fluidCollisionTriangles.size())return false;auto&t=fluidCollisionTriangles[triangle];V3 a=fluidCollisionVertices[t.a],b=fluidCollisionVertices[t.b],c=fluidCollisionVertices[t.c];p=a*(1-u-v)+b*u+c*v;n=Unit(Cross(b-a,c-a));return true;}
static bool FluidResolveSplat(const volumeFluid::FluidImpact& anchor,V3& p,V3& n){
 p=anchor.p;n=anchor.n;if(anchor.kind!=volumeFluid::FLUID_IMPACT_BODY)return true;
 unsigned ti=anchor.triangle;
 if(ti>=fluidCollisionTriangles.size()||fluidCollisionTriangles[ti].section!=anchor.section||fluidCollisionTriangles[ti].source!=anchor.sourceTriangle){
  ti=0;while(ti<fluidCollisionTriangles.size()&&(fluidCollisionTriangles[ti].section!=anchor.section||fluidCollisionTriangles[ti].source!=anchor.sourceTriangle))++ti;
 }
 if(!FluidResolveBodyAnchor(ti,anchor.baryU,anchor.baryV,p,n))return false;
 n=n*anchor.normalSign;return true;
}
static bool FluidProjectSplat(const volumeFluid::FluidImpact& receiver,V3 candidate,volumeFluid::FluidImpact& out){
 V3 p,n;if(!FluidResolveSplat(receiver,p,n))return false;
 V3 from=candidate+n*1.2f,to=candidate-n*1.2f;
 if(receiver.kind==volumeFluid::FLUID_IMPACT_BODY){float best=1.0001f;if(!FluidBodySweep(from,to,0,out,best,false))return false;}
 else {FluidQueryFrame();if(fluidNxQueries>=224)return false;++fluidNxQueries;V3 q,normal;if(!FluidPhysicsRay(from,n*-1.f,2.4f,q,normal))return false;out={};out.p=q;out.n=normal;out.kind=volumeFluid::FLUID_IMPACT_WORLD;out.shape=receiver.shape;}
 return Dot(out.n,n)>.4f&&Length(out.p-candidate)<1.21f;
}
static void FluidSimulationTransform(V3 componentPoint,V3 componentDirection,V3& point,V3& direction,float* clip){
 if(fluidCollisionWorld){point=FluidMatrixPoint(fluidCollisionLocal,componentPoint)+fluidCameraWorld;direction=Unit(FluidMatrixVector(fluidCollisionLocal,componentDirection));D3DXMATRIX translated,view,out;D3DXMatrixTranslation(&translated,-fluidCameraWorld.x,-fluidCameraWorld.y,-fluidCameraWorld.z);memcpy(&view,fluidCollisionView,64);D3DXMatrixMultiply(&out,&translated,&view);memcpy(clip,&out,64);}else{point=componentPoint;direction=componentDirection;D3DXMATRIX local,view,out;memcpy(&local,fluidCollisionLocal,64);memcpy(&view,fluidCollisionView,64);D3DXMatrixMultiply(&out,&local,&view);memcpy(clip,&out,64);}
}
static void SetFluidCollisionEmitter(V3 point){fluidCollisionEmitter=point;fluidCollisionEmitterValid=true;}
static void ResetFluidCollision(){fluidTopologyMask=0;ResetFluidWorldOrigin();fluidPhysicsScale=0;fluidScaleReference=0;fluidScaleMatches[0]=fluidScaleMatches[1]=0;fluidScaleFrame=-2;fluidPhysicsScene=nullptr;fluidScaleScenes[0]=fluidScaleScenes[1]=nullptr;fluidCollisionVertices.clear();fluidCollisionTriangles.clear();fluidTriangleOrder.clear();fluidBvh.clear();fluidGroundCells.clear();fluidGroundHitLogged=false;fluidNxSdk=nullptr;fluidNxRetryTick=0;fluidNxFrame=-2;fluidNxQueries=0;fluidBvhTopologyReady=false;fluidCollisionFrame=-2;fluidCameraTick=0;fluidCameraWorld={};fluidCollisionWorld=false;fluidCollisionEmitterValid=false;memset(fluidBodyPaletteCount,0,sizeof(fluidBodyPaletteCount));for(UINT i=0;i<fluidPixelCameraLayoutCount;i++)if(fluidPixelCameraLayouts[i].shader)fluidPixelCameraLayouts[i].shader->Release();memset(fluidPixelCameraLayouts,0,sizeof(fluidPixelCameraLayouts));fluidPixelCameraLayoutCount=0;volumeFluid::collisionSweep=nullptr;}
