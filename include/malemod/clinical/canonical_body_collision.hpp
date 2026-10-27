#pragma once
// Mechanically extracted canonical Wolverine body sweep math. Engine adapters
// provide already skinned vertices in source units and stable triangle IDs.
#include <malemod/clinical/fluid_types.h>
#include <cfloat>
#include <stdexcept>
namespace malemod::clinical {
class CanonicalBodyCollision {
public:
 struct Triangle {unsigned a,b,c;unsigned char section;unsigned source;};
private:
 using FluidTri=Triangle;
 struct FluidAabb {V3 lo,hi;};
 struct FluidBvhNode {FluidAabb box;int left=-1,right=-1;unsigned first=0,count=0;};
 std::vector<V3> fluidCollisionVertices;
 std::vector<FluidTri> fluidCollisionTriangles;
 std::vector<unsigned> fluidTriangleOrder;
 std::vector<FluidBvhNode> fluidBvh;
 V3 fluidCollisionEmitter{};bool fluidCollisionEmitterValid=false;
static FluidAabb FluidEmptyBox(){return {{FLT_MAX,FLT_MAX,FLT_MAX},{-FLT_MAX,-FLT_MAX,-FLT_MAX}};}
static void FluidGrow(FluidAabb& b,V3 p){b.lo.x=min(b.lo.x,p.x);b.lo.y=min(b.lo.y,p.y);b.lo.z=min(b.lo.z,p.z);b.hi.x=max(b.hi.x,p.x);b.hi.y=max(b.hi.y,p.y);b.hi.z=max(b.hi.z,p.z);}
static FluidAabb FluidMerge(FluidAabb a,FluidAabb b){FluidGrow(a,b.lo);FluidGrow(a,b.hi);return a;}
int FluidBuildBvh(unsigned first,unsigned count){
 int at=(int)fluidBvh.size();fluidBvh.push_back({});fluidBvh[at].first=first;fluidBvh[at].count=count;FluidAabb centroid=FluidEmptyBox();
 for(unsigned i=first;i<first+count;i++){auto&t=fluidCollisionTriangles[fluidTriangleOrder[i]];FluidGrow(centroid,(fluidCollisionVertices[t.a]+fluidCollisionVertices[t.b]+fluidCollisionVertices[t.c])/3.f);}
 if(count<=8)return at;V3 span=centroid.hi-centroid.lo;int axis=span.y>span.x?1:0;if((axis==0?span.x:span.y)<span.z)axis=2;unsigned mid=first+count/2;
 std::nth_element(fluidTriangleOrder.begin()+first,fluidTriangleOrder.begin()+mid,fluidTriangleOrder.begin()+first+count,[&](unsigned x,unsigned y){auto&a=fluidCollisionTriangles[x];auto&b=fluidCollisionTriangles[y];V3 ca=(fluidCollisionVertices[a.a]+fluidCollisionVertices[a.b]+fluidCollisionVertices[a.c])/3.f,cb=(fluidCollisionVertices[b.a]+fluidCollisionVertices[b.b]+fluidCollisionVertices[b.c])/3.f;return axis==0?ca.x<cb.x:axis==1?ca.y<cb.y:ca.z<cb.z;});
 int left=FluidBuildBvh(first,mid-first),right=FluidBuildBvh(mid,first+count-mid);fluidBvh[at].left=left;fluidBvh[at].right=right;fluidBvh[at].count=0;return at;
}
FluidAabb FluidRefitBvh(int index){auto& n=fluidBvh[index];FluidAabb b=FluidEmptyBox();if(n.count){for(unsigned i=n.first;i<n.first+n.count;i++){auto&t=fluidCollisionTriangles[fluidTriangleOrder[i]];FluidGrow(b,fluidCollisionVertices[t.a]);FluidGrow(b,fluidCollisionVertices[t.b]);FluidGrow(b,fluidCollisionVertices[t.c]);}}else b=FluidMerge(FluidRefitBvh(n.left),FluidRefitBvh(n.right));n.box=b;return b;}
static bool FluidSegmentBox(V3 a,V3 b,const FluidAabb& box,float radius){V3 d=b-a;float lo=0,hi=1;for(int k=0;k<3;k++){float p=k==0?a.x:k==1?a.y:a.z,v=k==0?d.x:k==1?d.y:d.z,mn=(k==0?box.lo.x:k==1?box.lo.y:box.lo.z)-radius,mx=(k==0?box.hi.x:k==1?box.hi.y:box.hi.z)+radius;if(fabsf(v)<1e-7f){if(p<mn||p>mx)return false;}else{float x=(mn-p)/v,y=(mx-p)/v;if(x>y)std::swap(x,y);lo=max(lo,x);hi=min(hi,y);if(lo>hi)return false;}}return true;}
static bool FluidSegmentTriangle(V3 from,V3 to,V3 a,V3 b,V3 c,float& fraction,float& u,float& v,V3& normal){V3 d=to-from,e1=b-a,e2=c-a,p=Cross(d,e2);float det=Dot(e1,p);if(fabsf(det)<1e-7f)return false;float inv=1.f/det;V3 s=from-a;u=Dot(s,p)*inv;if(u<0||u>1)return false;V3 q=Cross(s,e1);v=Dot(d,q)*inv;if(v<0||u+v>1)return false;fraction=Dot(e2,q)*inv;if(fraction<0||fraction>1)return false;normal=Unit(Cross(e1,e2));if(Dot(normal,d)>0)normal=normal*-1.f;return true;}
bool FluidBodySweep(V3 from,V3 to,float radius,volumeFluid::FluidImpact& hit,float& best,bool skipEmitter=true){
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
bool FluidResolveBodyAnchor(unsigned triangle,float u,float v,V3& p,V3& n){if(triangle>=fluidCollisionTriangles.size())return false;auto&t=fluidCollisionTriangles[triangle];V3 a=fluidCollisionVertices[t.a],b=fluidCollisionVertices[t.b],c=fluidCollisionVertices[t.c];p=a*(1-u-v)+b*u+c*v;n=Unit(Cross(b-a,c-a));return true;}
bool FluidResolveSplat(const volumeFluid::FluidImpact& anchor,V3& p,V3& n){
 p=anchor.p;n=anchor.n;if(anchor.kind!=volumeFluid::FLUID_IMPACT_BODY)return true;
 unsigned ti=anchor.triangle;
 if(ti>=fluidCollisionTriangles.size()||fluidCollisionTriangles[ti].section!=anchor.section||fluidCollisionTriangles[ti].source!=anchor.sourceTriangle){
  ti=0;while(ti<fluidCollisionTriangles.size()&&(fluidCollisionTriangles[ti].section!=anchor.section||fluidCollisionTriangles[ti].source!=anchor.sourceTriangle))++ti;
 }
 if(!FluidResolveBodyAnchor(ti,anchor.baryU,anchor.baryV,p,n))return false;
 n=n*anchor.normalSign;return true;
}
public:
 std::function<bool(const volumeFluid::FluidImpact&,V3,volumeFluid::FluidImpact&)> worldProject;
 void Clear(){fluidCollisionVertices.clear();fluidCollisionTriangles.clear();fluidTriangleOrder.clear();fluidBvh.clear();fluidCollisionEmitterValid=false;}
 void SetTopology(std::vector<Triangle> triangles){fluidCollisionTriangles=std::move(triangles);fluidTriangleOrder.clear();fluidBvh.clear();}
 void Refit(std::vector<V3> vertices){
  for(auto p:vertices)if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z))throw std::invalid_argument("Nonfinite body receiver");
  for(auto t:fluidCollisionTriangles)if(t.a>=vertices.size()||t.b>=vertices.size()||t.c>=vertices.size())throw std::invalid_argument("Body receiver index out of range");
  fluidCollisionVertices=std::move(vertices);
  if(fluidCollisionTriangles.empty()){fluidBvh.clear();return;}
  if(fluidBvh.empty()){fluidTriangleOrder.resize(fluidCollisionTriangles.size());for(unsigned i=0;i<fluidTriangleOrder.size();i++)fluidTriangleOrder[i]=i;FluidBuildBvh(0,unsigned(fluidTriangleOrder.size()));}
  FluidRefitBvh(0);
 }
 void SetEmitter(V3 point,bool valid=true){fluidCollisionEmitter=point;fluidCollisionEmitterValid=valid;}
 bool Sweep(V3 from,V3 to,float radius,volumeFluid::FluidImpact& hit,float& best,bool skipEmitter=true){return FluidBodySweep(from,to,radius,hit,best,skipEmitter);}
 bool Resolve(const volumeFluid::FluidImpact& anchor,V3& point,V3& normal){return FluidResolveSplat(anchor,point,normal);}
 bool Project(const volumeFluid::FluidImpact& receiver,V3 candidate,volumeFluid::FluidImpact& out){
  V3 p,n;if(!FluidResolveSplat(receiver,p,n))return false;
  V3 from=candidate+n*1.2f,to=candidate-n*1.2f;
  if(receiver.kind==volumeFluid::FLUID_IMPACT_BODY){float best=1.0001f;if(!FluidBodySweep(from,to,0,out,best,false))return false;}
  else if(!worldProject||!worldProject(receiver,candidate,out))return false;
  return Dot(out.n,n)>.4f&&Length(out.p-candidate)<1.21f;
 }
 const std::vector<Triangle>& Triangles()const{return fluidCollisionTriangles;}
 const std::vector<V3>& Vertices()const{return fluidCollisionVertices;}
};
}
