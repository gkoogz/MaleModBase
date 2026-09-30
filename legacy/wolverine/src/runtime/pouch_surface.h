#pragma once
#include <xmmintrin.h>
#include "pouch_surface_data.h"
static V3 cpSurfaceDelta[rsCount],cpTemplate[rsCount];
static V3 cpCenters[2],cpInverseBasis[2][3],cpWorldBasis[2][3],cpRenderRadii[2],cpSkinRadii[2];
static float cpSurfaceK=.55f;
static V3 CPLocalPoint(V3 p,int s){V3 d=p-cpCenters[s];return cpInverseBasis[s][0]*d.x+cpInverseBasis[s][1]*d.y+cpInverseBasis[s][2]*d.z;}
static V3 CPWorldPoint(V3 p,int s){return cpCenters[s]+cpWorldBasis[s][0]*p.x+cpWorldBasis[s][1]*p.y+cpWorldBasis[s][2]*p.z;}
static float CPPouchLevel(V3 local,V3 r){V3 a=CPDiv(local,r);float t=1.f-.13f*max(-1.f,min(1.f,a.z));a.x/=t;a.y/=t;return Length(a)-1.f;}
static __m128 CPRayLevel4(const __m128* origin,const __m128* direction,__m128 distance){
 const __m128 one=_mm_set1_ps(1.f);
 __m128 z=_mm_add_ps(origin[2],_mm_mul_ps(direction[2],distance));
 __m128 t=_mm_sub_ps(one,_mm_mul_ps(_mm_set1_ps(.13f),_mm_max_ps(_mm_set1_ps(-1.f),_mm_min_ps(one,z))));
 __m128 x=_mm_div_ps(_mm_add_ps(origin[0],_mm_mul_ps(direction[0],distance)),t);
 __m128 y=_mm_div_ps(_mm_add_ps(origin[1],_mm_mul_ps(direction[1],distance)),t);
 return _mm_sub_ps(_mm_sqrt_ps(_mm_add_ps(_mm_add_ps(_mm_mul_ps(x,x),_mm_mul_ps(y,y)),_mm_mul_ps(z,z))),one);
}
static float CPPouchField(V3 p){float a=CPPouchLevel(CPLocalPoint(p,0),cpSkinRadii[0]),b=CPPouchLevel(CPLocalPoint(p,1),cpSkinRadii[1]);float h=max(0.f,min(1.f,.5f+.5f*(b-a)/cpSurfaceK));return b*(1.f-h)+a*h-cpSurfaceK*h*(1.f-h);}
// Four independent skin vertices share each support's transform. Full
// precision division/square root and all five projections are retained.
struct CPPoints4 {__m128 x,y,z;};
static __m128 CPSplat(float f){return _mm_set1_ps(f);}
static __m128 CPSelect(__m128 mask,__m128 yes,__m128 no){return _mm_or_ps(_mm_and_ps(mask,yes),_mm_andnot_ps(mask,no));}
static CPPoints4 CPBasis4(CPPoints4 p,const V3* basis){
  return {
    _mm_add_ps(_mm_add_ps(_mm_mul_ps(CPSplat(basis[0].x),p.x),_mm_mul_ps(CPSplat(basis[1].x),p.y)),_mm_mul_ps(CPSplat(basis[2].x),p.z)),
    _mm_add_ps(_mm_add_ps(_mm_mul_ps(CPSplat(basis[0].y),p.x),_mm_mul_ps(CPSplat(basis[1].y),p.y)),_mm_mul_ps(CPSplat(basis[2].y),p.z)),
    _mm_add_ps(_mm_add_ps(_mm_mul_ps(CPSplat(basis[0].z),p.x),_mm_mul_ps(CPSplat(basis[1].z),p.y)),_mm_mul_ps(CPSplat(basis[2].z),p.z))};
}
static __m128 CPLevel4(CPPoints4 p,V3 radii){
  __m128 z=_mm_div_ps(p.z,CPSplat(radii.z));
  __m128 taper=_mm_sub_ps(CPSplat(1.f),_mm_mul_ps(CPSplat(.13f),_mm_max_ps(CPSplat(-1.f),_mm_min_ps(CPSplat(1.f),z))));
  __m128 x=_mm_div_ps(_mm_div_ps(p.x,CPSplat(radii.x)),taper),y=_mm_div_ps(_mm_div_ps(p.y,CPSplat(radii.y)),taper);
  return _mm_sub_ps(_mm_sqrt_ps(_mm_add_ps(_mm_add_ps(_mm_mul_ps(x,x),_mm_mul_ps(y,y)),_mm_mul_ps(z,z))),CPSplat(1.f));
}
template<bool packedFair=false> static void CPKeepSkinOutside(__m128* fairPositions=nullptr){
 GeometryFor((cpActiveCount+3)/4,[&](unsigned group){unsigned k=group*4;
  V3 p[4];for(unsigned lane=0;lane<4;lane++){
   unsigned id=cpActive[min(k+lane,cpActiveCount-1)];
   if constexpr(packedFair){float xyz[4];_mm_storeu_ps(xyz,fairPositions[id]);p[lane]={xyz[0],xyz[1],xyz[2]};}
   else p[lane]=rsPositions[id];
  }
  CPPoints4 points{_mm_set_ps(p[3].x,p[2].x,p[1].x,p[0].x),_mm_set_ps(p[3].y,p[2].y,p[1].y,p[0].y),_mm_set_ps(p[3].z,p[2].z,p[1].z,p[0].z)};
  for(int side=0;side<2;side++){
   V3 c=cpCenters[side];
   CPPoints4 local=CPBasis4({_mm_sub_ps(points.x,CPSplat(c.x)),_mm_sub_ps(points.y,CPSplat(c.y)),_mm_sub_ps(points.z,CPSplat(c.z))},cpInverseBasis[side]);
   __m128 inside=_mm_cmplt_ps(CPLevel4(local,cpRenderRadii[side]),CPSplat(.018f));
   if(!_mm_movemask_ps(inside))continue;
   for(int j=0;j<5;j++){
    __m128 factor=_mm_div_ps(CPSplat(1.020f),_mm_max_ps(CPSplat(1e-7f),_mm_add_ps(CPSplat(1.f),CPLevel4(local,cpRenderRadii[side]))));
    local={_mm_mul_ps(local.x,factor),_mm_mul_ps(local.y,factor),_mm_mul_ps(local.z,factor)};
   }
   // CPWorldPoint adds the center before the second/third basis vectors.
   const V3* basis=cpWorldBasis[side];
   __m128 x=_mm_add_ps(_mm_add_ps(_mm_add_ps(CPSplat(c.x),_mm_mul_ps(CPSplat(basis[0].x),local.x)),_mm_mul_ps(CPSplat(basis[1].x),local.y)),_mm_mul_ps(CPSplat(basis[2].x),local.z));
   __m128 y=_mm_add_ps(_mm_add_ps(_mm_add_ps(CPSplat(c.y),_mm_mul_ps(CPSplat(basis[0].y),local.x)),_mm_mul_ps(CPSplat(basis[1].y),local.y)),_mm_mul_ps(CPSplat(basis[2].y),local.z));
   __m128 z=_mm_add_ps(_mm_add_ps(_mm_add_ps(CPSplat(c.z),_mm_mul_ps(CPSplat(basis[0].z),local.x)),_mm_mul_ps(CPSplat(basis[1].z),local.y)),_mm_mul_ps(CPSplat(basis[2].z),local.z));
   points={CPSelect(inside,x,points.x),CPSelect(inside,y,points.y),CPSelect(inside,z,points.z)};
  }
  float x[4],y[4],z[4];_mm_storeu_ps(x,points.x);_mm_storeu_ps(y,points.y);_mm_storeu_ps(z,points.z);
  for(unsigned lane=0;lane<4&&k+lane<cpActiveCount;lane++){
   unsigned id=cpActive[k+lane];
   if constexpr(packedFair)fairPositions[id]=_mm_set_ps(0.f,z[lane],y[lane],x[lane]);
   else rsPositions[id]={x[lane],y[lane],z[lane]};
  }
 });
}
static void ApplyPouchSurface(){PerfScope perf(11);
 if(!eggRestReady||!constraintSolverReady)return;CPEnsure();
 for(int s=0;s<2;s++){
  cpCenters[s]=CPCenter(s);cpRenderRadii[s]=CPRadii(s);V3 scale=CPDiv(cpRenderRadii[s],s?V3{5.724f,4.86f,7.81f}:V3{5.724f,4.86f,7.93f});
  cpSkinRadii[s]=cpRenderRadii[s]+CPMul({.80f,.95f,.65f},scale);
  for(int j=0;j<3;j++){V3 axis{j==0?1.f:0.f,j==1?1.f:0.f,j==2?1.f:0.f};cpInverseBasis[s][j]=CPInverse(axis,s);cpWorldBasis[s][j]=CPTransform(axis,s);}
 }
 V3 center=(cpCenters[0]+cpCenters[1])*.5f,span=cpCenters[1]-cpCenters[0];
 V3 pair[3]={RotateFromTo({1,0,0},{0,7.5f,.35f},span),RotateFromTo({0,1,0},{0,7.5f,.35f},span),RotateFromTo({0,0,1},{0,7.5f,.35f},span)};
 V3 scale=CPDiv((cpRenderRadii[0]+cpRenderRadii[1])*.5f,{5.724f,4.86f,7.87f});
 float atCenter=max(CPPouchLevel(CPLocalPoint(center,0),cpSkinRadii[0]),CPPouchLevel(CPLocalPoint(center,1),cpSkinRadii[1]));cpSurfaceK=max(.55f,4.f*max(0.f,atCenter)+.08f);
 float reach=30.f*max(scale.x,max(scale.y,scale.z));
 // Four independent material rays share the same field and bisection depth.
 // This is full-precision SIMD, with the original arithmetic and branching;
 // no approximate reciprocal, relaxed accuracy or skipped ray samples.
 __m128 origins[2][3];
 for(int s=0;s<2;s++){V3 p=CPDiv(CPLocalPoint(center,s),cpSkinRadii[s]);origins[s][0]=_mm_set1_ps(p.x);origins[s][1]=_mm_set1_ps(p.y);origins[s][2]=_mm_set1_ps(p.z);}
 const __m128 zero=_mm_setzero_ps(),one=_mm_set1_ps(1.f),half=_mm_set1_ps(.5f),fieldK=_mm_set1_ps(cpSurfaceK);
 GeometryFor((cpActiveCount+3)/4,[&](unsigned group){unsigned k=group*4;
  V3 d[4],local[2][4];
  for(unsigned lane=0;lane<4;lane++){
   unsigned q=min(k+lane,cpActiveCount-1);V3 v=CPMul({cpDirections[q*3],cpDirections[q*3+1],cpDirections[q*3+2]},scale);
   d[lane]=Unit(pair[0]*v.x+pair[1]*v.y+pair[2]*v.z);
   for(int s=0;s<2;s++)local[s][lane]=CPDiv(cpInverseBasis[s][0]*d[lane].x+cpInverseBasis[s][1]*d[lane].y+cpInverseBasis[s][2]*d[lane].z,cpSkinRadii[s]);
  }
  __m128 directions[2][3];for(int s=0;s<2;s++){
   directions[s][0]=_mm_set_ps(local[s][3].x,local[s][2].x,local[s][1].x,local[s][0].x);
   directions[s][1]=_mm_set_ps(local[s][3].y,local[s][2].y,local[s][1].y,local[s][0].y);
   directions[s][2]=_mm_set_ps(local[s][3].z,local[s][2].z,local[s][1].z,local[s][0].z);
  }
  __m128 lo=zero,hi=_mm_set1_ps(reach);
  for(int j=0;j<18;j++){
   __m128 mid=_mm_mul_ps(_mm_add_ps(lo,hi),half),a=CPRayLevel4(origins[0],directions[0],mid),b=CPRayLevel4(origins[1],directions[1],mid);
   __m128 h=_mm_max_ps(zero,_mm_min_ps(one,_mm_add_ps(half,_mm_div_ps(_mm_mul_ps(half,_mm_sub_ps(b,a)),fieldK))));
   __m128 inv=_mm_sub_ps(one,h),field=_mm_sub_ps(_mm_add_ps(_mm_mul_ps(b,inv),_mm_mul_ps(a,h)),_mm_mul_ps(_mm_mul_ps(fieldK,h),inv));
   __m128 inside=_mm_cmplt_ps(field,zero);
   lo=_mm_or_ps(_mm_and_ps(inside,mid),_mm_andnot_ps(inside,lo));
   hi=_mm_or_ps(_mm_and_ps(inside,hi),_mm_andnot_ps(inside,mid));
  }
  float distance[4];_mm_storeu_ps(distance,_mm_mul_ps(_mm_add_ps(lo,hi),half));
  for(unsigned lane=0;lane<4&&k+lane<cpActiveCount;lane++){
   unsigned q=k+lane,id=cpActive[q];V3 target=center+d[lane]*distance[lane];rsPositions[id]=rsPositions[id]+(target-rsPositions[id])*cpWeights[q];
  }
 });
 V3 anchor=(BallAnchor(0)+BallAnchor(1))*.5f;
 V3 lateral=Unit(span),up=Unit(anchor-center-lateral*Dot(anchor-center,lateral)),forward=Unit(Cross(lateral,up));
 float neckLength=max(.2f,Length(anchor-center)/19.11f);
 static std::vector<unsigned> templateIDs;static V3 templateLocal[rsCount];
 if(templateIDs.empty()){
  bool needed[rsCount]{};
  for(unsigned k=0;k<cpNeckCount;k++)needed[cpNeckIDs[k]]=true;
  for(unsigned j=0;j<cpNeckRows[cpNeckCount];j++)needed[cpNeckSources[j]]=true;
  for(unsigned i=0;i<rsCount;i++)if(needed[i]){
   templateIDs.push_back(i);templateLocal[i]={cpReference[3*i]-18.85f,cpReference[3*i+1],cpReference[3*i+2]-64.275f};
  }
 }
 for(unsigned i:templateIDs){
  V3 local=templateLocal[i];
  cpTemplate[i]=center+forward*(local.x*scale.x)+lateral*(local.y*scale.y)+up*(local.z*neckLength);
  cpSurfaceDelta[i]=rsPositions[i]-cpTemplate[i];
 }
 GeometryFor(cpNeckCount,[&](unsigned k){
  unsigned id=cpNeckIDs[k];V3 t=cpTemplate[id];__m128 target=_mm_set_ps(0.f,t.z,t.y,t.x);
  for(unsigned j=cpNeckRows[k];j<cpNeckRows[k+1];j++){const V3& v=cpSurfaceDelta[cpNeckSources[j]];target=_mm_add_ps(target,_mm_mul_ps(_mm_set_ps(0.f,v.z,v.y,v.x),_mm_set1_ps(cpNeckWeights[j])));}
  float q[4];_mm_storeu_ps(q,target);V3 result{q[0],q[1],q[2]};
  rsPositions[id]=rsPositions[id]+(result-rsPositions[id])*cpNeckBlend[k];
 });
 // Fixed topology is prepared once; SIMD evaluates XYZ together without
 // changing neighbor order, pass count or the interleaved contact projection.
 // Keep the public V3 layout intact (GPU buffers and replay captures use it).
 static __m128 positions[rsCount],next[cpFairCount],counts[cpFairCount],amounts[cpFairCount];
 static bool fairReady=false;
 if(!fairReady){for(unsigned k=0;k<cpFairCount;k++){
   unsigned id=cpFair[k];counts[k]=_mm_set1_ps(float(cpRows[id+1]-cpRows[id]));amounts[k]=_mm_set1_ps(.4f*cpFairWeight[k]);
 }fairReady=true;}
 for(unsigned i=0;i<rsCount;i++){const V3& p=rsPositions[i];positions[i]=_mm_set_ps(0.f,p.z,p.y,p.x);}
 for(int pass=0;pass<60;pass++){
  for(unsigned k=0;k<cpFairCount;k++){
   unsigned id=cpFair[k];__m128 mean=_mm_setzero_ps();
   for(unsigned j=cpRows[id];j<cpRows[id+1];j++)mean=_mm_add_ps(mean,positions[cpNeighbors[j]]);
   next[k]=_mm_add_ps(positions[id],_mm_mul_ps(_mm_sub_ps(_mm_div_ps(mean,counts[k]),positions[id]),amounts[k]));
  }
  for(unsigned k=0;k<cpFairCount;k++)positions[cpFair[k]]=next[k];
  // Project directly in the Jacobi workspace: the same coordinates and
  // contact equations, without a V3 round trip after each five passes.
  if(pass%5==4)CPKeepSkinOutside<true>(positions);
 }
 for(unsigned k=0;k<cpFairCount;k++){unsigned id=cpFair[k];float p[4];_mm_storeu_ps(p,positions[id]);rsPositions[id]={p[0],p[1],p[2]};}
 for(unsigned k=0;k<cpActiveCount;k++){unsigned id=cpActive[k];float p[4];_mm_storeu_ps(p,positions[id]);rsPositions[id]={p[0],p[1],p[2]};}
 // The collider and visible lower skin share the same pressure transform.
 CPKeepSkinOutside();
 for(unsigned i=0;i<r14Count;i++)r14Positions[i]=rsPositions[i];
}
