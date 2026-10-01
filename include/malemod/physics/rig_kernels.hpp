#pragma once
#include "xpbd_kernels.hpp"
namespace malemod::physics {
// A curved authored rest metric. With zero rest curvature this is exactly the
// extracted Wolverine Kelvin-Voigt bend. Rest curvature is supplied by a spoke.
inline void SolveRestBend(State& state,int i,const PDBendData& data,V3 rest,V3 oldRest,V3& lambda){
 V3 value=state.position[i-1]-state.position[i]*2.f+state.position[i+1]-rest;
 V3 oldValue=data.oldValue-oldRest;
 V3 dl=(value+lambda*data.alpha+(value-oldValue)*data.gamma)*data.factor;
 lambda=lambda+dl;
 state.position[i-1]=state.position[i-1]+dl*state.invMass[i-1];
 state.position[i]=state.position[i]-dl*(2.f*state.invMass[i]);
 state.position[i+1]=state.position[i+1]+dl*state.invMass[i+1];
}
// Damped material shear, matching the source suspension's vector strain law.
// This point representation excludes angular effective mass and the source's
// full Hermite reaction Jacobian; adapters must report that distinction.
inline void SolveMaterial(State& state,int i,V3 target,V3 oldTarget,float compliance,float ratio,float dt,V3& lambda){
 float w=state.invMass[i],alpha=compliance/(dt*dt);
 float gamma=2.f*ratio*sqrtf(compliance/max(w,1e-8f))/dt;
 V3 value=state.position[i]-target;
 V3 rate=state.position[i]-state.oldPosition[i]-(target-oldTarget);
 V3 dl=(value+lambda*alpha+rate*gamma)*(-1.f/((1.f+gamma)*w+alpha));
 lambda=lambda+dl;state.position[i]=state.position[i]+dl*w;
}
inline void SolveReach(State& state,int i,V3 anchor,float limit){
 V3 delta=state.position[i]-anchor;float length=Length(delta);
 if(length>limit)state.position[i]=anchor+delta*(limit/length);
}
inline void SolveSuspension(State& state,int i,V3 anchor,V3 oldAnchor,float rest,float compliance,float ratio,float dt,float& lambda){
 V3 delta=state.position[i]-anchor;float distance=Length(delta);if(distance<1e-7f)return;
 V3 n=delta/distance;float w=state.invMass[i],alpha=compliance/(dt*dt);
 float gamma=2.f*ratio*sqrtf(compliance/max(w,1e-8f))/dt;
 float rate=Dot(n,state.position[i]-state.oldPosition[i]-(anchor-oldAnchor));
 float next=min(0.f,lambda+(-(distance-rest)-alpha*lambda-gamma*rate)/((1.f+gamma)*w+alpha));
 state.position[i]=state.position[i]+n*((next-lambda)*w);lambda=next;
}
inline void SolveMaterialAxis(State& state,int i,V3 target,V3 oldTarget,V3 axis,float compliance,float dt,float& lambda){
 float w=state.invMass[i],alpha=compliance/(dt*dt);
 float gamma=2.1f*sqrtf(compliance/max(w,1e-8f))/dt;
 float value=Dot(state.position[i]-target,axis);
 float rate=Dot(state.position[i]-state.oldPosition[i]-(target-oldTarget),axis);
 float dl=(-value-alpha*lambda-gamma*rate)/((1.f+gamma)*w+alpha);
 lambda=lambda+dl;state.position[i]=state.position[i]+axis*(dl*w);
}
// The source rod/lobe contact reaction, with point generalized masses.
inline void ProjectRodContact(State& state,int a,int b,int body,float part,V3 normal,float gap){
 if(gap>=0.f)return;
 float wa=state.invMass[a],wb=state.invMass[b],wc=state.invMass[body];
 float w=wa*(1.f-part)*(1.f-part)+wb*part*part+wc;if(w<1e-8f)return;
 V3 impulse=normal*(-gap/w);
 V3 ca=impulse*(-wa*(1.f-part)),cb=impulse*(-wb*part),cc=impulse*wc;
 state.position[a]=state.position[a]+ca;state.oldPosition[a]=state.oldPosition[a]+ca;
 state.position[b]=state.position[b]+cb;state.oldPosition[b]=state.oldPosition[b]+cb;
 state.position[body]=state.position[body]+cc;state.oldPosition[body]=state.oldPosition[body]+cc;
 V3 relative=state.position[body]-state.oldPosition[body]-(state.position[a]-state.oldPosition[a])*(1.f-part)-(state.position[b]-state.oldPosition[b])*part;
 float approach=Dot(relative,normal);
 if(approach<0.f){
  V3 stop=normal*(approach/w);
  state.oldPosition[a]=state.oldPosition[a]-stop*(wa*(1.f-part));
  state.oldPosition[b]=state.oldPosition[b]-stop*(wb*part);
  state.oldPosition[body]=state.oldPosition[body]+stop*wc;
 }
}
inline void ProjectPairContact(State& state,int a,int b,V3 normal,float gap){
 if(gap>=0.f)return;
 float wa=state.invMass[a],wb=state.invMass[b],w=wa+wb;if(w<1e-8f)return;
 V3 impulse=normal*(-gap/w),ca=impulse*(-wa),cb=impulse*wb;
 state.position[a]=state.position[a]+ca;state.oldPosition[a]=state.oldPosition[a]+ca;
 state.position[b]=state.position[b]+cb;state.oldPosition[b]=state.oldPosition[b]+cb;
 V3 relative=state.position[b]-state.oldPosition[b]-(state.position[a]-state.oldPosition[a]);
 float approach=Dot(relative,normal);
 if(approach<0.f){
  V3 stop=normal*(approach/w);
  state.oldPosition[a]=state.oldPosition[a]-stop*wa;state.oldPosition[b]=state.oldPosition[b]+stop*wb;
 }
}
// Position-level contact with actual moving-surface history. Source friction
// coefficients are .48 static / .32 sliding. Correction is excluded from the
// normal reconstructed velocity, preventing penetration-recovery rebound.
inline void ProjectMovingContact(State& state,int i,V3 normal,float gap,V3 surfaceMove){
 if(gap>=0.f)return;
 V3 correction=normal*(-gap);
 state.position[i]=state.position[i]+correction;
 state.oldPosition[i]=state.oldPosition[i]+correction;
 V3 relative=state.position[i]-state.oldPosition[i]-surfaceMove;
 float approach=Dot(relative,normal);
 if(approach<0.f)state.oldPosition[i]=state.oldPosition[i]+normal*approach;
 V3 tangent=relative-normal*approach;float slip=Length(tangent);
 float budget=.48f*(-gap);if(slip>budget)budget=.32f*(-gap);
 if(slip>1e-8f)state.oldPosition[i]=state.oldPosition[i]+tangent*(min(slip,budget)/slip);
}
}
