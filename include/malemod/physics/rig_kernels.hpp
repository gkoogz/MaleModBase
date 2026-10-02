#pragma once
#include "xpbd_kernels.hpp"
namespace malemod::physics {
// Motion is relative to a rigid attachment frame. The adapter measures the
// frame's world velocities, differentiates them, and rotates these inputs into
// solver space. Constant world translation must not become a drag force.
inline V3 FilterMotion(V3 previous,V3 measured,float response,float limit,float dt){
 V3 filtered=previous+(measured-previous)*(1.f-expf(-response*dt));
 float length=Length(filtered);
 if(length>limit)filtered=filtered*(limit/length);
 return filtered;
}
inline V3 FrameAcceleration(V3 position,V3 velocity,V3 linear,V3 angularVelocity,V3 angularAcceleration){
 return linear+Cross(angularAcceleration,position)+Cross(angularVelocity,Cross(angularVelocity,position))+Cross(angularVelocity,velocity)*2.f;
}
inline void IntegrateRelative(State& state,int i,V3 gravity,V3 linear,V3 angularVelocity,V3 angularAcceleration,float accelerationLimit,float drag,float dt){
 V3 acceleration=FrameAcceleration(state.position[i],state.velocity[i],linear,angularVelocity,angularAcceleration);
 float length=Length(acceleration);
 if(length>accelerationLimit)acceleration=acceleration*(accelerationLimit/length);
 state.velocity[i]=(state.velocity[i]+(gravity-acceleration)*dt)*expf(-drag*dt);
 state.position[i]=state.position[i]+state.velocity[i]*dt;
}
// Source SampleShaftChain's C1 Hermite guide. The first interval is kinematic;
// its chord therefore equals LiveRootDirection times the segment length.
inline void SampleGuide(const State& state,int count,float t,V3& center,V3& tangent){
 float u=max(0.f,min(1.f,t))*(count-1),q,q2,q3;
 int span=int(u);if(span>=count-1)span=count-2;
 q=u-span;q2=q*q;q3=q2*q;
 V3 p0=state.position[span],p1=state.position[span+1],m0,m1;
 if(span==0)m0=p1-p0;else m0=(state.position[span+1]-state.position[span-1])*.5f;
 if(span+1==count-1)m1=p1-p0;else m1=(state.position[span+2]-state.position[span])*.5f;
 center=p0*(2*q3-3*q2+1)+m0*(q3-2*q2+q)+p1*(-2*q3+3*q2)+m1*(q3-q2);
 tangent=Unit(p0*(6*q2-6*q)+m0*(3*q2-4*q+1)+p1*(-6*q2+6*q)+m1*(3*q2-2*q));
}
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
