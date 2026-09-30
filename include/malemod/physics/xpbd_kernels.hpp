#pragma once
// Generated from Wolverine; see provenance/physics-kernels.json.
#include "state.hpp"
namespace malemod::physics {
struct PDBendData {float alpha,gamma,factor;V3 oldValue;};
inline void SolveDistance(State& state,int a,int b,float rest,float compliance,float& lambda,float dt){
 V3 delta=state.position[b]-state.position[a];float distance=Length(delta);if(distance<1e-7f)return;
 float alpha=compliance/(dt*dt),dl=(-(distance-rest)-alpha*lambda)/(state.invMass[a]+state.invMass[b]+alpha);lambda+=dl;
 V3 impulse=delta*(dl/distance);state.position[a]=state.position[a]-impulse*state.invMass[a];state.position[b]=state.position[b]+impulse*state.invMass[b];
}
inline PDBendData PrepareBend(const State& state,int i,float compliance,float dt,float bounce01){
 float alpha=compliance/(dt*dt),w=state.invMass[i-1]+4.f*state.invMass[i]+state.invMass[i+1];
 // Kelvin-Voigt bending: dissipate changes in curvature, leaving rigid
 // translation and a common swing untouched. The ratio follows BOUNCE.
 float ratio=.10f+.40f*(1.f-max(0.f,min(1.f,bounce01)));
 float gamma=2.f*ratio*sqrtf(compliance/max(w,1e-8f))/dt;
 V3 oldValue=state.oldPosition[i-1]-state.oldPosition[i]*2.f+state.oldPosition[i+1];
 return {alpha,gamma,-1.f/((1.f+gamma)*w+alpha),oldValue};
}
inline void SolveBendPrepared(State& state,int i,const PDBendData& data,V3& lambda){
 V3 value=state.position[i-1]-state.position[i]*2.f+state.position[i+1];
 V3 dl=(value+lambda*data.alpha+(value-data.oldValue)*data.gamma)*data.factor;lambda=lambda+dl;
 state.position[i-1]=state.position[i-1]+dl*state.invMass[i-1];state.position[i]=state.position[i]-dl*(2.f*state.invMass[i]);state.position[i+1]=state.position[i+1]+dl*state.invMass[i+1];
}
}
