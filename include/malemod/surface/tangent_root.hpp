#pragma once
#include <malemod/surface/root_contact.hpp>
#include <vector>

namespace malemod::surface::tangent_root {
using root_contact::Point;
using root_contact::State;
using root_contact::Dot;
using root_contact::Scale;
using root_contact::Finite;
using Matrix=std::array<Point,3>;
inline void Positive(double x){if(!std::isfinite(x)||x<=0)throw std::invalid_argument("Invalid angular root coefficient");}
inline void Check(State s){for(double x:{s.pitch,s.yaw,s.pitchVelocity,s.yawVelocity})if(!std::isfinite(x))throw std::invalid_argument("Nonfinite angular root state");}
struct Basis {Point pitch,yaw;};
inline Basis Jacobian(State s,double length){
 Check(s);Positive(length);const double sp=std::sin(s.pitch),cp=std::cos(s.pitch),sy=std::sin(s.yaw),cy=std::cos(s.yaw);
 return {{-length*sp*cy,0,-length*cp*cy},{-length*cp*sy,length*cy,length*sp*sy}};
}
// Keep the angular state throughout projection. An isotropic point mass followed
// by radial normalization is not the same generalized mechanical system.
inline Point Response(Basis j,double inertia,Point impulse){
 Positive(inertia);Finite(j.pitch);Finite(j.yaw);Finite(impulse);
 auto p=Scale(j.pitch,Dot(j.pitch,impulse)/inertia),y=Scale(j.yaw,Dot(j.yaw,impulse)/inertia);
 for(unsigned k=0;k<3;k++)p[k]+=y[k];return p;
}
inline double EffectiveMass(Basis j,double inertia,Point gradient){return Dot(gradient,Response(j,inertia,gradient));}
inline Matrix Mobility(Basis j,double inertia){
 Positive(inertia);Finite(j.pitch);Finite(j.yaw);Matrix m{};
 for(unsigned a=0;a<3;a++)for(unsigned b=0;b<3;b++)m[a][b]=(j.pitch[a]*j.pitch[b]+j.yaw[a]*j.yaw[b])/inertia;return m;
}
inline State PositionImpulse(State state,Basis j,double inertia,Point impulse){
 Check(state);Positive(inertia);Finite(j.pitch);Finite(j.yaw);Finite(impulse);
 state.pitch+=Dot(j.pitch,impulse)/inertia;state.yaw+=Dot(j.yaw,impulse)/inertia;return state;
}
inline Point Velocity(Basis j,double pitchRate,double yawRate){
 if(!std::isfinite(pitchRate)||!std::isfinite(yawRate))throw std::invalid_argument("Nonfinite root rate");Finite(j.pitch);Finite(j.yaw);
 auto v=Scale(j.pitch,pitchRate);for(unsigned k=0;k<3;k++)v[k]+=j.yaw[k]*yawRate;return v;
}
inline State Rates(State state,Basis j,Point velocity){
 Check(state);Finite(velocity);Finite(j.pitch);Finite(j.yaw);const double p=Dot(j.pitch,j.pitch),y=Dot(j.yaw,j.yaw);
 if(p<1e-10||y<1e-10)throw std::invalid_argument("Singular angular root chart");
 state.pitchVelocity=Dot(velocity,j.pitch)/p;state.yawVelocity=Dot(velocity,j.yaw)/y;return state;
}
struct Station {double mass,arm;};
// Nominal straight-source rod mode. Pass only source-defined dynamic stations;
// lobes, driven root, and recruited body skin are not guessed participants.
inline double ModalInertia(double rootInertia,const std::vector<Station>& stations){
 Positive(rootInertia);double total=rootInertia;
 for(auto s:stations){Positive(s.mass);if(!std::isfinite(s.arm)||s.arm<0)throw std::invalid_argument("Invalid modal station arm");total+=s.mass*s.arm*s.arm;}
 Positive(total);return total;
}
struct Law {double stiffness,damping;};
// Preserve the source acceleration coefficients K/I and C/I in this nominal
// mode. This algebraic identity does not prove full coupled-response parity.
inline Law EquivalentLaw(double sourceInertia,double modalInertia,Law source){
 Positive(sourceInertia);Positive(modalInertia);Positive(source.stiffness);if(!std::isfinite(source.damping)||source.damping<0)throw std::invalid_argument("Invalid root damping");
 const double scale=modalInertia/sourceInertia;return {source.stiffness*scale,source.damping*scale};
}
struct Correction {double multiplier,angle;};
inline Correction Spring(double value,double oldValue,double target,double lambda,double inverseInertia,Law law,double dt){
 for(double x:{value,oldValue,target,lambda})if(!std::isfinite(x))throw std::invalid_argument("Nonfinite spring constraint");Positive(inverseInertia);Positive(dt);Positive(law.stiffness);
 if(!std::isfinite(law.damping)||law.damping<0)throw std::invalid_argument("Invalid root damping");
 const double alpha=1/(law.stiffness*dt*dt),gamma=law.damping/(law.stiffness*dt);
 const double delta=(-(value-target)-alpha*lambda-gamma*(value-oldValue))/((1+gamma)*inverseInertia+alpha);
 return {lambda+delta,delta*inverseInertia};
}
inline Correction SoftStop(double value,double onset,double lambda,double inverseInertia,double stiffness,double dt){
 for(double x:{value,lambda})if(!std::isfinite(x))throw std::invalid_argument("Nonfinite stop constraint");Positive(onset);Positive(inverseInertia);Positive(stiffness);Positive(dt);
 const double alpha=1/(stiffness*dt*dt),c=std::abs(value)-onset;
 const double next=(std::min)(0.,lambda-(c+alpha*lambda)/(inverseInertia+alpha));
 return {next,(value<0?-1:1)*(next-lambda)*inverseInertia};
}
// Symmetric positive definite 3x3 solve for root-adjacent vector bending.
// The caller adds scalar station mobility and compliance to J M^-1 J^T.
inline Point Solve(Matrix matrix,Point rhs){
 for(auto row:matrix)Finite(row);Finite(rhs);Matrix l{};
 for(unsigned a=0;a<3;a++)for(unsigned b=0;b<=a;b++){
  if(std::abs(matrix[a][b]-matrix[b][a])>1e-10*(1+std::abs(matrix[a][b])))throw std::invalid_argument("Nonsymmetric angular mobility");
  double v=matrix[a][b];for(unsigned k=0;k<b;k++)v-=l[a][k]*l[b][k];
  if(a==b){Positive(v);l[a][b]=std::sqrt(v);}else l[a][b]=v/l[b][b];
 }
 Point y{},x{};for(unsigned a=0;a<3;a++){double v=rhs[a];for(unsigned b=0;b<a;b++)v-=l[a][b]*y[b];y[a]=v/l[a][a];}
 for(int a=2;a>=0;a--){double v=y[a];for(unsigned b=a+1;b<3;b++)v-=l[b][a]*x[b];x[a]=v/l[a][a];}return x;
}
}
