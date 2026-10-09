#pragma once
#include <malemod/math.hpp>
#include <algorithm>

namespace malemod::physics {
// A small secondary angular degree of freedom around an authored rest hinge.
// The spoke supplies measured inertial/gravity torque in radians/squared second.
// This is a bounded accessory model, not a cloth collision solver.
struct HingeState { float angle=0,velocity=0; };
inline void StepHinge(HingeState& s,float torque,float omega,float dampingRatio,float limit,float dt){
 if(!std::isfinite(dt)||!std::isfinite(torque)||!std::isfinite(s.angle)||!std::isfinite(s.velocity)||
    !(omega>0)||!(dampingRatio>0&&dampingRatio<1)||!(limit>0)||dt>.5f){s={};return;}
 if(dt<=0)return;
 torque=std::clamp(torque,-omega*omega*limit,omega*omega*limit);
 const float equilibrium=torque/(omega*omega),a=dampingRatio*omega;
 const float b=omega*std::sqrt(1-dampingRatio*dampingRatio),decay=std::exp(-a*dt);
 const float x=s.angle-equilibrium,c=std::cos(b*dt),sn=std::sin(b*dt),q=(s.velocity+a*x)/b;
 s.angle=equilibrium+decay*(x*c+q*sn);
 s.velocity=decay*(s.velocity*c-(a*q+b*x)*sn);
 if(s.angle>limit){s.angle=limit;s.velocity=std::min(0.f,s.velocity);}
 if(s.angle<-limit){s.angle=-limit;s.velocity=std::max(0.f,s.velocity);}
}
inline V3 HingeOffset(V3 point,V3 pivot,V3 unitAxis,float angle,float influence){
 if(influence<=0)return {};
 V3 q=point-pivot;float c=std::cos(angle),s=std::sin(angle);
 V3 turned=q*c+Cross(unitAxis,q)*s+unitAxis*(Dot(unitAxis,q)*(1-c));
 return (turned-q)*std::clamp(influence,0.f,1.f);
}
}
