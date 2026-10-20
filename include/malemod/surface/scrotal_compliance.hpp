#pragma once
#include <array>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace malemod::surface::scrotal {
// Mechanical ownership is independent of the visible connected skin. The rod
// runs root-to-tip THROUGH the covered region; neither lobe replaces a span.
struct MechanicalPartition {
 static constexpr unsigned shaftNodes=12,lobes=2;
 static constexpr float suspensionCoordinate=.12f;
};
using Vector=std::array<float,3>;
inline float Unit(float x){return std::clamp(x,0.f,1.f);}
inline float Smooth(float x){x=Unit(x);return Unit(x*x*x*(10+x*(-15+6*x)));}
// Material donor weights, never current location: overlap cannot turn shaft
// vertices into sack vertices when either structure moves or changes size.
inline float SkinOwnership(float scrotum,float shaft){
 return Smooth((scrotum-.55f)/.35f)*(1-Smooth((shaft-.10f)/.40f));
}
struct SuspensionResponse {float angularStiffness,angularDamping,shearDamping;};
inline SuspensionResponse Response(float stiffness,float bounce){
 float w=12+24*Unit(stiffness/100),z=.35f+.50f*(1-Unit(bounce/100));
 return {w*w,2*z*w,2*z};
}
// Calibrate transverse suspension by response frequency, including the rod's
// effective inverse mass. A fixed tiny compliance acted as a positional lock;
// lowering its damping alone could not restore relative lobe motion.
inline float ShearCompliance(float inverseMass,float stiffness){
 const float omega=4.f+12.f*Unit(stiffness/100);
 return inverseMass/(omega*omega);
}

// A bounded secondary skin mode, not a replacement for the massive colliding
// contents or the continuous rod. Offsets are character-local; no feedback is
// sent into rod nodes. The existing suspension/contact solve owns that feedback.
class Envelope {
 Vector displacement_{},velocity_{},previousCenter_{};
 bool ready_=false;float previousRadius_=0;
 public:
 void Reset(){*this=Envelope{};}
 const Vector& Displacement()const{return displacement_;}
 void Step(float dt,Vector center,float radius,float stiffness,float bounce){
  if(!std::isfinite(dt)||dt<=0||dt>.02f||!std::isfinite(radius)||radius<=0)
   throw std::invalid_argument("Scrotal skin substep/radius");
  for(float x:center)if(!std::isfinite(x))throw std::invalid_argument("Scrotal skin center");
  if(!ready_||radius/previousRadius_<.8f||radius/previousRadius_>1.25f){
   displacement_={};velocity_={};previousCenter_=center;previousRadius_=radius;ready_=true;return;
  }
  // Frame-invariant follower: moving the contents leaves the skin behind.
  // Constant velocity reaches a bounded drag offset; stationary inputs decay.
  float w=14+18*Unit(stiffness/100),z=.35f+.45f*(1-Unit(bounce/100));
  float k=w*w,c=2*z*w,denominator=1+c*dt+k*dt*dt;
  float limit=.14f*radius;
  for(unsigned j=0;j<3;j++){
   float motion=center[j]-previousCenter_[j];
   // Shape switches/teleports reset instead of injecting artificial energy.
   if(std::abs(motion)>radius){Reset();Step(dt,center,radius,stiffness,bounce);return;}
   displacement_[j]-=motion;
   velocity_[j]=(velocity_[j]-k*dt*displacement_[j])/denominator;
   displacement_[j]+=velocity_[j]*dt;
  }
  float length=std::sqrt(displacement_[0]*displacement_[0]+displacement_[1]*displacement_[1]+displacement_[2]*displacement_[2]);
  if(length>limit){
   float outward=0;for(unsigned j=0;j<3;j++){displacement_[j]*=limit/length;outward+=velocity_[j]*displacement_[j]/limit;}
   if(outward>0)for(unsigned j=0;j<3;j++)velocity_[j]-=outward*displacement_[j]/limit;
  }
  previousCenter_=center;previousRadius_=radius;
 }
};
}
