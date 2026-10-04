#pragma once
#include "runtime.hpp"
#include <cmath>
#include <stdexcept>
namespace malemod::surface {
inline ImpulsePoint ImpulseAdd(ImpulsePoint a,ImpulsePoint b){return {a.x+b.x,a.y+b.y,a.z+b.z};}
inline ImpulsePoint ImpulseSubtract(ImpulsePoint a,ImpulsePoint b){return {a.x-b.x,a.y-b.y,a.z-b.z};}
inline void ValidateImpulse(ImpulsePoint p){for(double x:{p.x,p.y,p.z})if(!std::isfinite(x)||std::abs(x)>1e12)throw std::invalid_argument("Invalid cumulative garment impulse");}
// Producer accumulates EVERY completed cloth solve before replacing a latest
// output. A consumer can skip publications or retry a busy source submission
// without replaying or dropping the integrated reaction.
class GarmentImpulseLedger {
 Frame::GarmentSupport total_;
public:
 void Reset(std::uint64_t epoch){total_={};total_.contactEpoch=epoch;}
 const Frame::GarmentSupport& Append(const Frame::GarmentSupport& delta){
  if(!total_.contactEpoch)throw std::invalid_argument("Garment impulse ledger epoch missing");
  auto next=total_;
  if(delta.enabled&&delta.contactReaction){
   for(unsigned i=0;i<12;i++){ValidateImpulse(delta.rodImpulseTotals[i]);next.rodImpulseTotals[i]=ImpulseAdd(next.rodImpulseTotals[i],delta.rodImpulseTotals[i]);ValidateImpulse(next.rodImpulseTotals[i]);}
   for(unsigned i=0;i<2;i++){ValidateImpulse(delta.lobeImpulseTotals[i]);ValidateImpulse(delta.lobeAngularImpulseTotals[i]);next.lobeImpulseTotals[i]=ImpulseAdd(next.lobeImpulseTotals[i],delta.lobeImpulseTotals[i]);next.lobeAngularImpulseTotals[i]=ImpulseAdd(next.lobeAngularImpulseTotals[i],delta.lobeAngularImpulseTotals[i]);ValidateImpulse(next.lobeImpulseTotals[i]);ValidateImpulse(next.lobeAngularImpulseTotals[i]);}
  }
  next.enabled=true;next.contactReaction=true;next.contactSerial++;total_=next;return total_;
 }
};
struct GarmentImpulseDelta {std::array<ImpulsePoint,12> rod{};std::array<ImpulsePoint,2> lobes{},angular{};bool reset=false;};
class GarmentImpulseCursor {
 Frame::GarmentSupport last_;
public:
 GarmentImpulseDelta Consume(const Frame::GarmentSupport& total){
  GarmentImpulseDelta delta;
  if(!total.enabled||!total.contactReaction)return delta;
  if(!total.contactEpoch||!total.contactSerial)throw std::invalid_argument("Physical reaction publication identity absent");
  const bool changed=last_.contactEpoch!=total.contactEpoch;
  if(!changed&&total.contactSerial<last_.contactSerial)throw std::invalid_argument("Physical reaction publication regressed");
  if(!changed&&total.contactSerial==last_.contactSerial){
   auto equal=[](ImpulsePoint a,ImpulsePoint b){ValidateImpulse(a);return a.x==b.x&&a.y==b.y&&a.z==b.z;};
   for(unsigned i=0;i<12;i++)if(!equal(total.rodImpulseTotals[i],last_.rodImpulseTotals[i]))throw std::invalid_argument("Physical reaction identity reused with changed impulse");
   for(unsigned i=0;i<2;i++)if(!equal(total.lobeImpulseTotals[i],last_.lobeImpulseTotals[i])||!equal(total.lobeAngularImpulseTotals[i],last_.lobeAngularImpulseTotals[i]))throw std::invalid_argument("Physical reaction identity reused with changed impulse");
   return delta;
  }
  const auto previous=changed?Frame::GarmentSupport{}:last_;delta.reset=changed;
  for(unsigned i=0;i<12;i++){ValidateImpulse(total.rodImpulseTotals[i]);delta.rod[i]=ImpulseSubtract(total.rodImpulseTotals[i],previous.rodImpulseTotals[i]);}
  for(unsigned i=0;i<2;i++){ValidateImpulse(total.lobeImpulseTotals[i]);ValidateImpulse(total.lobeAngularImpulseTotals[i]);delta.lobes[i]=ImpulseSubtract(total.lobeImpulseTotals[i],previous.lobeImpulseTotals[i]);delta.angular[i]=ImpulseSubtract(total.lobeAngularImpulseTotals[i],previous.lobeAngularImpulseTotals[i]);}
  last_=total;return delta;
 }
};
}
