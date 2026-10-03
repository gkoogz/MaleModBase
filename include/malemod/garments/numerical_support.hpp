#pragma once
#include <malemod/garments/jockstrap.hpp>
#include <malemod/surface/runtime.hpp>
#include <malemod/surface/garment_support.hpp>
namespace malemod::garments {
enum class SupportBody {None,Shaft,Lobe0,Lobe1};
// classify uses measured source lineage, never guessed bone names. toSource
// converts acceleration directions/units, not positions (no origin offset).
// Normalize contact influence and each lineage independently, so tessellation
// and the number of support donors never multiply the total acceleration.
template<class Classify,class ToSource>
inline surface::Frame::GarmentSupport AggregateNumericalSupport(
 const Output& cloth,Classify classify,ToSource toSource){
 surface::Frame::GarmentSupport result;
 if(cloth.style==Style::Naked||!cloth.contactBudgetSatisfied||cloth.support.empty())return result;
 double total=0;for(const auto& s:cloth.support){
  if(!std::isfinite(s.influence)||s.influence<0||!Finite(s.acceleration)||!std::isfinite(s.separation))throw std::invalid_argument("Invalid measured garment support contact");
  total+=s.influence;
 }
 if(!std::isfinite(total)||total<=0)return result;
 std::array<Point,3> sum{};
 for(const auto& s:cloth.support){
  double donorTotal=0;for(const auto& d:s.lineage.donors){if(!std::isfinite(d.weight)||d.weight<0)throw std::invalid_argument("Invalid garment support lineage");donorTotal+=d.weight;}
  if(!std::isfinite(donorTotal)||donorTotal<=0)continue;
  auto acceleration=toSource(s.acceleration);if(!Finite(acceleration))throw std::invalid_argument("Invalid source garment acceleration transform");
  for(const auto& d:s.lineage.donors)if(d.weight>0){
   auto group=classify(d);int id=group==SupportBody::Shaft?0:group==SupportBody::Lobe0?1:group==SupportBody::Lobe1?2:-1;
   if(id>=0)sum[id]=Add(sum[id],Mul(acceleration,(s.influence/total)*(d.weight/donorTotal)));
  }
 }
 auto bounded=[](Point p,float limit){V3 q{float(p[0]),float(p[1]),float(p[2])};float length=Length(q);if(length>limit&&length>0)q=q*(limit/length);return surface::Point{q.x,q.y,q.z};};
 result.enabled=true;result.shaftAcceleration=bounded(sum[0],surface::maximumShaftSupportAcceleration);
 for(unsigned i=0;i<2;i++)result.lobeAcceleration[i]=bounded(sum[i+1],surface::maximumLobeSupportAcceleration);
 return result;
}
}
