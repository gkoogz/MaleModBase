#pragma once
#include "modular_motion.hpp"
#include <limits>

namespace malemod::surface::modular {
inline double SuspensionLiftWeight(double hang){
 if(!std::isfinite(hang)||hang<1||hang>100)throw std::invalid_argument("Invalid modular suspension control");
 const double u=std::clamp((50-hang)/49.,0.,1.);return u*u*(3-2*u);
}
// The modular neck has a measured free span that the legacy one-unit low-end
// control cannot retract. Close that span at rest, preserving the moving lobe
// offsets and its volume. Never derive this correction from the live pose.
inline double SuspensionLift(const std::vector<Vec>& material,const std::vector<Loft>& lofts,
 double radius,double hang,Vec up={0,0,1}){
 if(!std::isfinite(hang)||hang<1||hang>100)
  throw std::invalid_argument("Invalid modular suspension control");
 if(hang>=50)return 0;
 if(!std::isfinite(radius)||radius<=0)throw std::invalid_argument("Invalid modular suspension radius");
 up=Unit(up);double freeSpan=std::numeric_limits<double>::infinity();bool found=false;
 for(const auto& l:lofts)if(ShaftBinding(l.lastOwner)&&!ShaftBinding(l.firstOwner)){
  bool suspended=false;for(auto i:l.firstOwner.influences)suspended|=i.frame>=13&&i.weight>0;
  if(suspended){freeSpan=(std::min)(freeSpan,Dot(Sub(material.at(l.end),material.at(l.start)),up));found=true;}
 }
 if(!found)throw std::invalid_argument("Suspension has no measured neck span");
 const double weight=SuspensionLiftWeight(hang);
 // Leave a radius-scaled transition at the shortest boundary column. The
 // whole retained sac translates uniformly; connecting skin is lofted again.
 return .85*(std::max)(0.,freeSpan-.2*radius)*weight;
}
}
