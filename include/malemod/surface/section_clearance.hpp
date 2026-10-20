#pragma once
#include "attachment_corrective.hpp"
namespace malemod::surface::corrective {
// Response to an observed upper opening deficit, in the caller's X-forward,
// Z-up chart. Radius scaling is handled independently by the deformation field.
// Sampling excludes the body side of the sewn boundary and lower/side arcs.
inline double UpperClearanceResponse(const std::vector<Point>& points,const std::vector<unsigned>& ids,
 const std::vector<Point>& delta,Point center,Point axis,double radius,double boundaryX){
 if(ids.size()!=delta.size()||!Finite(center)||!Finite(axis)||!std::isfinite(radius+boundaryX)||radius<=0)throw std::invalid_argument("Invalid upper clearance sample");
 axis=Unit(axis);if(axis[0]<=.1)return 0;
 double maximum=0;for(auto d:delta)maximum=std::max(maximum,d[2]);if(maximum<1e-10)return 0;
 std::vector<double> required;
 for(unsigned j=0;j<ids.size();j++){
  if(ids[j]>=points.size())throw std::invalid_argument("Invalid upper clearance row");auto p=points[ids[j]];
  if(delta[j][2]<.2*maximum||p[0]<=boundaryX||std::abs(p[1]-center[1])>=.2*radius)continue;
  double lateral=p[1]-center[1],height=center[2]+(p[0]-center[0])*axis[2]/axis[0]+std::sqrt(std::max(0.,radius*radius-lateral*lateral))/axis[0];
  required.push_back((height-p[2])/delta[j][2]);
 }
 if(required.empty())return 0;
 // Discard the bottom decile of noisy support samples, then choose the
 // conservative deficit. No maximum-size slider fraction attenuates it again.
 auto index=(required.size()-1)/10;std::nth_element(required.begin(),required.begin()+index,required.end());
 return std::clamp(required[index],0.,1.);
}
}
