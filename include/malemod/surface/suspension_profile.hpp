#pragma once
#include "presentation.hpp"
#include <vector>

namespace malemod::surface {
// A material cross-section, from the retained sac toward the mounted rim.
// Endpoint values and first derivatives remain exact. Only generated skin is
// included: neither the complete shaft nor either retained lobe is reshaped.
struct SuspensionSection {double t=0;std::vector<unsigned> vertices;};
inline double SuspensionSectionScale(double t,double hang){
 if(!std::isfinite(t)||t<0||t>1||!std::isfinite(hang)||hang<1||hang>100)
  throw std::invalid_argument("Invalid suspended neck profile coordinate");
 const double u=(hang-1)/99.,h=u*u*(3-2*u);
 // The waist is above the lobes, at two thirds of the neck. The broad lower
 // transition and upper trumpet have zero displacement and slope at the joins.
 const double bell=(729./16.)*t*t*t*t*(1-t)*(1-t);
 return 1-(.38*h-.10*(1-h))*bell;
}
inline void ShapeSuspensionSections(std::vector<PresentationPoint>& points,
 const std::vector<SuspensionSection>& sections,double hang){
 for(const auto& section:sections){
  const double scale=SuspensionSectionScale(section.t,hang);
  const auto& ids=section.vertices;
  if(ids.size()<3)throw std::invalid_argument("Suspended neck section is not closed");
  PresentationPoint center{};double perimeter=0;
  for(unsigned j=0;j<ids.size();j++){
   const auto a=points.at(ids[j]),b=points.at(ids[(j+1)%ids.size()]);
   const double length=std::sqrt(Dot(Sub(b,a),Sub(b,a)));
   center=Add(center,Scale(Add(a,b),.5*length));perimeter+=length;
  }
  if(perimeter<1e-12)throw std::invalid_argument("Collapsed suspended neck section");
  center=Scale(center,1/perimeter);
  for(auto id:ids)points[id]=Add(center,Scale(Sub(points[id],center),scale));
 }
}
}
