#pragma once
#include "root_motion.hpp"

namespace malemod::surface::attachment_hinge {
namespace rm=root_motion;
using Point=rm::Point;
// A transient pose may have less clearance than its settled reference. Test
// the requested endpoint first, then restrict angular travel without changing
// the attachment, girth, or mechanical source state. The caller records gain.
template<class Accept> double AdmissibleGain(Accept&& accept){
 for(double gain:{1.,.75,.5,.25,.125,0.})if(accept(gain))return gain;
 throw std::runtime_error("No admissible attachment angle, including reference");
}
// Integrate a smooth spatial rotation field. Unlike a per-vertex ownership
// multiplier, material moving through the transition follows one common flow.
// The fixed half-space and measured protected box are exact equilibria.
class Flow {
 Point origin_,axis_,up_,minimum_,maximum_; double width_,angle_,far_;
 static double Smooth(double t){t=std::clamp(t,0.,1.);return t*t*t*(10+t*(-15+6*t));}
 Point Velocity(Point p)const{
  auto local=rm::Add(p,rm::Scale(origin_,-1));double s=rm::Dot(local,axis_),z=rm::Dot(local,up_),distance=0;
  for(unsigned k=0;k<3;k++){double d=std::max({minimum_[k]-p[k],p[k]-maximum_[k],0.});distance+=d*d;}
  const double gain=angle_*Smooth(s/width_)*Smooth(std::sqrt(distance)/width_);
  return rm::Scale(rm::Add(rm::Scale(up_,s),rm::Scale(axis_,-z)),gain);
 }
 Point Rotate(Point p)const{
  auto q=rm::Add(p,rm::Scale(origin_,-1));double x=rm::Dot(q,axis_),z=rm::Dot(q,up_),c=std::cos(angle_),s=std::sin(angle_);
  return rm::Add(p,rm::Add(rm::Scale(axis_,(c-1)*x-s*z),rm::Scale(up_,s*x+(c-1)*z)));
 }
 public:
 Flow(Point origin,Point axis,Point up,Point minimum,Point maximum,double width,double radians):origin_(origin),axis_(rm::Unit(axis)),minimum_(minimum),maximum_(maximum),width_(width),angle_(radians){
  rm::Finite(origin);rm::Finite(up);rm::Finite(minimum);rm::Finite(maximum);
  up_=rm::Unit(rm::Add(up,rm::Scale(axis_,-rm::Dot(up,axis_))));
  if(!std::isfinite(width+radians)||width<=0||std::abs(radians)>1.2)throw std::invalid_argument("Invalid attachment hinge dimensions");
  far_=width;double box=0;
  for(unsigned k=0;k<3;k++){if(minimum[k]>maximum[k])throw std::invalid_argument("Invalid attachment exclusion box");box+=axis_[k]*(axis_[k]>=0?maximum[k]-origin[k]:minimum[k]-origin[k]);}
  far_=std::max(far_,box+width);
 }
 Point Apply(Point p)const{
  rm::Finite(p);auto velocity=Velocity(p);if(angle_==0||rm::Dot(velocity,velocity)==0)return p;
  auto rotated=Rotate(p);
  // Positive axial coordinate is concave over this sub-90-degree rotation;
  // endpoint clearance therefore certifies the complete rigid arc.
  if(rm::Dot(rm::Add(p,rm::Scale(origin_,-1)),axis_)>far_&&rm::Dot(rm::Add(rotated,rm::Scale(origin_,-1)),axis_)>far_)return rotated;
  constexpr double step=1./24;
  for(unsigned i=0;i<24;i++){
   auto a=Velocity(p),b=Velocity(rm::Add(p,rm::Scale(a,step/2))),c=Velocity(rm::Add(p,rm::Scale(b,step/2))),d=Velocity(rm::Add(p,rm::Scale(c,step)));
   p=rm::Add(p,rm::Scale(rm::Add(rm::Add(a,d),rm::Scale(rm::Add(b,c),2)),step/6));
  }return p;
 }
 Point Direction(Point at,Point direction)const{
  auto v=rm::Unit(direction);double h=width_*1e-5;
  return rm::Unit(rm::Add(Apply(rm::Add(at,rm::Scale(v,h))),rm::Scale(Apply(rm::Add(at,rm::Scale(v,-h))),-1)));
 }
};
}
