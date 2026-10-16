#pragma once
#include "root_motion.hpp"
#include <memory>
namespace malemod::surface::root_articulation {
using Point=root_motion::Point;
using Face=std::array<unsigned,3>;
class Rotation {
 Point origin_,axis_,up_; double angle_;
 public:
 Rotation(Point origin,Point axis,Point up,double radians):origin_(origin),axis_(root_motion::Unit(axis)),up_(root_motion::Unit(up)),angle_(radians){
  root_motion::Finite(origin);if(!std::isfinite(radians)||std::abs(radians)>1.4||std::abs(root_motion::Dot(axis_,up_))>1e-6)throw std::invalid_argument("Invalid root articulation frame");
 }
 Point Vector(Point p)const{
  double x=root_motion::Dot(p,axis_),z=root_motion::Dot(p,up_),c=std::cos(angle_),s=std::sin(angle_);
  return root_motion::Add(p,root_motion::Add(root_motion::Scale(axis_,(c-1)*x-s*z),root_motion::Scale(up_,s*x+(c-1)*z)));
 }
 Point Apply(Point p)const{return root_motion::Add(origin_,Vector(root_motion::Add(p,root_motion::Scale(origin_,-1))));}
 double Radians()const{return angle_;}
};
// Rigid handles articulate the complete shaft about its measured root. Only
// the connecting material is solved. Fixed body/core and handle positions are
// exact Dirichlet constraints; no spatial angle mask bends the shaft itself.
// The junction minimizes the squared, degree-normalized graph Laplacian of
// displacement, including the rows on BOTH sides of each fixed boundary.
// Thus the attachment continues the boundary tangent without forcing a long
// stationary shaft neck. Reference detail is retained and rigid handles keep
// exact girth. This is not an intersection or orientation certificate.
class Junction {
 struct Impl;std::unique_ptr<Impl> impl_;
 public:
 Junction(const std::vector<Point>& reference,const std::vector<Face>& faces,const std::vector<bool>& fixed,const std::vector<bool>& moving);
 ~Junction();
 // Refresh the exact live metric without rebuilding immutable topology or
 // repeating sparse symbolic analysis. Does not freeze the input geometry.
 void UpdateReference(const std::vector<Point>& current);
 std::vector<Point> Apply(const Rotation&)const;
 // Reuse the immutable material metric and ownership with a new live pose.
 std::vector<Point> Apply(const std::vector<Point>& current,const Rotation&)const;
};
// Warm-start a difficult endpoint through the same rigid articulation path.
// Carry only the junction repair between stages. Hard handle displacement is
// still exact at the requested endpoint; this never substitutes a lesser angle.
template<class Target,class Project>
bool ContinueEndpoint(const std::vector<Point>& reference,std::vector<Point>& result,Target&& target,Project&& project,unsigned stages=8){
 if(!stages||stages>64)throw std::invalid_argument("Invalid articulation continuation");
 auto previous=reference,current=reference;
 for(unsigned step=1;step<=stages;step++){
  auto next=target(double(step)/stages);if(next.size()!=reference.size())throw std::invalid_argument("Articulation continuation topology changed");
  for(unsigned i=0;i<current.size();i++)for(unsigned k=0;k<3;k++)current[i][k]+=next[i][k]-previous[i][k];
  if(!project(current))return false;previous=std::move(next);
 }
 result=std::move(current);return true;
}
}
