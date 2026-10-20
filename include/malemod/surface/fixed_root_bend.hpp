#pragma once
#include "root_motion.hpp"

namespace malemod::surface::fixed_root {
namespace rm=root_motion;
using Point=root_motion::Point;
// A constant proximal region followed by an integrated, smooth bend. Rotating
// each cross section preserves its radius; integrating its tangent avoids the
// constriction produced by blending rotated vertex positions.
class Bend {
 Point origin_,axis_,up_; double length_,angle_;
 static double Smooth(double t){t=std::clamp(t,0.,1.);return t*t*t*(10+t*(-15+6*t));}
 double Theta(double s)const{return angle_*Smooth(s/length_);}
 Point Rotate(Point p,double theta)const{
  const double x=rm::Dot(p,axis_),z=rm::Dot(p,up_),c=std::cos(theta),s=std::sin(theta);
  return rm::Add(p,rm::Add(rm::Scale(axis_,(c-1)*x-s*z),rm::Scale(up_,s*x+(c-1)*z)));
 }
 Point Integral(double s)const{
  const double end=std::min(s,length_),step=end/32;
  Point sum{};
  for(unsigned i=0;i<=32;i++)sum=rm::Add(sum,rm::Scale(Rotate(axis_,Theta(i*step)),i==0||i==32?1:(i%2?4:2)));
  sum=rm::Scale(sum,step/3);
  if(s>length_)sum=rm::Add(sum,rm::Scale(Rotate(axis_,angle_),s-length_));
  return sum;
 }
 public:
 Bend(Point origin,Point axis,Point up,double length,double radius,double requestedRadians,double minimumJacobian=.65):origin_(origin),axis_(rm::Unit(axis)),length_(length){
  rm::Finite(origin);rm::Finite(up);
  up_=rm::Unit(rm::Add(up,rm::Scale(axis_,-rm::Dot(up,axis_))));
  if(!std::isfinite(length)||!std::isfinite(radius)||!std::isfinite(requestedRadians)||length<=0||radius<=0)throw std::invalid_argument("Invalid fixed-root bend dimensions");
  if(!std::isfinite(minimumJacobian)||minimumJacobian<=0||minimumJacobian>=1)throw std::invalid_argument("Invalid fixed-root minimum Jacobian");
  // max Smooth' = 1.875. Bound compression explicitly, retaining positive
  // orientation throughout the radial envelope even for a compact bend.
  const double limit=(1-minimumJacobian)*length/(1.875*radius);
  angle_=std::clamp(requestedRadians,-limit,limit);
 }
 double Angle()const{return angle_;}
 Point Apply(Point p)const{
  rm::Finite(p);const auto local=rm::Add(p,rm::Scale(origin_,-1));const double s=rm::Dot(local,axis_);
  if(s<=0||angle_==0)return p;
  return rm::Add(origin_,rm::Add(Integral(s),Rotate(rm::Add(local,rm::Scale(axis_,-s)),Theta(s))));
 }
 Point Vector(Point at,Point vector,bool normal=false)const{
  const auto local=rm::Add(at,rm::Scale(origin_,-1));const double s=rm::Dot(local,axis_);
  if(s<=0||angle_==0)return vector;
  const double t=std::clamp(s/length_,0.,1.);
  const double jacobian=1-angle_/length_*30*t*t*(1-t)*(1-t)*rm::Dot(local,up_);
  if(jacobian<=0)throw std::invalid_argument("Fixed-root bend exceeds its radial envelope");
  const double factor=normal?1/jacobian:jacobian;
  return Rotate(rm::Add(vector,rm::Scale(axis_,rm::Dot(vector,axis_)*(factor-1))),Theta(s));
 }
 Point Direction(Point at,Point vector)const{
  return rm::Unit(Vector(at,vector));
 }
};
}
