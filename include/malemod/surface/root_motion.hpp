#pragma once
#include <array>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace malemod::surface::root_motion {
using Point=std::array<double,3>;
inline Point Add(Point a,Point b){for(unsigned k=0;k<3;k++)a[k]+=b[k];return a;}
inline Point Scale(Point p,double s){for(auto& x:p)x*=s;return p;}
inline double Dot(Point a,Point b){double d=0;for(unsigned k=0;k<3;k++)d+=a[k]*b[k];return d;}
inline Point Cross(Point a,Point b){return {a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]};}
inline Point Unit(Point p){double n=std::sqrt(Dot(p,p));if(!std::isfinite(n)||n<1e-12)throw std::invalid_argument("Invalid root motion direction");return Scale(p,1/n);}
inline void Finite(Point p){for(double x:p)if(!std::isfinite(x))throw std::invalid_argument("Nonfinite root motion point");}
class SewnWeights {
 std::vector<double> distances_;
 public:
 SewnWeights(const std::vector<Point>& rest,const std::vector<Point>& opening){
  if(rest.empty()||opening.size()<3)throw std::invalid_argument("Incomplete sewn root support");
  for(auto p:opening)Finite(p);
  for(auto p:rest){Finite(p);double closest=INFINITY;for(auto q:opening){auto d=Add(p,Scale(q,-1));closest=std::min(closest,Dot(d,d));}distances_.push_back(std::sqrt(closest));}
 }
 double Weight(unsigned i,double reach)const{
  if(i>=distances_.size()||!std::isfinite(reach)||reach<=0)throw std::invalid_argument("Invalid sewn root reach");
  const double t=std::clamp(distances_[i]/reach,0.,1.);return t*t*t*(t*(t*6-15)+10);
 }
};

// Opt-in interpolation of an observed moving attachment. A linear blend of
// rotated positions collapses its cross section (and collapses an antipodal
// section completely). Interpolate the proper rotation, with morphology and
// elastic residuals expressed in its material frame. No game units or joints.
// This preserves the exact fixed/moving endpoints; it is not a collision or
// orientation certificate for a spatially varying transition field.
class Rotation {
 Point axis_{};
 double angle_=0;
 public:
 Rotation(Point neutralDirection,Point movingDirection,Point halfTurnAxis){
  const auto a=Unit(neutralDirection),b=Unit(movingDirection);
  const double dot=std::clamp(Dot(a,b),-1.,1.);auto cross=Cross(a,b);
  const double sine=std::sqrt(Dot(cross,cross));angle_=std::atan2(sine,dot);
  if(sine>1e-12)axis_=Scale(cross,1/sine);
  else if(dot<0){
   Finite(halfTurnAxis);axis_=Unit(Add(halfTurnAxis,Scale(a,-Dot(halfTurnAxis,a))));
  }else{axis_={1,0,0};angle_=0;}
 }
 Point Apply(Point p,double fraction)const{
  Finite(p);if(!std::isfinite(fraction))throw std::invalid_argument("Invalid root rotation fraction");
  if(angle_==0||fraction==0)return p;
  const double angle=angle_*fraction,c=std::cos(angle),s=std::sin(angle);
  return Add(Add(Scale(p,c),Scale(Cross(axis_,p),s)),Scale(axis_,Dot(axis_,p)*(1-c)));
 }
 double Angle()const{return angle_;}
 Point Blend(Point rest,Point root,Point fixedDelta,Point movingDelta,double weight)const{
  Finite(rest);Finite(root);Finite(fixedDelta);Finite(movingDelta);
  if(!std::isfinite(weight)||weight<0||weight>1)throw std::invalid_argument("Invalid sewn root motion weight");
  if(weight==0)return fixedDelta;
  if(weight==1)return movingDelta;
  if(angle_==0)return Add(Scale(fixedDelta,1-weight),Scale(movingDelta,weight));
  const auto local=Add(rest,Scale(root,-1));
  const auto material=Apply(Add(local,movingDelta),-1);
  const auto blended=Add(Scale(Add(local,fixedDelta),1-weight),Scale(material,weight));
  return Add(Apply(blended,weight),Scale(local,-1));
 }
};
}
