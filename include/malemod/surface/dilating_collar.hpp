#pragma once
#include "graft_runtime.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace malemod::surface {
// Diagnostic radial recruitment. The adapter must constrain original edge
// donors and validate the final composed surface; this field alone is not
// proof of a valid attachment. Coordinates are adapter-calibrated.
class DilatingCollar {
 using P=PrecisePoint;
 P root_{};
 std::vector<P> rest_;
 std::vector<std::pair<double,double>> section_;
 std::vector<bool> mouth_;
 static double Smooth(double t){t=std::clamp(t,0.,1.);return t*t*t*(10+t*(-15+6*t));}
 double RestRadius(double phi)const{
  auto hi=std::upper_bound(section_.begin(),section_.end(),std::pair<double,double>{phi,1e100});
  const auto b=hi==section_.end()?section_.front():*hi;
  const auto a=hi==section_.begin()?section_.back():*(hi-1);
  double span=b.first-a.first,q=phi-a.first;constexpr double tau=6.283185307179586;
  if(span<=0)span+=tau;if(q<0)q+=tau;
  // Ray intersection with the original polygon edge, rather than averaging
  // radii: original seam vertices and edge donors remain on their own cutout.
  const double den=b.second*std::sin(span-q)+a.second*std::sin(q);
  return den>1e-10?a.second*b.second*std::sin(span)/den:(a.second+b.second)*.5;
 }
 public:
 DilatingCollar(std::vector<P> rest,const std::vector<P>& commonOpening,const std::vector<unsigned>& mouth):rest_(std::move(rest)),mouth_(rest_.size()){
  if(commonOpening.size()<3)throw std::invalid_argument("Missing measured collar boundary");
  for(auto p:commonOpening)for(unsigned a=0;a<3;a++)root_[a]+=p[a]/commonOpening.size();
  for(auto p:commonOpening){double y=p[1]-root_[1],z=p[2]-root_[2];section_.emplace_back(std::atan2(z,y),std::hypot(y,z));}
  std::sort(section_.begin(),section_.end());
  for(auto i:mouth){if(i>=rest_.size())throw std::invalid_argument("Invalid collar boundary index");mouth_[i]=true;}
 }
 P Displacement(unsigned i,P center,double radius,double neutral,double radians=0)const{
  if(i>=rest_.size()||!std::isfinite(radius+neutral+radians)||radius<=0||neutral<=0||std::abs(radians)>1.1)throw std::invalid_argument("Invalid collar section");
  auto p=rest_[i];double y=p[1]-root_[1],z=p[2]-root_[2],rho=std::hypot(y,z);
  if(rho<1e-9)return {};
  const double uy=y/rho,uz=z/rho,phi=std::atan2(z,y);
  // Edge donors are not all on the opening itself. Projecting every donor
  // onto the same radius erases their radial spacing and folds the body faces
  // between them. Use the measured boundary section for every donor as well.
  const double r0=RestRadius(phi);
  const double target=radius*(1+.10*std::pow(std::max(0.,-uz),2));
  const double shift=std::hypot(center[1]-root_[1],center[2]-root_[2]);
  const double outer=std::max(r0,target)+.8*r0+3*(std::abs(target-r0)+shift);
  if(rho>=outer)return {};
  const double length=outer-r0,width=outer-target,t=std::max(0.,(rho-r0)/length);
  const double m0=.5*length/width,m1=length/width,t3=t*t*t,t4=t3*t,t5=t4*t;
  const double h=m0*t+(10-6*m0-4*m1)*t3+(8*m0+7*m1-15)*t4+(6-3*m0-3*m1)*t5;
  const double next=rho<r0?target+.5*(rho-r0):target+width*h;
  const double bell=1-Smooth(t),front=mouth_[i]?1:Smooth((p[0]-root_[0]+3*neutral)/(1.5*neutral));
  const double relief=1-Smooth(std::max(0.,next-target)/(.5*radius));
  const double forward=std::max(0.,center[0]+.10*radius-p[0])*relief*front;
  P d{forward,
      ((next-rho)*uy+(center[1]-root_[1])*bell)*front,
      ((next-rho)*uz+(center[2]-root_[2]+target*uz*(1/std::cos(radians)-1)+std::tan(radians)*(p[0]+forward-center[0]))*bell)*front};
  return d;
 }
};
}
