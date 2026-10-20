#pragma once
#include "graft_runtime.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace malemod::surface {
// One nested-section map owns recruitment. Its anterior travel is independent
// of girth. Both elliptical radii increase monotonically toward the exterior;
// the migrating centre has a smaller derivative than either radius.
class DirectPelvicOpening {
 PrecisePoint root_{},forward_,up_,side_;
 double openingRadius_=0;
 static double Dot(PrecisePoint a,PrecisePoint b){double x=0;for(unsigned i=0;i<3;i++)x+=a[i]*b[i];return x;}
 static double Smooth(double t){t=std::clamp(t,0.,1.);return t*t*t*(10+t*(-15+6*t));}
 static double Section(double rho,double r0,double inner,double outer){
  if(rho<r0)return inner+.5*(rho-r0);
  const double length=outer-r0,width=outer-inner,t=(rho-r0)/length;
  const double m0=.5*length/width,m1=length/width,t3=t*t*t,t4=t3*t,t5=t4*t;
  return inner+width*(m0*t+(10-6*m0-4*m1)*t3+(8*m0+7*m1-15)*t4+(6-3*m0-3*m1)*t5);
 }
 public:
 DirectPelvicOpening(const std::vector<PrecisePoint>& opening,PrecisePoint forward={1,0,0},PrecisePoint up={0,0,1}):forward_(forward),up_(up){
  if(opening.size()<3||std::abs(Dot(forward,forward)-1)>1e-6||std::abs(Dot(up,up)-1)>1e-6||std::abs(Dot(up,forward))>1e-6)throw std::invalid_argument("Invalid opening frame");
  side_={up[1]*forward[2]-up[2]*forward[1],up[2]*forward[0]-up[0]*forward[2],up[0]*forward[1]-up[1]*forward[0]};
  for(auto p:opening)for(unsigned a=0;a<3;a++){if(!std::isfinite(p[a]))throw std::invalid_argument("Nonfinite opening");root_[a]+=p[a]/opening.size();}
  openingRadius_=1e100;for(auto p:opening){for(unsigned a=0;a<3;a++)p[a]-=root_[a];openingRadius_=std::min(openingRadius_,std::hypot(Dot(p,side_),Dot(p,up_)));}
  if(openingRadius_<1e-8)throw std::invalid_argument("Degenerate opening radius");
 }
 PrecisePoint Root()const{return root_;}
 PrecisePoint Displacement(PrecisePoint point,const GraftFrame& pose,double neutralRadius)const{
  if(!std::isfinite(pose.radius)||pose.radius<=0||!std::isfinite(neutralRadius)||neutralRadius<=0)throw std::invalid_argument("Invalid opening dimensions");
  const double growth=Smooth((pose.radius-neutralRadius)/(.75*neutralRadius));if(growth==0)return {};
  for(unsigned a=0;a<3;a++)if(!std::isfinite(point[a])||!std::isfinite(pose.root[a])||!std::isfinite(pose.axis[a]))throw std::invalid_argument("Nonfinite opening pose");
  const double fx=Dot(pose.axis,forward_),az=Dot(pose.axis,up_);
  if(std::abs(Dot(pose.axis,pose.axis)-1)>1e-5||fx<.12||std::abs(Dot(pose.axis,side_))>1e-5)throw std::invalid_argument("Opening pose must cross its forward plane");
  PrecisePoint q=point,live=pose.root;for(unsigned a=0;a<3;a++){q[a]-=root_[a];live[a]-=root_[a];}
  const double s=Dot(q,forward_),y=Dot(q,side_),z=Dot(q,up_),rho=std::hypot(y,z),r0=openingRadius_;
  if(rho<1e-10)return {};
  const double radius=1.08*pose.radius,allowance=.18*growth;
  const double fullY=radius*std::sqrt(1+allowance),fullZ=radius*(1+allowance)/fx;
  const double forwardOuter=r0+3*std::max({0.,fullY-r0,fullZ-r0})+2*neutralRadius;
  const double forwardBell=1-Smooth(std::max(0.,rho-r0)/(forwardOuter-r0));
  const double front=Smooth((s+3*neutralRadius)/(1.5*neutralRadius));
  const double travel=std::max(0.,Dot(live,forward_))*growth*forwardBell*front;
  const double nextS=s+travel,weight=growth*Smooth((nextS+3*neutralRadius)/(1.5*neutralRadius));
  if(weight==0)return {};
  const double innerY=r0+std::max(0.,fullY-r0)*weight,innerZ=r0+std::max(0.,fullZ-r0)*weight;
  const double centre=(Dot(live,up_)+(nextS-Dot(live,forward_))*az/fx-allowance*radius/fx)*weight;
  const double dilation=std::max(innerY,innerZ)-r0;
  const double outer=std::max(innerY,innerZ)+.81*r0+4*(dilation+std::abs(centre));
  double nextY=y,nextZ=z;
  if(rho<outer){
   const double t=std::max(0.,(rho-r0)/(outer-r0)),t3=t*t*t,t4=t3*t,t5=t4*t;
   const double bell=rho<r0?1-(rho-r0)/(outer-r0):1-t-4*t3+7*t4-3*t5;
   nextY=Section(rho,r0,innerY,outer)*y/rho;
   nextZ=Section(rho,r0,innerZ,outer)*z/rho+centre*bell;
  }
  PrecisePoint result{};for(unsigned a=0;a<3;a++)result[a]=forward_[a]*travel+side_[a]*(nextY-y)+up_[a]*(nextZ-z);return result;
 }
};
}
