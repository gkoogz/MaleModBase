#pragma once
#include "graft_runtime.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace malemod::surface {
// A measured rest opening and the independently moving anatomy share a C2
// displacement field at the attachment. No game units, bones or vertex IDs.
class RootTransition {
 std::vector<PrecisePoint> rest_;
 std::vector<double> seamDistance_;
 unsigned bodyCount_;
 PrecisePoint root_{};
 PrecisePoint forward_;
 PrecisePoint openingUp_;
 PrecisePoint openingSide_;
 std::vector<double> openingRadius_;
 static double Smooth(double t){t=std::clamp(t,0.,1.);return t*t*t*(t*(t*6-15)+10);}
 public:
 RootTransition(std::vector<PrecisePoint> rest,unsigned bodyCount,const std::vector<PrecisePoint>& opening,PrecisePoint forward={1,0,0},PrecisePoint openingUp={0,0,1}):rest_(std::move(rest)),bodyCount_(bodyCount),forward_(forward),openingUp_(openingUp){
  if(opening.size()<3||!bodyCount_||bodyCount_>=rest_.size())throw std::invalid_argument("Incomplete measured root transition");
  double f=0;for(double x:forward_){if(!std::isfinite(x))throw std::invalid_argument("Nonfinite pelvic forward");f+=x*x;}if(f<1e-12)throw std::invalid_argument("Degenerate pelvic forward");for(auto& x:forward_)x/=std::sqrt(f);
  for(auto p:opening)for(unsigned a=0;a<3;a++){if(!std::isfinite(p[a]))throw std::invalid_argument("Nonfinite opening");root_[a]+=p[a]/opening.size();}
  double u=0,fu=0;for(unsigned a=0;a<3;a++){if(!std::isfinite(openingUp_[a]))throw std::invalid_argument("Nonfinite opening up");u+=openingUp_[a]*openingUp_[a];fu+=openingUp_[a]*forward_[a];}if(std::abs(u-1)>1e-5||std::abs(fu)>1e-5)throw std::invalid_argument("Opening frame must be orthonormal");
  openingSide_={openingUp_[1]*forward_[2]-openingUp_[2]*forward_[1],openingUp_[2]*forward_[0]-openingUp_[0]*forward_[2],openingUp_[0]*forward_[1]-openingUp_[1]*forward_[0]};
  // Cache a smooth angular section of this measured opening, not the source
  // character's nominal circular radius. This is immutable instance data.
  std::vector<std::array<double,3>> section;
  for(auto p:opening){double z=0,y=0;for(unsigned a=0;a<3;a++){z+=(p[a]-root_[a])*openingUp_[a];y+=(p[a]-root_[a])*openingSide_[a];}double r=std::hypot(z,y);if(r>1e-10)section.push_back({z/r,y/r,r});}
  if(section.size()<3)throw std::invalid_argument("Degenerate opening cross section");
  for(auto p:rest_){double z=0,y=0;for(unsigned a=0;a<3;a++){z+=(p[a]-root_[a])*openingUp_[a];y+=(p[a]-root_[a])*openingSide_[a];}double r=std::hypot(z,y),total=0,weighted=0;const double uz=r>1e-10?z/r:1,uy=r>1e-10?y/r:0;for(auto q:section){double w=std::exp(24*(uz*q[0]+uy*q[1]-1));weighted+=w*q[2];total+=w;}openingRadius_.push_back(weighted/total);}
  for(auto p:rest_){double best=1e100;for(auto q:opening){double ds=0;for(unsigned a=0;a<3;a++){if(!std::isfinite(p[a]))throw std::invalid_argument("Nonfinite rest transition");ds+=(p[a]-q[a])*(p[a]-q[a]);}best=std::min(best,std::sqrt(ds));}seamDistance_.push_back(best);}
 }
 PrecisePoint Root()const{return root_;}
 PrecisePoint BodyDisplacement(unsigned i,const GraftFrame& frame,double neutralRadius)const{
  if(i>=rest_.size()||!std::isfinite(neutralRadius)||neutralRadius<=0||!std::isfinite(frame.radius)||frame.radius<=0)throw std::invalid_argument("Invalid root transition dimensions");
  if(!std::isfinite(frame.sourceLengthScale)||frame.sourceLengthScale<=0||!std::isfinite(frame.length)||frame.length<=0)throw std::invalid_argument("Invalid root transition scale or length");
  double aa=0,uu=0,au=0;for(unsigned a=0;a<3;a++){aa+=frame.axis[a]*frame.axis[a];uu+=frame.up[a]*frame.up[a];au+=frame.axis[a]*frame.up[a];}if(!std::isfinite(aa+uu+au)||std::abs(aa-1)>1e-5||std::abs(uu-1)>1e-5||std::abs(au)>1e-5)throw std::invalid_argument("Invalid transition frame");
  PrecisePoint q{},d{};double s=0,z=0,y=0;
  for(unsigned a=0;a<3;a++){q[a]=rest_[i][a]-root_[a];s+=q[a]*forward_[a];z+=q[a]*openingUp_[a];y+=q[a]*openingSide_[a];}
  const double rho=std::hypot(y,z),scale=frame.sourceLengthScale,r0=openingRadius_[i];
  const double growth=Smooth((frame.radius-neutralRadius)/(.75*neutralRadius));
  const double dilation=std::max(0.,frame.radius*1.04-r0)*growth;
  if(dilation<=1e-12*scale)return d;
  const double inner=r0+dilation,width=.45*r0+1.5*dilation,outer=inner+width,length=outer-r0;
  if(rho>=outer)return d;
  const double innerSlope=.5;
  const double t=(rho-r0)/length,m0=innerSlope*length/width,m1=length/width;
  // A monotone Hermite bell connects the expanded section to the unchanged
  // pelvis. Its outer position, tangent and curvature match the original
  // surface. A small positive inner radial slope preserves distinct rings.
  const double clamped=std::max(0.,t),t2=clamped*clamped,t3=t2*clamped,t4=t3*clamped,t5=t4*clamped;
  const double hermite=m0*clamped+(10-6*m0-4*m1)*t3+(8*m0+7*m1-15)*t4+(6-3*m0-3*m1)*t5;
  const double radius=t<0?inner+innerSlope*(rho-r0):inner+width*hermite;
  const double bell=t<0?1-t:1-clamped-4*t3+7*t4-3*t5;
  const double anterior=Smooth((s+3*neutralRadius)/(1.5*neutralRadius));
  const double depth=1-Smooth(std::max(0.,s)/(2*inner));
  const double weight=anterior*depth;
  if(rho>1e-8*scale)for(unsigned a=0;a<3;a++)d[a]=(openingUp_[a]*z+openingSide_[a]*y)*(radius-rho)*weight/rho;
  for(unsigned a=0;a<3;a++)d[a]+=forward_[a]*(.6*dilation*bell*weight);
  // The lower ramp may grow down/out, never backward into the thighs.
  double backward=0;for(unsigned a=0;a<3;a++)backward+=d[a]*forward_[a];if(backward<0)for(unsigned a=0;a<3;a++)d[a]-=backward*forward_[a];return d;
 }
 std::vector<PrecisePoint> Evaluate(const std::vector<PrecisePoint>& completeDelta,const GraftFrame& frame,double neutralRadius)const{
  if(completeDelta.size()!=rest_.size())throw std::invalid_argument("Root transition topology differs");
  auto out=completeDelta;const double reach=std::max(2*neutralRadius,1.5*frame.radius);
  for(unsigned i=0;i<out.size();i++){
   auto d=BodyDisplacement(i,frame,neutralRadius);
   // Preserve the complete morphology field outside the sewn neighborhood,
   // including recruitment that crosses existing torso/leg resource joins.
   // Only the local opening replaces that field with the measured transition.
   const double t=Smooth(seamDistance_[i]/reach);
   for(unsigned a=0;a<3;a++){if(!std::isfinite(out[i][a]))throw std::invalid_argument("Nonfinite transition input");out[i][a]=(1-t)*d[a]+t*out[i][a];}
  }
  return out;
 }
};
}
