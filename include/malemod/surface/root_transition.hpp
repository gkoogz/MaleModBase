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
 static double Smooth(double t){t=std::clamp(t,0.,1.);return t*t*t*(t*(t*6-15)+10);}
 public:
 RootTransition(std::vector<PrecisePoint> rest,unsigned bodyCount,const std::vector<PrecisePoint>& opening,PrecisePoint forward={1,0,0}):rest_(std::move(rest)),bodyCount_(bodyCount),forward_(forward){
  if(opening.size()<3||!bodyCount_||bodyCount_>=rest_.size())throw std::invalid_argument("Incomplete measured root transition");
  double f=0;for(double x:forward_){if(!std::isfinite(x))throw std::invalid_argument("Nonfinite pelvic forward");f+=x*x;}if(f<1e-12)throw std::invalid_argument("Degenerate pelvic forward");for(auto& x:forward_)x/=std::sqrt(f);
  for(auto p:opening)for(unsigned a=0;a<3;a++){if(!std::isfinite(p[a]))throw std::invalid_argument("Nonfinite opening");root_[a]+=p[a]/opening.size();}
  for(auto p:rest_){double best=1e100;for(auto q:opening){double ds=0;for(unsigned a=0;a<3;a++){if(!std::isfinite(p[a]))throw std::invalid_argument("Nonfinite rest transition");ds+=(p[a]-q[a])*(p[a]-q[a]);}best=std::min(best,std::sqrt(ds));}seamDistance_.push_back(best);}
 }
 PrecisePoint Root()const{return root_;}
 PrecisePoint BodyDisplacement(unsigned i,const GraftFrame& frame,double neutralRadius)const{
  if(i>=rest_.size()||!std::isfinite(neutralRadius)||neutralRadius<=0||!std::isfinite(frame.radius)||frame.radius<=0)throw std::invalid_argument("Invalid root transition dimensions");
  if(!std::isfinite(frame.sourceLengthScale)||frame.sourceLengthScale<=0||!std::isfinite(frame.length)||frame.length<=0)throw std::invalid_argument("Invalid root transition scale or length");
  PrecisePoint lateral={frame.up[1]*frame.axis[2]-frame.up[2]*frame.axis[1],frame.up[2]*frame.axis[0]-frame.up[0]*frame.axis[2],frame.up[0]*frame.axis[1]-frame.up[1]*frame.axis[0]};
  double aa=0,uu=0,au=0;for(unsigned a=0;a<3;a++){aa+=frame.axis[a]*frame.axis[a];uu+=frame.up[a]*frame.up[a];au+=frame.axis[a]*frame.up[a];}if(!std::isfinite(aa+uu+au)||std::abs(aa-1)>1e-5||std::abs(uu-1)>1e-5||std::abs(au)>1e-5)throw std::invalid_argument("Invalid transition frame");
  PrecisePoint q{},d{};double s=0,z=0,y=0;
  for(unsigned a=0;a<3;a++){q[a]=rest_[i][a]-root_[a];s+=q[a]*frame.axis[a];z+=q[a]*frame.up[a];y+=q[a]*lateral[a];}
  const double rho=std::hypot(y,z),scale=frame.sourceLengthScale;
  const double envelope=frame.radius*1.55+2*scale,reach=frame.radius*1.25+3*scale;
  const double w=(1-Smooth((rho-envelope)/(5*scale)))*(1-Smooth(std::abs(s)/reach));
  // Recruit toward a bounded barrel, rather than scaling the whole pelvic
  // cross-section. Multiplying distant radial coordinates by the shaft size
  // creates an abdominal spike even when the attachment itself stays sewn.
  const double growth=Smooth((frame.radius-neutralRadius)/(4.72*scale));
  const double profile=1-Smooth((s/frame.length+.03)/.26);
  const double barrel=frame.radius*(1.025+(.06+.12*growth)*profile);
  const double neutralBarrel=neutralRadius*(1.025+.06*profile);
  const double gap=std::max(0.,std::max(0.,barrel-rho)-std::max(0.,neutralBarrel-rho))*w;
  if(rho>1e-8*scale)for(unsigned a=0;a<3;a++)d[a]=(frame.up[a]*z+lateral[a]*y)*gap/rho;
  // The lower ramp may grow down/out, never backward into the thighs.
  double backward=0;for(unsigned a=0;a<3;a++)backward+=d[a]*forward_[a];if(backward<0)for(unsigned a=0;a<3;a++)d[a]-=backward*forward_[a];return d;
 }
 std::vector<PrecisePoint> Evaluate(const std::vector<PrecisePoint>& completeDelta,const GraftFrame& frame,double neutralRadius)const{
  if(completeDelta.size()!=rest_.size())throw std::invalid_argument("Root transition topology differs");
  auto out=completeDelta;const double reach=2*neutralRadius;
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
