#pragma once
#include "root_motion.hpp"

namespace malemod::surface {
// Outward-only support for the dorsal proximal shaft. The adapter supplies
// immutable material ownership and every original edge donor. Apply after
// articulation, using the same posed mechanical guide as the visible shaft.
// This preserves axial placement and does not reshape the scrotal surface.
class CollarRadialSupport {
 using P=root_motion::Point;
 std::vector<double> weights_;
 static double Smooth(double x){x=std::clamp(x,0.,1.);return x*x*x*(10+x*(-15+6*x));}
 public:
 CollarRadialSupport(const std::vector<bool>& body,const std::vector<double>& station,
  const std::vector<double>& angle,const std::vector<double>& lobeRole,const std::vector<unsigned>& fixed):weights_(body.size()){
  if(body.empty()||station.size()!=body.size()||angle.size()!=body.size()||lobeRole.size()!=body.size())throw std::invalid_argument("Invalid collar material topology");
  for(unsigned i=0;i<body.size();i++){
   if(!std::isfinite(station[i])||!std::isfinite(angle[i])||!std::isfinite(lobeRole[i])||station[i]<0||lobeRole[i]<0||lobeRole[i]>1)throw std::invalid_argument("Invalid collar material coordinates");
   if(!body[i])weights_[i]=(1-Smooth((lobeRole[i]-.05)/.40))*(1-Smooth((station[i]-.20)/.10))*Smooth((std::sin(angle[i])+.15)/.50);
  }
  for(auto i:fixed){if(i>=weights_.size())throw std::invalid_argument("Invalid collar donor");weights_[i]=0;}
 }
 std::vector<P> Apply(const std::vector<P>& rest,const std::vector<P>& displacement,const std::vector<P>& guide,double radius)const{
  using namespace root_motion;
  if(guide.size()<2||rest.size()!=weights_.size()||displacement.size()!=rest.size()||!std::isfinite(radius)||radius<=0)throw std::invalid_argument("Invalid collar support frame");
  bool valid=false;for(auto p:guide)Finite(p);
  for(unsigned j=1;j<guide.size();j++){auto d=Add(guide[j],Scale(guide[j-1],-1));valid=valid||Dot(d,d)>1e-12;}
  if(!valid)throw std::invalid_argument("Degenerate collar guide");
  auto out=displacement;
  for(unsigned i=0;i<rest.size();i++){
   Finite(rest[i]);Finite(displacement[i]);if(weights_[i]==0)continue;
   P current=Add(rest[i],out[i]),center{};double closest=INFINITY;
   for(unsigned j=0;j+1<guide.size();j++){
    auto delta=Add(guide[j+1],Scale(guide[j],-1));double length2=Dot(delta,delta);if(length2<1e-12)continue;
    double fraction=Dot(Add(current,Scale(guide[j],-1)),delta)/length2;
    if(j>0)fraction=std::max(0.,fraction);if(j+2<guide.size())fraction=std::min(1.,fraction);
    auto sample=Add(guide[j],Scale(delta,fraction)),v=Add(current,Scale(sample,-1));double squared=Dot(v,v);
    if(squared<closest){closest=squared;center=sample;}
   }
   auto radial=Add(current,Scale(center,-1));double measured=std::sqrt(Dot(radial,radial));
   if(measured>1e-9&&measured<radius)out[i]=Add(out[i],Scale(radial,weights_[i]*(radius/measured-1)));
  }
  return out;
 }
};
}
