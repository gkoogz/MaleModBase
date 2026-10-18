#pragma once
#include "modular_motion.hpp"

namespace malemod::surface::modular {
// Fair the longitudinal ventral crest in immutable material space. The old
// combined source applies several overlapping underside supports and local
// triangle backoffs. Their radial bulge must not become permanent rod shape.
// This is a surface correction, not a force or a world-space straightener.
inline std::vector<Vec> RapheProfileCorrection(const std::vector<Vec>& points,
 const std::vector<Binding>& owners,const PresentationFrames& frames,
 const std::vector<double>& crown,double radius){
 if(points.size()!=owners.size()||points.size()!=crown.size()||!std::isfinite(radius)||radius<=0)
  throw std::invalid_argument("Invalid raphe material profile");
 std::vector<Vec> delta(points.size()),local(points.size());std::vector<double> station(points.size());
 auto binding=BindMaterialStations(points,owners,frames);std::array<double,12> arc{};
 for(unsigned i=1;i<12;i++){auto d=Sub(frames[i+1].origin,frames[i].origin);arc[i]=arc[i-1]+std::sqrt(Dot(d,d));}
 if(arc.back()<1e-8)throw std::invalid_argument("Collapsed raphe material guide");
 std::vector<double> reference;
 for(unsigned k=0;k<points.size();k++)if(ShaftBinding(owners[k])){
  auto a=binding[k].influences[0],b=binding[k].influences[1];
  auto frame=MaterialSection(frames,a.frame,b.frame,b.weight);
  local[k]=Unrotate(frame.basis,Sub(points[k],frame.origin));
  station[k]=(arc[a.frame-1]*(1-b.weight)+arc[b.frame-1]*b.weight)/arc.back();
  const double depth=-local[k][2];
  // Measure the existing proximal crest, not a hardcoded anatomical size.
  if(station[k]>.02&&station[k]<.20&&std::abs(local[k][1])<.15*radius&&depth>.8*radius&&crown[k]==0)
   reference.push_back(depth);
 }
 if(reference.size()<3)return delta; // No measured exposed crest in this asset.
 std::sort(reference.begin(),reference.end());const double depth=reference[reference.size()/2];
 auto ease=[](double t){t=std::clamp(t,0.,1.);return t*t*t*(10+t*(-15+6*t));};
 for(unsigned k=0;k<points.size();k++)if(ShaftBinding(owners[k])&&crown[k]<1){
  const double s=station[k],y=std::abs(local[k][1]),z=-local[k][2];
  // Root/socket and the whole authored crown/frenulum stay exact. The distal
  // fade restores the source taper with zero first and second derivatives.
  const double axial=ease((s-.12)/.20)*(1-ease((s-.48)/.26))*(1-ease(crown[k]));
  const double lateral=1-ease((y/radius-.20)/.45),ventral=ease((z/radius-.75)/.35);
  if(axial*lateral*ventral==0)continue;
  const double cap=.45*radius,u=(std::min)(y,.8*cap),height=std::sqrt(cap*cap-u*u);
  const double target=depth-cap+height-(y-u)*u/height;
  const double amount=std::clamp(target-z,-.20*radius,.20*radius)*axial*lateral*ventral;
  auto a=binding[k].influences[0],b=binding[k].influences[1];
  auto frame=MaterialSection(frames,a.frame,b.frame,b.weight);
  delta[k]=Scale(frame.basis[2],-amount);
 }
 return delta;
}
}
