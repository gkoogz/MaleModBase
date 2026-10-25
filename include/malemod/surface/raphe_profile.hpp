#pragma once
#include "modular_motion.hpp"

namespace malemod::surface::modular {
inline double RapheCorrectionFraction(const std::vector<Vec>& before,
 const std::vector<Vec>& after,const std::vector<std::array<unsigned,3>>& faces);
// The retained shaft/crown join can double back axially and form a ventral
// spur. A radial clamp alone flattens that fold into overlapping triangles.
// Fair both axial and radial coordinates in a compact material-space patch,
// retaining lateral coordinates and coincident aliases across both LODs.
inline std::vector<Vec> FrenulumProfileCorrection(const std::vector<Vec>& points,
 const std::vector<Binding>& owners,const PresentationFrames& frames,
 const std::vector<std::array<unsigned,3>>& faces,double radius){
 if(points.size()!=owners.size()||!std::isfinite(radius)||radius<=0)throw std::invalid_argument("Invalid frenulum material profile");
 auto bindings=BindMaterialStations(points,owners,frames);std::array<double,12> arc{};
 for(unsigned i=1;i<12;i++){auto d=Sub(frames[i+1].origin,frames[i].origin);arc[i]=arc[i-1]+std::sqrt(Dot(d,d));}
 std::vector<Vec> delta(points.size()),local(points.size()),side(points.size()),up(points.size()),axis(points.size());std::vector<double> station(points.size());double extent=0;
 PreparedMaterialSections sections(frames);
 for(unsigned i=0;i<points.size();i++)if(ShaftBinding(owners[i])){
  const auto a=bindings[i].influences[0],b=bindings[i].influences[1];auto frame=sections.Section(a.frame,b.frame,b.weight);
  local[i]=Unrotate(frame.basis,Sub(points[i],frame.origin));side[i]=frame.basis[1];up[i]=frame.basis[2];axis[i]=frame.basis[0];
  // The crown extends past the last rod station. Retain that material offset
  // instead of clamping its entire frenulum to station one.
  station[i]=arc[a.frame-1]*(1-b.weight)+arc[b.frame-1]*b.weight+local[i][0];extent=(std::max)(extent,station[i]);
 }
 if(extent<1e-8)return delta;
 auto ease=[](double t){t=std::clamp(t,0.,1.);return t*t*t*(10+t*(-15+6*t));};
 std::vector<double> proximal,distal;
 for(unsigned i=0;i<points.size();i++)if(ShaftBinding(owners[i])&&std::abs(local[i][1])<.12*radius&&-local[i][2]>.3*radius){
  const double s=station[i]/extent;
  if(std::abs(s-.70)<=.015+1e-12)proximal.push_back(-local[i][2]);
  if(std::abs(s-.91)<=.010+1e-12)distal.push_back(-local[i][2]);
 }
 const bool profile=proximal.size()>=3&&distal.size()>=3;
 auto median=[](auto& values){std::sort(values.begin(),values.end());return .5*(values[(values.size()-1)/2]+values[values.size()/2]);};
 const double proximalDepth=profile?median(proximal):0,distalDepth=profile?median(distal):0;
 std::map<Vec,unsigned> unique;std::vector<unsigned> group(points.size()),representative;
 std::vector<Vec> rest;std::vector<double> weight;
 for(unsigned i=0;i<points.size();i++){
  auto pair=unique.emplace(points[i],unsigned(rest.size()));group[i]=pair.first->second;
  if(pair.second){rest.push_back(points[i]);representative.push_back(i);weight.push_back(0);}
  if(!ShaftBinding(owners[i]))continue;
  const double s=station[i]/extent,y=std::abs(local[i][1])/radius,z=-local[i][2]/radius;
  const double width=1.0-.20*ease((s-.70)/.21);
  const double w=ease((s-.69)/.045)*(1-ease((s-.90)/.04))*(1-ease((y-.10)/(width-.10)))*ease((z-.3)/.3);
  if(w>weight[group[i]]){weight[group[i]]=w;representative[group[i]]=i;}
 }
 std::vector<std::vector<unsigned>> adjacent(rest.size());std::vector<std::array<unsigned,3>> patch;
 for(auto f:faces){for(auto i:f)if(i>=points.size())throw std::invalid_argument("Invalid frenulum face");
  std::array<unsigned,3> g{group[f[0]],group[f[1]],group[f[2]]};
  if(!(weight[g[0]]||weight[g[1]]||weight[g[2]]))continue;
  if(g[0]==g[1]||g[1]==g[2]||g[2]==g[0])continue;
  patch.push_back(g);for(unsigned i=0;i<3;i++)if(weight[g[i]])for(unsigned j=0;j<3;j++)if(i!=j)adjacent[g[i]].push_back(g[j]);
 }
 std::vector<unsigned> active;
 for(unsigned i=0;i<adjacent.size();i++)if(!adjacent[i].empty()){
  auto& row=adjacent[i];std::sort(row.begin(),row.end());row.erase(std::unique(row.begin(),row.end()),row.end());active.push_back(i);
 }
 auto current=rest;
 for(unsigned iteration=0;iteration<100;iteration++){
  auto next=current;
  for(auto i:active){Vec mean{};for(auto j:adjacent[i])mean=Add(mean,current[j]);mean=Scale(mean,1./adjacent[i].size());
   auto move=Scale(Sub(mean,current[i]),.35*weight[i]);const auto id=representative[i];const auto lateral=side[id];move=Sub(move,Scale(lateral,Dot(move,lateral)));
   if(profile){
    const auto displacement=Sub(current[i],rest[i]);const double s=(station[id]+Dot(displacement,axis[id]))/extent,y=std::abs(local[id][1])/radius,z=-(local[id][2]+Dot(displacement,up[id]));
    const double t=ease((s-.70)/.21),crest=proximalDepth*(1-t)+distalDepth*t;
    // Continue the raphe as a narrowing ridge while its crest gently dips
    // into the distal fold. A broad support removes the surrounding bulb;
    // only its central relief narrows, so no steep wall replaces the bulb.
    const double ridgeWidth=.40*(1-t)+.12*t,ridgeHeight=.10*radius*(1-.4*t);
    const double target=crest+radius*(std::sqrt((std::max)(0.,1-y*y))-1)-ridgeHeight*(1-std::exp(-std::pow(y/ridgeWidth,2)));
    move=Add(move,Scale(up[id],.05*weight[i]*(std::max)(0.,z-target)));
   }
   auto total=Sub(Add(current[i],move),rest[i]);const double length=std::sqrt(Dot(total,total));if(length>.6*radius)total=Scale(total,.6*radius/length);next[i]=Add(rest[i],total);
  }
  // Bound every incremental step. A repaired sharp fold can legitimately
  // rotate more than90 degrees overall; comparing only to its original normal
  // would preserve the spike rather than test continuous triangle integrity.
  const double fraction=RapheCorrectionFraction(current,next,patch);
  for(auto i:active)current[i]=Add(current[i],Scale(Sub(next[i],current[i]),fraction));
 }
 for(unsigned i=0;i<points.size();i++)delta[i]=Sub(current[group[i]],rest[group[i]]);
 return delta;
}
// A common amplitude preserves a smooth profile when a short/wide shape has
// nearly folded source triangles. Independent per-vertex backoffs would put
// the very ripples being removed back into the longitudinal crest.
inline double RapheCorrectionFraction(const std::vector<Vec>& before,
 const std::vector<Vec>& after,const std::vector<std::array<unsigned,3>>& faces){
 if(before.size()!=after.size())throw std::invalid_argument("Raphe correction topology differs");
 double fraction=1;
 for(auto f:faces){
  auto e1=Sub(before[f[1]],before[f[0]]),e2=Sub(before[f[2]],before[f[0]]),n=Cross(e1,e2);
  const double area2=Dot(n,n);if(area2<1e-24)continue;
  auto d1=Sub(Sub(after[f[1]],after[f[0]]),e1),d2=Sub(Sub(after[f[2]],after[f[0]]),e2);
  const double a=Dot(n,Cross(d1,d2))/area2,b=Dot(n,Add(Cross(e1,d2),Cross(d1,e2)))/area2,c=.75;
  double limit=1;
  if(std::abs(a)<1e-12){if(b<0)limit=(std::min)(limit,-c/b);}
  else {const double discriminant=b*b-4*a*c;if(discriminant>=0){
   // Stable quadratic roots; the first positive root is the first crossing
   // of 25 percent of the original oriented area along this correction.
   const double q=-.5*(b+std::copysign(std::sqrt(discriminant),b));
   for(double root:{q/a,q==0?1.:c/q})if(root>0)limit=(std::min)(limit,root);
  }}
  if(limit<1)fraction=(std::min)(fraction,.98*limit);
 }
 return std::clamp(fraction,0.,1.);
}
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
