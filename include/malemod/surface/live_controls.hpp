#pragma once
#include "presentation.hpp"
#include <algorithm>
#include <vector>
#include <optional>
#include <malemod/task_arena.hpp>

namespace malemod::surface {
// A presentation-only deformation of an immutable, prepared mesh. The physics
// worker remains authoritative for dynamics and the settled authored shape.
// Always evaluate from the controls belonging to that mesh, never incrementally
// from the preceding displayed mesh: input frequency cannot accumulate scale.
struct LiveControlBinding {
 double shaft=0,lobe=0,crown=0,body=0;
 bool fixed=false;
};
struct LiveControlFrame {
 PresentationPoint root{},tip{},lobes{},bodyOrigin{};
 PresentationBasis basis{};
 double radius=1,bodyRadius=1;
};
inline double ControlCurve(double u,double low,double middle,double high){
 u=(std::max)(1.,u);
 return u<=50?low+(middle-low)*(u-1)/49:middle+(high-middle)*(u-50)/50;
}
inline double ControlSmooth(double t){t=std::clamp(t,0.,1.);return t*t*t*(10+t*(-15+6*t));}
// Source contract: runtime.cpp Map and source MapLength100/GlansIndependentScale.
inline double ControlLength(double u){
 if(u<50)return .4+1.2*ControlSmooth(u/50);
 const double t=std::clamp((u-50)/50,0.,1.);
 return 1.6+.4*t+.4*ControlSmooth(t);
}
inline double ControlCrown(double u){
 u=u<=50?u*.7:35+(u-50)*1.2;
 return u<50?.82+.18*ControlSmooth(u/50):ControlCurve((std::max)(0.,(u-50)*2),1,1.4,1.6);
}
inline double ControlAngle(double u){
 u=std::clamp(u,10.,90.);
 return (50-u)/(u<50?49.:50.)*(u<50?45.:30.)*3.14159265358979323846/180.;
}
class LiveControlDeformation {
 using Vec=PresentationPoint;
 LiveControlFrame frame_;
 double axial_=1,radial_=1,lobe_=1,crown_=1,angle_=0,hang_=0,overall_=1;
 Vec shift_{},tip_{},lobeCenter_{};double angleCos_=1,angleSin_=0;
 static double Overall(const Controls& c){return ControlCurve(c.values[1],.85,1.2,2.5);}
 static double Ball(const Controls& c){return Overall(c)*ControlCurve(c.values[5],1,1.53,2);}
 static double Hang(const Controls& c){
  const double u=(c.values[6]-50)/(c.values[6]<50?49.:50.);
  return -u*(u<0?1.:4.5)*std::sqrt((std::max)(.35,Ball(c)/(1.5*1.25)));
 }
 Vec RotateDelta(Vec p)const{
  const auto axis=frame_.basis[1];const double c=angleCos_,s=angleSin_;
  // Positive UI elevation rotates forward toward up (about negative side).
  return Add(Add(Scale(p,c),Scale(Cross(axis,p),-s)),Scale(axis,Dot(axis,p)*(1-c)));
 }
 Vec Shaft(Vec p)const{
  auto q=Unrotate(frame_.basis,Sub(p,frame_.root));
  // The clipped root circumference is a material boundary. Length changes
  // stretch the free shaft; they cannot retract the socket through the pelvis.
  q={q[0]*overall_+(axial_-overall_)*(std::max)(0.,q[0]),q[1]*radial_,q[2]*radial_};
  return Add(Add(frame_.root,RotateDelta(Rotate(frame_.basis,q))),shift_);
 }
public:
 double LobeScale()const{return lobe_;}
 LiveControlDeformation(const LiveControlFrame& frame,const Controls& from,const Controls& to):frame_(frame){
  if(frame.radius<=0||frame.bodyRadius<=0)throw std::invalid_argument("Invalid live control support");
  for(unsigned i=0;i<18;i++)if(!std::isfinite(from.values[i])||!std::isfinite(to.values[i]))throw std::invalid_argument("Nonfinite live controls");
  Quaternion(frame.basis);
  const double overall=Overall(to)/Overall(from);overall_=overall;
  axial_=overall*ControlLength(to.values[2])/ControlLength(from.values[2]);
  radial_=overall*ControlCurve(to.values[3],.95,1.59,2)/ControlCurve(from.values[3],.95,1.59,2);
  lobe_=Ball(to)/Ball(from);crown_=ControlCrown(to.values[4])/ControlCrown(from.values[4]);
  angle_=ControlAngle(to.values[7])-ControlAngle(from.values[7]);hang_=Hang(to)-Hang(from);
  angleCos_=std::cos(angle_);angleSin_=std::sin(angle_);
  shift_={ControlCurve(to.values[8],-2,-.7,6)-ControlCurve(from.values[8],-2,-.7,6),0,
          ControlCurve(to.values[9],-3,.400001,3)-ControlCurve(from.values[9],-3,.400001,3)};
  tip_=Shaft(frame_.tip);
  const auto attachment=Add(frame_.root,Scale(frame_.basis[0],
   (std::max)(0.,Dot(Sub(frame_.lobes,frame_.root),frame_.basis[0]))));
  lobeCenter_=Add(frame_.lobes,Add(Add(Sub(Shaft(attachment),attachment),
   Scale(Sub(frame_.lobes,attachment),overall_-1)),Vec{0,0,hang_}));
 }
 Vec Point(Vec p,LiveControlBinding binding)const{
  if(binding.fixed)return p;
  auto shaft=Shaft(p);
  if(binding.crown>0){
   shaft=Add(shaft,Scale(Sub(shaft,tip_),(crown_-1)*binding.crown));
  }
  // Move the suspended system with its attachment station, not with the
  // shaft's rotated radial coordinates. Gravity remains in body space.
  auto sack=Add(lobeCenter_,Scale(Sub(p,frame_.lobes),lobe_));
  auto out=Add(p,Add(Scale(Sub(shaft,p),binding.shaft),Scale(Sub(sack,p),binding.lobe)));
  if(binding.body>0){
   const auto q=Sub(p,frame_.bodyOrigin);
   // This is one shared field across both body resources and the outer ring.
   // The prepared boundary mask vanishes at protected stock/waist vertices.
   auto radial=Sub(q,Scale(Vec{1,0,0},q[0]));
   out=Add(out,Scale(Add(Scale(radial,radial_-1),shift_),binding.body));
  }
  return out;
 }
 PresentationBasis Differential(Vec p,LiveControlBinding binding)const{
  PresentationBasis j{};const double h=(std::max)(1e-5,frame_.radius*1e-5);
  for(unsigned k=0;k<3;k++){auto a=p,b=p;a[k]-=h;b[k]+=h;j[k]=Scale(Sub(Point(b,binding),Point(a,binding)),.5/h);}
  return j;
 }
 static Vec Tangent(Vec t,const PresentationBasis& j){return Unit(Rotate(j,t));}
 static Vec Normal(Vec n,const PresentationBasis& j){
  // Cofactor transform, not the position transform: nonuniform dilation
  // otherwise makes the seam's normals disagree with the actual surface.
  auto cofactor=Add(Add(Scale(Cross(j[1],j[2]),n[0]),Scale(Cross(j[2],j[0]),n[1])),Scale(Cross(j[0],j[1]),n[2]));
  if(Dot(j[0],Cross(j[1],j[2]))<=1e-10)throw std::runtime_error("Live controls inverted a local deformation");
  return Unit(cofactor);
 }
};
struct LiveControlLoft {unsigned vertex=0,start=0,end=0;double t=0;};
struct LiveControlTangents {PresentationPoint first{},last{};};
struct LiveControlDonor {unsigned vertex=0;std::array<unsigned,3> parents{};std::array<double,3> weights{};};
inline void DeformLiveControlDonors(const std::vector<LiveControlDonor>& donors,
 const std::vector<PresentationPoint>& before,std::vector<PresentationPoint>& after){
 // All original parent vertices are evaluated before any refined vertex.
 // Subdivision points must inherit that affine displacement: independently
 // evaluating a nonlinear field at an edge split bends the original edge,
 // reverses needle triangles, and breaks source-edge seam correspondence.
 std::vector<PresentationPoint> deltas;deltas.reserve(donors.size());
 for(const auto& d:donors){PresentationPoint delta{};for(unsigned k=0;k<3;k++)delta=Add(delta,Scale(Sub(after.at(d.parents[k]),before.at(d.parents[k])),d.weights[k]));deltas.push_back(delta);}
 for(unsigned i=0;i<donors.size();i++)after.at(donors[i].vertex)=Add(before.at(donors[i].vertex),deltas[i]);
}
inline void DeformLiveControlLofts(const LiveControlDeformation& morph,
 const std::vector<LiveControlBinding>& bindings,const std::vector<LiveControlLoft>& lofts,
 const std::vector<LiveControlTangents>& tangents,
 const std::vector<PresentationPoint>& before,std::vector<PresentationPoint>& after,TaskArena* arena=nullptr,
 unsigned first=0,unsigned end=~0u){
 if(lofts.size()!=tangents.size()||before.size()!=after.size()||before.size()!=bindings.size())throw std::invalid_argument("Live loft layout differs");
 if(end==~0u)end=unsigned(lofts.size());
 if(first>end||end>lofts.size())throw std::invalid_argument("Live loft range differs");
 auto hermite=[](PresentationPoint a,PresentationPoint b,PresentationPoint u,PresentationPoint v,double t){
  const double t2=t*t,t3=t2*t;return Add(Add(Scale(a,2*t3-3*t2+1),Scale(b,-2*t3+3*t2)),Add(Scale(u,t3-2*t2+t),Scale(v,t3-t2)));
 };
 std::vector<std::optional<PresentationBasis>> jacobians(before.size());
 auto jacobian=[&](unsigned i)->const PresentationBasis&{auto& value=jacobians.at(i);if(!value)value=morph.Differential(before[i],bindings[i]);return *value;};
 if(arena){
  std::vector<bool> generated(before.size());
  for(unsigned i=first;i<end;i++){const auto& l=lofts[i];if(generated.at(l.vertex))throw std::invalid_argument("Duplicate live loft vertex");generated[l.vertex]=true;}
  for(unsigned i=first;i<end;i++){const auto& l=lofts[i];
   if(generated.at(l.start)||generated.at(l.end))throw std::invalid_argument("Parallel live loft endpoints must be independent");
   jacobian(l.start);jacobian(l.end);
  }
 }
 auto evaluate=[&](unsigned i){
  const auto l=lofts[i];const auto& tangent=tangents[i];
  auto a=arena?*jacobians[l.start]:jacobian(l.start),b=arena?*jacobians[l.end]:jacobian(l.end);
  const auto residual=Sub(before.at(l.vertex),hermite(before[l.start],before[l.end],tangent.first,tangent.last,l.t));
  const auto transported=Add(Scale(Rotate(a,residual),1-ControlSmooth(l.t)),Scale(Rotate(b,residual),ControlSmooth(l.t)));
  after.at(l.vertex)=Add(hermite(after[l.start],after[l.end],Rotate(a,tangent.first),Rotate(b,tangent.last),l.t),transported);
 };
 if(arena)arena->For(end-first,[&](unsigned i){evaluate(first+i);});else for(unsigned i=first;i<end;i++)evaluate(i);
}
}
