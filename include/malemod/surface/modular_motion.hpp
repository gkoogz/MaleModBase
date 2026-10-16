#pragma once
#include "presentation.hpp"
#include <vector>
#include <functional>
#include <memory>

namespace malemod::surface::modular {
// Semantic mechanical ownership is authored once, never inferred from posed
// vertex height. The complete shaft and suspended skin use independent frames.
using Vec=PresentationPoint;
struct Influence {unsigned frame=0;double weight=0;Vec local{};};
struct Binding {std::array<Influence,2> influences{};};
struct State {
 PresentationFrames frames{};
 // Rest/control geometry owns the pelvic opening; simulated motion must not
 // recut the body or rebuild topology on every physics step.
 PresentationFrames shapeFrames{};
 bool hasShapeFrames=false;
 std::shared_ptr<const std::vector<Vec>> offsets;
 std::shared_ptr<const std::vector<Vec>> surfaceNormals;
 std::array<Vec,15> scales{};
 State(){for(auto& s:scales)s={1,1,1};}
};
struct Dimensions {double radius=1,length=1;std::array<Vec,2> lobes{{{1,1,1},{1,1,1}}};Vec anchor{};};
inline State DrivenState(const State& reference,const Dimensions& dimensions,const Output& source){
 if(dimensions.radius<=0||dimensions.length<=0||source.proximalRadius<=0||source.restLength<=0)throw std::invalid_argument("Invalid measured modular dimensions");
 State state;state.frames=MaterialFrames(source);
 for(unsigned i=1;i<=12;i++)state.scales[i]={source.restLength/dimensions.length,source.proximalRadius/dimensions.radius,source.proximalRadius/dimensions.radius};
 const auto& a=reference.frames[1];const auto& b=state.frames[1];
 auto local=Unrotate(a.basis,Sub(dimensions.anchor,a.origin));
 auto predicted=Add(b.origin,Rotate(b.basis,{local[0]*state.scales[1][0],local[1]*state.scales[1][1],local[2]*state.scales[1][2]}));
 auto desired=Add(dimensions.anchor,Sub(b.origin,a.origin));auto correction=Sub(desired,predicted);
 for(unsigned i=1;i<=12;i++)state.frames[i].origin=Add(state.frames[i].origin,correction);
 auto restSource=source;restSource.shaftGuide=source.restGuide;
 state.shapeFrames=MaterialFrames(restSource);state.hasShapeFrames=true;
 const auto& shapeRoot=state.shapeFrames[1];
 auto shapePredicted=Add(shapeRoot.origin,Rotate(shapeRoot.basis,{local[0]*state.scales[1][0],local[1]*state.scales[1][1],local[2]*state.scales[1][2]}));
 auto shapeDesired=Add(dimensions.anchor,Sub(shapeRoot.origin,a.origin));
 auto shapeCorrection=Sub(shapeDesired,shapePredicted);
 for(unsigned i=1;i<=12;i++)state.shapeFrames[i].origin=Add(state.shapeFrames[i].origin,shapeCorrection);
 for(unsigned i=0;i<2;i++)for(unsigned k=0;k<3;k++){
  const auto current=Precise(source.lobeRadii[i]);if(dimensions.lobes[i][k]<=0||current[k]<=0)throw std::invalid_argument("Invalid measured lobe dimensions");
  state.scales[13+i][k]=current[k]/dimensions.lobes[i][k];
 }
 return state;
}
struct Loft {
 unsigned vertex=0,start=0,end=0;double t=0;
 Vec first{},last{},restValue{};
 Binding firstOwner{},lastOwner{};
};
inline Vec Components(Vec a,Vec b){return {a[0]*b[0],a[1]*b[1],a[2]*b[2]};}
inline void ValidateBinding(const Binding& b){
 double sum=0;
 for(const auto& v:b.influences){
  if(v.frame>=15||!std::isfinite(v.weight)||v.weight<0)throw std::invalid_argument("Invalid modular mechanical ownership");
  for(double x:v.local)if(!std::isfinite(x))throw std::invalid_argument("Nonfinite modular binding offset");
  sum+=v.weight;
 }
 if(std::abs(sum-1)>1e-8)throw std::invalid_argument("Unnormalized modular mechanical ownership");
}
inline void ValidateState(const State& state){
 for(const auto& s:state.scales)for(double x:s)if(!std::isfinite(x)||x<=0)throw std::invalid_argument("Invalid modular morphology scale");
 for(const auto& f:state.frames){for(const auto& a:f.basis)for(double x:a)if(!std::isfinite(x))throw std::invalid_argument("Nonfinite modular frame");Quaternion(f.basis);for(double x:f.origin)if(!std::isfinite(x))throw std::invalid_argument("Nonfinite modular pose");}
}
inline State Interpolate(const State& a,const State& b,double t){
 if(!std::isfinite(t)||t<0||t>1)throw std::invalid_argument("Invalid modular interpolation phase");
 State s;
 if(a.offsets&&b.offsets){if(a.offsets->size()!=b.offsets->size())throw std::invalid_argument("Modular shape transfer size changed");auto values=std::make_shared<std::vector<Vec>>(a.offsets->size());for(unsigned i=0;i<values->size();i++)(*values)[i]=Add(Scale((*a.offsets)[i],1-t),Scale((*b.offsets)[i],t));s.offsets=std::move(values);}
 else s.offsets=t<1?a.offsets:b.offsets;
 if(a.surfaceNormals&&b.surfaceNormals){if(a.surfaceNormals->size()!=b.surfaceNormals->size())throw std::invalid_argument("Modular normal transfer size changed");auto values=std::make_shared<std::vector<Vec>>(a.surfaceNormals->size());for(unsigned i=0;i<values->size();i++){auto n=Add(Scale((*a.surfaceNormals)[i],1-t),Scale((*b.surfaceNormals)[i],t));(*values)[i]=Dot(n,n)>1e-12?Unit(n):Vec{};}s.surfaceNormals=std::move(values);}else s.surfaceNormals=t<1?a.surfaceNormals:b.surfaceNormals;
 s.hasShapeFrames=a.hasShapeFrames&&b.hasShapeFrames;
 for(unsigned i=0;i<15;i++){
  s.frames[i].origin=Add(Scale(a.frames[i].origin,1-t),Scale(b.frames[i].origin,t));
  s.frames[i].basis=InterpolateBasis(a.frames[i].basis,b.frames[i].basis,t);
  s.scales[i]=Add(Scale(a.scales[i],1-t),Scale(b.scales[i],t));
  if(s.hasShapeFrames){s.shapeFrames[i].origin=Add(Scale(a.shapeFrames[i].origin,1-t),Scale(b.shapeFrames[i].origin,t));s.shapeFrames[i].basis=InterpolateBasis(a.shapeFrames[i].basis,b.shapeFrames[i].basis,t);}
 }
 return s;
}
inline Vec Point(const Binding& b,const State& state){
 Vec out{};double sum=0;
 for(const auto& v:b.influences){
  if(!v.weight)continue;
  if(v.frame>=15||!std::isfinite(v.weight)||v.weight<0)throw std::invalid_argument("Invalid modular mechanical ownership");
  const auto& f=state.frames[v.frame];auto p=Add(f.origin,Rotate(f.basis,Components(v.local,state.scales[v.frame])));
  out=Add(out,Scale(p,v.weight));sum+=v.weight;
 }
 if(std::abs(sum-1)>1e-8)throw std::invalid_argument("Unnormalized modular mechanical ownership");
 return out;
}
inline Vec Direction(Vec original,const Binding& b,const State& reference,const State& state){
 Vec out{};
 for(const auto& v:b.influences)if(v.weight){
  auto local=Unrotate(reference.frames[v.frame].basis,original);
  Vec scale{};for(unsigned k=0;k<3;k++)scale[k]=state.scales[v.frame][k]/reference.scales[v.frame][k];
  out=Add(out,Scale(Rotate(state.frames[v.frame].basis,Components(local,scale)),v.weight));
 }
 return out;
}
inline Vec Hermite(Vec a,Vec b,Vec da,Vec db,double t){
 const double t2=t*t,t3=t2*t;
 return Add(Add(Scale(a,2*t3-3*t2+1),Scale(da,t3-2*t2+t)),Add(Scale(b,-2*t3+3*t2),Scale(db,t3-t2)));
}
class Plan {
 std::vector<Vec> rest_;std::vector<Binding> bindings_;std::vector<Loft> lofts_;State reference_;
 public:
 Plan(std::vector<Vec> rest,std::vector<Binding> bindings,std::vector<Loft> lofts,State reference)
 :rest_(std::move(rest)),bindings_(std::move(bindings)),lofts_(std::move(lofts)),reference_(reference){
  if(rest_.empty()||rest_.size()!=bindings_.size())throw std::invalid_argument("Modular topology/binding count differs");
  ValidateState(reference_);
  for(const auto& b:bindings_)ValidateBinding(b);
  for(const auto& p:rest_)for(double x:p)if(!std::isfinite(x))throw std::invalid_argument("Nonfinite modular rest point");
  std::vector<bool> generated(rest_.size());
  for(const auto& l:lofts_){
   if(l.vertex>=rest_.size()||l.start>=rest_.size()||l.end>=rest_.size()||l.t<=0||l.t>=1||!std::isfinite(l.t)||generated[l.vertex])throw std::invalid_argument("Invalid immutable loft binding");
   generated[l.vertex]=true;
   ValidateBinding(l.firstOwner);ValidateBinding(l.lastOwner);
   for(const auto& p:{l.first,l.last,l.restValue})for(double x:p)if(!std::isfinite(x))throw std::invalid_argument("Nonfinite modular loft");
  }
  for(const auto& l:lofts_)if(generated[l.start]||generated[l.end])throw std::invalid_argument("Loft endpoint depends on a generated interior");
  for(unsigned i=0;i<rest_.size();i++){
   auto p=Point(bindings_[i],reference_);if(!generated[i]&&std::sqrt(Dot(Sub(p,rest_[i]),Sub(p,rest_[i])))>1e-6)throw std::invalid_argument("Mechanical binding does not reproduce reference surface");
  }
 }
 std::vector<Vec> Evaluate(const State& state,const std::function<void(std::vector<Vec>&)>& moveOwners={})const{
  ValidateState(state);
  std::vector<Vec> out;out.reserve(rest_.size());for(const auto& b:bindings_)out.push_back(Point(b,state));
  if(state.offsets){if(state.offsets->size()!=out.size())throw std::invalid_argument("Modular shape transfer size differs");for(unsigned i=0;i<out.size();i++){for(double x:(*state.offsets)[i])if(!std::isfinite(x))throw std::invalid_argument("Nonfinite modular shape transfer");out[i]=Add(out[i],(*state.offsets)[i]);}}
  if(moveOwners)moveOwners(out);
  for(const auto& l:lofts_){
   auto a=Direction(l.first,l.firstOwner,reference_,state),b=Direction(l.last,l.lastOwner,reference_,state);
   if(state.surfaceNormals){
    auto tangent=[&](Vec v,unsigned index){auto n=state.surfaceNormals->at(index);if(Dot(n,n)<1e-12)return v;n=Unit(n);auto projected=Sub(v,Scale(n,Dot(n,v)));return Dot(projected,projected)>1e-12?Scale(Unit(projected),std::sqrt(Dot(v,v))):v;};
    a=tangent(a,l.start);b=tangent(b,l.end);
   }
   const auto before=Hermite(rest_[l.start],rest_[l.end],l.first,l.last,l.t);
   const auto after=Hermite(out[l.start],out[l.end],a,b,l.t);
   // Preserve authored local correction exactly at rest. Transport its offset
   // with endpoint owners instead of baking suspended motion into the shaft.
   const auto residual=Sub(l.restValue,before);
   const auto ra=Direction(residual,l.firstOwner,reference_,state),rb=Direction(residual,l.lastOwner,reference_,state);
   const double w=l.t*l.t*l.t*(10+l.t*(-15+6*l.t));
   out[l.vertex]=Add(after,Add(Scale(ra,1-w),Scale(rb,w)));
  }
  return out;
 }
};
}
