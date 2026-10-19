#pragma once
#include "presentation.hpp"
#include <vector>
#include <functional>
#include <memory>
#include <map>
#include "suspension_profile.hpp"

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
 // A dynamics-free material shape; never apply legacy posed skin twice.
 std::shared_ptr<const std::vector<Vec>> shaftMaterial;
 std::shared_ptr<const std::vector<Binding>> materialBindings;
 std::shared_ptr<const std::vector<double>> crownWeights;
 double sectionRadius=0,hangControl=50;
 bool suspensionProfile=false;
 std::array<Vec,15> scales{};
 State(){for(auto& s:scales)s={1,1,1};}
};
struct Dimensions {double radius=1,length=1;std::array<Vec,2> lobes{{{1,1,1},{1,1,1}}};Vec anchor{};};
inline void AnchorRootPhysics(State& state,double radius,double length){
 if(!state.hasShapeFrames)return;
 // The opening is attached to the body. Rest-angle controls rotate its root,
 // but the first simulated tangent must not rotate that fixed boundary again.
 // Release the boundary constraint within the first guide interval;
 // the suspended bodies and the remainder retain their full simulated pose.
 const double span=(std::max)(1e-6,length/11.);double station=0;
 for(unsigned i=1;i<=12;i++){
  if(i>1){auto d=Sub(state.shapeFrames[i].origin,state.shapeFrames[i-1].origin);station+=std::sqrt(Dot(d,d));}
  const double t=std::clamp(station/span,0.,1.),w=t*t*t*(10+t*(-15+6*t));
  state.frames[i].origin=Add(Scale(state.shapeFrames[i].origin,1-w),Scale(state.frames[i].origin,w));
  state.frames[i].basis=InterpolateBasis(state.shapeFrames[i].basis,state.frames[i].basis,w);
 }
}
inline State DrivenState(const State& reference,const Dimensions& dimensions,const Output& source){
 if(dimensions.radius<=0||dimensions.length<=0||source.proximalRadius<=0||source.restLength<=0)throw std::invalid_argument("Invalid measured modular dimensions");
 State state;state.frames=MaterialFrames(source);state.sectionRadius=source.proximalRadius;
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
 AnchorRootPhysics(state,source.proximalRadius,source.restLength);
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
// Preserve rest segment lengths after interpolation or root constraints.
// Temporal interpolation of bent node positions otherwise shortens the rod.
inline void PreserveMaterialLengths(State& s){
 if(!s.hasShapeFrames)return;
 const auto original=s.frames;
 Vec previous=s.frames[1].basis[0],previousRest=s.shapeFrames[1].basis[0];
 for(unsigned i=2;i<=12;i++){
  auto d=Sub(original[i].origin,original[i-1].origin);
  const auto r=Sub(s.shapeFrames[i].origin,s.shapeFrames[i-1].origin);
  const double length=std::sqrt(Dot(r,r));
  if(Dot(d,d)<1e-16)d=r;
  auto direction=Unit(d),restDirection=Unit(r);
  if(s.sectionRadius>0){
   // Carry the authored rest curvature exactly, then bound additional live
   // curvature by section radius. A centerline can be length-preserving yet
   // fold its inner surface if a wide section turns through a tight hinge.
   auto axis=Cross(previousRest,restDirection);const double cosine=std::clamp(Dot(previousRest,restDirection),-1.,1.);
   auto target=cosine>-.9999?Add(Add(previous,Cross(axis,previous)),Scale(Cross(axis,Cross(axis,previous)),1/(1+cosine))):previous;
   target=Unit(target);const double angle=std::acos(std::clamp(Dot(target,direction),-1.,1.));
   // Half a segment from the clamped boundary to the first segment center.
   // The remaining margin covers interpolated section angular strain.
   const double limit=.4*length/s.sectionRadius*(i==2?.5:1.);
   if(angle>limit){auto side=Sub(direction,Scale(target,Dot(direction,target)));if(Dot(side,side)<1e-16)side=s.frames[i].basis[1];side=Unit(side);direction=Add(Scale(target,std::cos(limit)),Scale(side,std::sin(limit)));}
  }
  s.frames[i].origin=Add(s.frames[i-1].origin,Scale(direction,length));previous=direction;previousRest=restDirection;
 }
 for(unsigned i=2;i<=12;i++){
  auto tangent=Unit(Sub(s.frames[(std::min)(12u,i+1)].origin,s.frames[i-1].origin));
  auto side=Sub(s.frames[i].basis[1],Scale(tangent,Dot(s.frames[i].basis[1],tangent)));
  if(Dot(side,side)<1e-12)side=Cross(s.frames[i].basis[2],tangent);
  side=Unit(side);s.frames[i].basis={tangent,side,Unit(Cross(tangent,side))};
 }
}
inline State Interpolate(const State& a,const State& b,double t){
 if(!std::isfinite(t)||t<0||t>1)throw std::invalid_argument("Invalid modular interpolation phase");
 State s;s.materialBindings=b.materialBindings;s.shaftMaterial=b.shaftMaterial;s.crownWeights=b.crownWeights;s.sectionRadius=a.sectionRadius*(1-t)+b.sectionRadius*t;s.hangControl=b.hangControl;s.suspensionProfile=b.suspensionProfile;
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
 if(s.shaftMaterial)PreserveMaterialLengths(s);
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
inline bool ShaftBinding(const Binding& b){
 for(auto v:b.influences)if(v.weight>0&&(v.frame<1||v.frame>12))return false;
 return true;
}
inline std::vector<Binding> BindMaterialStations(const std::vector<Vec>& points,const std::vector<Binding>& ownership,const PresentationFrames& frames){
 if(points.size()!=ownership.size())throw std::invalid_argument("Material station topology differs");
 auto bindings=ownership;
 for(unsigned i=0;i<points.size();i++)if(ShaftBinding(ownership[i])){
  double best=1e300;unsigned station=1;double fraction=0;
  for(unsigned j=1;j<12;j++){
   auto d=Sub(frames[j+1].origin,frames[j].origin),v=Sub(points[i],frames[j].origin);const double length2=Dot(d,d);
   if(length2<1e-16)throw std::invalid_argument("Collapsed material guide interval");
   const double t=std::clamp(Dot(v,d)/length2,0.,1.);auto delta=Sub(v,Scale(d,t));const double e=Dot(delta,delta);
   if(e<best){best=e;station=j;fraction=t;}
  }
  bindings[i].influences={Influence{station,1-fraction,{}},Influence{station+1,fraction,{}}};
 }
 return bindings;
}
// A continuous section frame is needed as well as a continuous centerline.
// Pairwise SLERP has a different angular derivative on each side of a guide
// knot, and a nonzero derivative immediately after the fixed root. A wide
// section multiplies that jump by its radius, producing a visible step even
// though every center and vertex is position-continuous.
inline PresentationFrame MaterialSection(const PresentationFrames& f,unsigned i,unsigned j,double t){
 if(i==j)return f[i];
 const auto d=Sub(f[j].origin,f[i].origin);const double length=std::sqrt(Dot(d,d));
 auto qa=Quaternion(f[i].basis),qb=Quaternion(f[j].basis);
 auto aligned=[](PresentationQuaternion q,const PresentationQuaternion& reference){double dot=0;for(unsigned k=0;k<4;k++)dot+=q[k]*reference[k];if(dot<0)for(auto& x:q)x=-x;return q;};
 qb=aligned(qb,qa);
 auto derivative=[&](unsigned node,PresentationQuaternion q){
  PresentationQuaternion v{};
  // The body and rigid crown continue with constant orientation. Match their
  // zero angular strain, rather than introducing a hinge at either boundary.
  if(node==1||node==12)return v;
  auto before=aligned(Quaternion(f[node-1].basis),q),after=aligned(Quaternion(f[node+1].basis),q);
  auto a=Sub(f[node].origin,f[node-1].origin),b=Sub(f[node+1].origin,f[node].origin);
  const double span=std::sqrt(Dot(a,a))+std::sqrt(Dot(b,b));
  if(span<1e-12)return v;
  double normal=0;for(unsigned k=0;k<4;k++){v[k]=(after[k]-before[k])/span;normal+=v[k]*q[k];}
  for(unsigned k=0;k<4;k++)v[k]-=normal*q[k];return v;
 };
 auto va=derivative(i,qa),vb=derivative(j,qb);PresentationQuaternion q{};double norm=0;
 const double t2=t*t,t3=t2*t;
 for(unsigned k=0;k<4;k++){q[k]=(2*t3-3*t2+1)*qa[k]+(t3-2*t2+t)*length*va[k]+(-2*t3+3*t2)*qb[k]+(t3-t2)*length*vb[k];norm+=q[k]*q[k];}
 if(norm<1e-16)throw std::invalid_argument("Collapsed material orientation interpolant");
 for(auto& x:q)x/=std::sqrt(norm);
 return {Hermite(f[i].origin,f[j].origin,Scale(f[i].basis[0],length),Scale(f[j].basis[0],length),t),Basis(q)};
}
// The actual socket can be ahead of the source solver's hidden root after
// clearance. Start the material rod at that socket, preserving the selected
// rest surface and retaining twelve free-section samples through the tip.
inline State SocketMaterialState(const State& original,double start){
 State out=original;std::array<double,12> station{};
 for(unsigned i=1;i<12;i++){auto d=Sub(original.shapeFrames[i+1].origin,original.shapeFrames[i].origin);station[i]=station[i-1]+std::sqrt(Dot(d,d));}
 if(start<0||start>=station.back()-1e-6)throw std::invalid_argument("Socket leaves no free material rod");
 for(unsigned i=0;i<12;i++){
  const double x=start+(station.back()-start)*i/11.;
  unsigned k=1;while(k<11&&station[k]<x)++k;
  const double t=std::clamp((x-station[k-1])/(station[k]-station[k-1]),0.,1.);
  out.shapeFrames[i+1]=MaterialSection(original.shapeFrames,k,k+1,t);
  out.frames[i+1]=MaterialSection(original.frames,k,k+1,t);
 }
 out.frames[1]=out.shapeFrames[1];auto neutral=out;neutral.frames=neutral.shapeFrames;
 PreserveMaterialLengths(out);PreserveMaterialLengths(neutral);
 for(unsigned i=1;i<=12;i++){auto basis=out.frames[i].basis;for(unsigned k=0;k<3;k++)out.frames[i].basis[k]=Rotate(basis,Unrotate(neutral.frames[i].basis,out.shapeFrames[i].basis[k]));}
 return out;
}
// Rotate one material section instead of blending rotated vertex positions.
// Its orthonormal frame preserves diameter; the crown preserves its shape.
inline Vec MaterialPoint(const Binding& binding,const State& state,Vec p,double crown=0){
 const auto a=binding.influences[0],b=binding.influences[1];
 const unsigned i=a.weight>0?a.frame:b.frame,j=b.weight>0?b.frame:i;
 const double t=i==j?0:b.weight;
 auto rest=MaterialSection(state.shapeFrames,i,j,t),live=MaterialSection(state.frames,i,j,t);
 auto result=Add(live.origin,Rotate(live.basis,Unrotate(rest.basis,Sub(p,rest.origin))));
 if(crown>0){const auto& r=state.shapeFrames[12];const auto& f=state.frames[12];auto rigid=Add(f.origin,Rotate(f.basis,Unrotate(r.basis,Sub(p,r.origin))));result=Add(Scale(result,1-crown),Scale(rigid,crown));}
 return result;
}
class Plan {
 std::vector<Vec> rest_;std::vector<Binding> bindings_;std::vector<Loft> lofts_;State reference_;
 std::vector<SuspensionSection> suspensionSections_;
 public:
 void TranslateSuspension(std::vector<Vec>& points,Vec offset)const{
  if(points.size()!=bindings_.size())throw std::invalid_argument("Suspended translation topology differs");
  for(unsigned i=0;i<points.size();i++){
   bool suspended=true;for(auto owner:bindings_[i].influences)if(owner.weight>0&&owner.frame<13)suspended=false;
   if(suspended)points[i]=Add(points[i],offset);
  }
 }
 Vec SuspensionRestOffset(const std::vector<Vec>& material,const State& rest,Vec up={0,0,1})const{
  // The independent suspended bodies hang below their actual mounted rim,
  // not the legacy combined mesh's forward-offset neck. Measure the rim once
  // in the zero-motion shape; never cancel any live swing or inertial motion.
  std::vector<unsigned> rim;std::vector<bool> seen(material.size());
  for(const auto& l:lofts_)if(ShaftBinding(l.lastOwner)&&!ShaftBinding(l.firstOwner)){
   bool suspended=false;for(auto i:l.firstOwner.influences)suspended|=i.frame>=13&&i.weight>0;
   if(suspended&&!seen[l.end]){seen[l.end]=true;rim.push_back(l.end);}
  }
  if(rim.size()<3)throw std::invalid_argument("Suspended attachment has no closed support rim");
  Vec center{};double total=0;
  for(unsigned i=0;i<rim.size();i++){auto a=material[rim[i]],b=material[rim[(i+1)%rim.size()]];const double length=std::sqrt(Dot(Sub(b,a),Sub(b,a)));center=Add(center,Scale(Add(a,b),.5*length));total+=length;}
  if(total<1e-8)throw std::invalid_argument("Suspended support rim collapsed");
  center=Scale(center,1/total);auto lobes=Scale(Add(rest.frames[13].origin,rest.frames[14].origin),.5);
  auto offset=Sub(center,lobes);up=Unit(up);return Sub(offset,Scale(up,Dot(offset,up)));
 }
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
  std::map<double,std::vector<unsigned>> sections;
  for(const auto& l:lofts_)if(ShaftBinding(l.lastOwner)&&!ShaftBinding(l.firstOwner)){
   bool suspended=false;for(auto v:l.firstOwner.influences)suspended|=v.frame>=13&&v.weight>0;
   if(suspended)sections[l.t].push_back(l.vertex);
  }
  for(auto& entry:sections)suspensionSections_.push_back({entry.first,std::move(entry.second)});
  for(unsigned i=0;i<rest_.size();i++){
   auto p=Point(bindings_[i],reference_);if(!generated[i]&&std::sqrt(Dot(Sub(p,rest_[i]),Sub(p,rest_[i])))>1e-6)throw std::invalid_argument("Mechanical binding does not reproduce reference surface");
  }
 }
 std::vector<Vec> Evaluate(const State& state,const std::function<void(std::vector<Vec>&)>& moveOwners={},const State* materialState=nullptr)const{
  ValidateState(state);
  std::vector<Vec> out;out.reserve(rest_.size());for(const auto& b:bindings_)out.push_back(Point(b,state));
  if(state.offsets){if(state.offsets->size()!=out.size())throw std::invalid_argument("Modular shape transfer size differs");for(unsigned i=0;i<out.size();i++){for(double x:(*state.offsets)[i])if(!std::isfinite(x))throw std::invalid_argument("Nonfinite modular shape transfer");out[i]=Add(out[i],(*state.offsets)[i]);}}
  if(state.shaftMaterial){if(!state.hasShapeFrames||state.shaftMaterial->size()!=out.size())throw std::invalid_argument("Invalid shaft material field");const auto& motion=materialState?*materialState:state;for(unsigned i=0;i<out.size();i++)if(ShaftBinding(bindings_[i]))out[i]=MaterialPoint(motion.materialBindings?motion.materialBindings->at(i):bindings_[i],motion,(*state.shaftMaterial)[i],state.crownWeights?state.crownWeights->at(i):0);}
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
  if(state.suspensionProfile)ShapeSuspensionSections(out,suspensionSections_,state.hangControl);
  return out;
 }
};
}
