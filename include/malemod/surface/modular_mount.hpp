#pragma once
#include "modular_motion.hpp"

namespace malemod::surface::modular {
// A measured front-surface chart lets the opening recruit material along the
// original body, instead of scaling the whole pelvis or holding a fixed hole.
// The adapter supplies the body samples; no game axes or vertex IDs are assumed.
struct MountData {
 Vec origin{};PresentationBasis basis{};double step=1;
 unsigned side=0;std::vector<double> height;
 std::vector<double> bodyWeight,shaftWeight;
 std::vector<unsigned> outer,inner;
 std::vector<double> theta;
 double station=0,length=1;
};
class MountPlan {
 MountData data_;std::vector<Vec> rest_;std::vector<double> radii_;
 double Height(double y,double z)const{
  const double middle=(data_.side-1)*.5;
  const double fy=std::clamp(y/data_.step+middle,0.,double(data_.side-1)),fz=std::clamp(z/data_.step+middle,0.,double(data_.side-1));
  const auto i=(std::min)(unsigned(fy),data_.side-2),j=(std::min)(unsigned(fz),data_.side-2);
  const double a=fy-i,b=fz-j;const auto& h=data_.height;
  return (1-a)*(1-b)*h[j*data_.side+i]+a*(1-b)*h[j*data_.side+i+1]+(1-a)*b*h[(j+1)*data_.side+i]+a*b*h[(j+1)*data_.side+i+1];
 }
 double Radius(double angle,const std::vector<double>& values)const{
  constexpr double pi=3.14159265358979323846;
  while(angle<data_.theta.front())angle+=2*pi;while(angle>=data_.theta.front()+2*pi)angle-=2*pi;
  auto b=std::upper_bound(data_.theta.begin(),data_.theta.end(),angle);const unsigned j=unsigned(b-data_.theta.begin()),i=j?j-1:unsigned(radii_.size()-1);
  const double a=data_.theta[i],c=j<radii_.size()?data_.theta[j]:data_.theta.front()+2*pi,t=(angle-a)/(c-a);
  return values[i]*(1-t)+values[j%values.size()]*t;
 }
 public:
 MountPlan(MountData data,const std::vector<Vec>& rest):data_(std::move(data)),rest_(rest){
  if(data_.side<3||data_.side>1025||data_.height.size()!=std::size_t(data_.side)*data_.side||!std::isfinite(data_.step)||data_.step<=0||data_.bodyWeight.size()!=rest.size()||data_.shaftWeight.size()!=rest.size()||data_.outer.size()<8||data_.outer.size()!=data_.inner.size()||data_.outer.size()!=data_.theta.size())throw std::invalid_argument("Invalid measured modular mount");
  Quaternion(data_.basis);
  for(double h:data_.height)if(!std::isfinite(h))throw std::invalid_argument("Nonfinite body chart");
  for(const auto& weights:{data_.bodyWeight,data_.shaftWeight})for(double w:weights)if(!std::isfinite(w)||w<0||w>1)throw std::invalid_argument("Invalid mount ownership");
  for(unsigned i=0;i<data_.outer.size();i++){
   if(data_.outer[i]>=rest.size()||data_.inner[i]>=rest.size()||!std::isfinite(data_.theta[i])||(i&&data_.theta[i]<=data_.theta[i-1]))throw std::invalid_argument("Invalid mount boundary ordering");
   auto q=Unrotate(data_.basis,Sub(rest[data_.outer[i]],data_.origin));radii_.push_back(std::hypot(q[1],q[2]));
  }
 }
 void Apply(std::vector<Vec>& points,const State& reference,const State& state)const{
  const auto currentAxis=Unrotate(data_.basis,state.frames[1].basis[0]);
  const auto referenceAxis=Unrotate(data_.basis,reference.frames[1].basis[0]);
  const double scale=state.scales[1][1]/reference.scales[1][1];
  if(currentAxis[0]<.15)throw std::invalid_argument("Modular mount leaves forward body chart");
  std::vector<double> targets;targets.reserve(radii_.size());
  for(unsigned i=0;i<radii_.size();i++){
   const double y=std::cos(data_.theta[i]),z=std::sin(data_.theta[i]),radius=radii_[i];
   // The opening side compressed by tilt needs the swept intersection. The
   // opposite arc keeps its size-driven footprint: chasing an underside ray
   // through empty perineal space would incorrectly recruit the rear thighs.
   if(y*(currentAxis[1]-referenceAxis[1])+z*(currentAxis[2]-referenceAxis[2])<=0){targets.push_back(radius*scale);continue;}
   const Vec original{Height(y*radius,z*radius),y*radius,z*radius};
   const double axial=Dot(original,referenceAxis),distance2=(Dot(original,original)-axial*axial)*scale*scale;
   auto value=[&](double r){const Vec q{Height(y*r,z*r),y*r,z*r};const double axial=Dot(q,currentAxis);return Dot(q,q)-axial*axial-distance2;};
   double low=0,high=4*radius*(std::max)(1.,scale);
   if(value(low)>0||value(high)<0)throw std::invalid_argument("No forward body/shaft section intersection");
   for(unsigned iteration=0;iteration<32;iteration++){const double mid=(low+high)*.5;if(value(mid)>0)high=mid;else low=mid;}
   targets.push_back((std::max)(radius*scale,(low+high)*.5));
  }
  for(unsigned i=0;i<points.size();i++)if(data_.bodyWeight[i]>0){
   const auto q=Unrotate(data_.basis,Sub(rest_[i],data_.origin));const double r=std::hypot(q[1],q[2]);if(r<1e-8)continue;
   const double angle=std::atan2(q[2],q[1]),initial=Radius(angle,radii_),target=Radius(angle,targets),support=4*(std::max)(initial,target);
   if(r>=support)continue;
   double changed;
   if(r<=initial)changed=r*target/initial;
   else{const double t=(r-initial)/(support-initial),slope=(support-target)/(support-initial),d=(std::min)(1.,1.5*slope)*(support-initial);changed=Hermite({target,0,0},{support,0,0},{d,0,0},{d,0,0},t)[0];}
   const double y=q[1]*changed/r,z=q[2]*changed/r;
   const Vec delta{Height(y,z)-Height(q[1],q[2]),y-q[1],z-q[2]};
   points[i]=Add(points[i],Scale(Rotate(data_.basis,delta),data_.bodyWeight[i]));
  }
  // The exposed cylinder begins beyond the foremost body support. Move its
  // parameterized root rows along the continuous shaft, not the rotation pivot.
  double bodyFar=-1e100,shaftNear=1e100;
  for(unsigned i:data_.outer)bodyFar=(std::max)(bodyFar,Dot(Sub(points[i],data_.origin),data_.basis[0]));
  for(unsigned i:data_.inner)shaftNear=(std::min)(shaftNear,Dot(Sub(points[i],data_.origin),data_.basis[0]));
  double referenceFar=-1e100,referenceNear=1e100;
  for(unsigned i:data_.outer)referenceFar=(std::max)(referenceFar,Dot(Sub(rest_[i],data_.origin),data_.basis[0]));
  for(unsigned i:data_.inner)referenceNear=(std::min)(referenceNear,Dot(Sub(rest_[i],data_.origin),data_.basis[0]));
  const double clearance=(std::max)(0.,referenceNear-referenceFar)*scale;
  const double advance=(std::max)(0.,bodyFar+clearance-shaftNear)/currentAxis[0];
  const double span=(std::max)(data_.length*.7*state.scales[1][0],2.1*advance);
  for(unsigned i=0;i<points.size();i++)if(data_.shaftWeight[i]>0){
   const double station=Dot(Sub(rest_[i],data_.origin),data_.basis[0]);
   double t=std::clamp((station-data_.station)*state.scales[1][0]/span,0.,1.);const double fade=1-t*t*t*(10+t*(-15+6*t));
   points[i]=Add(points[i],Scale(state.frames[1].basis[0],advance*fade*data_.shaftWeight[i]));
  }
 }
 void Finish(std::vector<Vec>& points,const std::vector<Loft>& lofts,const State& reference,const State& state)const{
  // Rebuild the pelvic loft with endpoint tangents for the current opening.
  // Scrotal lofts retain their independent mechanical endpoint ownership.
  std::vector<int> membership(points.size(),-1);for(unsigned i=0;i<data_.outer.size();i++)membership[data_.outer[i]]=int(i);
  for(const auto& l:lofts)if(membership[l.start]>=0){
   const auto a=points[l.start],b=points[l.end];const auto d=Sub(b,a);const double distance=std::sqrt(Dot(d,d));
   auto q=Unrotate(data_.basis,Sub(a,data_.origin));const double eps=data_.step*.2;
   const double dy=(Height(q[1]+eps,q[2])-Height(q[1]-eps,q[2]))/(2*eps),dz=(Height(q[1],q[2]+eps)-Height(q[1],q[2]-eps))/(2*eps);
   auto chartFirst=Unit(Rotate(data_.basis,{-dy*q[1]-dz*q[2],-q[1],-q[2]}));
   const double scale=state.scales[1][1]/reference.scales[1][1];
   auto rq=Unrotate(data_.basis,Sub(rest_[l.start],data_.origin));
   const double rdy=(Height(rq[1]+eps,rq[2])-Height(rq[1]-eps,rq[2]))/(2*eps),rdz=(Height(rq[1],rq[2]+eps)-Height(rq[1],rq[2]-eps))/(2*eps);
   auto referenceChart=Unit(Rotate(data_.basis,{-rdy*rq[1]-rdz*rq[2],-rq[1],-rq[2]}));
   auto first=Scale(Unit(Add(Unit(l.first),Sub(chartFirst,referenceChart))),(std::min)(distance,std::sqrt(Dot(l.first,l.first))*scale));
   auto last=Scale(Unit(Direction(l.last,l.lastOwner,reference,state)),distance);
   const auto after=Hermite(a,b,first,last,l.t);
   auto residual=Sub(l.restValue,Hermite(rest_[l.start],rest_[l.end],l.first,l.last,l.t));
   points[l.vertex]=Add(after,Scale(residual,scale));
  }
 }
};
}
