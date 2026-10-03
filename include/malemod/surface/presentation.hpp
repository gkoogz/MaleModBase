#pragma once
#include "runtime.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace malemod::surface {
// Render between completed fixed-step solutions in material frames. This does
// not alter dynamics or claim a higher numerical solve rate. A rigid glans or
// lobe rotates through SO(3); linear world-position interpolation would shrink it.
using PresentationPoint=std::array<double,3>;
using PresentationBasis=std::array<PresentationPoint,3>; // orthonormal columns
inline PresentationPoint Add(PresentationPoint a,PresentationPoint b){return {a[0]+b[0],a[1]+b[1],a[2]+b[2]};}
inline PresentationPoint Sub(PresentationPoint a,PresentationPoint b){return {a[0]-b[0],a[1]-b[1],a[2]-b[2]};}
inline PresentationPoint Scale(PresentationPoint a,double s){return {a[0]*s,a[1]*s,a[2]*s};}
inline double Dot(PresentationPoint a,PresentationPoint b){return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];}
inline PresentationPoint Cross(PresentationPoint a,PresentationPoint b){return {a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]};}
inline PresentationPoint Unit(PresentationPoint a){double n=std::sqrt(Dot(a,a));if(!std::isfinite(n)||n<1e-12)throw std::invalid_argument("Invalid presentation direction");return Scale(a,1/n);}
inline PresentationPoint Rotate(const PresentationBasis& b,PresentationPoint p){return Add(Add(Scale(b[0],p[0]),Scale(b[1],p[1])),Scale(b[2],p[2]));}
inline PresentationPoint Unrotate(const PresentationBasis& b,PresentationPoint p){return {Dot(b[0],p),Dot(b[1],p),Dot(b[2],p)};}
struct PresentationFrame {PresentationPoint origin{};PresentationBasis basis{{{1,0,0},{0,1,0},{0,0,1}}};};
struct PresentationBinding {unsigned frame=0;double amount=0;};
using PresentationFrames=std::array<PresentationFrame,15>; // identity, 12 guide, 2 lobes
inline PresentationPoint Precise(Point p){return {p.x,p.y,p.z};}
inline PresentationFrames MaterialFrames(const Output& s){
 PresentationFrames out{};
 for(unsigned i=0;i<12;i++){
  auto tangent=Unit(Sub(Precise(s.shaftGuide[std::min(11u,i+1)]),Precise(s.shaftGuide[i?i-1:0])));
  auto side=Sub(PresentationPoint{0,1,0},Scale(tangent,tangent[1]));
  if(Dot(side,side)<1e-8)side=Sub(PresentationPoint{0,0,1},Scale(tangent,tangent[2]));
  side=Unit(side);out[i+1]={Precise(s.shaftGuide[i]),{tangent,side,Unit(Cross(tangent,side))}};
 }
 for(unsigned i=0;i<2;i++)out[13+i]={Precise(s.lobeCenters[i]),{Precise(s.lobeAxes[i][0]),Precise(s.lobeAxes[i][1]),Precise(s.lobeAxes[i][2])}};
 return out;
}
using PresentationQuaternion=std::array<double,4>; // x y z w
inline PresentationQuaternion Quaternion(const PresentationBasis& b){
 for(unsigned i=0;i<3;i++)for(unsigned j=0;j<3;j++)if(std::abs(Dot(b[i],b[j])-(i==j?1.:0.))>1e-4)throw std::invalid_argument("Presentation frame is not orthonormal");
 if(Dot(Cross(b[0],b[1]),b[2])<.9999)throw std::invalid_argument("Presentation frame is reflected");
 const double m[3][3]={{b[0][0],b[1][0],b[2][0]},{b[0][1],b[1][1],b[2][1]},{b[0][2],b[1][2],b[2][2]}};
 PresentationQuaternion q{};double trace=m[0][0]+m[1][1]+m[2][2];
 if(trace>0){double s=2*std::sqrt(trace+1);q={(m[2][1]-m[1][2])/s,(m[0][2]-m[2][0])/s,(m[1][0]-m[0][1])/s,s/4};}
 else{unsigned i=m[1][1]>m[0][0]?1:0;if(m[2][2]>m[i][i])i=2;unsigned j=(i+1)%3,k=(i+2)%3;double s=2*std::sqrt(1+m[i][i]-m[j][j]-m[k][k]);q[i]=s/4;q[j]=(m[j][i]+m[i][j])/s;q[k]=(m[k][i]+m[i][k])/s;q[3]=(m[k][j]-m[j][k])/s;}
 double n=0;for(double x:q)n+=x*x;for(auto& x:q)x/=std::sqrt(n);return q;
}
inline PresentationBasis Basis(PresentationQuaternion q){
 const double x=q[0],y=q[1],z=q[2],w=q[3];
 return {{{1-2*(y*y+z*z),2*(x*y+z*w),2*(x*z-y*w)},
          {2*(x*y-z*w),1-2*(x*x+z*z),2*(y*z+x*w)},
          {2*(x*z+y*w),2*(y*z-x*w),1-2*(x*x+y*y)}}};
}
inline PresentationBasis InterpolateBasis(const PresentationBasis& a,const PresentationBasis& b,double t){
 auto qa=Quaternion(a),qb=Quaternion(b);double dot=0;for(unsigned i=0;i<4;i++)dot+=qa[i]*qb[i];
 if(dot<0){for(auto& x:qb)x=-x;dot=-dot;}dot=std::clamp(dot,-1.,1.);
 double wa=1-t,wb=t;if(dot<.9995){double angle=std::acos(dot),s=std::sin(angle);wa=std::sin((1-t)*angle)/s;wb=std::sin(t*angle)/s;}
 PresentationQuaternion q{};double n=0;for(unsigned i=0;i<4;i++){q[i]=qa[i]*wa+qb[i]*wb;n+=q[i]*q[i];}for(auto& x:q)x/=std::sqrt(n);return Basis(q);
}
class PresentationPlan {
 struct Map {PresentationBasis a{},b{};PresentationPoint offset{};};
 std::array<Map,15> maps_{};double t_;
 public:
 PresentationPlan(const PresentationFrames& a,const PresentationFrames& b,double t):t_(t){
  if(!std::isfinite(t)||t<0||t>1)throw std::invalid_argument("Invalid presentation phase");
  for(unsigned i=0;i<15;i++){
   const auto basis=InterpolateBasis(a[i].basis,b[i].basis,t);auto& m=maps_[i];
   for(unsigned k=0;k<3;k++){
    PresentationPoint axis{};axis[k]=1;
    m.a[k]=Scale(Rotate(basis,Unrotate(a[i].basis,axis)),1-t);
    m.b[k]=Scale(Rotate(basis,Unrotate(b[i].basis,axis)),t);
   }
   m.offset=Sub(Add(Scale(a[i].origin,1-t),Scale(b[i].origin,t)),Add(Rotate(m.a,a[i].origin),Rotate(m.b,b[i].origin)));
  }
 }
 PresentationPoint Point(PresentationPoint a,PresentationPoint b,PresentationBinding binding)const{
  if(binding.frame>=15||!std::isfinite(binding.amount)||binding.amount<0||binding.amount>1)throw std::invalid_argument("Invalid presentation binding");
  if(t_==0)return a;if(t_==1)return b;
  const auto& m=maps_[binding.frame];auto linear=Add(Scale(a,1-t_),Scale(b,t_));
  auto framed=Add(Add(Rotate(m.a,a),Rotate(m.b,b)),m.offset);
  return Add(Scale(linear,1-binding.amount),Scale(framed,binding.amount));
 }
 PresentationPoint Direction(PresentationPoint a,PresentationPoint b,PresentationBinding binding)const{
  if(binding.frame>=15||!std::isfinite(binding.amount)||binding.amount<0||binding.amount>1)throw std::invalid_argument("Invalid presentation binding");
  if(t_==0)return a;if(t_==1)return b;
  const auto& m=maps_[binding.frame];auto linear=Add(Scale(a,1-t_),Scale(b,t_));auto framed=Add(Rotate(m.a,a),Rotate(m.b,b));
  return Unit(Add(Scale(linear,1-binding.amount),Scale(framed,binding.amount)));
 }
};
}
