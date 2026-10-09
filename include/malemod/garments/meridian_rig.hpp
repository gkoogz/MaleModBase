#pragma once
#include "meridian_runtime.hpp"
#include <algorithm>
namespace malemod::garments::meridian {
struct CircularSection {Vec center,u,v;float radius;};
inline float CircleSupport(const CircularSection& circle,Vec n){float x=Dot(n,circle.u),y=Dot(n,circle.v);return Dot(n,circle.center)+circle.radius*std::sqrt(x*x+y*y);}
inline float LinkSupport(const CircularSection& a,const CircularSection& b,Vec n){return (std::max)(CircleSupport(a,n),CircleSupport(b,n));}
inline float DomeSupport(const CircularSection& rim,Vec apex,Vec n){
 float x=Dot(n,rim.u)*rim.radius,y=Dot(n,rim.v)*rim.radius,z=Dot(n,Sub(apex,rim.center));
 return Dot(n,rim.center)+std::sqrt(x*x+y*y+(z>0?z*z:0));
}
inline float OvoidSupport(const Vec* controls,Vec n){
 Vec center=Mul(Add(controls[0],controls[1]),.5f);
 double h=Dot(n,Mul(Sub(controls[0],controls[1]),.5f)),x=Dot(n,Mul(Sub(controls[2],controls[3]),.5f)),y=Dot(n,Mul(Sub(controls[4],controls[5]),.5f));
 double radial=std::sqrt(x*x+y*y);if(radial<1e-12)return Dot(n,center)+float(std::abs(h));
 // The tapered radial profile is strictly concave on [-1,1]. Its derivative
 // has one zero, so bounded bisection gives its support without tessellation.
 double low=-1,high=1;
 for(unsigned i=0;i<24;i++){double z=(low+high)*.5,root=std::sqrt(1-z*z),derivative=h+radial*(.26*z*z-z-.13)/root;if(derivative>0)low=z;else high=z;}
 double z=(low+high)*.5;
 return Dot(n,center)+float(h*z+radial*(1-.13*z)*std::sqrt(1-z*z))+1e-4f;
}
inline CircularSection FitCircularSection(Vec a,Vec b,Vec c,Vec d){
 CircularSection out;out.center=Mul(Add(Add(a,b),Add(c,d)),.25f);out.u=Unit(Sub(a,c));
 Vec normal=Unit(Cross(out.u,Sub(b,d)));out.v=Cross(normal,out.u);out.radius=0;
 for(Vec p:{a,b,c,d}){p=Sub(p,out.center);out.radius=(std::max)(out.radius,std::sqrt(Dot(p,p)));}
 return out;
}
inline Vec SectionPoint(const CircularSection& section,unsigned index,unsigned segments){
 float angle=6.28318530718f*index/segments;return Add(section.center,Mul(Add(Mul(section.u,std::cos(angle)),Mul(section.v,std::sin(angle))),section.radius));
}
inline void WriteLink(Vec* points,const CircularSection& first,const CircularSection& second,unsigned segments){
 for(unsigned i=0;i<segments;i++){points[i]=SectionPoint(first,i,segments);points[i+segments]=SectionPoint(second,i,segments);}
 points[segments*2]=first.center;points[segments*2+1]=second.center;
}
inline void AlignDomeRim(CircularSection& rim,Vec apex){
 Vec normal=Unit(Sub(apex,rim.center));rim.u=Unit(Sub(rim.u,Mul(normal,Dot(rim.u,normal))));rim.v=Cross(normal,rim.u);
}
inline void WriteDome(Vec* points,const CircularSection& rim,Vec apex,unsigned rows,unsigned segments){
 Vec axial=Sub(apex,rim.center);
 for(unsigned row=0;row<rows;row++){float phi=1.57079632679f*row/rows;for(unsigned col=0;col<segments;col++)points[row*segments+col]=Add(rim.center,Add(Mul(axial,std::sin(phi)),Mul(Sub(SectionPoint(rim,col,segments),rim.center),std::cos(phi))));}
 points[rows*segments]=apex;points[rows*segments+1]=rim.center;
}
inline void WriteOvoid(Vec* points,Vec north,Vec south,Vec east,Vec west,Vec front,Vec back,unsigned rows,unsigned segments){
 Vec center=Mul(Add(north,south),.5f),z=Mul(Sub(north,south),.5f),x=Mul(Sub(east,west),.5f),y=Mul(Sub(front,back),.5f);
 for(unsigned row=0;row<=rows;row++){
  float phi=.001f+(3.14159265359f-.002f)*row/rows,h=std::cos(phi),r=(1-.13f*h)*std::sin(phi);
  for(unsigned col=0;col<segments;col++){float angle=6.28318530718f*col/segments;points[row*segments+col]=Add(center,Add(Mul(z,h),Mul(Add(Mul(x,std::cos(angle)),Mul(y,std::sin(angle))),r)));}
 }
 points[(rows+1)*segments]=north;points[(rows+1)*segments+1]=south;
}
}
