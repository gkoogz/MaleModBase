#pragma once
// Source-derived float support; see provenance/ovoid-support.json.
#include "state.hpp"
namespace malemod::physics {
inline V3 OvoidSupport(V3 n,V3 r){
 float h=sqrtf(n.x*n.x*r.x*r.x+n.y*n.y*r.y*r.y),v=n.z*r.z;
 float sign=r.z;if(v<0.f)sign=-r.z;
 if(h<1e-8f)return {0,0,sign};
 // The tapered ellipse has strictly negative support-objective curvature.
 // Newton from the untapered ellipse converges without the quantization of
 // a 22-step float bisection, and is cheaper during normal refinement.
 float z=(v)/sqrtf((h)*h+(v)*v),ratio=(v)/h;
 z=max(-.999999f,min(.999999f,z));
 for(int i=0;i<6;i++){
  float radial=sqrtf(max(1e-24f,1.f-z*z));
  float derivative=-.13f*radial-(1.f-.13f*z)*z/radial+ratio;
  float curvature=(-1.f+.39f*z-.26f*z*z*z)/(radial*radial*radial);
  z=max(-.999999f,min(.999999f,z-derivative/curvature));
 }
 float section=float((1.f-.13f*z)*sqrtf(max(0.f,1.f-z*z)));
 return {r.x*r.x*n.x/h*section,r.y*r.y*n.y/h*section,(r.z*z)};
}
}
