#pragma once
#include <array>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace malemod::surface::angle_collar {
using Point=std::array<double,3>;
inline double Smooth(double t){if(!std::isfinite(t))throw std::invalid_argument("Nonfinite collar parameter");t=std::clamp(t,0.,1.);return std::clamp(t*t*t*(10+t*(-15+6*t)),0.,1.);}
inline double Dot(Point a,Point b){return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];}
inline Point Sub(Point a,Point b){return {a[0]-b[0],a[1]-b[1],a[2]-b[2]};}
inline Point Unit(Point a){double n=std::sqrt(Dot(a,a));if(!std::isfinite(n)||n<1e-12)throw std::invalid_argument("Invalid collar frame");for(auto& x:a)x/=n;return a;}
// A pose corrective driven by the authored rest guide, never the oscillating
// live guide. Downward/horizontal shapes take the exact original path. The
// response reaches full strength at 25 degrees upward with C2 endpoints.
inline double Response(Point authoredDirection,Point forward,Point up){
 authoredDirection=Unit(authoredDirection);forward=Unit(forward);up=Unit(up);
 if(std::abs(Dot(forward,up))>1e-6)throw std::invalid_argument("Nonorthogonal collar frame");
 const double pitch=std::atan2(Dot(authoredDirection,up),Dot(authoredDirection,forward));
 return Smooth(pitch/(25.*3.14159265358979323846/180.));
}
// Reference 1..100 angle contract: low=-80deg, neutral=30deg at UI50.
// Use the requested slider, not the measured/rest guide (which also changes
// with shape fitting). The downward half is exactly inactive.
inline double SliderResponse(double ui){
 if(!std::isfinite(ui)||ui<1||ui>100)throw std::invalid_argument("Invalid angle slider");
 if(ui>=50)return 0;
 const double degrees=-80.+110.*(ui-1.)/49.;
 return Smooth(-degrees/25.);
}
inline double GrowthResponse(double radius,double neutralRadius){
 if(!std::isfinite(radius)||!std::isfinite(neutralRadius)||radius<=0||neutralRadius<=0)throw std::invalid_argument("Invalid collar radius");
 return Smooth((radius-neutralRadius)/(.75*neutralRadius));
}
class RestWeights {
 std::vector<double> distance_;
 public:
 RestWeights(const std::vector<Point>& rest,const std::vector<Point>& opening){
  if(opening.size()<3)throw std::invalid_argument("Incomplete collar opening");
  for(auto p:rest){double best=1e100;for(auto q:opening){auto d=Sub(p,q);double ds=Dot(d,d);if(!std::isfinite(ds))throw std::invalid_argument("Nonfinite collar binding");best=std::min(best,ds);}distance_.push_back(std::sqrt(best));}
 }
 double Weight(unsigned i,double reach)const{
  if(i>=distance_.size()||!std::isfinite(reach)||reach<=0)throw std::invalid_argument("Invalid collar reach");
  return Smooth(distance_[i]/reach);
 }
};
// Immutable material-sector weight. The ventral surface receives exactly zero;
// the dorsal correction joins it with zero first/second angular derivatives.
inline double DorsalWeight(Point rest,Point root,Point side,Point up){
 const auto q=Sub(rest,root);side=Unit(side);up=Unit(up);
 const double z=Dot(q,up),y=Dot(q,side),r=std::hypot(y,z);
 return r>1e-12?Smooth(z/r):0.;
}
// Reconcile the original solved surface with the same solve whose recruited
// dorsal boundary is preserved and whose module reach uses rest dimensions.
// Only the upward component is restored: no lateral narrowing or forward wedge.
// Exact seams and orientation constraints must be projected after this field.
inline Point Reconcile(Point original,Point supported,double materialWeight,double response,Point up){
 if(!std::isfinite(materialWeight)||!std::isfinite(response)||materialWeight<0||materialWeight>1||response<0||response>1)throw std::invalid_argument("Invalid collar response");
 if(response==0||materialWeight==0)return original;
 up=Unit(up);double lift=std::max(0.,Dot(Sub(supported,original),up))*materialWeight*response;
 for(unsigned a=0;a<3;a++)original[a]+=up[a]*lift;
 return original;
}
}
