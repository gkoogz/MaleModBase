#pragma once
#include "convex_surface_cut.hpp"
#include <limits>

namespace malemod::surface {
struct PelvicAperture {
 SurfaceCut body;
 PresentationPoint anchor{},translation{};
 std::vector<std::array<double,2>> footprint;
 double maximumOriginalDisplacement=0;
};
class PelvicAperturePlan {
 using Vec=PresentationPoint;using UV=std::array<double,2>;
 std::vector<Vec> body_,section_;std::vector<std::array<unsigned,3>> faces_;std::vector<bool> front_;
 Vec origin_;PresentationBasis basis_;double tolerance_,extent_,depth_;
 static double Cross2(UV a,UV b,UV c){return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0]);}
 static std::vector<UV> Hull(std::vector<UV> p){
  std::sort(p.begin(),p.end());p.erase(std::unique(p.begin(),p.end()),p.end());if(p.size()<3)throw std::invalid_argument("Degenerate aperture section");
  std::vector<UV> h;for(auto a:p){while(h.size()>1&&Cross2(h[h.size()-2],h.back(),a)<=1e-12)h.pop_back();h.push_back(a);}const auto lower=h.size();
  for(auto i=p.rbegin()+1;i!=p.rend();i++){while(h.size()>lower&&Cross2(h[h.size()-2],h.back(),*i)<=1e-12)h.pop_back();h.push_back(*i);}h.pop_back();return h;
 }
 bool Ray(Vec start,Vec direction,Vec& point,double* facing=nullptr)const{
  double nearest=std::numeric_limits<double>::infinity();bool found=false;
  for(unsigned i=0;i<faces_.size();i++)if(front_[i]){
   auto f=faces_[i];auto edge1=Sub(body_[f[1]],body_[f[0]]),edge2=Sub(body_[f[2]],body_[f[0]]),h=Cross(direction,edge2);const double determinant=Dot(edge1,h);if(std::abs(determinant)<1e-12)continue;
   auto s=Sub(start,body_[f[0]]);const double u=Dot(s,h)/determinant;if(u< -1e-9||u>1+1e-9)continue;
   auto q=Cross(s,edge1);const double v=Dot(direction,q)/determinant;if(v< -1e-9||u+v>1+1e-9)continue;
   const double t=Dot(edge2,q)/determinant;if(t>=0&&t<nearest){nearest=t;point=Add(start,Scale(direction,t));found=true;if(facing)*facing=Dot(Unit(Cross(edge1,edge2)),basis_[0]);}
  }return found;
 }
 public:
 PelvicAperturePlan(std::vector<Vec> body,std::vector<std::array<unsigned,3>> faces,Vec origin,PresentationBasis basis,std::vector<Vec> section,double tolerance)
 :body_(std::move(body)),section_(std::move(section)),faces_(std::move(faces)),origin_(origin),basis_(basis),tolerance_(tolerance){
  if(body_.empty()||section_.size()<8||tolerance<=0)throw std::invalid_argument("Invalid measured aperture domain");Quaternion(basis_);
  Vec low=body_[0],high=body_[0];for(auto p:body_)for(unsigned k=0;k<3;k++){low[k]=(std::min)(low[k],p[k]);high[k]=(std::max)(high[k],p[k]);}extent_=2*std::sqrt(Dot(Sub(high,low),Sub(high,low)));
  for(auto f:faces_){for(auto v:f)if(v>=body_.size())throw std::invalid_argument("Aperture face leaves original body");front_.push_back(Dot(Cross(Sub(body_[f[1]],body_[f[0]]),Sub(body_[f[2]],body_[f[0]])),basis_[0])>1e-10);}
  Vec hit;if(!Ray(Add(origin_,Scale(basis_[0],extent_)),Scale(basis_[0],-1),hit))throw std::invalid_argument("Reference anchor has no measured original body surface");depth_=Dot(Sub(hit,origin_),basis_[0]);
 }
 PelvicAperture Solve(Vec axis,double scale,double margin=.12,double underside=.08,double minimumChartFacing=.35)const{
  axis=Unit(axis);if(!std::isfinite(scale)||scale<=0||margin<=0||underside<0||Dot(axis,basis_[0])<.15)throw std::invalid_argument("Unsupported aperture frame");
  auto side=Unit(Sub(basis_[1],Scale(axis,Dot(axis,basis_[1])))),up=Unit(Cross(axis,side));
  double radius=0;for(auto p:section_)radius=(std::max)(radius,std::hypot(Dot(p,basis_[1]),Dot(p,basis_[2]))*scale);
  // The unmodified perineal silhouette has finite room. Find the lowest
  // measured mount which contains the requested circumference; move anatomy
  // together instead of chasing a missing ray onto the back of a thigh.
  for(unsigned attempt=0;attempt<=96;attempt++){
   const double lift=radius*2*attempt/96.;auto anchor=Add(origin_,Scale(basis_[2],lift));Vec center;
   if(!Ray(Add(anchor,Scale(basis_[0],extent_)),Scale(basis_[0],-1),center))continue;
   anchor=Sub(center,Scale(basis_[0],depth_));std::vector<UV> cloud;bool valid=true;
   for(auto original:section_){
    const double y=Dot(original,basis_[1])*scale,z=Dot(original,basis_[2])*scale,r=std::hypot(y,z),ventral=std::pow((std::max)(0.,-z/r),4),support=1+margin+underside*ventral;
    UV neutral{y*support,z*support};Vec sample=Add(anchor,Add(Scale(basis_[1],neutral[0]),Scale(basis_[2],neutral[1]))),hit;
    double facing=0;if(!Ray(Add(sample,Scale(basis_[0],extent_)),Scale(basis_[0],-1),hit,&facing)||facing<minimumChartFacing){valid=false;break;}
    cloud.push_back(neutral);
    const auto radial=Add(Scale(basis_[1],y),Scale(basis_[2],z));const auto tilt=Sub(axis,Scale(basis_[0],Dot(axis,basis_[0])));
    if(Dot(radial,tilt)>0){
     auto start=Add(anchor,Add(Scale(side,neutral[0]),Scale(up,neutral[1])));
     if(!Ray(Add(start,Scale(axis,extent_)),Scale(axis,-1),hit)){valid=false;break;}
     auto q=Sub(hit,anchor);cloud.push_back({Dot(q,basis_[1]),Dot(q,basis_[2])});
    }
   }
   if(!valid)continue;auto footprint=Hull(std::move(cloud));
   // Verify the entire polygon edges, not just their vertices, against the
   // original surface. This rejects a chord spanning the gap between legs.
   for(unsigned i=0;i<footprint.size()&&valid;i++)for(unsigned j=0;j<5;j++){
    const double t=j/4.;auto a=footprint[i],b=footprint[(i+1)%footprint.size()];auto p=Add(anchor,Add(Scale(basis_[1],a[0]*(1-t)+b[0]*t),Scale(basis_[2],a[1]*(1-t)+b[1]*t)));Vec hit;
    double facing=0;if(!Ray(Add(p,Scale(basis_[0],extent_)),Scale(basis_[0],-1),hit,&facing)||facing<minimumChartFacing){valid=false;break;}
   }
   if(!valid)continue;
   try{
    ConvexSurfaceCut cutter(body_,faces_,front_,anchor,{{basis_[1],basis_[2]}},tolerance_);auto cut=cutter.Cut(footprint);
    return {std::move(cut),anchor,Sub(anchor,origin_),std::move(footprint),0};
   }catch(const std::runtime_error& e){
    // At the perineal turn a projected circle can enclose a second, rear
    // surface component. It is not a valid front attachment. Only that
    // measured topological fit failure permits trying a higher anchor;
    // numerical, nonmanifold and donor errors remain fatal.
    if(std::string(e.what()).find("Opening has multiple boundary components")!=0)throw;
   }
  }
  throw std::runtime_error("Requested aperture cannot fit the measured original pelvis");
 }
};
}
