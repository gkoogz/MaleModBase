#pragma once
#include "presentation.hpp"
#include <vector>
#include <stdexcept>
#include <cmath>
#include <utility>

namespace malemod::surface {
// A rim can cross retained body, a generated root ramp and moving shaft.
// Its owner is the evaluated supporting triangle, not one skeletal frame.
// The caller must finish support recruitment/physics BEFORE evaluating this
// patch, and must share its end vertices with that support in the visible mesh.
struct CompoundSurfaceDonor {
 std::array<unsigned,3> triangle{};
 std::array<double,3> barycentric{};
};
class CompoundGraftPlan {
 using Vec=PresentationPoint;
 struct Endpoint {
  CompoundSurfaceDonor donor;
  Vec a,b,normal;double aa=0,ab=0,bb=0,det=0,area=0;
  Endpoint(const std::vector<Vec>& source,CompoundSurfaceDonor d):donor(d){
   double sum=0;for(unsigned k=0;k<3;k++){
    if(d.triangle[k]>=source.size()||!std::isfinite(d.barycentric[k])||d.barycentric[k]<-1e-10)
     throw std::invalid_argument("Invalid compound surface donor");
    sum+=d.barycentric[k];
   }
   if(std::abs(sum-1)>1e-9)throw std::invalid_argument("Compound donor weights do not sum to one");
   a=Sub(source[d.triangle[1]],source[d.triangle[0]]);b=Sub(source[d.triangle[2]],source[d.triangle[0]]);
   auto n=Cross(a,b);area=std::sqrt(Dot(n,n));
   aa=Dot(a,a);ab=Dot(a,b);bb=Dot(b,b);det=aa*bb-ab*ab;
   if(area<=1e-14||det<=0)throw std::invalid_argument("Degenerate compound reference triangle");
   normal=Scale(n,1/area);
  }
  Vec Point(const std::vector<Vec>& source)const{
   Vec p{};for(unsigned k=0;k<3;k++){
    const auto v=source.at(donor.triangle[k]);for(double x:v)if(!std::isfinite(x))throw std::invalid_argument("Nonfinite live compound donor");
    p=Add(p,Scale(v,donor.barycentric[k]));
   }return p;
  }
  Vec Direction(Vec v,const std::vector<Vec>& source)const{
   auto u=Sub(source.at(donor.triangle[1]),source.at(donor.triangle[0]));
   auto w=Sub(source.at(donor.triangle[2]),source.at(donor.triangle[0]));
   auto n=Cross(u,w);const double liveArea=std::sqrt(Dot(n,n));
   if(!std::isfinite(liveArea)||liveArea<=1e-14)throw std::invalid_argument("Compound support collapsed during motion");
   const double x=(Dot(v,a)*bb-Dot(v,b)*ab)/det,y=(Dot(v,b)*aa-Dot(v,a)*ab)/det;
   return Add(Add(Scale(u,x),Scale(w,y)),Scale(n,Dot(v,normal)*std::sqrt(liveArea/area)/liveArea));
  }
 };
 std::vector<Endpoint> first_,last_;
 std::vector<Vec> reference_,start_,end_,firstTangent_,lastTangent_;
 std::vector<double> parameters_;
 unsigned count_=0;
 static Vec Hermite(Vec a,Vec b,Vec first,Vec last,double t){
  return Add(Add(Scale(a,2*t*t*t-3*t*t+1),Scale(b,-2*t*t*t+3*t*t)),
             Add(Scale(first,t*t*t-2*t*t+t),Scale(last,t*t*t-t*t)));
 }
public:
 CompoundGraftPlan(const std::vector<Vec>& source,const std::vector<Vec>& support,
     const std::vector<CompoundSurfaceDonor>& first,const std::vector<CompoundSurfaceDonor>& last,
     std::vector<Vec> firstTangents,std::vector<Vec> lastTangents,
     std::vector<double> parameters,std::vector<Vec> patch)
 :reference_(std::move(patch)),firstTangent_(std::move(firstTangents)),lastTangent_(std::move(lastTangents)),parameters_(std::move(parameters)),count_(unsigned(first.size())){
  if(!count_||last.size()!=count_||firstTangent_.size()!=count_||lastTangent_.size()!=count_||parameters_.size()<3||reference_.size()!=parameters_.size()*count_)
   throw std::invalid_argument("Compound graft dimensions differ");
  if(parameters_.front()!=0||parameters_.back()!=1)throw std::invalid_argument("Compound graft endpoints missing");
  for(unsigned i=0;i<parameters_.size();i++)if(!std::isfinite(parameters_[i])||(i&&parameters_[i]<=parameters_[i-1]))throw std::invalid_argument("Invalid compound row parameter");
  const std::array<const std::vector<Vec>*,5> fields{&source,&support,&reference_,&firstTangent_,&lastTangent_};
  for(const auto* values:fields)for(auto p:*values)for(double v:p)if(!std::isfinite(v))throw std::invalid_argument("Nonfinite compound geometry");
  for(unsigned j=0;j<count_;j++){
   first_.emplace_back(source,first[j]);last_.emplace_back(support,last[j]);
   start_.push_back(first_.back().Point(source));end_.push_back(last_.back().Point(support));
   for(auto pair:{std::pair<Vec,Vec>{reference_[j],start_.back()},std::pair<Vec,Vec>{reference_[reference_.size()-count_+j],end_.back()}}){
    auto delta=Sub(pair.first,pair.second);if(Dot(delta,delta)>1e-14)throw std::invalid_argument("Compound rim differs from its surface donor");
   }
  }
 }
 std::vector<Vec> Evaluate(const std::vector<Vec>& source,const std::vector<Vec>& support)const{
  std::vector<Vec> out(reference_.size());
  for(unsigned j=0;j<count_;j++){
   const auto a=first_[j].Point(source),b=last_[j].Point(support);
   const auto first=first_[j].Direction(firstTangent_[j],source),last=last_[j].Direction(lastTangent_[j],support);
   out[j]=a;out[out.size()-count_+j]=b;
   for(unsigned row=1;row+1<parameters_.size();row++){
    const double t=parameters_[row],w=t*t*t*(10+t*(-15+6*t));
    const auto residual=Sub(reference_[row*count_+j],Hermite(start_[j],end_[j],firstTangent_[j],lastTangent_[j],t));
    out[row*count_+j]=Add(Hermite(a,b,first,last,t),Add(Scale(first_[j].Direction(residual,source),1-w),Scale(last_[j].Direction(residual,support),w)));
   }
  }
  return out;
 }
};
}
