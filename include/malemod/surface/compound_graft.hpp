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
 static void PreserveStripOrientation(std::vector<Vec>& points,const std::vector<double>& parameters_,unsigned count_){
  if(parameters_.size()<3||points.size()!=parameters_.size()*count_)throw std::invalid_argument("Compound strip dimensions differ");
  if(count_<3)return;
  std::vector<Vec> ruled(points.size());
  for(unsigned row=0;row<parameters_.size();row++)for(unsigned j=0;j<count_;j++)
   ruled[row*count_+j]=Add(Scale(points[j],1-parameters_[row]),Scale(points[points.size()-count_+j],parameters_[row]));
  auto triangles=[&](auto&& visit){for(unsigned row=0;row+1<parameters_.size();row++)for(unsigned j=0;j<count_;j++){
   unsigned k=(j+1)%count_,a=row*count_+j,b=row*count_+k,c=(row+1)*count_+j,d=(row+1)*count_+k;
   visit(b,a,c);visit(b,c,d);
  }};
  auto normal=[](const std::vector<Vec>& p,unsigned a,unsigned b,unsigned c){return Cross(Sub(p[b],p[a]),Sub(p[c],p[a]));};
  // The fallback must not contain the very endpoint overshoot being limited.
  // A boundary-weighted Hermite fallback can already fold near either rim;
  // comparing against its normals then certifies the fold as valid. Use the
  // endpoint-to-endpoint strip as the orientation reference and retain the
  // largest safe amount of the authored curvature. This changes only the
  // connecting skin: both retained surfaces and both rim positions stay exact.
  // Tangency can be relaxed when necessary to prevent a fold. This local
  // orientation test is not a substitute for whole-surface collision review.
  const auto& base=ruled;std::vector<Vec> delta(points.size());
  for(unsigned i=0;i<points.size();i++)delta[i]=Sub(points[i],base[i]);
  bool valid=true;
  triangles([&](unsigned a,unsigned b,unsigned c){auto n=normal(base,a,b,c);const double nn=Dot(n,n);if(nn>1e-28&&Dot(normal(points,a,b,c),n)<.02*nn)valid=false;});
  if(valid)return;
  auto limit=[&](unsigned i,unsigned j,unsigned k,Vec di,Vec dj,Vec dk){
   double fraction=1;
   auto n=normal(base,i,j,k);double nn=Dot(n,n);if(nn<=1e-28)return 1.;
   auto a=Sub(base[j],base[i]),b=Sub(base[k],base[i]),da=Sub(dj,di),db=Sub(dk,di);
   double c=Dot(Cross(a,b),n)-.02*nn;
   if(c<0)throw std::runtime_error("Compound boundary strip reverses its parameterization");
   double linear=Dot(Add(Cross(da,b),Cross(a,db)),n),quadratic=Dot(Cross(da,db),n);
   auto root=[&](double x){if(x>=0&&x<fraction)fraction=x;};
   double scale=(std::max)({std::abs(c),std::abs(linear),std::abs(quadratic),1e-300});
   if(std::abs(quadratic)<1e-14*scale){if(linear<0)root(-c/linear);}
   else{double discriminant=linear*linear-4*quadratic*c;if(discriminant>0){double q=-.5*(linear+std::copysign(std::sqrt(discriminant),linear));root(q/quadratic);if(q!=0)root(c/q);}}
   return fraction;
  };
  double fraction=1;
  triangles([&](unsigned i,unsigned j,unsigned k){fraction=(std::min)(fraction,limit(i,j,k,delta[i],delta[j],delta[k]));});fraction*=.98;
  for(unsigned i=0;i<points.size();i++)points[i]=Add(base[i],Scale(delta[i],fraction));
 }
public:
 static void ConformSourceEntry(std::vector<Vec>& points,const std::vector<double>& parameters,const std::vector<Vec>& normals){
  const unsigned count=unsigned(normals.size());
  if(count<3||parameters.size()<3||points.size()!=count*parameters.size())throw std::invalid_argument("Compound entry dimensions differ");
  const auto base=points;std::vector<Vec> delta(points.size());std::vector<double> weights(count,1.);
  unsigned rows=0;
  for(unsigned row=1;row+1<parameters.size()&&parameters[row]<.2;row++){
   rows=row;double t=parameters[row]/.2,fade=1-t*t*t*(10+t*(-15+6*t));
   for(unsigned j=0;j<count;j++){auto n=normals[j];double nn=Dot(n,n);if(nn<=1e-20)throw std::invalid_argument("Collapsed compound entry normal");delta[row*count+j]=Scale(n,-Dot(Sub(base[row*count+j],base[j]),n)*fade/nn);}
  }
  // This fillet changes only the source end of the generated skin. It keeps
  // the retained sac and the support end exact, and restores the source's
  // tangent plane without projecting the opposite end into its support.
  auto triangles=[&](auto&& visit){for(unsigned row=0;row<=rows;row++)for(unsigned j=0;j<count;j++){unsigned k=(j+1)%count,a=row*count+j,b=row*count+k,c=(row+1)*count+j,d=(row+1)*count+k;visit(b,a,c);visit(b,c,d);}};
  bool settled=false;
  for(unsigned iteration=0;iteration<48;iteration++){
   std::vector<bool> reduce(count);bool valid=true;
   triangles([&](unsigned i,unsigned j,unsigned k){auto a=Sub(base[j],base[i]),b=Sub(base[k],base[i]),n=Cross(a,b);double nn=Dot(n,n);if(nn<=1e-28)return;
    auto di=Scale(delta[i],weights[i%count]),dj=Scale(delta[j],weights[j%count]),dk=Scale(delta[k],weights[k%count]);
    if(Dot(Cross(Add(a,Sub(dj,di)),Add(b,Sub(dk,di))),n)<.1*nn){valid=false;reduce[i%count]=reduce[j%count]=reduce[k%count]=true;}
   });
   if(valid){settled=true;break;}for(unsigned j=0;j<count;j++)if(reduce[j])weights[j]*=.7;
  }
  if(!settled)return;
  for(unsigned row=1;row<=rows;row++)for(unsigned j=0;j<count;j++)points[row*count+j]=Add(base[row*count+j],Scale(delta[row*count+j],weights[j]));
 }
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
 std::vector<Vec> Evaluate(const std::vector<Vec>& source,const std::vector<Vec>& support,std::vector<Vec>* hermiteTrace=nullptr,
     std::vector<Vec>* firstHandles=nullptr,std::vector<Vec>* lastHandles=nullptr)const{
  std::vector<Vec> out(reference_.size());
  if(hermiteTrace)hermiteTrace->resize(reference_.size());
  if(firstHandles)firstHandles->resize(count_);if(lastHandles)lastHandles->resize(count_);
  for(unsigned j=0;j<count_;j++){
   const auto a=first_[j].Point(source),b=last_[j].Point(support);
   auto first=first_[j].Direction(firstTangent_[j],source),last=last_[j].Direction(lastTangent_[j],support);
   // A surface differential's parameter density is not the length of skin
   // spanning between surfaces. Preserve its tangent direction (and hence
   // the join's tangent plane), but scale each authored handle by the actual
   // endpoint separation. Otherwise a compressed gap can receive a handle
   // several times longer than the gap and fold back through the retained skin.
   auto span=Sub(b,a),referenceSpan=Sub(end_[j],start_[j]);
   double distance=std::sqrt(Dot(span,span)),referenceDistance=std::sqrt(Dot(referenceSpan,referenceSpan));
   if(referenceDistance<=1e-14||distance<=1e-14)throw std::runtime_error("Compound connection span collapsed");
   auto matchLength=[&](Vec direction,Vec authored){double length=std::sqrt(Dot(direction,direction)),restLength=std::sqrt(Dot(authored,authored));if(restLength==0)return Vec{};if(length<=1e-14)throw std::runtime_error("Compound tangent collapsed");return Scale(direction,(std::min)(1.,restLength/referenceDistance*distance/length));};
   first=matchLength(first,firstTangent_[j]);last=matchLength(last,lastTangent_[j]);
   if(firstHandles)(*firstHandles)[j]=first;if(lastHandles)(*lastHandles)[j]=last;
   out[j]=a;out[out.size()-count_+j]=b;
   if(hermiteTrace){(*hermiteTrace)[j]=a;(*hermiteTrace)[out.size()-count_+j]=b;}
   for(unsigned row=1;row+1<parameters_.size();row++){
    const double t=parameters_[row],w=t*t*t*(10+t*(-15+6*t));
    const auto residual=Sub(reference_[row*count_+j],Hermite(start_[j],end_[j],firstTangent_[j],lastTangent_[j],t));
    if(hermiteTrace)(*hermiteTrace)[row*count_+j]=Hermite(a,b,first,last,t);
    out[row*count_+j]=Add(Hermite(a,b,first,last,t),Add(Scale(first_[j].Direction(residual,source),1-w),Scale(last_[j].Direction(residual,support),w)));
   }
  }
  PreserveStripOrientation(out,parameters_,count_);
  return out;
 }
 CompoundGraftPlan Rebased(const std::vector<Vec>& source,const std::vector<Vec>& support)const{
  std::vector<Vec> first,last;auto patch=Evaluate(source,support,nullptr,&first,&last);
  std::vector<CompoundSurfaceDonor> starts,ends;for(const auto& endpoint:first_)starts.push_back(endpoint.donor);for(const auto& endpoint:last_)ends.push_back(endpoint.donor);
  return CompoundGraftPlan(source,support,starts,ends,std::move(first),std::move(last),parameters_,std::move(patch));
 }
};
}
