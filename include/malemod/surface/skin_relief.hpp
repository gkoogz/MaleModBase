#pragma once
#include "presentation.hpp"
#include <queue>
#include <limits>

namespace malemod::surface {
// Material-space skin relief. This changes display geometry only: mechanical
// owners, volumes, socket positions and simulation cadence remain independent.
class SkinRelief {
 using Vec=PresentationPoint;
 struct Sample {double x=0,z=0,front=0,fade=0,metric=0;};
 std::vector<Sample> samples_;
 std::vector<std::array<unsigned,3>> faces_;
 std::vector<std::array<unsigned,2>> edges_;
 double radius_=1;
 static double Smooth(double t){t=std::clamp(t,0.,1.);return t*t*t*(10+t*(-15+6*t));}
 public:
 SkinRelief()=default;
 SkinRelief(const std::vector<Vec>& rest,const std::vector<std::array<unsigned,3>>& faces,
  const std::vector<bool>& active,Vec center,PresentationBasis chart,double radius):radius_(radius){
  if(rest.size()!=active.size()||!std::isfinite(radius)||radius<=0)throw std::invalid_argument("Invalid skin relief material");
  samples_.resize(rest.size());std::vector<Vec> normals(rest.size());
  std::vector<std::vector<std::pair<unsigned,double>>> adjacency(rest.size());
  std::vector<std::array<unsigned,2>> edges;
  for(auto f:faces){
   for(auto i:f)if(i>=rest.size())throw std::invalid_argument("Skin relief face outside material");
   auto n=Cross(Sub(rest[f[1]],rest[f[0]]),Sub(rest[f[2]],rest[f[0]]));
   for(auto i:f)normals[i]=Add(normals[i],n);
   if(active[f[0]]||active[f[1]]||active[f[2]]){
    faces_.push_back(f);
    for(unsigned k=0;k<3;k++){auto a=f[k],b=f[(k+1)%3];edges.push_back({(std::min)(a,b),(std::max)(a,b)});}
   }
  }
  std::sort(edges.begin(),edges.end());edges.erase(std::unique(edges.begin(),edges.end()),edges.end());edges_=std::move(edges);
  for(auto e:edges_){double length=std::sqrt(Dot(Sub(rest[e[0]],rest[e[1]]),Sub(rest[e[0]],rest[e[1]])));
   for(unsigned k=0;k<2;k++){adjacency[e[k]].push_back({e[1-k],length});samples_[e[k]].metric+=length;}}
  // Geodesic boundary fade has zero first/second derivatives. The complete
  // shaft attachment remains exact; folds continue across the lower skin join.
  std::vector<double> distance(rest.size(),std::numeric_limits<double>::infinity());
  using Node=std::pair<double,unsigned>;std::priority_queue<Node,std::vector<Node>,std::greater<Node>> queue;
  for(unsigned i=0;i<rest.size();i++)if(!active[i]){distance[i]=0;queue.push({0,i});}
  while(!queue.empty()){auto [d,i]=queue.top();queue.pop();if(d!=distance[i])continue;
   for(auto [j,length]:adjacency[i])if(d+length<distance[j]){distance[j]=d+length;queue.push({distance[j],j});}}
  for(unsigned i=0;i<rest.size();i++)if(active[i]){
   if(Dot(normals[i],normals[i])<1e-24)throw std::invalid_argument("Skin relief has undefined normal");
   auto delta=Scale(Sub(rest[i],center),1/radius);auto& s=samples_[i];
   s.x=Dot(delta,chart[1]);s.z=Dot(delta,chart[2]);
   s.front=Smooth((Dot(Unit(normals[i]),chart[0])+.15)/.8);
   s.fade=Smooth(distance[i]/(.35*radius));
  }
 }
 bool Active(unsigned i)const{return i<samples_.size()&&samples_[i].fade>0;}
 struct Result {double amplitude=1,maximumDisplacement=0;unsigned changed=0;};
 Result Apply(std::vector<Vec>& points,double scale=1)const{
  if(samples_.empty())return {};
  if(points.size()!=samples_.size()||!std::isfinite(scale)||scale<=0)throw std::invalid_argument("Invalid posed skin relief");
  std::vector<Vec> normals(points.size()),delta(points.size());std::vector<double> metric(points.size());
  for(auto f:faces_){auto n=Cross(Sub(points[f[1]],points[f[0]]),Sub(points[f[2]],points[f[0]]));for(auto i:f)normals[i]=Add(normals[i],n);}
  for(auto e:edges_){auto d=Sub(points[e[0]],points[e[1]]);double length=std::sqrt(Dot(d,d));for(auto i:e)metric[i]+=length;}
  for(unsigned i=0;i<points.size();i++)if(Active(i)&&Dot(normals[i],normals[i])>1e-24){
   const auto& s=samples_[i];const double stretch=metric[i]/(s.metric*scale);
   // No time-dependent noise: a feature is fixed to the same skin material.
   // Compression gathers folds; extension relaxes them without erasing them.
   const double gather=std::clamp(1+1.6*(1-stretch),.18,1.7);
   const double x=s.x,z=s.z;
   const double neck=Smooth((z-.25)/1.25),bottom=1-Smooth((z+.8)/1.5);
   const double phase=13.5*z+1.25*std::sin(2.7*x)+.6*std::sin(3.1*z+1.8*x);
   const double wrinkles=(.012+.006*bottom)*(std::sin(phase)+.32*std::sin(1.79*phase+2.3*x));
   const double neckFolds=.019*neck*std::sin(10*x+.65*std::sin(2.1*z)+.3*x*z);
   // A shallow longitudinal web beside the raphe, fading into the sac and
   // the existing attachment. Its varying width avoids a second cord/seam.
   const double lateral=.22+.065*Smooth((z-.1)/2),width=.17+.055*(1-neck);
   const double ridge=std::exp(-std::pow((std::abs(x)-lateral)/width,2));
   const double central=Smooth(std::abs(x)/.12);
   const double web=.045*ridge*central*Smooth((z+.05)/.85)*s.front;
   const double height=radius_*scale*s.fade*(gather*(wrinkles+neckFolds)*(.55+.45*s.front)+web);
   delta[i]=Scale(Unit(normals[i]),height);
  }
  // A common amplitude, never independent vertex clipping, preserves the
  // smooth field while preventing new collapsed/reversed display triangles.
  double amplitude=1;
  for(unsigned iteration=0;iteration<16;iteration++){
   bool valid=true;
   for(auto f:faces_){auto a=Sub(points[f[1]],points[f[0]]),b=Sub(points[f[2]],points[f[0]]),n=Cross(a,b);
    auto de=Sub(delta[f[1]],delta[f[0]]),df=Sub(delta[f[2]],delta[f[0]]);
    auto after=Cross(Add(a,Scale(de,amplitude)),Add(b,Scale(df,amplitude)));
    if(Dot(after,n)<.2*Dot(n,n)){valid=false;break;}}
   if(valid)break;
   amplitude*=.5;if(iteration==15)amplitude=0;
  }
  Result result;result.amplitude=amplitude;
  for(unsigned i=0;i<points.size();i++)if(Active(i)){
   auto d=Scale(delta[i],amplitude);double length=std::sqrt(Dot(d,d));
   if(length>0){points[i]=Add(points[i],d);++result.changed;result.maximumDisplacement=(std::max)(result.maximumDisplacement,length);}}
  return result;
 }
};
}
