#pragma once
#include "jockstrap.hpp"
namespace malemod::garments {
// Stable triangle ownership, refitted from measured vertices. Each convex
// capsule encloses every vertex of its assigned triangles, hence those faces.
// Source topology and semantic regions come from the adapter, never bone names.
class MeasuredCapsules {
 std::vector<std::vector<unsigned>> supports_;
 std::vector<std::array<std::vector<unsigned>,2>> ends_;
 static Point Axis(const std::vector<Point>& points,Point center){
  double matrix[3][3]{};for(auto p:points){auto d=Sub(p,center);for(unsigned i=0;i<3;i++)for(unsigned j=0;j<3;j++)matrix[i][j]+=d[i]*d[j];}
  unsigned major=0;for(unsigned i=1;i<3;i++)if(matrix[i][i]>matrix[major][major])major=i;
  Point axis{};axis[major]=1;
  for(unsigned n=0;n<16;n++){Point next{};for(unsigned i=0;i<3;i++)for(unsigned j=0;j<3;j++)next[i]+=matrix[i][j]*axis[j];if(Length(next)<1e-20)break;axis=Unit(next);}return axis;
 }
public:
 static double RayExit(Point origin,Point direction,const Capsule& capsule){
  direction=Unit(direction);double far=0;
  auto roots=[&](double a,double b,double c,auto accept){
   double discriminant=b*b-a*c;if(a<1e-14||discriminant<0)return;
   double root=std::sqrt(discriminant);for(double t:{(-b-root)/a,(-b+root)/a})if(t>=0&&accept(t))far=(std::max)(far,t);
  };
  for(auto center:{capsule.a,capsule.b}){auto d=Sub(origin,center);roots(1,Dot(d,direction),Dot(d,d)-capsule.radius*capsule.radius,[](double){return true;});}
  auto u=Sub(capsule.b,capsule.a);double length=Length(u);
  if(length>1e-12){u=Mul(u,1/length);auto d=Sub(origin,capsule.a);double along=Dot(d,u),rate=Dot(direction,u);auto radial=Sub(d,Mul(u,along)),velocity=Sub(direction,Mul(u,rate));roots(Dot(velocity,velocity),Dot(radial,velocity),Dot(radial,radial)-capsule.radius*capsule.radius,[&](double t){double x=along+t*rate;return x>=0&&x<=length;});}
  return far;
 }
 void Build(const Input& reference){
  supports_.clear();ends_.clear();const unsigned bins[4]{6,2,3,3};unsigned offsets[4]{0,6,8,11};
  std::vector<unsigned> region(reference.anatomy.size(),0);std::array<Point,4> centers{},axes{};double low[4],high[4];
  for(unsigned r=0;r<4;r++){
   std::vector<Point> points;for(auto id:reference.anatomyRegions[r]){if(id>=region.size())throw std::invalid_argument("Anatomy region outside surface");region[id]=r;points.push_back(reference.frame.Local(reference.anatomy[id].position));centers[r]=Add(centers[r],points.back());}
   if(points.empty())throw std::invalid_argument("Measured capsule fitting requires all four anatomy regions");
   centers[r]=Mul(centers[r],1./points.size());axes[r]=Axis(points,centers[r]);low[r]=1e100;high[r]=-1e100;
   for(auto p:points){double t=Dot(Sub(p,centers[r]),axes[r]);low[r]=(std::min)(low[r],t);high[r]=(std::max)(high[r],t);}
  }
  std::vector<std::set<unsigned>> groups(14);
  for(auto face:reference.anatomyTriangles){
   for(auto id:face)if(id>=region.size())throw std::invalid_argument("Anatomy face outside surface");
   unsigned r=region[face[0]];if(region[face[1]]==region[face[2]])r=region[face[1]];
   Point center{};for(auto id:face)center=Add(center,reference.frame.Local(reference.anatomy[id].position));center=Mul(center,1./3);
   double t=(Dot(Sub(center,centers[r]),axes[r])-low[r])/(std::max)(1e-12,high[r]-low[r]);
   unsigned bin=(std::min)(bins[r]-1,unsigned(std::clamp(t,0.,1.)*bins[r]));
   for(auto id:face)groups[offsets[r]+bin].insert(id);
  }
  for(const auto& g:groups)if(!g.empty())supports_.emplace_back(g.begin(),g.end());
  if(supports_.empty())throw std::invalid_argument("No measured capsule support faces");
  for(const auto& group:supports_){
   std::vector<Point> points;Point center{};for(auto id:group){auto p=reference.frame.Local(reference.anatomy[id].position);points.push_back(p);center=Add(center,p);}center=Mul(center,1./points.size());auto axis=Axis(points,center);
   std::vector<std::pair<double,unsigned>> order;for(unsigned i=0;i<group.size();i++)order.push_back({Dot(Sub(points[i],center),axis),group[i]});std::sort(order.begin(),order.end());
   std::array<std::vector<unsigned>,2> ends;unsigned count=(std::max)(1u,unsigned(order.size()/4));for(unsigned k=0;k<count;k++){ends[0].push_back(order[k].second);ends[1].push_back(order[order.size()-1-k].second);}ends_.push_back(std::move(ends));
  }
 }
 std::vector<Capsule> Fit(const Input& input,double scale,double clearance)const{
  if(!std::isfinite(scale)||scale<=0||clearance<0)throw std::invalid_argument("Invalid capsule fit scale");
  std::vector<Capsule> result;
  for(unsigned group=0;group<supports_.size();group++){
   const auto& support=supports_[group];
   std::vector<Point> points;Point center{};
   for(auto id:support){if(id>=input.anatomy.size())throw std::invalid_argument("Measured capsule topology changed");auto p=Mul(input.frame.Local(input.anatomy[id].position),1/scale);if(!Finite(p))throw std::invalid_argument("Nonfinite capsule support");points.push_back(p);center=Add(center,p);}
   center=Mul(center,1./points.size());std::array<Point,2> ends{};
   for(unsigned side=0;side<2;side++){for(auto id:ends_[group][side])ends[side]=Add(ends[side],Mul(input.frame.Local(input.anatomy[id].position),1/scale));ends[side]=Mul(ends[side],1./ends_[group][side].size());}
   // Persistent source bindings avoid PCA eigenvector jumps on round cross
   // sections. Such jumps create fictitious swept collider motion.
   auto axis=Unit(Sub(ends[1],ends[0]));double low=1e100,high=-1e100,radius=0;
   for(auto p:points){auto d=Sub(p,center);double t=Dot(d,axis);low=(std::min)(low,t);high=(std::max)(high,t);radius=(std::max)(radius,Length(Sub(d,Mul(axis,t))));}
   // Endpoint extrema are the surface extent, not the capsule's sphere
   // centers. Search enclosed axial capsules instead of adding a whole radius
   // beyond both anatomical ends (which falsely engulfs sewn attachments).
   const double middle=(low+high)*.5,span=(high-low)*.5;double best=1e100,bestHalf=0,bestRadius=0;
   std::vector<std::array<double,2>> radial;for(auto p:points){auto d=Sub(p,center);double t=Dot(d,axis);radial.push_back({std::abs(t-middle),Dot(d,d)-t*t});}
   for(unsigned step=0;step<=24;step++){
    double half=span*step/24,r2=0;for(auto p:radial){double end=(std::max)(0.,p[0]-half);r2=(std::max)(r2,p[1]+end*end);}double r=std::sqrt((std::max)(0.,r2));double volume=r2*(2*half+4*r/3);
    if(volume<best){best=volume;bestHalf=half;bestRadius=r;}
   }
   result.push_back({Add(center,Mul(axis,middle-bestHalf)),Add(center,Mul(axis,middle+bestHalf)),bestRadius+clearance});
  }return result;
 }
 const std::vector<std::vector<unsigned>>& Supports()const{return supports_;}
};
}
