#pragma once
#include "presentation.hpp"
#include <algorithm>
#include <limits>
#include <numeric>
#include <vector>

namespace malemod::surface {
// Conservative broad phase only. Intersection arithmetic and source-face tie
// order are identical to the measured-surface scan used by the aperture fitter.
class TriangleRayIndex {
 using Vec=PresentationPoint;
 struct Node {Vec low{},high{};unsigned begin=0,end=0,left=0,right=0;};
 std::vector<Node> nodes_;
 std::vector<unsigned> ids_;
 unsigned Build(const std::vector<Vec>& points,const std::vector<std::array<unsigned,3>>& faces,unsigned begin,unsigned end){
  Node n;n.begin=begin;n.end=end;n.low=points[faces[ids_[begin]][0]];n.high=n.low;
  for(unsigned i=begin;i<end;i++)for(auto v:faces[ids_[i]])for(unsigned k=0;k<3;k++){n.low[k]=(std::min)(n.low[k],points[v][k]);n.high[k]=(std::max)(n.high[k],points[v][k]);}
  // The narrow phase admits barycentric excursions of 1e-9 on each edge.
  for(unsigned k=0;k<3;k++){double pad=4e-9*(n.high[k]-n.low[k])+1e-10;n.low[k]-=pad;n.high[k]+=pad;}
  unsigned id=unsigned(nodes_.size());nodes_.push_back(n);
  if(end-begin>8){unsigned axis=0;for(unsigned k=1;k<3;k++)if(n.high[k]-n.low[k]>n.high[axis]-n.low[axis])axis=k;
   auto center=[&](unsigned f){return points[faces[f][0]][axis]+points[faces[f][1]][axis]+points[faces[f][2]][axis];};
   unsigned mid=begin+(end-begin)/2;
   std::nth_element(ids_.begin()+begin,ids_.begin()+mid,ids_.begin()+end,[&](unsigned a,unsigned b){double x=center(a),y=center(b);return x!=y?x<y:a<b;});
   unsigned left=Build(points,faces,begin,mid),right=Build(points,faces,mid,end);nodes_[id].left=left;nodes_[id].right=right;
  }return id;
 }
 static bool Intersects(const Node& n,Vec start,Vec direction,double nearest){
  double lo=0,hi=nearest;
  for(unsigned k=0;k<3;k++){
   if(direction[k]==0){if(start[k]<n.low[k]||start[k]>n.high[k])return false;continue;}
   double a=(n.low[k]-start[k])/direction[k],b=(n.high[k]-start[k])/direction[k];if(a>b)std::swap(a,b);
   lo=(std::max)(lo,a);hi=(std::min)(hi,b);if(lo>hi)return false;
  }return true;
 }
 public:
 TriangleRayIndex()=default;
 TriangleRayIndex(const std::vector<Vec>& points,const std::vector<std::array<unsigned,3>>& faces,const std::vector<bool>& eligible){
  if(eligible.size()!=faces.size())throw std::invalid_argument("Ray eligibility differs from source faces");
  for(unsigned i=0;i<faces.size();i++){for(auto v:faces[i])if(v>=points.size())throw std::invalid_argument("Ray face leaves source points");if(eligible[i])ids_.push_back(i);}
  nodes_.reserve(ids_.size()*2);if(!ids_.empty())Build(points,faces,0,unsigned(ids_.size()));
 }
 bool Ray(const std::vector<Vec>& points,const std::vector<std::array<unsigned,3>>& faces,Vec start,Vec direction,Vec& point,Vec facingAxis,double* facing=nullptr)const{
  if(nodes_.empty())return false;
  double nearest=std::numeric_limits<double>::infinity();unsigned best=~0u;
  // Median splitting bounds depth to log2(face count); no per-ray allocation.
  std::array<unsigned,64> stack{};unsigned count=1;
  while(count){const auto& n=nodes_[stack[--count]];if(!Intersects(n,start,direction,nearest))continue;
   if(n.left){if(count+2>stack.size())throw std::runtime_error("Ray traversal stack exceeded");stack[count++]=n.right;stack[count++]=n.left;continue;}
   for(unsigned j=n.begin;j<n.end;j++){
    unsigned i=ids_[j];auto f=faces[i];auto edge1=Sub(points[f[1]],points[f[0]]),edge2=Sub(points[f[2]],points[f[0]]),h=Cross(direction,edge2);const double determinant=Dot(edge1,h);if(std::abs(determinant)<1e-12)continue;
    auto s=Sub(start,points[f[0]]);const double u=Dot(s,h)/determinant;if(u< -1e-9||u>1+1e-9)continue;
    auto q=Cross(s,edge1);const double v=Dot(direction,q)/determinant;if(v< -1e-9||u+v>1+1e-9)continue;
    const double t=Dot(edge2,q)/determinant;
    if(t>=0&&(t<nearest||(t==nearest&&i<best))){nearest=t;best=i;point=Add(start,Scale(direction,t));if(facing)*facing=Dot(Unit(Cross(edge1,edge2)),facingAxis);}
   }
  }return best!=~0u;
 }
};
}
