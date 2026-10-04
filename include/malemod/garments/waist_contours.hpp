#pragma once
// SDK-free transverse body cuts. Attachment indices refer to the supplied
// surface, so adapters retain native lineage and part boundaries themselves.
#include <array>
#include <vector>
#include <map>
#include <set>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <functional>

namespace malemod::garments::waist {
using Point=std::array<double,3>;
struct Attachment {
 Point position{},normal{};
 std::array<std::uint32_t,4> vertices{};
 std::array<double,4> weights{};
};
struct Contour {
 std::vector<Attachment> points;
 double circumference=0,signedArea=0,height=0;
};
struct BandContours {Contour bottom,top;};
namespace detail {
inline Point Add(Point a,Point b){return {a[0]+b[0],a[1]+b[1],a[2]+b[2]};}
inline Point Sub(Point a,Point b){return {a[0]-b[0],a[1]-b[1],a[2]-b[2]};}
inline Point Mul(Point a,double s){return {a[0]*s,a[1]*s,a[2]*s};}
inline double Length(Point a){return std::sqrt(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]);}
inline bool Finite(Point p){return std::isfinite(p[0])&&std::isfinite(p[1])&&std::isfinite(p[2]);}
inline Point Unit(Point p){double n=Length(p);if(n<1e-14||!std::isfinite(n))throw std::invalid_argument("Waist attachment normal is degenerate");return Mul(p,1/n);}
inline Attachment Blend(const Attachment& a,const Attachment& b,double t){
 Attachment out;out.position=Add(Mul(a.position,1-t),Mul(b.position,t));
 auto normal=Add(Mul(a.normal,1-t),Mul(b.normal,t));out.normal=Length(normal)>1e-14?Unit(normal):(t<.5?a.normal:b.normal);
 unsigned used=0;for(unsigned side=0;side<2;side++){const auto& s=side?b:a;double factor=side?t:1-t;
  for(unsigned k=0;k<4;k++){double w=s.weights[k]*factor;if(w<=1e-15)continue;unsigned at=0;while(at<used&&out.vertices[at]!=s.vertices[k])at++;if(at==used){if(used==4)throw std::invalid_argument("Waist interpolation exceeds four surface donors");out.vertices[used++]=s.vertices[k];}out.weights[at]+=w;}}
 return out;
}
template<class Samples,class Triangles,class Frame>
Contour Slice(const Samples& surface,const Triangles& triangles,const Frame& frame,double height,unsigned samples,const std::function<double(Point)>& seating={},const Point* radialCenter=nullptr){
 if(surface.size()<3||surface.size()>262144||triangles.empty()||triangles.size()>524288||samples<8||samples>256||!std::isfinite(height))throw std::invalid_argument("Invalid transverse waist surface or resolution");
 std::vector<Point> local;local.reserve(surface.size());std::vector<double> level;level.reserve(surface.size());Point lo{1e100,1e100,1e100},hi{-1e100,-1e100,-1e100};
 for(const auto& s:surface){if(!Finite(s.position)||!Finite(s.normal))throw std::invalid_argument("Nonfinite waist surface");auto p=frame.Local(s.position);local.push_back(p);double offset=seating?seating(p):0;if(!std::isfinite(offset))throw std::invalid_argument("Nonfinite seated waist profile");level.push_back(p[2]-height-offset);for(unsigned k=0;k<3;k++){lo[k]=(std::min)(lo[k],p[k]);hi[k]=(std::max)(hi[k],p[k]);}}
 double scale=Length(Sub(hi,lo));if(scale<=1e-14)throw std::invalid_argument("Waist surface is degenerate");
 const double epsilon=scale*1e-9,weld=scale*1e-8;
 std::vector<Attachment> nodes;std::map<std::array<long long,2>,unsigned> lookup;
 std::set<std::pair<unsigned,unsigned>> edges;
 auto node=[&](Attachment a){auto p=frame.Local(a.position);std::array<long long,2> key{std::llround(p[0]/weld),std::llround(p[1]/weld)};auto found=lookup.find(key);if(found!=lookup.end())return found->second;unsigned id=unsigned(nodes.size());lookup.emplace(key,id);nodes.push_back(a);return id;};
 auto vertex=[&](unsigned i){Attachment a;a.position=surface[i].position;a.normal=Unit(surface[i].normal);a.vertices[0]=i;a.weights[0]=1;return a;};
 for(auto triangle:triangles){for(auto i:triangle)if(i>=surface.size())throw std::invalid_argument("Waist triangle outside supplied body");
  std::array<double,3> d{};unsigned coplanar=0;for(unsigned k=0;k<3;k++){d[k]=level[triangle[k]];if(std::abs(d[k])<=epsilon){d[k]=0;coplanar++;}}
  // A face lying in the plane contributes no crossing. Its boundary is supplied
  // by adjacent noncoplanar faces, avoiding internal edges of a flat body cap.
  if(coplanar==3)continue;std::vector<unsigned> cut;
  for(unsigned k=0;k<3;k++){unsigned j=(k+1)%3;if(d[k]==0)cut.push_back(node(vertex(triangle[k])));
   if(d[k]*d[j]<0)cut.push_back(node(Blend(vertex(triangle[k]),vertex(triangle[j]),d[k]/(d[k]-d[j]))));}
  std::sort(cut.begin(),cut.end());cut.erase(std::unique(cut.begin(),cut.end()),cut.end());
  if(cut.size()==2&&cut[0]!=cut[1])edges.emplace(cut[0],cut[1]);
 }
 if(edges.empty())throw std::invalid_argument("Waist plane does not intersect a closed body contour");
 std::vector<std::vector<unsigned>> adjacency(nodes.size());for(auto e:edges){adjacency[e.first].push_back(e.second);adjacency[e.second].push_back(e.first);}
 for(const auto& links:adjacency)if(!links.empty()&&links.size()!=2)throw std::invalid_argument("Waist intersection is open or branched");
 std::vector<bool> visited(nodes.size());std::vector<unsigned> chosen;double chosenArea=0;
 for(unsigned start=0;start<nodes.size();start++)if(!visited[start]&&!adjacency[start].empty()){
  std::vector<unsigned> loop;unsigned current=start,previous=unsigned(-1);
  do{if(visited[current])throw std::invalid_argument("Waist contour revisited before closure");visited[current]=true;loop.push_back(current);auto& links=adjacency[current];unsigned next=links[0]==previous?links[1]:links[0];previous=current;current=next;}while(current!=start);
  if(loop.size()<3)throw std::invalid_argument("Waist intersection has fewer than three points");
  double area=0;for(unsigned k=0;k<loop.size();k++){auto a=frame.Local(nodes[loop[k]].position),b=frame.Local(nodes[loop[(k+1)%loop.size()]].position);area+=a[0]*b[1]-b[0]*a[1];}area*=.5;
  // At a waist plane the torso is the largest cross section. Small disconnected
  // arm/accessory loops do not become waistband anchors.
  if(std::abs(area)>std::abs(chosenArea)){chosenArea=area;chosen=std::move(loop);}
 }
 if(chosen.empty()||std::abs(chosenArea)<scale*scale*1e-12)throw std::invalid_argument("Waist cross section has no area");
 if(chosenArea<0){std::reverse(chosen.begin(),chosen.end());chosenArea=-chosenArea;}
 // Stable seam at the forward ray, independent of triangle enumeration and
 // native UV aliases. Both band edges start on the same anatomical side.
 auto first=std::max_element(chosen.begin(),chosen.end(),[&](unsigned a,unsigned b){auto p=frame.Local(nodes[a].position),q=frame.Local(nodes[b].position);return p[1]!=q[1]?p[1]<q[1]:p[0]<q[0];});std::rotate(chosen.begin(),first,chosen.end());
 std::vector<double> arc(chosen.size()+1);for(unsigned k=0;k<chosen.size();k++)arc[k+1]=arc[k]+Length(Sub(nodes[chosen[k]].position,nodes[chosen[(k+1)%chosen.size()]].position));
 Contour out;out.circumference=arc.back();out.signedArea=chosenArea;out.height=height;out.points.reserve(samples);
 if(out.circumference<=epsilon)throw std::invalid_argument("Waist contour has zero circumference");
 if(radialCenter){
  // A single angular material coordinate aligns all stripe rows at the same
  // transverse rays. Independent arc-length phases can twist a strip when the
  // lower graft expands farther forward than its upper edge. Intersect the
  // ORIGINAL cut segments, preserving their exact four supplied-surface donors.
  if(!Finite(*radialCenter))throw std::invalid_argument("Nonfinite radial waist center");
  constexpr double pi=3.14159265358979323846;
  for(unsigned k=0;k<samples;k++){
   const double theta=-2*pi*k/samples;Point direction{std::sin(theta),std::cos(theta),0};
   double best=-1;Attachment picked;
   for(unsigned j=0;j<chosen.size();j++){
    const auto& a=nodes[chosen[j]];const auto& b=nodes[chosen[(j+1)%chosen.size()]];
    auto p=Sub(frame.Local(a.position),*radialCenter),edge=Sub(frame.Local(b.position),frame.Local(a.position));
    double divisor=direction[0]*edge[1]-direction[1]*edge[0];if(std::abs(divisor)<=scale*1e-14)continue;
    double t=(p[0]*direction[1]-p[1]*direction[0])/divisor;
    if(t<-1e-10||t>1+1e-10)continue;t=std::clamp(t,0.,1.);
    auto q=Add(p,Mul(edge,t));double distance=q[0]*direction[0]+q[1]*direction[1];
    if(distance>best){best=distance;picked=Blend(a,b,t);}
   }
   if(best<=epsilon)throw std::invalid_argument("Measured waist contour does not cover a material ray");
   out.points.push_back(picked);
  }
  return out;
 }
 for(unsigned k=0;k<samples;k++){double at=out.circumference*k/samples;unsigned high=unsigned(std::upper_bound(arc.begin(),arc.end(),at)-arc.begin());high=(std::min)(unsigned(chosen.size()),(std::max)(1u,high));unsigned low=high-1;double t=(at-arc[low])/(arc[high]-arc[low]);out.points.push_back(Blend(nodes[chosen[low]],nodes[chosen[high%chosen.size()]],t));}
 return out;
}
}
// The frame's local Z is the character's transverse-plane normal. Heights and
// body positions use adapter-observed units, not assumed metres or joint names.
// Clearance and elastic pressure belong to the consuming garment solver; this
// function returns exact body intersections without shrinking them through skin.
template<class Samples,class Triangles,class Frame>
BandContours Intersect(const Samples& surface,const Triangles& triangles,const Frame& frame,double bottomHeight,double topHeight,unsigned samples=64){
 frame.Validate();if(!std::isfinite(bottomHeight)||!std::isfinite(topHeight)||topHeight<=bottomHeight)throw std::invalid_argument("Waistband planes must be ordered and finite");
 return {detail::Slice(surface,triangles,frame,bottomHeight,samples),detail::Slice(surface,triangles,frame,topHeight,samples)};
}
}
