#pragma once
#include "presentation.hpp"
#include <map>
#include <set>
#include <vector>
#include <algorithm>
#include <cstdint>
#include <string>

namespace malemod::surface {
// Cut a convex opening in a measured surface without moving that surface.
// Every output corner retains its original triangle and barycentric donor.
// Resource boundaries and UV aliases are adapter data, never welded by UV.
struct SurfaceCutCorner {unsigned vertex=0;std::array<double,3> donor{};};
struct SurfaceCutFace {unsigned parent=0;std::array<SurfaceCutCorner,3> corners;};
struct SurfaceCut {
 std::vector<PresentationPoint> points;
 std::vector<SurfaceCutFace> faces;
 std::vector<unsigned> opening;
};
class ConvexSurfaceCut {
 using Vec=PresentationPoint;using UV=std::array<double,2>;
 struct Vertex {Vec p{},b{};UV uv{};};
 std::vector<Vec> rest_;std::vector<std::array<unsigned,3>> faces_;
 std::vector<bool> eligible_;Vec origin_;std::array<Vec,2> chart_;double tolerance_;
 static double Cross2(UV a,UV b){return a[0]*b[1]-a[1]*b[0];}
 static UV Sub2(UV a,UV b){return {a[0]-b[0],a[1]-b[1]};}
 UV Project(Vec p)const{p=Sub(p,origin_);return {Dot(p,chart_[0]),Dot(p,chart_[1])};}
 static Vertex Mix(const Vertex& a,const Vertex& b,double t){Vertex v;v.p=Add(Scale(a.p,1-t),Scale(b.p,t));v.b=Add(Scale(a.b,1-t),Scale(b.b,t));v.uv={a.uv[0]*(1-t)+b.uv[0]*t,a.uv[1]*(1-t)+b.uv[1]*t};return v;}
 public:
 ConvexSurfaceCut(std::vector<Vec> rest,std::vector<std::array<unsigned,3>> faces,std::vector<bool> eligible,Vec origin,std::array<Vec,2> chart,double tolerance)
 :rest_(std::move(rest)),faces_(std::move(faces)),eligible_(std::move(eligible)),origin_(origin),chart_(chart),tolerance_(tolerance){
  if(rest_.empty()||faces_.size()!=eligible_.size()||!std::isfinite(tolerance)||tolerance<=0)throw std::invalid_argument("Invalid surface cut domain");
  if(std::abs(Dot(chart[0],chart[0])-1)>1e-8||std::abs(Dot(chart[1],chart[1])-1)>1e-8||std::abs(Dot(chart[0],chart[1]))>1e-8)throw std::invalid_argument("Surface cut chart is not orthonormal");
  for(auto p:rest_)for(double x:p)if(!std::isfinite(x))throw std::invalid_argument("Nonfinite surface cut point");
  for(auto f:faces_)for(auto v:f)if(v>=rest_.size())throw std::invalid_argument("Surface cut face leaves domain");
 }
 SurfaceCut Cut(const std::vector<UV>& polygon)const{
  if(polygon.size()<3||polygon.size()>4096)throw std::invalid_argument("Invalid opening polygon");
  for(unsigned i=0;i<polygon.size();i++){
   for(double x:polygon[i])if(!std::isfinite(x))throw std::invalid_argument("Nonfinite opening polygon");
   const auto edge=Sub2(polygon[(i+1)%polygon.size()],polygon[i]);
   if(std::hypot(edge[0],edge[1])<=tolerance_)throw std::invalid_argument("Duplicate opening knot");
   for(auto p:polygon)if(Cross2(edge,Sub2(p,polygon[i])) < -tolerance_)throw std::invalid_argument("Opening must be convex and counterclockwise");
  }
  SurfaceCut result;result.points=rest_;
  // Shared original edges get one intersection ID. Interior corners of a
  // clipped triangle are shared by all pieces of that same triangle.
  using Key=std::array<std::int64_t,3>;std::map<Key,std::vector<unsigned>> inserted;
  auto cell=[&](Vec p){Key key;for(unsigned k=0;k<3;k++)key[k]=std::int64_t(std::floor(p[k]/tolerance_));return key;};
  for(unsigned i=0;i<rest_.size();i++)inserted[cell(rest_[i])].push_back(i);
  auto index=[&](const Vertex& v,unsigned parent){
   auto f=faces_[parent];for(unsigned k=0;k<3;k++)if(v.b[k]>1-1e-12)return f[k];
   const auto key=cell(v.p);
   // Adjacent faces calculate the same intersection through different
   // barycentric paths. Compare positions within the declared source-unit
   // tolerance instead of rounding a barycentric fraction into different bins.
   for(int x=-1;x<=1;x++)for(int y=-1;y<=1;y++)for(int z=-1;z<=1;z++){
    auto found=inserted.find({key[0]+x,key[1]+y,key[2]+z});if(found==inserted.end())continue;
    for(auto id:found->second){auto d=Sub(v.p,result.points[id]);if(Dot(d,d)<=tolerance_*tolerance_)return id;}
   }
   unsigned id=unsigned(result.points.size());result.points.push_back(v.p);inserted[key].push_back(id);return id;
  };
  struct Piece {unsigned parent;std::vector<Vertex> vertices;};std::vector<Piece> pieces;
  std::map<unsigned,std::vector<Vertex>> parentVertices;
  std::map<std::pair<unsigned,unsigned>,std::vector<Vec>> edgeVertices;
  auto emit=[&](const std::vector<Vertex>& poly,unsigned parent){
   if(poly.size()<3)return;
   Vec area{};for(unsigned k=1;k+1<poly.size();k++)area=Add(area,Cross(Sub(poly[k].p,poly[0].p),Sub(poly[k+1].p,poly[0].p)));
   if(Dot(area,area)<=tolerance_*tolerance_*tolerance_*tolerance_)return;
   pieces.push_back({parent,poly});auto f=faces_[parent];
   for(const auto& v:poly){
    // Original corners cannot introduce an interior split on their own edges.
    // Keep only new cut intersections in the conformance lookup.
    if(v.b==Vec{1,0,0}||v.b==Vec{0,1,0}||v.b==Vec{0,0,1})continue;
    parentVertices[parent].push_back(v);
    for(unsigned k=0;k<3;k++)if(std::abs(v.b[k])<1e-10)edgeVertices[std::minmax(f[(k+1)%3],f[(k+2)%3])].push_back(v.p);
   }
  };
  UV low=polygon[0],high=polygon[0];for(auto p:polygon)for(unsigned k=0;k<2;k++){low[k]=(std::min)(low[k],p[k]);high[k]=(std::max)(high[k],p[k]);}
  for(unsigned parent=0;parent<faces_.size();parent++){
   std::vector<Vertex> remaining;for(unsigned k=0;k<3;k++){Vertex v;v.p=rest_[faces_[parent][k]];v.b[k]=1;v.uv=Project(v.p);remaining.push_back(v);}
   bool disjoint=false;for(unsigned k=0;k<2;k++)disjoint|=(remaining[0].uv[k]<low[k]&&remaining[1].uv[k]<low[k]&&remaining[2].uv[k]<low[k])||(remaining[0].uv[k]>high[k]&&remaining[1].uv[k]>high[k]&&remaining[2].uv[k]>high[k]);
   if(!eligible_[parent]||disjoint){emit(remaining,parent);continue;}
   for(unsigned plane=0;plane<polygon.size()&&!remaining.empty();plane++){
    const auto start=polygon[plane],edge=Sub2(polygon[(plane+1)%polygon.size()],start);
    std::vector<Vertex> inside,outside;
    for(unsigned k=0;k<remaining.size();k++){
     const auto& a=remaining[k];const auto& b=remaining[(k+1)%remaining.size()];
     double da=Cross2(edge,Sub2(a.uv,start)),db=Cross2(edge,Sub2(b.uv,start));
     if(std::abs(da)<tolerance_*tolerance_)da=0;if(std::abs(db)<tolerance_*tolerance_)db=0;
     if(da>=0)inside.push_back(a);if(da<=0)outside.push_back(a);
     if((da<0&&db>0)||(da>0&&db<0)){auto v=Mix(a,b,da/(da-db));inside.push_back(v);outside.push_back(v);}
    }
    emit(outside,parent);remaining=std::move(inside);
   }
  }
  // Difference clipping emits several convex pieces per source face. Later
  // planes can subdivide their common edges; conform all pieces AND their
  // original neighbors before triangulation, otherwise those are T-junctions.
  for(const auto& piece:pieces){
   const unsigned parent=piece.parent;const auto source=faces_[parent];std::vector<Vertex> boundary;
   if(piece.vertices.size()==3&&piece.vertices[0].b==Vec{1,0,0}&&piece.vertices[1].b==Vec{0,1,0}&&piece.vertices[2].b==Vec{0,0,1}){
    bool split=false;for(unsigned k=0;k<3;k++)split|=edgeVertices.count(std::minmax(source[k],source[(k+1)%3]))!=0;
    if(!split){SurfaceCutFace face;face.parent=parent;for(unsigned k=0;k<3;k++)face.corners[k]={source[k],piece.vertices[k].b};result.faces.push_back(face);continue;}
   }
   for(unsigned k=0;k<piece.vertices.size();k++){
    const auto& a=piece.vertices[k];const auto& b=piece.vertices[(k+1)%piece.vertices.size()];const auto edge=Sub(b.p,a.p);const double length2=Dot(edge,edge);if(length2<=tolerance_*tolerance_)continue;
    std::vector<double> knots{0};
    auto add=[&](Vec p){const double t=Dot(Sub(p,a.p),edge)/length2;if(t<=1e-10||t>=1-1e-10)return;const auto delta=Sub(p,Add(a.p,Scale(edge,t)));if(Dot(delta,delta)<tolerance_*tolerance_)knots.push_back(t);};
    bool originalEdge=false;
    for(unsigned j=0;j<3;j++)if(std::abs(a.b[j])<1e-10&&std::abs(b.b[j])<1e-10){originalEdge=true;auto found=edgeVertices.find(std::minmax(source[(j+1)%3],source[(j+2)%3]));if(found!=edgeVertices.end())for(auto p:found->second)add(p);}
    if(!originalEdge){auto found=parentVertices.find(parent);if(found!=parentVertices.end())for(const auto& v:found->second)add(v.p);}
    std::sort(knots.begin(),knots.end());knots.erase(std::unique(knots.begin(),knots.end(),[](double a,double b){return std::abs(a-b)<1e-10;}),knots.end());
    for(double t:knots)boundary.push_back(Mix(a,b,t));
   }
   if(boundary.size()<3)continue;
   Vertex center{};for(const auto& v:boundary){center.p=Add(center.p,v.p);center.b=Add(center.b,v.b);}center.p=Scale(center.p,1./boundary.size());center.b=Scale(center.b,1./boundary.size());center.uv=Project(center.p);
   // Preserve untouched original triangles byte for byte. Otherwise an
   // interior fan preserves every collinear edge donor without zero-area ears.
   const unsigned count=boundary.size()==3?1:unsigned(boundary.size());
   for(unsigned k=0;k<count;k++){
    std::array<Vertex,3> tri=boundary.size()==3?std::array<Vertex,3>{boundary[0],boundary[1],boundary[2]}:std::array<Vertex,3>{center,boundary[k],boundary[(k+1)%boundary.size()]};
    SurfaceCutFace face;face.parent=parent;for(unsigned j=0;j<3;j++)face.corners[j]={index(tri[j],parent),tri[j].b};
    if(face.corners[0].vertex==face.corners[1].vertex||face.corners[0].vertex==face.corners[2].vertex||face.corners[1].vertex==face.corners[2].vertex)continue;
    result.faces.push_back(face);
   }
  }
  // Distinguish the new hole from every pre-existing open resource boundary.
  using Edge=std::pair<unsigned,unsigned>;std::map<Edge,std::pair<unsigned,Edge>> edges;
  for(const auto& f:result.faces)for(unsigned k=0;k<3;k++){Edge directed{f.corners[k].vertex,f.corners[(k+1)%3].vertex},key=std::minmax(directed.first,directed.second);auto& e=edges[key];e.first++;e.second=directed;if(e.first>2)throw std::runtime_error("Cut generated nonmanifold edge");}
  std::map<Edge,unsigned> original;
  for(auto f:faces_)for(unsigned k=0;k<3;k++)original[std::minmax(f[k],f[(k+1)%3])]++;
  std::vector<Edge> originalBoundary;for(auto e:original)if(e.second==1)originalBoundary.push_back(e.first);
  auto originalBoundaryChild=[&](Edge edge){
   for(auto old:originalBoundary){auto a=rest_[old.first],delta=Sub(rest_[old.second],a);const double length2=Dot(delta,delta);bool both=true;
    for(auto id:{edge.first,edge.second}){const auto q=Sub(result.points[id],a);const double t=Dot(q,delta)/length2;auto residual=Sub(q,Scale(delta,t));both&=t>=-1e-10&&t<=1+1e-10&&Dot(residual,residual)<=tolerance_*tolerance_;}
    if(both)return true;
   }return false;
  };
  std::map<unsigned,unsigned> next;
  for(const auto& item:edges)if(item.second.first==1){if(originalBoundaryChild(item.first))continue;const auto e=item.second.second;if(!next.emplace(e.first,e.second).second)throw std::runtime_error("Opening branches at a vertex");}
  if(next.empty())throw std::runtime_error("Opening did not intersect the eligible surface");
  unsigned start=next.begin()->first,current=start;
  do{result.opening.push_back(current);auto found=next.find(current);if(found==next.end())throw std::runtime_error("Opening meets an existing boundary");current=found->second;if(result.opening.size()>next.size())throw std::runtime_error("Opening does not close");}while(current!=start);
  if(result.opening.size()!=next.size()){
   std::set<unsigned> seen(result.opening.begin(),result.opening.end());std::string detail;
   for(auto e:next)if(!seen.count(e.first)){auto p=result.points[e.first];detail+=" ["+std::to_string(e.first)+":"+std::to_string(p[0])+","+std::to_string(p[1])+","+std::to_string(p[2])+"]";if(detail.size()>600)break;}
   auto p=result.points[result.opening.front()];throw std::runtime_error("Opening has multiple boundary components: primary="+std::to_string(result.opening.size())+" start="+std::to_string(result.opening.front())+":"+std::to_string(p[0])+","+std::to_string(p[1])+","+std::to_string(p[2])+" total="+std::to_string(next.size())+detail);
  }
  return result;
 }
};
}
