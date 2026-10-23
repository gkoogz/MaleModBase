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
 std::vector<UV> coordinates_;
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
 // Explicit charts let one cut span independently owned curved surfaces.
 ConvexSurfaceCut(std::vector<Vec> rest,std::vector<std::array<unsigned,3>> faces,
     std::vector<bool> eligible,std::vector<UV> coordinates,double tolerance)
 :ConvexSurfaceCut(std::move(rest),std::move(faces),std::move(eligible),Vec{},{{{1,0,0},{0,1,0}}},tolerance){
  if(coordinates.size()!=rest_.size())throw std::invalid_argument("Surface chart dimensions differ");
  for(auto uv:coordinates)for(auto x:uv)if(!std::isfinite(x))throw std::invalid_argument("Nonfinite surface chart");
  coordinates_=std::move(coordinates);
 }
 SurfaceCut CutSimple(std::vector<UV> polygon)const{
  if(polygon.size()<3||polygon.size()>4096)throw std::invalid_argument("Invalid opening polygon");
  double area=0;for(unsigned i=0;i<polygon.size();i++){
   for(double x:polygon[i])if(!std::isfinite(x))throw std::invalid_argument("Nonfinite opening polygon");
   auto d=Sub2(polygon[(i+1)%polygon.size()],polygon[i]);
   if(std::hypot(d[0],d[1])<=tolerance_)throw std::invalid_argument("Duplicate opening knot");
   area+=Cross2(polygon[i],polygon[(i+1)%polygon.size()]);
  }
  if(std::abs(area)<=tolerance_*tolerance_)throw std::invalid_argument("Degenerate opening polygon");
  if(area<0)std::reverse(polygon.begin(),polygon.end());
  std::vector<unsigned> active;for(unsigned i=0;i<polygon.size();i++)active.push_back(i);
  std::vector<std::vector<unsigned>> partition;
  while(active.size()>3){bool removed=false;
   for(unsigned i=0;i<active.size();i++){
    const unsigned a=active[(i+active.size()-1)%active.size()],b=active[i],c=active[(i+1)%active.size()];
    if(Cross2(Sub2(polygon[b],polygon[a]),Sub2(polygon[c],polygon[a]))<=tolerance_*tolerance_)continue;
    bool contains=false;
    for(auto j:active)if(j!=a&&j!=b&&j!=c){
     const auto p=polygon[j];
     if(Cross2(Sub2(polygon[b],polygon[a]),Sub2(p,polygon[a]))>=-tolerance_*tolerance_&&
        Cross2(Sub2(polygon[c],polygon[b]),Sub2(p,polygon[b]))>=-tolerance_*tolerance_&&
        Cross2(Sub2(polygon[a],polygon[c]),Sub2(p,polygon[c]))>=-tolerance_*tolerance_){contains=true;break;}
    }
    if(contains)continue;
    partition.push_back({a,b,c});active.erase(active.begin()+i);removed=true;break;
   }
   if(!removed)throw std::invalid_argument("Opening is not a triangulable simple polygon");
  }
  partition.push_back(active);
  // Merge adjacent ears before subtraction. Cutting hundreds of narrow ear
  // triangles separately creates unnecessary interior partitions and makes a
  // live control rebuild dramatically more expensive than the actual rim.
  bool merged=true;
  while(merged){merged=false;
   using Edge=std::pair<unsigned,unsigned>;
   std::map<Edge,std::pair<unsigned,unsigned>> owner;
   for(unsigned i=0;i<partition.size()&&!merged;i++)for(unsigned j=0;j<partition[i].size()&&!merged;j++){
    const auto a=partition[i][j],b=partition[i][(j+1)%partition[i].size()];
    auto other=owner.find({b,a});
    if(other!=owner.end()){
     unsigned oi=other->second.first,oj=other->second.second;
     const auto& p=partition[i];const auto& q=partition[oi];std::vector<unsigned> join;
     for(unsigned k=0;k<p.size();k++)join.push_back(p[(j+1+k)%p.size()]);
     for(unsigned k=0;k+2<q.size();k++)join.push_back(q[(oj+2+k)%q.size()]);
     bool convex=true;
     for(unsigned k=0;k<join.size();k++)if(Cross2(Sub2(polygon[join[(k+1)%join.size()]],polygon[join[k]]),Sub2(polygon[join[(k+2)%join.size()]],polygon[join[(k+1)%join.size()]])) < -tolerance_*tolerance_){convex=false;break;}
     if(convex){partition[oi]=std::move(join);partition.erase(partition.begin()+i);merged=true;break;}
    }
    owner[{a,b}]={i,j};
   }
  }
  std::vector<std::vector<UV>> regions;
  for(const auto& piece:partition){std::vector<UV> p;for(auto id:piece)p.push_back(polygon[id]);regions.push_back(std::move(p));}
  return CutRegions(regions);
 }
 SurfaceCut Cut(const std::vector<UV>& polygon)const{
  if(polygon.size()<3||polygon.size()>4096)throw std::invalid_argument("Invalid opening polygon");
  for(unsigned i=0;i<polygon.size();i++){
   for(double x:polygon[i])if(!std::isfinite(x))throw std::invalid_argument("Nonfinite opening polygon");
   const auto edge=Sub2(polygon[(i+1)%polygon.size()],polygon[i]);
   if(std::hypot(edge[0],edge[1])<=tolerance_)throw std::invalid_argument("Duplicate opening knot");
   for(auto p:polygon)if(Cross2(edge,Sub2(p,polygon[i])) < -tolerance_)throw std::invalid_argument("Opening must be convex and counterclockwise");
  }
  return CutRegions({polygon});
 }
 private:
 SurfaceCut CutRegions(const std::vector<std::vector<UV>>& regions)const{
  SurfaceCut result;result.points=rest_;
  // Shared original edges get one intersection ID. Interior corners of a
  // clipped triangle are shared by all pieces of that same triangle.
  using Key=std::array<std::int64_t,3>;std::map<Key,std::vector<unsigned>> inserted;
  auto cell=[&](Vec p){Key key;for(unsigned k=0;k<3;k++)key[k]=std::int64_t(std::floor(p[k]/tolerance_));return key;};
  // A compound support may carry hidden deformation samples. They are not
  // surface vertices and must never win a spatial weld against a visible edge.
  std::vector<bool> referenced(rest_.size());for(auto f:faces_)for(auto id:f)referenced[id]=true;
  for(unsigned i=0;i<rest_.size();i++)if(referenced[i])inserted[cell(rest_[i])].push_back(i);
  std::vector<std::pair<unsigned,Vec>> insertedOwners;
  constexpr double donorTolerance=1e-10;
  auto compatible=[&](const Vertex& v,unsigned parent,unsigned id){
   const auto f=faces_[parent];
   if(id<rest_.size()){for(unsigned j=0;j<3;j++)if(f[j]==id)return v.b[j]>1-donorTolerance;return false;}
   const auto& owner=insertedOwners.at(id-rest_.size());
   if(owner.first==parent){auto d=Sub(v.b,owner.second);return Dot(d,d)<=donorTolerance*donorTolerance;}
   const auto other=faces_[owner.first];unsigned common=0;
   for(unsigned k=0;k<3;k++){
    bool shared=std::find(other.begin(),other.end(),f[k])!=other.end();common+=shared;
    if(!shared&&std::abs(v.b[k])>donorTolerance)return false;
    if(std::find(f.begin(),f.end(),other[k])==f.end()&&std::abs(owner.second[k])>donorTolerance)return false;
    for(unsigned j=0;j<3;j++)if(f[k]==other[j]&&std::abs(v.b[k]-owner.second[j])>donorTolerance)return false;
   }return common>=2;
  };
  auto index=[&](const Vertex& v,unsigned parent){
   auto f=faces_[parent];for(unsigned k=0;k<3;k++)if(v.b[k]>1-1e-12)return f[k];
   const auto key=cell(v.p);
   // Adjacent faces calculate the same intersection through different
   // barycentric paths. Compare positions within the declared source-unit
   // tolerance instead of rounding a barycentric fraction into different bins.
   for(int x=-1;x<=1;x++)for(int y=-1;y<=1;y++)for(int z=-1;z<=1;z++){
    auto found=inserted.find({key[0]+x,key[1]+y,key[2]+z});if(found==inserted.end())continue;
    for(auto id:found->second){auto d=Sub(v.p,result.points[id]);if(Dot(d,d)<=tolerance_*tolerance_&&compatible(v,parent,id))return id;}
   }
  unsigned id=unsigned(result.points.size());result.points.push_back(v.p);insertedOwners.push_back({parent,v.b});inserted[key].push_back(id);return id;
  };
  struct Piece {unsigned parent;std::vector<Vertex> vertices;};std::vector<Piece> pieces;
  std::map<unsigned,std::vector<Vertex>> parentVertices;
  std::map<std::pair<unsigned,unsigned>,std::vector<double>> edgeVertices;
  auto hasArea=[&](const std::vector<Vertex>& poly){
   if(poly.size()<3)return false;
   // Eligibility/AABB overlap does not imply this triangle was cut. Preserve
   // an unchanged source face at every clipping stage, including the inside
   // path before a later plane establishes that it lies outside the opening.
   if(poly.size()==3&&poly[0].b==Vec{1,0,0}&&poly[1].b==Vec{0,1,0}&&poly[2].b==Vec{0,0,1})return true;
   // Coincident clipping planes can leave a line with distinct knots. Reject
   // those fragments only after separating-plane rejection below has protected
   // triangles that do not actually intersect the opening.
   unsigned end=1;double longest=0;
   for(unsigned k=1;k<poly.size();k++){auto d=Sub(poly[k].b,poly[0].b);double l=Dot(d,d);if(l>longest){longest=l;end=k;}}
   if(longest<=donorTolerance*donorTolerance)return false;
   const auto axis=Sub(poly[end].b,poly[0].b);double width2=0;
   for(const auto& v:poly){auto n=Cross(axis,Sub(v.b,poly[0].b));width2=(std::max)(width2,Dot(n,n)/longest);}
   if(width2<=1e-28)return false;
   return true;
  };
  auto emit=[&](const std::vector<Vertex>& poly,unsigned parent){
   // Reject line-like fragments produced by clipping, never an untouched
   // source triangle. Dropping a thin retained triangle punches an unrelated
   // second hole in the surface and makes opening discovery fail.
   const bool untouched=poly.size()==3&&poly[0].b==Vec{1,0,0}&&poly[1].b==Vec{0,1,0}&&poly[2].b==Vec{0,0,1};
   if(!untouched&&!hasArea(poly))return;
   pieces.push_back({parent,poly});auto f=faces_[parent];
   for(const auto& v:poly){
    // Original corners cannot introduce an interior split on their own edges.
    // Keep only new cut intersections in the conformance lookup.
    if(v.b==Vec{1,0,0}||v.b==Vec{0,1,0}||v.b==Vec{0,0,1})continue;
    parentVertices[parent].push_back(v);
    for(unsigned k=0;k<3;k++)if(std::abs(v.b[k])<donorTolerance){unsigned a=(k+1)%3,b=(k+2)%3;edgeVertices[std::minmax(f[a],f[b])].push_back(f[a]<f[b]?v.b[b]:v.b[a]);}
   }
  };
  UV low=regions[0][0],high=low;for(const auto& polygon:regions)for(auto p:polygon)for(unsigned k=0;k<2;k++){low[k]=(std::min)(low[k],p[k]);high[k]=(std::max)(high[k],p[k]);}
  for(unsigned parent=0;parent<faces_.size();parent++){
   std::vector<Vertex> remaining;for(unsigned k=0;k<3;k++){Vertex v;v.p=rest_[faces_[parent][k]];v.b[k]=1;v.uv=coordinates_.empty()?Project(v.p):coordinates_[faces_[parent][k]];remaining.push_back(v);}
   bool disjoint=false;for(unsigned k=0;k<2;k++)disjoint|=(remaining[0].uv[k]<low[k]&&remaining[1].uv[k]<low[k]&&remaining[2].uv[k]<low[k])||(remaining[0].uv[k]>high[k]&&remaining[1].uv[k]>high[k]&&remaining[2].uv[k]>high[k]);
   if(!eligible_[parent]||disjoint){emit(remaining,parent);continue;}
   std::vector<std::vector<Vertex>> fragments{remaining};
   for(const auto& polygon:regions){
    UV regionLow=polygon[0],regionHigh=polygon[0];for(auto p:polygon)for(unsigned k=0;k<2;k++){regionLow[k]=(std::min)(regionLow[k],p[k]);regionHigh[k]=(std::max)(regionHigh[k],p[k]);}
    std::vector<std::vector<Vertex>> next;
    for(auto fragment:fragments){
     bool apart=false;for(unsigned k=0;k<2;k++){bool below=true,above=true;for(auto v:fragment){below&=v.uv[k]<regionLow[k];above&=v.uv[k]>regionHigh[k];}apart|=below||above;}
     // AABB overlap is not polygon overlap. Reject a separating clipping plane
     // before any partition: otherwise an early plane subdivides an unrelated
     // thin face, and tolerance filtering deletes it before a later plane can
     // establish that the whole face belongs to the retained surface.
     if(!apart)for(unsigned j=0;j<polygon.size();j++){
      auto a=polygon[j],edge=Sub2(polygon[(j+1)%polygon.size()],a);bool outside=true;
      for(auto v:fragment)outside&=Cross2(edge,Sub2(v.uv,a)) < -tolerance_*tolerance_;
      if(outside){apart=true;break;}
     }
     if(apart){next.push_back(std::move(fragment));continue;}
     remaining=std::move(fragment);
     for(unsigned plane=0;plane<polygon.size()&&!remaining.empty();plane++){
    const auto start=polygon[plane],edge=Sub2(polygon[(plane+1)%polygon.size()],start);
    std::vector<Vertex> inside,outside;
    for(unsigned k=0;k<remaining.size();k++){
     const auto& a=remaining[k];const auto& b=remaining[(k+1)%remaining.size()];
     double da=Cross2(edge,Sub2(a.uv,start)),db=Cross2(edge,Sub2(b.uv,start));
     if(std::abs(da)<tolerance_*tolerance_)da=0;if(std::abs(db)<tolerance_*tolerance_)db=0;
     if(da>=0)inside.push_back(a);if(da<=0)outside.push_back(a);
     if((da<0&&db>0)||(da>0&&db<0)){
      auto v=Mix(a,b,da/(da-db));
      // Convex partitions share authored polygon corners. Evaluate those
      // corners once in the original triangle chart instead of accumulating
      // slightly different donors through many successive narrow clips.
      bool snapped=false;for(auto endpoint:{start,polygon[(plane+1)%polygon.size()]}){
       auto d=Sub2(v.uv,endpoint);if(d[0]*d[0]+d[1]*d[1]<=tolerance_*tolerance_){v.uv=endpoint;snapped=true;break;}
      }
      if(snapped){
       auto source=faces_[parent];std::array<UV,3> q;for(unsigned j=0;j<3;j++)q[j]=coordinates_.empty()?Project(rest_[source[j]]):coordinates_[source[j]];
       auto x=Sub2(q[1],q[0]),y=Sub2(q[2],q[0]),d=Sub2(v.uv,q[0]);double den=Cross2(x,y);
       if(std::abs(den)>1e-20){double u=Cross2(d,y)/den,w=Cross2(x,d)/den;v.b={1-u-w,u,w};v.p={};for(unsigned j=0;j<3;j++)v.p=Add(v.p,Scale(rest_[source[j]],v.b[j]));}
      }
      inside.push_back(v);outside.push_back(v);
     }
    }
      if(hasArea(outside))next.push_back(std::move(outside));if(hasArea(inside))remaining=std::move(inside);else remaining.clear();
     }
    }
    fragments=std::move(next);if(fragments.empty())break;
   }
   for(const auto& fragment:fragments)emit(fragment,parent);
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
    const auto& a=piece.vertices[k];const auto& b=piece.vertices[(k+1)%piece.vertices.size()];const auto edge=Sub(b.b,a.b);const double length2=Dot(edge,edge);if(length2<=donorTolerance*donorTolerance)continue;
    std::vector<double> knots{0};
    // Source barycentric coordinates remain well conditioned even when the
    // world triangle or its chart is extremely thin. Topology is not a
    // proximity operation on those flattened surfaces.
    auto add=[&](Vec p){auto q=Sub(p,a.b);const double t=Dot(q,edge)/length2;if(t<=donorTolerance||t>=1-donorTolerance)return;auto d=Sub(q,Scale(edge,t));if(Dot(d,d)<1e-26)knots.push_back(t);};
    bool originalEdge=false;
    for(unsigned j=0;j<3;j++)if(std::abs(a.b[j])<donorTolerance&&std::abs(b.b[j])<donorTolerance){originalEdge=true;unsigned x=(j+1)%3,y=(j+2)%3;auto found=edgeVertices.find(std::minmax(source[x],source[y]));if(found!=edgeVertices.end())for(double t:found->second){Vec p{};p[x]=source[x]<source[y]?1-t:t;p[y]=1-p[x];add(p);}}
    if(!originalEdge){auto found=parentVertices.find(parent);if(found!=parentVertices.end())for(const auto& v:found->second)add(v.b);}
    std::sort(knots.begin(),knots.end());knots.erase(std::unique(knots.begin(),knots.end(),[](double a,double b){return std::abs(a-b)<1e-10;}),knots.end());
    for(double t:knots)boundary.push_back(Mix(a,b,t));
   }
   // Resolve the same donor identities before triangulation. Resolving them
   // only afterwards lets repeated corners make overlapping fans when a
   // clipping sliver is smaller than the declared donor tolerance.
   std::set<unsigned> uniqueCorners;std::vector<Vertex> uniqueBoundary;
   for(const auto& v:boundary)if(uniqueCorners.insert(index(v,parent)).second)uniqueBoundary.push_back(v);
   boundary=std::move(uniqueBoundary);
   if(boundary.size()<3)continue;
   // Convex clipping pieces need no extra center vertex. A center fan makes
   // thin strips even thinner and can strand long near-zero-area ears. Choose
   // the triangulation with the largest minimum triangle area while preserving
   // every conformed boundary knot and its exact source-triangle donor.
   const unsigned count=unsigned(boundary.size());
   if(count>512)throw std::runtime_error("Clipped polygon exceeds triangulation bound");
   std::vector<double> quality(count*count,-1);std::vector<unsigned> split(count*count,~0u);
   for(unsigned i=0;i+1<count;i++)quality[i*count+i+1]=std::numeric_limits<double>::infinity();
   for(unsigned gap=2;gap<count;gap++)for(unsigned i=0;i+gap<count;i++){
    unsigned j=i+gap;
    for(unsigned k=i+1;k<j;k++){
     auto n=Cross(Sub(boundary[k].p,boundary[i].p),Sub(boundary[j].p,boundary[i].p));
     double score=(std::min)({Dot(n,n),quality[i*count+k],quality[k*count+j]});
     if(score>quality[i*count+j]){quality[i*count+j]=score;split[i*count+j]=k;}
    }
   }
   std::vector<std::pair<unsigned,unsigned>> pending{{0,count-1}};
   while(!pending.empty()){
    auto [i,j]=pending.back();pending.pop_back();if(j<=i+1)continue;
    unsigned k=split[i*count+j];if(k==~0u)throw std::runtime_error("Clipped polygon cannot be triangulated");
    std::array<Vertex,3> tri{boundary[i],boundary[k],boundary[j]};
    SurfaceCutFace face;face.parent=parent;for(unsigned d=0;d<3;d++)face.corners[d]={index(tri[d],parent),tri[d].b};
    if(face.corners[0].vertex!=face.corners[1].vertex&&face.corners[0].vertex!=face.corners[2].vertex&&face.corners[1].vertex!=face.corners[2].vertex)result.faces.push_back(face);
    pending.push_back({i,k});pending.push_back({k,j});
   }
  }
  // Distinguish the new hole from every pre-existing open resource boundary.
  using Edge=std::pair<unsigned,unsigned>;std::map<Edge,std::pair<unsigned,Edge>> edges;
  for(const auto& f:result.faces)for(unsigned k=0;k<3;k++){Edge directed{f.corners[k].vertex,f.corners[(k+1)%3].vertex},key=std::minmax(directed.first,directed.second);auto& e=edges[key];e.first++;e.second=directed;if(e.first>2){std::string detail="Cut generated nonmanifold edge "+std::to_string(key.first)+","+std::to_string(key.second)+" parents";for(const auto& face:result.faces){bool a=false,b=false;for(auto c:face.corners){a|=c.vertex==key.first;b|=c.vertex==key.second;}if(a&&b){detail+=" "+std::to_string(face.parent)+"[";for(auto c:face.corners){detail+=std::to_string(c.vertex)+"(";for(auto w:c.donor)detail+=std::to_string(w)+",";detail+=")";}detail+="]";}}auto p=result.points[key.first],q=result.points[key.second];detail+=" length="+std::to_string(std::sqrt(Dot(Sub(p,q),Sub(p,q))));throw std::runtime_error(detail);}}
  std::map<Edge,unsigned> original;
  for(auto f:faces_)for(unsigned k=0;k<3;k++)original[std::minmax(f[k],f[(k+1)%3])]++;
  std::vector<Edge> originalBoundary;for(auto e:original)if(e.second==1)originalBoundary.push_back(e.first);
  auto originalBoundaryChild=[&](Edge edge){
   for(auto old:originalBoundary){bool both=true;
    for(auto id:{edge.first,edge.second}){
     if(id<rest_.size()){both&=id==old.first||id==old.second;continue;}
     auto owner=insertedOwners.at(id-rest_.size());auto f=faces_[owner.first];
     bool a=false,b=false;for(unsigned k=0;k<3;k++){a|=f[k]==old.first;b|=f[k]==old.second;if(f[k]!=old.first&&f[k]!=old.second&&std::abs(owner.second[k])>donorTolerance)both=false;}
     both&=a&&b;
    }
    if(both)return true;
   }return false;
  };
  std::map<unsigned,unsigned> next;
  for(const auto& item:edges)if(item.second.first==1){if(originalBoundaryChild(item.first))continue;const auto e=item.second.second;if(!next.emplace(e.first,e.second).second){auto p=result.points[e.first];std::string owner=e.first>=rest_.size()?" parent="+std::to_string(insertedOwners[e.first-rest_.size()].first):" original";throw std::runtime_error("Opening branches at vertex "+std::to_string(e.first)+" at "+std::to_string(p[0])+","+std::to_string(p[1])+","+std::to_string(p[2])+" to "+std::to_string(e.second)+" and "+std::to_string(next[e.first])+owner);}}
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
