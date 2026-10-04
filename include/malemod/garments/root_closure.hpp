#pragma once
// Classification closure only. Returned triangles never become rendered
// geometry or physical contact faces; they use existing measured indices only.
#include <array>
#include <cstdint>
#include <vector>
#include <unordered_map>
namespace malemod::garments {
struct Sample;
inline std::vector<std::array<std::uint32_t,3>> RootCap(const std::vector<Sample>&,const std::vector<std::array<std::uint32_t,3>>&,const std::vector<Sample>&,double);
inline std::vector<std::array<std::uint32_t,3>> SurfaceCaps(const std::vector<Sample>&,const std::vector<std::array<std::uint32_t,3>>&,double);
inline std::vector<std::array<std::uint32_t,3>> RefineClassificationBoundary(const std::vector<Sample>&,const std::vector<std::array<std::uint32_t,3>>&,const std::vector<std::uint32_t>&,double);
}
#include "jockstrap.hpp"
namespace malemod::garments {
namespace closure_detail {
struct Boundaries {Point origin{};std::vector<Point> points;std::vector<std::vector<unsigned>> loops;};
inline Boundaries Extract(const std::vector<Sample>& surface,const std::vector<std::array<std::uint32_t,3>>& triangles,double scale){
 if(!std::isfinite(scale)||scale<=0||surface.size()>131072||triangles.size()>262144)throw std::invalid_argument("Invalid measured closure budget");
 if(surface.empty()!=triangles.empty())throw std::invalid_argument("Incomplete measured surface closure");
 if(surface.empty())return {};
    const Point origin=surface.front().position;
    std::vector<Point> points;points.reserve(surface.size());
    for(const auto& sample:surface){auto p=Mul(Sub(sample.position,origin),1/scale);if(!Finite(p))throw std::invalid_argument("Nonfinite measured root");points.push_back(p);}
    // Weld only positional aliases. Keep the first original index as the
    // representative; no synthetic cap vertex or guessed character frame.
    constexpr double epsilon=1e-9;
    using Cell=std::array<long long,3>;
    struct CellHash {std::size_t operator()(const Cell& key)const{std::uint64_t value=1469598103934665603ull;for(auto k:key){value^=std::uint64_t(k);value*=1099511628211ull;}return std::size_t(value);}};
    std::unordered_map<Cell,std::vector<unsigned>,CellHash> cells;cells.reserve(points.size()*2);
    std::vector<unsigned> aliases(points.size());
    auto cell=[&](Point p){Cell key{};for(unsigned a=0;a<3;a++){double v=std::floor(p[a]/epsilon);if(std::abs(v)>9e18)throw std::invalid_argument("Measured root coordinate exceeds weld budget");key[a]=static_cast<long long>(v);}return key;};
    for(unsigned i=0;i<points.size();i++){
        auto key=cell(points[i]);unsigned representative=i;
        for(int x=-1;x<=1;x++)for(int y=-1;y<=1;y++)for(int z=-1;z<=1;z++){
            auto found=cells.find({key[0]+x,key[1]+y,key[2]+z});if(found==cells.end())continue;
            for(auto id:found->second)if(Length(Sub(points[i],points[id]))<=epsilon)representative=(std::min)(representative,id);
        }
        aliases[i]=representative;if(representative==i)cells[key].push_back(i);
    }
    struct Edge {unsigned count=0,from=0,to=0;};
    std::unordered_map<std::uint64_t,Edge> edges;edges.reserve(triangles.size()*2);
    for(auto face:triangles){
        for(auto& id:face){if(id>=aliases.size())throw std::invalid_argument("Measured root triangle outside surface");id=aliases[id];}
        if(face[0]==face[1]||face[1]==face[2]||face[2]==face[0])continue;
        for(unsigned j=0;j<3;j++){
            auto a=face[j],b=face[(j+1)%3];auto key=(std::uint64_t((std::min)(a,b))<<32)|(std::max)(a,b);auto& e=edges[key];
            if(!e.count){e.from=a;e.to=b;}else if(e.count==1&&(e.from!=b||e.to!=a))throw std::invalid_argument("Measured surface has inconsistent edge winding");
            if(++e.count>2)throw std::invalid_argument("Measured surface is nonmanifold");
        }
    }
    std::map<unsigned,unsigned> next;std::set<unsigned> incoming;
    for(auto row:edges)if(row.second.count==1){auto e=row.second;if(!next.emplace(e.from,e.to).second||!incoming.insert(e.to).second)throw std::invalid_argument("Measured root branches");}

 if(next.size()>1024)throw std::invalid_argument("Measured surface boundary exceeds closure budget");
 Boundaries out;out.origin=origin;out.points=std::move(points);std::set<unsigned> visited;
 for(auto edge:next){unsigned start=edge.first;if(visited.count(start))continue;std::vector<unsigned> loop;unsigned id=start;
  do{if(!visited.insert(id).second)throw std::invalid_argument("Measured surface boundary does not close simply");loop.push_back(id);auto found=next.find(id);if(found==next.end())throw std::invalid_argument("Measured surface boundary is open");id=found->second;}while(id!=start);
  if(loop.size()<3||out.loops.size()>=64)throw std::invalid_argument("Measured surface closure loop outside budget");out.loops.push_back(std::move(loop));
 }
 return out;
}
inline std::vector<std::array<std::uint32_t,3>> EarClip(const std::vector<Point>& points,std::vector<unsigned> loop){
    // Reverse existing unmatched edge winding, then ear-clip a simple
    // projected polygon. A centroid fan is invalid for concave measured roots.
    std::reverse(loop.begin(),loop.end());std::rotate(loop.begin(),std::min_element(loop.begin(),loop.end()),loop.end());
    Point center{};for(auto i:loop)center=Add(center,points[i]);center=Mul(center,1./loop.size());
    Point area{};for(unsigned k=0;k<loop.size();k++)area=Add(area,Cross(Sub(points[loop[k]],center),Sub(points[loop[(k+1)%loop.size()]],center)));
    if(Length(area)<1e-14)throw std::invalid_argument("Measured root projection is degenerate");auto normal=Unit(area);
    unsigned axis=0;for(unsigned k=1;k<3;k++)if(std::abs(normal[k])<std::abs(normal[axis]))axis=k;Point direction{};direction[axis]=1;auto u=Unit(Cross(normal,direction)),v=Cross(normal,u);
    using XY=std::array<double,2>;std::vector<XY> xy(points.size());for(auto i:loop){auto p=Sub(points[i],center);xy[i]={Dot(p,u),Dot(p,v)};}
    auto turn=[&](unsigned a,unsigned b,unsigned c){return (xy[b][0]-xy[a][0])*(xy[c][1]-xy[a][1])-(xy[b][1]-xy[a][1])*(xy[c][0]-xy[a][0]);};
    constexpr double tolerance=1e-14;
    auto on=[&](unsigned a,unsigned b,unsigned p){return std::abs(turn(a,b,p))<=tolerance&&xy[p][0]>=(std::min)(xy[a][0],xy[b][0])-tolerance&&xy[p][0]<=(std::max)(xy[a][0],xy[b][0])+tolerance&&xy[p][1]>=(std::min)(xy[a][1],xy[b][1])-tolerance&&xy[p][1]<=(std::max)(xy[a][1],xy[b][1])+tolerance;};
    for(unsigned i=0;i<loop.size();i++)for(unsigned j=i+2;j<loop.size();j++){
        if(i==0&&j+1==loop.size())continue;auto a=loop[i],b=loop[(i+1)%loop.size()],c=loop[j],d=loop[(j+1)%loop.size()];double x=turn(a,b,c),y=turn(a,b,d),z=turn(c,d,a),w=turn(c,d,b);
        if((x*y<0&&z*w<0)||on(a,b,c)||on(a,b,d)||on(c,d,a)||on(c,d,b))throw std::invalid_argument("Measured root projection self intersects");
    }
    std::vector<std::array<std::uint32_t,3>> cap;cap.reserve(loop.size()-2);
    while(loop.size()>3){bool clipped=false;for(unsigned k=0;k<loop.size();k++){
        auto a=loop[(k+loop.size()-1)%loop.size()],b=loop[k],c=loop[(k+1)%loop.size()];if(turn(a,b,c)<=tolerance)continue;
        bool contains=false;for(auto p:loop)if(p!=a&&p!=b&&p!=c&&turn(a,b,p)>=-tolerance&&turn(b,c,p)>=-tolerance&&turn(c,a,p)>=-tolerance){contains=true;break;}
        if(contains)continue;cap.push_back({a,b,c});loop.erase(loop.begin()+k);clipped=true;break;
    }if(!clipped)throw std::invalid_argument("Measured root could not be triangulated");}
    if(turn(loop[0],loop[1],loop[2])<=tolerance)throw std::invalid_argument("Measured root closure has degenerate final face");cap.push_back({loop[0],loop[1],loop[2]});return cap;
}
}
// Refine a measured coarse body opening to the existing anatomical root
// indices before inserting its reversed cap. This changes classification
// triangles only. Float interpolation residuals are bounded in measured
// circumference units; no new point or guessed body frame is introduced.
inline std::vector<std::array<std::uint32_t,3>> RefineClassificationBoundary(const std::vector<Sample>& surface,const std::vector<std::array<std::uint32_t,3>>& faces,const std::vector<std::uint32_t>& root,double scale){
 if(root.size()<3||root.size()>1024)throw std::invalid_argument("Measured classification root outside refinement budget");
 auto boundary=closure_detail::Extract(surface,faces,scale);constexpr double tolerance=1e-7;
 std::set<unsigned> unique;for(auto id:root)if(id>=surface.size()||!unique.insert(id).second)throw std::invalid_argument("Invalid classification root index");
 const auto& points=boundary.points;
 for(unsigned i=0;i<root.size();i++)for(unsigned j=0;j<i;j++)if(Length(Sub(points[root[i]],points[root[j]]))<=1e-9)throw std::invalid_argument("Classification root repeats a positional alias");
 const std::vector<unsigned>* selected=nullptr;
 for(const auto& loop:boundary.loops){bool matched=true;for(auto endpoint:loop){double best=1e100;for(auto id:root)best=(std::min)(best,Length(Sub(points[endpoint],points[id])));if(best>tolerance){matched=false;break;}}
  if(matched){if(selected)throw std::invalid_argument("Classification root matches multiple body boundaries");selected=&loop;}}
 if(!selected)throw std::invalid_argument("Classification root does not match a complete body opening");
 const auto& loop=*selected;std::vector<unsigned> endpoints;std::set<unsigned> endpointIds;
 for(auto endpoint:loop){double best=1e100;unsigned match=0;for(auto id:root){double d=Length(Sub(points[endpoint],points[id]));if(d<best){best=d;match=id;}}
  if(!endpointIds.insert(match).second)throw std::invalid_argument("Classification root merges distinct body endpoints");endpoints.push_back(match);}
 std::vector<std::vector<std::pair<double,unsigned>>> interior(loop.size());
 for(auto id:root){if(endpointIds.count(id))continue;double best=1e100,bestT=0;unsigned at=0;
  for(unsigned edge=0;edge<loop.size();edge++){auto a=points[loop[edge]],b=points[loop[(edge+1)%loop.size()]],direction=Sub(b,a);double square=Dot(direction,direction);if(square<=1e-18)throw std::invalid_argument("Degenerate classification body edge");double t=std::clamp(Dot(Sub(points[id],a),direction)/square,0.,1.);double d=Length(Sub(points[id],Add(a,Mul(direction,t))));if(d<best){best=d;bestT=t;at=edge;}}
  if(best>tolerance||bestT<=1e-9||bestT>=1-1e-9)throw std::invalid_argument("Classification root vertex is not on its measured body edge");
  interior[at].push_back({bestT,id});}
 std::map<unsigned,unsigned> remap;
 // Include original UV/shading aliases at the coarse endpoints. They must
 // all use the same existing fine-root index in the classification copy.
 for(unsigned id=0;id<points.size();id++)for(unsigned k=0;k<loop.size();k++)if(Length(Sub(points[id],points[loop[k]]))<=1e-9){remap[id]=endpoints[k];break;}
 std::map<std::pair<unsigned,unsigned>,std::vector<unsigned>> splits;
 for(unsigned edge=0;edge<loop.size();edge++){auto& list=interior[edge];std::sort(list.begin(),list.end());for(unsigned k=1;k<list.size();k++)if(list[k].first-list[k-1].first<=1e-9)throw std::invalid_argument("Classification root has duplicate edge parameters");
  auto a=endpoints[edge],b=endpoints[(edge+1)%loop.size()];std::vector<unsigned> ids;for(auto value:list)ids.push_back(value.second);splits[{a,b}]=ids;std::reverse(ids.begin(),ids.end());splits[{b,a}]=ids;}
 std::vector<std::array<std::uint32_t,3>> result;result.reserve(faces.size()+2*root.size());
 for(auto face:faces){for(auto& id:face){auto found=remap.find(id);if(found!=remap.end())id=found->second;}
  std::vector<unsigned> polygon;bool changed=false;for(unsigned k=0;k<3;k++){auto a=face[k],b=face[(k+1)%3];polygon.push_back(a);auto found=splits.find({a,b});if(found!=splits.end()&&!found->second.empty()){polygon.insert(polygon.end(),found->second.begin(),found->second.end());changed=true;}}
  if(!changed){result.push_back(face);continue;}
  // EarClip reverses its boundary argument to form a cap. Reverse this
  // triangle polygon first, preserving the body's original face winding.
  std::reverse(polygon.begin(),polygon.end());auto refined=closure_detail::EarClip(points,std::move(polygon));result.insert(result.end(),refined.begin(),refined.end());}
 auto check=closure_detail::Extract(surface,result,scale);bool complete=false;
 for(const auto& refined:check.loops){if(refined.size()!=root.size())continue;bool matched=true;for(auto id:root){double best=1e100;for(auto candidate:refined)best=(std::min)(best,Length(Sub(points[id],points[candidate])));if(best>1e-9){matched=false;break;}}if(matched)complete=true;}
 if(!complete)throw std::invalid_argument("Classification boundary refinement did not preserve the complete fine root");
 return result;
}
inline std::vector<std::array<std::uint32_t,3>> SurfaceCaps(const std::vector<Sample>& surface,const std::vector<std::array<std::uint32_t,3>>& triangles,double scale){
 auto boundary=closure_detail::Extract(surface,triangles,scale);std::vector<std::array<std::uint32_t,3>> result;
 for(const auto& loop:boundary.loops){auto cap=closure_detail::EarClip(boundary.points,loop);result.insert(result.end(),cap.begin(),cap.end());}
 return result;
}
inline std::vector<std::array<std::uint32_t,3>> RootCap(const std::vector<Sample>& anatomy,const std::vector<std::array<std::uint32_t,3>>& triangles,const std::vector<Sample>& opening,double scale){
 if(opening.size()>1024)throw std::invalid_argument("Invalid measured root opening budget");
 auto boundary=closure_detail::Extract(anatomy,triangles,scale);if(boundary.loops.empty())return {};
 if(boundary.loops.size()!=1||opening.empty())throw std::invalid_argument("Measured anatomy has multiple or unmatched open loops");
 const auto& loop=boundary.loops.front();const auto& points=boundary.points;const auto origin=boundary.origin;
 std::vector<bool> matched(loop.size());
 for(const auto& sample:opening){auto p=Mul(Sub(sample.position,origin),1/scale);if(!Finite(p))throw std::invalid_argument("Nonfinite measured opening");double best=1e100;unsigned at=0;for(unsigned k=0;k<loop.size();k++){double d=Length(Sub(p,points[loop[k]]));if(d<best){best=d;at=k;}}if(best>1e-6)throw std::invalid_argument("Measured opening differs from anatomy boundary");matched[at]=true;}
 for(bool value:matched)if(!value)throw std::invalid_argument("Measured opening omits an anatomy boundary vertex");
 return closure_detail::EarClip(points,loop);
}
}
