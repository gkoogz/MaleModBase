#pragma once
#include "triangle_ray_index.hpp"
#include <malemod/task_arena.hpp>
namespace malemod::surface {
// The cut support remains the material envelope of the shaft. A connector
// between a suspended rim and an oblique upper rim must follow that envelope,
// rather than taking a chord through the volume removed by the topological cut.
inline unsigned ConformCompoundExterior(std::vector<PresentationPoint>& points,
 const std::vector<std::vector<unsigned>>& rings,const std::vector<PresentationPoint>& support,
 const std::vector<std::array<unsigned,3>>& faces,const std::vector<bool>& eligible,
 const std::vector<PresentationPoint>& guide,TaskArena* arena=nullptr){
 using Vec=PresentationPoint;
 if(guide.size()<2||rings.size()<3)throw std::invalid_argument("Compound exterior lacks a material guide");
 TriangleRayIndex index(support,faces,eligible);
 auto project=[&](Vec p){
  Vec center{};double best=std::numeric_limits<double>::infinity();
  for(unsigned j=0;j+1<guide.size();j++){
   auto edge=Sub(guide[j+1],guide[j]);const double square=Dot(edge,edge);
   if(square<=1e-20)throw std::invalid_argument("Compound exterior guide collapsed");
   // The first section continues into the pelvic support. All later sections
   // use finite closest segments, including a bent full-floppy centerline.
   double t=Dot(Sub(p,guide[j]),edge)/square;t=(std::min)(1.,j?std::clamp(t,0.,1.):t);
   auto q=Add(guide[j],Scale(edge,t)),d=Sub(p,q);double distance=Dot(d,d);
   if(distance<best){best=distance;center=q;}
  }
  auto direction=Sub(p,center);double distance=std::sqrt(Dot(direction,direction));if(distance<1e-10)return p;
  direction=Scale(direction,1/distance);Vec hit{};double facing=0;
  if(index.Ray(support,faces,center,direction,hit,direction,&facing)&&facing>0){
   const double outer=Dot(Sub(hit,center),direction);
   if(outer>distance+1e-8)return hit;
  }
  return p;
 };
 const unsigned rows=unsigned(rings.size());
 std::vector<unsigned> changes(rings.front().size());
 auto column=[&](unsigned col){
  std::vector<Vec> path(rows);std::vector<double> originalArc(rows),contactArc(rows);bool active=false;
  for(unsigned row=0;row<rows;row++){
   auto id=rings[row][col];path[row]=row&&row+1<rows?project(points[id]):points[id];
   if(path[row]!=points[id]){active=true;changes[col]++;}
   if(row){auto a=Sub(points[id],points[rings[row-1][col]]),b=Sub(path[row],path[row-1]);originalArc[row]=originalArc[row-1]+std::sqrt(Dot(a,a));contactArc[row]=contactArc[row-1]+std::sqrt(Dot(b,b));}
  }
  if(!active)return;
  if(originalArc.back()<1e-12||contactArc.back()<1e-12)throw std::runtime_error("Compound exterior meridian collapsed");
  // Contact can place several radial samples on the same surface point.
  // Redistribute along the resolved path using the original material spacing;
  // otherwise a correct envelope would still contain collapsed strip faces.
  for(unsigned row=1;row+1<rows;row++){
   double at=originalArc[row]/originalArc.back()*contactArc.back();
   unsigned end=unsigned(std::upper_bound(contactArc.begin(),contactArc.end(),at)-contactArc.begin());end=(std::min)(end,rows-1);
   unsigned begin=end-1;double length=contactArc[end]-contactArc[begin];
   if(length<=1e-14)throw std::runtime_error("Compound exterior contact interval collapsed");
   double t=(at-contactArc[begin])/length;
   points[rings[row][col]]=project(Add(Scale(path[begin],1-t),Scale(path[end],t)));
  }
 };
 if(arena)arena->For(unsigned(changes.size()),column);else for(unsigned col=0;col<changes.size();col++)column(col);
 return std::accumulate(changes.begin(),changes.end(),0u);
}
}
