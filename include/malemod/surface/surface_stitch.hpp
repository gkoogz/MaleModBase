#pragma once
#include "convex_surface_cut.hpp"
#include "modular_motion.hpp"
#include <limits>

namespace malemod::surface {
// Impose the retained surface's shading direction with a zero-slope boundary
// condition. Adding only the endpoint error preserves an interior normal spike
// caused by needle triangles, even when the boundary vertex itself is exact.
inline PresentationPoint BlendEndpointNormal(PresentationPoint interior,PresentationPoint endpoint,double distance,double span=.25){
 if(!std::isfinite(distance)||!std::isfinite(span)||span<=0)throw std::invalid_argument("Invalid normal transition");
 interior=Unit(interior);endpoint=Unit(endpoint);
 if(distance<=0)return endpoint;if(distance>=span)return interior;
 const double x=distance/span,w=1-x*x*x*(10+x*(-15+6*x));
 const double c=std::clamp(Dot(interior,endpoint),-1.,1.);
 if(c>.999999)return Unit(Add(Scale(interior,1-w),Scale(endpoint,w)));
 auto side=Sub(endpoint,Scale(interior,c));
 if(Dot(side,side)<1e-20){PresentationPoint axis{1,0,0};if(std::abs(interior[0])>.9)axis={0,1,0};side=Cross(interior,axis);}
 side=Unit(side);const double angle=std::acos(c)*w;
 return Add(Scale(interior,std::cos(angle)),Scale(side,std::sin(angle)));
}
struct SurfaceLoopChart {
 std::vector<unsigned> loop;std::vector<double> angles;
 std::vector<std::array<double,2>> coordinates;
};
inline SurfaceLoopChart ChartLoop(const SurfaceCut& mesh,PresentationPoint origin,std::array<PresentationPoint,2> axes,const std::vector<PresentationPoint>* reference=nullptr){
 constexpr double pi=3.14159265358979323846;SurfaceLoopChart chart;const auto& p=reference?*reference:mesh.points;
 if(p.size()>mesh.points.size())throw std::invalid_argument("Invalid loop reference positions");
 std::vector<std::pair<double,unsigned>> order;
 for(unsigned id:mesh.opening){if(id>=p.size())throw std::invalid_argument("Loop leaves reference positions");auto d=Sub(p[id],origin);double a=std::atan2(Dot(d,axes[1]),Dot(d,axes[0]));if(a<0)a+=2*pi;order.emplace_back(a,id);}
 std::sort(order.begin(),order.end());
 for(auto v:order){if(!chart.angles.empty()&&v.first-chart.angles.back()<1e-10)throw std::invalid_argument("Loop is not strictly radial");chart.angles.push_back(v.first);chart.loop.push_back(v.second);auto d=Sub(p[v.second],origin);chart.coordinates.push_back({Dot(d,axes[0]),Dot(d,axes[1])});}
 return chart;
}
inline std::vector<double> CommonLoopKnots(const SurfaceLoopChart& a,const SurfaceLoopChart& b){
 constexpr double pi=3.14159265358979323846;auto knots=a.angles;knots.insert(knots.end(),b.angles.begin(),b.angles.end());std::sort(knots.begin(),knots.end());
 knots.erase(std::unique(knots.begin(),knots.end(),[](double x,double y){return std::abs(x-y)<1e-9;}),knots.end());if(knots.back()-knots.front()>2*pi-1e-9)knots.pop_back();return knots;
}
inline std::vector<unsigned> RefineSurfaceLoop(SurfaceCut& mesh,const SurfaceLoopChart& chart,const std::vector<double>& knots){
 constexpr double pi=3.14159265358979323846;using Edge=std::pair<unsigned,unsigned>;
 struct Insert {unsigned vertex;double t;};std::map<Edge,std::vector<Insert>> insertions;std::vector<unsigned> seam;
 for(double angle:knots){
  auto upper=std::upper_bound(chart.angles.begin(),chart.angles.end(),angle);unsigned next=unsigned(upper-chart.angles.begin())%chart.loop.size(),previous=(next+unsigned(chart.loop.size())-1)%chart.loop.size();
  auto distance=[&](unsigned i){double d=std::abs(chart.angles[i]-angle);return (std::min)(d,2*pi-d);};
  if(distance(previous)<1e-9){seam.push_back(chart.loop[previous]);continue;}if(distance(next)<1e-9){seam.push_back(chart.loop[next]);continue;}
  const auto a=chart.coordinates[previous],b=chart.coordinates[next];const double y=std::cos(angle),z=std::sin(angle),den=(b[0]-a[0])*z-(b[1]-a[1])*y;
  if(std::abs(den)<1e-14)throw std::runtime_error("Radial refinement has no edge intersection");
  const double t=std::clamp(-(a[0]*z-a[1]*y)/den,0.,1.);unsigned ia=chart.loop[previous],ib=chart.loop[next],id=unsigned(mesh.points.size());
  mesh.points.push_back(Add(Scale(mesh.points[ia],1-t),Scale(mesh.points[ib],t)));seam.push_back(id);
  if(ia<ib)insertions[{ia,ib}].push_back({id,t});else insertions[{ib,ia}].push_back({id,1-t});
 }
 for(auto& entry:insertions)std::sort(entry.second.begin(),entry.second.end(),[](auto a,auto b){return a.t<b.t;});
 std::vector<SurfaceCutFace> faces;faces.reserve(mesh.faces.size()+knots.size()*3);
 for(auto face:mesh.faces){
  bool split=false;for(unsigned k=0;k<3;k++)split|=insertions.count(std::minmax(face.corners[k].vertex,face.corners[(k+1)%3].vertex))!=0;
  if(!split){faces.push_back(face);continue;}
  std::vector<SurfaceCutCorner> poly;
  for(unsigned k=0;k<3;k++){
   const auto a=face.corners[k],b=face.corners[(k+1)%3];poly.push_back(a);auto found=insertions.find(std::minmax(a.vertex,b.vertex));if(found==insertions.end())continue;
   auto append=[&](Insert item){const double t=a.vertex<b.vertex?item.t:1-item.t;poly.push_back({item.vertex,Add(Scale(a.donor,1-t),Scale(b.donor,t))});};
   if(a.vertex<b.vertex)for(auto item:found->second)append(item);else for(auto i=found->second.rbegin();i!=found->second.rend();i++)append(*i);
  }
  if(poly.size()==3){faces.push_back(face);continue;}
  SurfaceCutCorner center;center.vertex=unsigned(mesh.points.size());PresentationPoint p{};
  for(auto v:poly){p=Add(p,mesh.points[v.vertex]);center.donor=Add(center.donor,v.donor);}p=Scale(p,1./poly.size());center.donor=Scale(center.donor,1./poly.size());mesh.points.push_back(p);
  for(unsigned k=0;k<poly.size();k++)faces.push_back({face.parent,{center,poly[k],poly[(k+1)%poly.size()]}});
 }
 mesh.faces=std::move(faces);mesh.opening=seam;return seam;
}
struct SurfaceJoinCorner {unsigned vertex=0,domain=0,parent=0;std::array<double,3> donor{};std::array<double,2> uv{};};
struct SurfaceJoin {
 std::vector<PresentationPoint> points;
 std::vector<std::array<SurfaceJoinCorner,3>> faces;
 std::vector<std::vector<unsigned>> rings;
 std::vector<PresentationPoint> firstTangents;
 // Rest-space correction measured on the original body, cached once per
 // morphology. Live reposing must retain the same geometric boundary field.
 std::vector<PresentationPoint> bodyDisplacements;
 std::vector<PresentationPoint> bodyGradients;
 PresentationPoint bodyAxis{};
 std::vector<PresentationPoint> entryPoints,entryNormals;
};
// Project onto the front surface of the measured body along the mount axis.
// A shading normal is not the plane of the triangle that is actually retained.
inline PresentationPoint BodySurfacePoint(PresentationPoint p,PresentationPoint axis,const std::vector<PresentationPoint>& points,const std::vector<std::array<unsigned,3>>& faces,PresentationPoint* normal=nullptr,const std::vector<unsigned>* candidates=nullptr){
 bool found=false;double front=-std::numeric_limits<double>::infinity();PresentationPoint result{};
 for(unsigned index=0;index<(candidates?candidates->size():faces.size());index++){
  const auto f=faces[candidates?(*candidates)[index]:index];
  const auto a=points[f[0]],e=Sub(points[f[1]],a),g=Sub(points[f[2]],a),n=Cross(e,g);const double facing=Dot(n,axis);
  if(facing<=1e-12)continue;
  const auto q=Sub(p,Scale(axis,Dot(Sub(p,a),n)/facing)),v=Sub(q,a);
  const double ee=Dot(e,e),eg=Dot(e,g),gg=Dot(g,g),det=ee*gg-eg*eg;if(det<=1e-20)continue;
  const double u=(Dot(v,e)*gg-Dot(v,g)*eg)/det,w=(Dot(v,g)*ee-Dot(v,e)*eg)/det;
  if(u< -1e-8||w< -1e-8||u+w>1+1e-8)continue;
  const double depth=Dot(q,axis);if(depth>front){front=depth;result=q;found=true;if(normal)*normal=Unit(n);}
 }
 if(!found)throw std::runtime_error("Root ramp leaves the measured body chart");
 return result;
}
// Index only the two coordinates perpendicular to the projection ray. The
// exact triangle test above remains authoritative, including original face
// ordering for coplanar ties. This avoids a full body scan for every ramp row
// whenever a shape/angle change rebuilds the attachment.
class BodySurfaceChart {
 static constexpr unsigned resolution=64;
 using Vec=PresentationPoint;using UV=std::array<double,2>;
 std::array<Vec,2> basis_;UV low_{},high_{};
 std::vector<std::vector<unsigned>> cells_{resolution*resolution};
 std::vector<unsigned> empty_;
 UV Project(Vec p)const{return {Dot(p,basis_[0]),Dot(p,basis_[1])};}
 unsigned Cell(double x,unsigned d)const{return unsigned(std::clamp((x-low_[d])/(high_[d]-low_[d])*resolution,0.,double(resolution-1)));}
public:
 BodySurfaceChart(Vec axis,const std::vector<Vec>& points,const std::vector<std::array<unsigned,3>>& faces){
  Vec reference=std::abs(axis[2])<.9?Vec{0,0,1}:Vec{0,1,0};
  basis_[0]=Unit(Cross(reference,axis));basis_[1]=Unit(Cross(axis,basis_[0]));
  const double inf=std::numeric_limits<double>::infinity();low_={inf,inf};high_={-inf,-inf};
  struct Box{unsigned face;UV low,high;};std::vector<Box> boxes;
  for(unsigned i=0;i<faces.size();i++){
   auto f=faces[i];if(Dot(Cross(Sub(points[f[1]],points[f[0]]),Sub(points[f[2]],points[f[0]])),axis)<=1e-12)continue;
   Box b{i,{inf,inf},{-inf,-inf}};
   for(auto id:f){auto uv=Project(points[id]);for(unsigned d=0;d<2;d++){b.low[d]=(std::min)(b.low[d],uv[d]);b.high[d]=(std::max)(b.high[d],uv[d]);}}
   for(unsigned d=0;d<2;d++){
    // Include the same barycentric tolerance as the exact test, even at an
    // original edge or a spatial-cell boundary.
    const double pad=(std::max)(1e-10,2e-8*(b.high[d]-b.low[d]));b.low[d]-=pad;b.high[d]+=pad;
    low_[d]=(std::min)(low_[d],b.low[d]);high_[d]=(std::max)(high_[d],b.high[d]);
   }boxes.push_back(b);
  }
  if(boxes.empty())throw std::invalid_argument("Measured body has no forward chart");
  for(const auto& b:boxes)for(unsigned y=Cell(b.low[1],1);y<=Cell(b.high[1],1);y++)for(unsigned x=Cell(b.low[0],0);x<=Cell(b.high[0],0);x++)cells_[y*resolution+x].push_back(b.face);
 }
 const std::vector<unsigned>& Candidates(Vec p)const{
  const auto q=Project(p);for(unsigned d=0;d<2;d++)if(q[d]<low_[d]||q[d]>high_[d])return empty_;
  return cells_[Cell(q[1],1)*resolution+Cell(q[0],0)];
 }
};
// A Hermite derivative can pull the connecting skin inside BOTH endpoints,
// even when the aperture and shaft diameters are correct. Bound its Bezier
// control hull in the radial direction. Shortening (never rotating) the two
// derivatives preserves the source tangent planes and the welded endpoints.
inline std::array<PresentationPoint,2> BoundRampTangents(PresentationPoint a,PresentationPoint b,PresentationPoint first,PresentationPoint last,PresentationPoint radial,double tolerance){
 radial=Unit(radial);const double da=Dot(a,radial),db=Dot(b,radial);
 const double low=(std::min)(da,db)-tolerance,high=(std::max)(da,db)+tolerance;
 auto limit=[&](PresentationPoint tangent,double endpoint,double sign){
  const double change=sign*Dot(tangent,radial)/3.;double factor=1;
  if(change>0)factor=(std::min)(1.,(high-endpoint)/change);
  if(change<0)factor=(std::min)(1.,(low-endpoint)/change);
  return Scale(tangent,std::clamp(factor,0.,1.));
 };
 return {limit(first,da,1),limit(last,db,-1)};
}
inline PresentationPoint CrossSectionRadial(PresentationPoint radial,PresentationPoint shaftAxis){
 shaftAxis=Unit(shaftAxis);return Sub(radial,Scale(shaftAxis,Dot(radial,shaftAxis)));
}
// Match the retained measured body surface at the outer rim, then release
// that constraint only in the first quarter of the connector. Extending the
// projection across its whole width creates a planar annulus before the bend.
// The local C2 release leaves the remaining fillet free to turn into the shaft.
inline PresentationPoint RampEntryPoint(PresentationPoint p,PresentationPoint bodyPoint,PresentationPoint bodyNormal,PresentationPoint axis,double t){
 const double facing=Dot(bodyNormal,axis);if(facing<=.15)return p;
 const double u=std::clamp(t/.25,0.,1.),w=1-u*u*u*(10+u*(-15+6*u));
 return Sub(p,Scale(axis,w*Dot(Sub(p,bodyPoint),bodyNormal)/facing));
}
// Use the same boundary construction for full fitting and compact-frame motion.
// Entry supplies the immutable measured body point/normal for (vertex,ordinal).
template<class Entry> void ReposeRootRamp(std::vector<PresentationPoint>& points,
 const std::vector<std::vector<unsigned>>& rings,const std::vector<PresentationPoint>& firstTangents,
 PresentationPoint anchor,PresentationPoint mountAxis,PresentationPoint shaftAxis,Entry entry,TaskArena* arena=nullptr){
 if(rings.size()<2||rings.front().empty()||firstTangents.size()!=rings.front().size())throw std::invalid_argument("Invalid prepared root ramp");
 const auto columns=unsigned(firstTangents.size());for(const auto& ring:rings)if(ring.size()!=columns)throw std::invalid_argument("Root ramp circumference changed");
 auto first=firstTangents;std::vector<PresentationPoint> last(columns);double firstScale=1,lastScale=1;
 for(unsigned i=0;i<columns;i++){
  auto a=points[rings.front()[i]],b=points[rings.back()[i]],d=Sub(a,b);const double distance=std::sqrt(Dot(d,d));last[i]=Scale(shaftAxis,distance);
  auto radial=Sub(Sub(a,anchor),Scale(mountAxis,Dot(Sub(a,anchor),mountAxis)));
  auto bounded=BoundRampTangents(a,b,first[i],last[i],CrossSectionRadial(radial,shaftAxis),.005*std::sqrt(Dot(radial,radial)));
  if(Dot(first[i],first[i])>1e-20)firstScale=(std::min)(firstScale,std::sqrt(Dot(bounded[0],bounded[0])/Dot(first[i],first[i])));
  if(Dot(last[i],last[i])>1e-20)lastScale=(std::min)(lastScale,std::sqrt(Dot(bounded[1],bounded[1])/Dot(last[i],last[i])));
 }
 for(auto& p:first)p=Scale(p,firstScale);for(auto& p:last)p=Scale(p,lastScale);
 auto pose=[&](unsigned index){const unsigned row=1+index/columns,col=index%columns,id=rings[row][col];
  const double t=.5*(1-std::cos(3.14159265358979323846*row/(rings.size()-1)));
  auto p=modular::Hermite(points[rings.front()[col]],points[rings.back()[col]],first[col],last[col],t);const auto field=entry(id,index);
  points[id]=RampEntryPoint(p,field.first,field.second,mountAxis,t);
 };
 const auto count=unsigned(rings.size()-2)*columns;if(arena)arena->For(count,pose);else for(unsigned i=0;i<count;i++)pose(i);
}
inline SurfaceJoin StitchSurfaces(SurfaceCut body,SurfaceCut module,const SurfaceLoopChart& bodyChart,const SurfaceLoopChart& moduleChart,PresentationPoint origin,PresentationPoint mountAxis,PresentationPoint shaftAxis,const std::vector<PresentationPoint>& bodyNormals,const std::vector<std::array<unsigned,3>>& bodyParents,double radialMargin=.18,double undersideExtra=.08,unsigned rows=12,const std::vector<PresentationPoint>* originalBody=nullptr){
 using Vec=PresentationPoint;const auto knots=CommonLoopKnots(bodyChart,moduleChart);auto outer=RefineSurfaceLoop(body,bodyChart,knots),inner=RefineSurfaceLoop(module,moduleChart,knots);
 if(rows<2||rows>128)throw std::invalid_argument("Invalid joining ramp density");

 // Boundary normals come from the intact source surface, not the remaining
 // one-sided cut triangles. Exact position/skin donors survive independently.
 std::vector<Vec> outerNormals(body.points.size());
 for(auto face:body.faces)for(auto c:face.corners){Vec n{};for(unsigned k=0;k<3;k++)n=Add(n,Scale(bodyNormals[bodyParents[face.parent][k]],c.donor[k]));outerNormals[c.vertex]=Unit(n);}
 std::set<std::pair<unsigned,unsigned>> directed;for(auto f:body.faces)for(unsigned k=0;k<3;k++)directed.emplace(f.corners[k].vertex,f.corners[(k+1)%3].vertex);
 const bool reversed=!directed.count({outer[0],outer[1]});if(reversed){std::reverse(outer.begin(),outer.end());std::reverse(inner.begin(),inner.end());}
 SurfaceJoin out;out.points=body.points;const unsigned offset=unsigned(body.points.size());out.points.insert(out.points.end(),module.points.begin(),module.points.end());
 for(unsigned domain=0;domain<2;domain++)for(auto f:domain?module.faces:body.faces){std::array<SurfaceJoinCorner,3> row;for(unsigned k=0;k<3;k++){auto c=f.corners[k];row[k]={c.vertex+(domain?offset:0),domain,f.parent,c.donor,{}};}out.faces.push_back(row);}
 for(auto& i:inner)i+=offset;out.rings.push_back(outer);
 out.bodyDisplacements.resize(out.points.size());out.bodyGradients.resize(out.points.size());out.bodyAxis=mountAxis;

 std::vector<Vec> first(outer.size()),last(outer.size());double firstScale=1,lastScale=1;
 for(unsigned i=0;i<outer.size();i++){
  auto a=out.points[outer[i]],b=out.points[inner[i]],d=Sub(b,a),radial=Sub(Sub(a,origin),Scale(mountAxis,Dot(Sub(a,origin),mountAxis)));const double distance=std::sqrt(Dot(d,d)),radius=std::sqrt(Dot(radial,radial));
  auto tangent=Scale(radial,-1);tangent=Sub(tangent,Scale(outerNormals[outer[i]],Dot(tangent,outerNormals[outer[i]])));
  // The caller's chart convention has positive second coordinate upward.
  const double ventral=std::pow((std::max)(0.,-std::sin(knots[reversed?knots.size()-1-i:i])),4);
  first[i]=Scale(Unit(tangent),(std::min)(distance,2*(radialMargin+undersideExtra*ventral)*radius/(1+radialMargin)));
  last[i]=Scale(shaftAxis,distance);
  auto bounded=BoundRampTangents(a,b,first[i],last[i],CrossSectionRadial(radial,shaftAxis),.005*radius);
  if(Dot(first[i],first[i])>1e-20)firstScale=(std::min)(firstScale,std::sqrt(Dot(bounded[0],bounded[0])/Dot(first[i],first[i])));
  if(Dot(last[i],last[i])>1e-20)lastScale=(std::min)(lastScale,std::sqrt(Dot(bounded[1],bounded[1])/Dot(last[i],last[i])));
 }
 // A common derivative scale preserves adjacent-column ordering, including
 // almost coincident original-edge knots. Per-column clamps can fold the
 // first strip despite bounding each individual curve correctly.
 for(unsigned i=0;i<first.size();i++){first[i]=Scale(first[i],firstScale);last[i]=Scale(last[i],lastScale);}
 out.firstTangents=first;
 for(unsigned row=1;row<=rows;row++){
  const double t=.5*(1-std::cos(3.14159265358979323846*row/rows));std::vector<unsigned> ring;
  for(unsigned i=0;i<outer.size();i++){if(row==rows)ring.push_back(inner[i]);else{ring.push_back(unsigned(out.points.size()));out.points.push_back(modular::Hermite(out.points[outer[i]],out.points[inner[i]],first[i],last[i],t));}}
  const auto& previous=out.rings.back();const double before=.5*(1-std::cos(3.14159265358979323846*(row-1)/rows));
  for(unsigned i=0;i<outer.size();i++){
   const unsigned j=(i+1)%unsigned(outer.size());const double u0=double(i)/outer.size(),u1=double(i+1)/outer.size();
   SurfaceJoinCorner a{previous[i],2,0,{}, {u0,before}},b{previous[j],2,0,{}, {u1,before}},c{ring[i],2,0,{}, {u0,t}},d{ring[j],2,0,{}, {u1,t}};
   out.faces.push_back({b,a,c});out.faces.push_back({b,c,d});
  }out.rings.push_back(std::move(ring));
 }
 out.bodyDisplacements.resize(out.points.size());
 out.bodyGradients.resize(out.points.size());
 out.entryPoints.resize(out.points.size());out.entryNormals.resize(out.points.size());
 if(originalBody){BodySurfaceChart chart(mountAxis,*originalBody,bodyParents);
 for(unsigned row=1;row+1<out.rings.size();row++){
  const double t=.5*(1-std::cos(3.14159265358979323846*row/rows));
  for(unsigned column=0;column<outer.size();column++){
   const auto id=out.rings[row][column];auto p=out.points[id];Vec n{},q{};
   try{q=BodySurfacePoint(p,mountAxis,*originalBody,bodyParents,&n,&chart.Candidates(p));}
   catch(const std::runtime_error&){q=out.points[outer[column]];n=outerNormals[outer[column]];}
   if(Dot(n,mountAxis)<=.15)continue;
   out.entryPoints[id]=q;out.entryNormals[id]=n;
   out.points[id]=RampEntryPoint(p,q,n,mountAxis,t);
  }
 }
 }
 return out;
}
}
