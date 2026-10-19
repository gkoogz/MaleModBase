#include <malemod/surface/surface_stitch.hpp>
#include <iostream>
using namespace malemod::surface;
void Check(bool ok,const char* message){if(!ok)throw std::runtime_error(message);}
int main(){try{
 constexpr double pi=3.14159265358979323846;using Vec=PresentationPoint;
 // A raised annulus must leave the source rims and tangent planes exact,
 // scale with the model, and never inflate the circumference itself.
 for(double scale:{.01,1.,100.}){
  Vec a{0,0,1.18*scale},b{.3*scale,0,scale},first{0,0,-.3*scale},last{.3*scale,0,0};
  Check(RaisedRampPoint(a,b,first,last,{0,0,0},{1,0,0},0)==a,"Outer rim moved");
  Check(RaisedRampPoint(a,b,first,last,{0,0,0},{1,0,0},1)==b,"Inner rim moved");
  double previous=-1;
  for(unsigned j=0;j<=100;j++){
   const double t=j/100.;auto original=modular::Hermite(a,b,first,last,t),p=RaisedRampPoint(a,b,first,last,{0,0,0},{1,0,0},t);
   Check(p[1]==original[1]&&p[2]==original[2],"Shoulder widened the circumference");
   Check(p[0]>=original[0]-1e-14&&p[0]-original[0]<=.08*1.18*scale,"Shoulder escaped local envelope");
   Check(p[0]>=previous,"Raised ramp folded backward");previous=p[0];
  }
  for(double t:{1e-5,1-1e-5}){auto d=Sub(RaisedRampPoint(a,b,first,last,{0,0,0},{1,0,0},t),modular::Hermite(a,b,first,last,t));Check(std::sqrt(Dot(d,d))<1e-12*scale,"Shoulder changed boundary differential");}
 }
 // A high-frequency interior normal must not survive arbitrarily close to
 // an exact boundary. This models the observed last-strip shading spike.
 for(Vec interior:{Vec{1,0,0},Vec{-.8,.6,0},Vec{0,0,-1}}){
  const Vec endpoint{0,0,1};auto a=BlendEndpointNormal(interior,endpoint,.004);
  Check(Dot(a,endpoint)>.999999,"Endpoint shading spike survived");
  Check(std::abs(Dot(a,a)-1)<1e-12,"Nonunit stitched normal");
  Check(BlendEndpointNormal(interior,endpoint,0)==endpoint,"Endpoint normal changed");
  auto b=BlendEndpointNormal(interior,endpoint,.25);Check(Dot(Sub(b,Unit(interior)),Sub(b,Unit(interior)))<1e-24,"Interior shading changed outside strip");
 }
 // Level endpoints with opposing slopes previously formed the dorsal saddle.
 for(double slope:{-.9,-.3,0.,.3,.9})for(double height:{-.2,0.,.2}){
  Vec a{0,0,1},b{1,0,1+height},first{.2,0,-1},last{.7,0,slope};
  auto tangents=BoundRampTangents(a,b,first,last,{0,0,1},.005);
  Check(Dot(Cross(first,tangents[0]),Cross(first,tangents[0]))<1e-20,"Body tangent plane changed");
  Check(Dot(Cross(last,tangents[1]),Cross(last,tangents[1]))<1e-20,"Shaft tangent plane changed");
  for(unsigned j=0;j<=100;j++){auto p=modular::Hermite(a,b,tangents[0],tangents[1],j/100.);Check(p[2]>=(std::min)(a[2],b[2])-.005-1e-12&&p[2]<=(std::max)(a[2],b[2])+.005+1e-12,"Ramp escaped its radial envelope");}
 }
 std::vector<Vec> p{{0,-2,-2},{0,2,-2},{0,2,2},{0,-2,2}};std::vector<std::array<unsigned,3>> bodyFaces{{0,1,2},{0,2,3}};
 ConvexSurfaceCut plan(p,bodyFaces,{true,true},{0,0,0},{{{0,1,0},{0,0,1}}},1e-9);std::vector<std::array<double,2>> opening;
 for(unsigned i=0;i<31;i++){double a=2*pi*i/31;opening.push_back({.6*std::cos(a),.6*std::sin(a)});}auto body=plan.Cut(opening);
 SurfaceCut module;std::vector<std::array<unsigned,3>> moduleFaces;
 for(unsigned row=0;row<2;row++)for(unsigned i=0;i<23;i++){double a=2*pi*(i+.07)/23;module.points.push_back({1.+row,.5*std::cos(a),.5*std::sin(a)});if(!row)module.opening.push_back(i);}
 module.points.push_back({2,0,0});for(unsigned i=0;i<23;i++){unsigned j=(i+1)%23;moduleFaces.push_back({i,j,i+23});moduleFaces.push_back({j,j+23,i+23});moduleFaces.push_back({46,i+23,j+23});}
 for(unsigned i=0;i<moduleFaces.size();i++){SurfaceCutFace f;f.parent=i;for(unsigned k=0;k<3;k++){f.corners[k].vertex=moduleFaces[i][k];f.corners[k].donor[k]=1;}module.faces.push_back(f);}
 const auto originalModule=module.points;auto bodyChart=ChartLoop(body,{0,0,0},{{{0,1,0},{0,0,1}}}),moduleChart=ChartLoop(module,{0,0,0},{{{0,1,0},{0,0,1}}});
 auto out=StitchSurfaces(body,module,bodyChart,moduleChart,{0,0,0},{1,0,0},{1,0,0},std::vector<Vec>(p.size(),{1,0,0}),bodyFaces);
 std::map<std::pair<unsigned,unsigned>,std::pair<unsigned,int>> edges;
 for(auto f:out.faces){auto a=out.points[f[0].vertex],b=out.points[f[1].vertex],c=out.points[f[2].vertex];auto n=Cross(Sub(b,a),Sub(c,a));Check(Dot(n,n)>1e-20,"Stitch made a degenerate face");
  for(unsigned k=0;k<3;k++){
   auto corner=f[k];if(corner.domain<2){const auto& source=corner.domain?originalModule:p;auto parent=corner.domain?moduleFaces[corner.parent]:bodyFaces[corner.parent];Vec expected{};for(unsigned j=0;j<3;j++)expected=Add(expected,Scale(source[parent[j]],corner.donor[j]));Check(Dot(Sub(expected,out.points[corner.vertex]),Sub(expected,out.points[corner.vertex]))<1e-18,"Refinement lost position/attribute donors");}
   unsigned x=f[k].vertex,y=f[(k+1)%3].vertex;auto& e=edges[std::minmax(x,y)];e.first++;e.second+=x<y?1:-1;
  }
 }
 for(auto e:edges){Check(e.second.first<=2,"Nonmanifold stitched surface");if(e.second.first==2)Check(e.second.second==0,"Inconsistent stitched winding");else{auto a=out.points[e.first.first],b=out.points[e.first.second];Check(std::abs(a[0])<1e-9&&std::abs(b[0])<1e-9,"Stitch left an open attachment");}}
 Check(out.rings.front().size()>31+23,"Did not refine measured cut intersections");
 std::cout<<"PASS unequal-loop sewing, original donor reconstruction, manifoldness and winding\n";
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
