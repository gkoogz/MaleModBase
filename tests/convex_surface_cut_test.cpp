#include <malemod/surface/convex_surface_cut.hpp>
#include <iostream>
#include <malemod/surface/triangle_ray_index.hpp>
using namespace malemod::surface;
void Check(bool ok,const char* message){if(!ok)throw std::runtime_error(message);}
int main(){try{
 std::vector<PresentationPoint> p{{-2,-2,0},{2,-2,0},{2,2,0},{-2,2,0}};
 std::vector<std::array<unsigned,3>> f{{0,1,2},{0,2,3}};
 ConvexSurfaceCut plan(p,f,{true,true},{0,0,0},{{{1,0,0},{0,1,0}}},1e-9);
 for(double radius:{.2,.5,1.,1.7}){
  std::vector<std::array<double,2>> loop;constexpr unsigned count=37;constexpr double pi=3.14159265358979323846;
  for(unsigned i=0;i<count;i++){double a=2*pi*i/count;loop.push_back({radius*std::cos(a),radius*std::sin(a)});}
  auto cut=plan.Cut(loop);double area=0;
  for(const auto& face:cut.faces){auto a=cut.points[face.corners[0].vertex],b=cut.points[face.corners[1].vertex],c=cut.points[face.corners[2].vertex];const auto normal=Cross(Sub(b,a),Sub(c,a));Check(normal[2]>0,"Reversed clipped face");area+=normal[2]*.5;
   for(auto corner:face.corners){PresentationPoint expected{};for(unsigned k=0;k<3;k++)expected=Add(expected,Scale(p[f[face.parent][k]],corner.donor[k]));Check(Dot(Sub(expected,cut.points[corner.vertex]),Sub(expected,cut.points[corner.vertex]))<1e-18,"Lost exact triangle donor");}
  }
  const double removed=.5*count*radius*radius*std::sin(2*pi/count);Check(std::abs(area+removed-16)<1e-8,"Cut area is not conservative");Check(cut.opening.size()>=count,"Opening lost polygon corners");
  for(unsigned i=0;i<p.size();i++)Check(cut.points[i]==p[i],"Original surface moved");
 }
 {
  // A valid material chart can cover a nearly flat world-space sliver.
  // Its opposite edges must not be welded or treated as old boundaries just
  // because they are physically closer than the spatial tolerance.
  auto thin=p;std::vector<std::array<double,2>> chart;
  for(unsigned i=0;i<p.size();i++){chart.push_back({p[i][0],p[i][1]});thin[i][1]*=1e-10;}
  auto cut=ConvexSurfaceCut(thin,f,{true,true},chart,1e-8).CutSimple({{-.8,-.7},{.8,-.7},{.3,0},{.8,.7},{-.8,.7},{-.3,0}});
  Check(cut.opening.size()>=6,"Thin world surface lost its chart opening");
  double retained=0;
  for(auto face:cut.faces){std::array<PresentationPoint,3> tri{};for(unsigned j=0;j<3;j++)for(unsigned k=0;k<3;k++)tri[j]=Add(tri[j],Scale(p[f[face.parent][k]],face.corners[j].donor[k]));
   double a=Cross(Sub(tri[1],tri[0]),Sub(tri[2],tri[0]))[2]*.5;Check(a>0,"Thin world cut inverted a chart triangle");retained+=a;}
  Check(std::abs(retained-14.46)<1e-8,"Thin world cut changed retained chart area");
 }
 {
  auto points=p;points.insert(points.end(),{{10,0,0},{11,0,0},{10,2e-10,0},{10,0,1}});
  auto faces=f;faces.insert(faces.end(),{{4,6,5},{4,5,7},{5,6,7},{6,4,7}});
  ConvexSurfaceCut preserved(points,faces,{true,true,false,false,false,false},{0,0,0},{{{1,0,0},{0,1,0}}},1e-8);
  auto cut=preserved.CutSimple({{-.5,-.5},{.5,-.5},{.5,.5},{-.5,.5}});
  bool found=false;for(const auto& face:cut.faces)if(face.parent==2){found=true;for(unsigned k=0;k<3;k++)Check(face.corners[k].vertex==faces[2][k],"Untouched thin triangle was changed");}
  Check(found,"Clipping deleted an unrelated thin source triangle");
 }
 {
  auto points=p;points.insert(points.end(),{{.8,.8,0},{1.,.8,0},{.9,.8+2e-10,0},{.8,.8,1}});
  auto faces=f;faces.insert(faces.end(),{{4,5,6},{4,7,5},{5,7,6},{6,7,4}});
  ConvexSurfaceCut preserved(points,faces,{true,true,true,false,false,false},{0,0,0},{{{1,0,0},{0,1,0}}},1e-8);
  std::vector<std::array<double,2>> loop;for(unsigned j=0;j<37;j++){double a=6.283185307179586*j/37;loop.push_back({std::cos(a),std::sin(a)});}
  auto cut=preserved.Cut(loop);bool found=false;for(auto face:cut.faces)found|=face.parent==2;
  Check(found,"Eligible untouched sliver vanished during clipping");
 }
 {
  // The rear footprint narrows into a concave waist in a compound chart.
  // Cutting its convex hull would remove surrounding tissue incorrectly.
  std::vector<std::array<double,2>> footprint{{-1,-1},{1,-1},{.3,0},{1,1},{-1,1},{-.3,0}};
  std::vector<std::array<double,2>> chart;for(auto v:p)chart.push_back({v[0],v[1]});
  auto curved=p;curved[2][2]=.7;
  ConvexSurfaceCut compound(curved,f,{true,true},chart,1e-9);
  auto cut=compound.CutSimple(footprint);
  double chartArea=0;
  for(const auto& face:cut.faces){
   std::array<PresentationPoint,3> xy{};
   for(unsigned j=0;j<3;j++){
    auto c=face.corners[j];PresentationPoint expected{};
    for(unsigned k=0;k<3;k++){auto id=f[face.parent][k];expected=Add(expected,Scale(curved[id],c.donor[k]));xy[j]=Add(xy[j],Scale(p[id],c.donor[k]));}
    Check(Dot(Sub(expected,cut.points[c.vertex]),Sub(expected,cut.points[c.vertex]))<1e-18,"Compound cut lost exact support donor");
   }
   double area=Cross(Sub(xy[1],xy[0]),Sub(xy[2],xy[0]))[2]*.5;
   Check(area>0,"Compound cut reversed a chart triangle");chartArea+=area;
  }
  Check(std::abs(chartArea-13.4)<1e-8,"Concave compound cut removed the convex hull");
  Check(cut.opening.size()>=footprint.size(),"Compound cut lost rim knots");
  for(unsigned i=0;i<curved.size();i++)Check(cut.points[i]==curved[i],"Compound cut moved retained support");
 }
 {
  std::vector<PresentationPoint> points;std::vector<std::array<unsigned,3>> faces;
  for(unsigned layer=0;layer<3;layer++)for(unsigned y=0;y<12;y++)for(unsigned x=0;x<12;x++){
   unsigned n=unsigned(points.size());double z=layer==2?0.:double(layer);
   points.insert(points.end(),{{double(x),double(y),z},{double(x+1),double(y),z},{double(x),double(y+1),z}});faces.push_back({n,n+1,n+2});
  }
  std::vector<bool> eligible(faces.size(),true);TriangleRayIndex tree(points,faces,eligible);
  for(unsigned trial=0;trial<1000;trial++){
   PresentationPoint start{double(trial%31)*.4-1,double((trial*17)%31)*.4-1,4},direction{trial%3==0?.13:0.,trial%5==0?-.17:0.,-1};
   double nearest=1e300;bool found=false;PresentationPoint expected{},actual{};
   for(auto f:faces){auto a=Sub(points[f[1]],points[f[0]]),b=Sub(points[f[2]],points[f[0]]),h=Cross(direction,b);double det=Dot(a,h);if(std::abs(det)<1e-12)continue;auto r=Sub(start,points[f[0]]);double u=Dot(r,h)/det;if(u< -1e-9||u>1+1e-9)continue;auto q=Cross(r,a);double v=Dot(direction,q)/det;if(v< -1e-9||u+v>1+1e-9)continue;double t=Dot(b,q)/det;if(t>=0&&t<nearest){found=true;nearest=t;expected=Add(start,Scale(direction,t));}}
   bool hit=tree.Ray(points,faces,start,direction,actual,{0,0,1});Check(hit==found&&(!hit||actual==expected),"Ray broad phase changed exact measured hit");
  }
 }
 bool rejected=false;try{plan.Cut({{-3,-3},{3,-3},{3,3},{-3,3}});}catch(const std::runtime_error&){rejected=true;}Check(rejected,"Removed entire surface without rejection");
 std::cout<<"PASS exact original surface, winding, area, shared intersections, donors and boundary protection\n";
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
