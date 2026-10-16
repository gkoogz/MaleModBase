#include <malemod/surface/convex_surface_cut.hpp>
#include <iostream>
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
 bool rejected=false;try{plan.Cut({{-3,-3},{3,-3},{3,3},{-3,3}});}catch(const std::runtime_error&){rejected=true;}Check(rejected,"Removed entire surface without rejection");
 std::cout<<"PASS exact original surface, winding, area, shared intersections, donors and boundary protection\n";
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
