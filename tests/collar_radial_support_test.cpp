#include <malemod/surface/collar_radial_support.hpp>
#include <malemod/surface/root_articulation.hpp>
#include <iostream>
using namespace malemod::surface;
using namespace malemod::surface::root_motion;
int main(){try{
 std::vector<bool> body{false,true,false,false,false,false};
 std::vector<double> station{.1,.1,.1,.6,.1,.1},angle(6,1.5707963267948966),role{0,0,1,0,0,0};
 CollarRadialSupport support(body,station,angle,role,{4});
 for(double radians:{-.5,0.,.7})for(double radius:{.5,2.,8.}){
  root_articulation::Rotation rotation({0,0,0},{1,0,0},{0,0,1},radians);
  std::vector<Point> rest(6,rotation.Apply({3,0,radius*.5})),delta(6),guide{rotation.Apply({0,0,0}),rotation.Apply({10,0,0})};
  rest[5]=rotation.Apply({3,0,radius*1.5});auto out=support.Apply(rest,delta,guide,radius);
  auto expected=rotation.Apply({3,0,radius});auto error=Add(Add(rest[0],out[0]),Scale(expected,-1));
  if(Dot(error,error)>1e-20)throw std::runtime_error("Girth or longitudinal placement changed");
  for(unsigned i=1;i<6;i++)if(out[i]!=Point{})throw std::runtime_error("Protected body, core, distal, donor or existing girth changed");
 }
 bool rejected=false;try{support.Apply(std::vector<Point>(6),std::vector<Point>(6),{{0,0,0},{0,0,0}},1);}catch(const std::invalid_argument&){rejected=true;}
 if(!rejected)throw std::runtime_error("Degenerate guide accepted");
 std::cout<<"PASS posed girth, axial placement, donor/body/core/distal preservation and no shrink\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
