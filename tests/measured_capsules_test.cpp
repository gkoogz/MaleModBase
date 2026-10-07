#include <malemod/garments/measured_capsules.hpp>
#include <iostream>
using namespace malemod::garments;
int main(){try{
 auto need=[](bool ok,const char* message){if(!ok)throw std::runtime_error(message);};
 Input input;
 for(unsigned region=0;region<4;region++){
  unsigned start=unsigned(input.anatomy.size());
  for(unsigned row=0;row<=12;row++)for(unsigned col=0;col<24;col++){
   double phi=3.14159265358979323846*row/12,theta=2*3.14159265358979323846*col/24;
   Point p{region*.3+.08*std::sin(phi)*std::cos(theta),.04*std::sin(phi)*std::sin(theta),.14*std::cos(phi)};
   input.anatomy.push_back({p,{0,0,1},{}});input.anatomyRegions[region].push_back(unsigned(input.anatomy.size()-1));
  }
  for(unsigned row=0;row<12;row++)for(unsigned col=0;col<24;col++){
   unsigned a=start+row*24+col,b=start+row*24+(col+1)%24,c=b+24,d=a+24;
   input.anatomyTriangles.push_back({a,b,c});input.anatomyTriangles.push_back({a,c,d});
  }
 }
 MeasuredCapsules fitter;fitter.Build(input);auto capsules=fitter.Fit(input,1,.003);
 need(!capsules.empty()&&capsules.size()<=16,"Collider budget exceeded");
 for(unsigned group=0;group<capsules.size();group++)for(auto id:fitter.Supports()[group])need(detail::CapsuleDistance(input.anatomy[id].position,capsules[group])<=-.003+1e-10,"Measured source escaped its conservative collider");
 for(auto face:input.anatomyTriangles){bool owned=false;for(const auto& group:fitter.Supports()){bool complete=true;for(auto id:face)complete=complete&&std::binary_search(group.begin(),group.end(),id);owned=owned||complete;}need(owned,"A physical triangle lost its support ownership");}
 auto moved=input;moved.frame.origin={20,-30,40};moved.frame.lateral={0,1,0};moved.frame.forward={-1,0,0};
 for(auto& sample:moved.anatomy)sample.position=moved.frame.World(Mul(sample.position,3));
 auto transformed=fitter.Fit(moved,3,.003);for(unsigned i=0;i<capsules.size();i++)need(Length(Sub(capsules[i].a,transformed[i].a))<1e-9&&Length(Sub(capsules[i].b,transformed[i].b))<1e-9&&std::abs(capsules[i].radius-transformed[i].radius)<1e-9,"Collider fit changed under a rigid frame/unit transform");
 Capsule capsule{{0,0,-1},{0,0,1},.2};
 need(std::abs(MeasuredCapsules::RayExit({0,0,0},{1,0,0},capsule)-.2)<1e-12,"Inside cylinder ray exit");
 need(std::abs(MeasuredCapsules::RayExit({-2,0,0},{1,0,0},capsule)-2.2)<1e-12,"Outside cylinder ray exit");
 need(std::abs(MeasuredCapsules::RayExit({0,0,2},{0,0,-1},capsule)-3.2)<1e-12,"Capsule cap ray exit");
 need(MeasuredCapsules::RayExit({1,0,0},{0,1,0},capsule)==0,"Missed capsule ray should not fabricate a hit");
 std::cout<<"Measured triangle enclosure, stable source binding, units and ray exits passed\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
