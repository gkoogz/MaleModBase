#include <malemod/surface/suspension_profile.hpp>
#include <iostream>
using namespace malemod::surface;
void Check(bool value,const char* message){if(!value)throw std::runtime_error(message);}
int main(){try{
 for(double h:{1.,10.,25.,50.,75.,100.}){
  Check(SuspensionSectionScale(0,h)==1&&SuspensionSectionScale(1,h)==1,"Attachment endpoints moved");
  const double epsilon=1e-5;
  Check(std::abs(SuspensionSectionScale(epsilon,h)-1)/epsilon<.001,"Lower join tangent changed");
  Check(std::abs(SuspensionSectionScale(1-epsilon,h)-1)/epsilon<.001,"Upper join tangent changed");
  for(unsigned i=0;i<=100;i++)Check(SuspensionSectionScale(i/100.,h)>=.619,"Neck collapsed");
 }
 double previous=2;
 for(unsigned h=1;h<=100;h++){double value=SuspensionSectionScale(2./3.,h);Check(value<=previous,"Hang waist is not monotone");previous=value;}
 Check(SuspensionSectionScale(2./3.,1)>1,"Low Hang is not fuller");
 Check(SuspensionSectionScale(2./3.,100)<.65,"Long Hang lacks a waist");
 std::vector<PresentationPoint> points{{2,0,3},{0,2,3},{-2,0,3},{0,-2,3},{10,20,30}};
 const auto original=points;std::vector<SuspensionSection> sections{{2./3.,{0,1,2,3}}};
 ShapeSuspensionSections(points,sections,100);Check(points[4]==original[4],"Retained tissue changed");
 auto translated=original;for(auto& p:translated)p=Add(Scale(p,3),{9,-4,7});
 ShapeSuspensionSections(translated,sections,100);
 for(unsigned i=0;i<points.size();i++){auto error=Sub(translated[i],Add(Scale(points[i],3),{9,-4,7}));Check(Dot(error,error)<1e-20,"Profile depends on scale/origin");}
 bool rejected=false;try{SuspensionSectionScale(.5,0);}catch(const std::invalid_argument&){rejected=true;}Check(rejected,"Invalid Hang accepted");
 std::cout<<"PASS attachment tangents, bounded monotone Hang waist, retained ownership and scale invariance\n";return 0;
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
