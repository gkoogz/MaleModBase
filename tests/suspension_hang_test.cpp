#include <malemod/surface/suspension_hang.hpp>
#include <iostream>
using namespace malemod::surface::modular;
void Check(bool v,const char* s){if(!v)throw std::runtime_error(s);}
int main(){try{
 std::vector<Vec> p={{0,0,0},{1,0,0},{0,1,0},{0,0,10},{1,0,8},{0,1,9}};
 Binding sack,shaft;sack.influences[0]={13,1,{}};shaft.influences[0]={1,1,{}};
 std::vector<Loft> skin;
 for(unsigned i=0;i<3;i++){Loft l;l.start=i;l.end=i+3;l.firstOwner=sack;l.lastOwner=shaft;skin.push_back(l);}
 double last=1e9;
 for(double h:{1.,10.,25.,40.,50.,75.,100.}){
  double lift=SuspensionLift(p,skin,2,h);
  Check(lift<=last,"Hang is not monotone");Check(lift>=0&&lift<8-.4,"Neck collapsed");
  if(h>=50)Check(lift==0,"Default/high Hang changed");last=lift;
 }
 Check(SuspensionLift(p,skin,2,1)>6,"Low Hang did not retract the measured neck");
 auto scaled=p;for(auto& q:scaled)for(auto& x:q)x=x*3+4;
 Check(std::abs(SuspensionLift(scaled,skin,6,25)-3*SuspensionLift(p,skin,2,25))<1e-12,"Hang is not scale/translation invariant");
 p[4][2]=.2;Check(SuspensionLift(p,skin,2,1)==0,"No-clearance neck moved into shaft");
 bool bad=false;try{SuspensionLift(p,skin,2,0);}catch(const std::invalid_argument&){bad=true;}Check(bad,"Invalid slider accepted");
 std::cout<<"PASS measured neck retraction, monotonic controls, clearance, scale invariance and exact neutral/high range\n";
 return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
