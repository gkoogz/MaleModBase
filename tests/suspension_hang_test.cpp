#include <malemod/surface/suspension_hang.hpp>
#include <malemod/surface/compound_dynamic.hpp>
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
 // The captured minimum-size compound rim is lower than the retired loft
 // support: a 2.078659 lift inverted 214 columns. Measure the new rim instead.
 using malemod::surface::CompoundDynamicPlan;
 std::vector<Vec> lower={{0,0,82.22314},{1,0,82.4},{0,1,82.6}};
 std::vector<Vec> upper={{0,0,81.462715},{1,0,84.2},{0,1,85.2}};
 const double oldCapacity=2.078659,radius=1.5;
 const auto cap=CompoundDynamicPlan::BoundSuspensionLift(lower,upper,radius,oldCapacity,oldCapacity);
 Check(cap>0&&cap<1,"Compound rim did not bound the obsolete loft lift");
 for(unsigned i=0;i<lower.size();i++)Check(upper[i][2]-(lower[i][2]-oldCapacity+cap)>.2*radius,"Compound lifted rim crossed its support");
 auto unlifted=lower;for(auto& q:unlifted)q[2]-=oldCapacity;
 Check(std::abs(cap-CompoundDynamicPlan::BoundSuspensionLift(unlifted,upper,radius,oldCapacity,0))<1e-12,"Capacity depends on applied Hang weight");
 auto sideLower=lower,sideUpper=upper;
 for(auto* field:{&sideLower,&sideUpper})for(auto& q:*field){std::swap(q[0],q[2]);for(auto& x:q)x=3*x+7;}
 Check(std::abs(CompoundDynamicPlan::BoundSuspensionLift(sideLower,sideUpper,3*radius,3*oldCapacity,3*oldCapacity,{1,0,0})-3*cap)<1e-11,"Compound clearance ignored lift direction or scale");
 for(auto& q:upper)q[2]+=10;
 Check(CompoundDynamicPlan::BoundSuspensionLift(lower,upper,radius,oldCapacity,oldCapacity)==oldCapacity,"Adequate compound clearance changed capacity");
 upper[0][2]=lower[0][2]-oldCapacity-.1;
 Check(CompoundDynamicPlan::BoundSuspensionLift(lower,upper,radius,oldCapacity,oldCapacity)==0,"Already reversed compound column was lifted farther");
 std::cout<<"PASS measured neck retraction, monotonic controls, clearance, scale invariance and exact neutral/high range\n";
 return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
