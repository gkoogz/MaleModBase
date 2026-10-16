#include <malemod/surface/modular_motion.hpp>
#include <iostream>
using namespace malemod::surface;
using namespace malemod::surface::modular;
void Check(bool v,const char* message){if(!v)throw std::runtime_error(message);}
int main(){try{
 State rest;std::vector<Vec> p{{0,0,0},{1,0,0},{1,0,-2},{1,0,-1}};
 std::vector<Binding> b(4);for(unsigned i=0;i<4;i++)b[i].influences[0]={i==2?13u:i==0?0u:1u,1,p[i]};
 Loft l{3,2,1,.5,{0,0,2},{0,0,2},p[3],b[2],b[1]};
 Plan plan(p,b,{l},rest);auto baseline=plan.Evaluate(rest);
 for(unsigned i=0;i<p.size();i++)Check(Dot(Sub(baseline[i],p[i]),Sub(baseline[i],p[i]))<1e-24,"Rest geometry changed");
 State moved=rest;moved.frames[13].origin={.4,.2,-.3};auto result=plan.Evaluate(moved);
 Check(result[0]==p[0]&&result[1]==p[1],"Suspended motion changed fixed body or complete shaft");
 Check(result[2]==Add(p[2],moved.frames[13].origin),"Suspended skin failed to follow its lobe");
 Check(result[3]!=p[3],"Connecting skin is rigid");
 moved=rest;moved.frames[1].origin={0,0,1};result=plan.Evaluate(moved);
 Check(result[2]==p[2],"Shaft motion was incorrectly baked into the suspended lobe");
 Check(result[1]==Add(p[1],Vec{0,0,1}),"Shaft frame failed to drive shaft");
 moved=rest;moved.scales[1]={1,2,2};result=plan.Evaluate(moved);Check(result[2]==p[2],"Shaft width changed lobe size");
 bool rejected=false;try{moved.scales[1][1]=0;plan.Evaluate(moved);}catch(const std::invalid_argument&){rejected=true;}Check(rejected,"Collapsed morphology accepted");
 std::cout<<"PASS independent shaft/lobe ownership, live connecting skin, exact rest and invalid-scale rejection\n";
 return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
