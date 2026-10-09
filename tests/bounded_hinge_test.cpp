#include <malemod/physics/bounded_hinge.hpp>
#include <iostream>
#include <limits>
using namespace malemod;using namespace malemod::physics;
#define CHECK(x) do{if(!(x)){std::cerr<<"FAIL "<<__LINE__<<'\n';return 1;}}while(0)
int main(){
 HingeState a,b;for(int i=0;i<30;i++)StepHinge(a,4,12,.7f,.06f,1.f/30);
 for(int i=0;i<240;i++)StepHinge(b,4,12,.7f,.06f,1.f/240);
 CHECK(std::abs(a.angle-b.angle)<1e-6f);CHECK(std::abs(a.velocity-b.velocity)<1e-5f);
 for(int i=0;i<2000;i++){StepHinge(a,i%2?1e6f:-1e6f,12,.7f,.06f,.2f);CHECK(std::abs(a.angle)<=.06f);CHECK(std::isfinite(a.velocity));}
 for(int i=0;i<100;i++)StepHinge(a,0,12,.7f,.06f,.1f);CHECK(std::abs(a.angle)<1e-8f);
 a={.03f,.4f};b=a;StepHinge(a,2,12,.7f,.06f,0);CHECK(a.angle==b.angle&&a.velocity==b.velocity);
 CHECK(Length(HingeOffset({1,2,3},{1,2,3},{0,0,1},.3f,1))==0);
 CHECK(Length(HingeOffset({4,2,3},{1,2,3},{0,0,1},.3f,0))==0);
 V3 p={4,2,3},h={1,2,3};auto q=p+HingeOffset(p,h,{0,0,1},.3f,1);CHECK(std::abs(Length(q-h)-Length(p-h))<1e-6f);
 StepHinge(a,std::numeric_limits<float>::quiet_NaN(),12,.7f,.06f,.1f);CHECK(a.angle==0&&a.velocity==0);
 a={.03f,.4f};StepHinge(a,2,12,.7f,.06f,1);CHECK(a.angle==0&&a.velocity==0);
 std::cout<<"PASS timestep invariance, bounds, decay, pause, pinned seam, rigid length, invalid/stall reset\n";
}
