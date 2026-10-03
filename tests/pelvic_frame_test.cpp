#include <malemod/surface/pelvic_frame.hpp>
#include <cmath>
#include <cstdio>
using namespace malemod;
static bool Near(V3 a,V3 b){return Length(a-b)<2e-6f;}
int main(){
 const float a=30.f*3.1415926535f/180.f;
 auto f=collar::StableRecruitmentFrame({9,0,84.3f},{std::cos(a),0,-std::sin(a)},{0,1,0});
 if(!Near(f.root,{9,0,84.3f})||!Near(f.lateral,{0,1,0})||!Near(Cross(f.axis,f.lateral),f.up))return 1;
 if(std::abs(Dot(f.axis,f.up))>2e-6f||std::abs(Length(f.up)-1)>2e-6f)return 2;
 // A measured rigid character transform carries the whole support frame.
 auto turn=[](V3 p){return V3{-p.y,p.x,p.z};};
 auto g=collar::StableRecruitmentFrame(turn(f.root)+V3{3,-2,7},turn(f.axis)*4,turn(f.lateral)*2);
 if(!Near(g.axis,turn(f.axis))||!Near(g.up,turn(f.up))||!Near(g.root,turn(f.root)+V3{3,-2,7}))return 3;
 // Anatomical command angle never enters this measured frame API.
 for(float angle:{1.f,10.f,50.f,64.f,100.f}){
  (void)angle;auto same=collar::StableRecruitmentFrame(f.root,f.axis,f.lateral);
  if(!Near(same.axis,f.axis)||!Near(same.up,f.up))return 4;
 }
 for(int test=0;test<4;test++){
  bool rejected=false;
  try{collar::StableRecruitmentFrame(test==0?V3{NAN,0,0}:V3{},test==1?V3{}:V3{1,0,0},test==2?V3{}:test==3?V3{2,0,0}:V3{0,1,0});}
  catch(const std::invalid_argument&){rejected=true;}
  if(!rejected)return 5;
 }
 std::puts("PASS measured stable pelvis frame, rigid transform and degenerate input rejection");
}
