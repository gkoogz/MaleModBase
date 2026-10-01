#include <malemod/physics/rig_kernels.hpp>
#include <malemod/physics/ovoid_support.hpp>
#include <random>
#include <iostream>
namespace original {
using malemod::V3;using std::min;using std::max;
#include "data/wolverine-ovoid-support.inc"
}
using namespace malemod;using namespace malemod::physics;
#define CHECK(x) do{if(!(x)){std::cerr<<"FAIL "<<__LINE__<<" "<<#x<<'\n';return 1;}}while(0)
int main(){
 std::mt19937 random(729);std::uniform_real_distribution<float> v(-1,1);float worst=0;
 for(int i=0;i<5000;i++){
  V3 n=Unit({v(random),v(random),v(random)}),r={5.724f,4.86f,7.93f};
  auto a=OvoidSupport(n,r),b=original::CPSupportLocal(n,r);
  worst=max(worst,Length(a-b));CHECK(std::isfinite(Length(a)));CHECK(Length(a-b)<.0001f);
 }
 CHECK(OvoidSupport({0,0,1},{2,3,4}).z==4);CHECK(OvoidSupport({0,0,-1},{2,3,4}).z==-4);
 State a,b;a.position={{0,0,0},{0,1,0},{.2f,2,0}};a.oldPosition=a.position;a.invMass={0,1,1};b=a;
 auto data=PrepareBend(a,1,.00008f,1.f/120,.5f);V3 la{},lb{};
 SolveBendPrepared(a,1,data,la);SolveRestBend(b,1,data,{},{},lb);
 for(int i=0;i<3;i++)CHECK(Length(a.position[i]-b.position[i])<1e-7f);
 State contact;contact.position={{0,0,.05f}};contact.oldPosition={{0,0,.05f}};contact.invMass={1};
 ProjectMovingContact(contact,0,{0,0,1},-.1f,{});
 CHECK(std::abs(contact.position[0].z-.15f)<1e-6f);
 CHECK(Length(contact.position[0]-contact.oldPosition[0])==0);
 contact.position[0]={0,0,.01f};contact.oldPosition[0]={0,0,.05f};
 ProjectMovingContact(contact,0,{0,0,1},-.1f,{0,0,.02f});
 CHECK(std::abs((contact.position[0]-contact.oldPosition[0]).z-.02f)<1e-6f);
 State tether;tether.position={{0,0,-2}};tether.oldPosition=tether.position;tether.invMass={1};float lm=0;
 SolveSuspension(tether,0,{},{},1,.00001f,.4f,1.f/60,lm);
 CHECK(tether.position[0].z>-2);CHECK(lm<0);
 SolveReach(tether,0,{},1.1f);CHECK(Length(tether.position[0])<=1.100001f);
 tether.position[0]={0,0,-.5f};tether.oldPosition=tether.position;lm=0;
 SolveSuspension(tether,0,{},{},1,.00001f,.4f,1.f/60,lm);CHECK(tether.position[0].z==-.5f);CHECK(lm==0);
 State pair;pair.position={{0,0,0},{0,0,.1f}};pair.oldPosition=pair.position;pair.invMass={1,.5f};
 ProjectPairContact(pair,0,1,{0,0,1},-.2f);
 CHECK(std::abs(pair.position[1].z-pair.position[0].z-.3f)<1e-6f);
 CHECK(Length(pair.position[0]-pair.oldPosition[0])==0);CHECK(Length(pair.position[1]-pair.oldPosition[1])==0);
 CHECK(std::abs(pair.position[0].z+2*pair.position[1].z-.2f)<1e-6f);
 State rod;rod.position={{0,0,0},{1,0,0},{.25f,0,.1f}};rod.oldPosition=rod.position;rod.invMass={1,1,1};
 ProjectRodContact(rod,0,1,2,.25f,{0,0,1},-.2f);
 CHECK(std::abs((rod.position[2]-(rod.position[0]*.75f+rod.position[1]*.25f)).z-.3f)<1e-6f);
 CHECK(Length(rod.position[0]-rod.oldPosition[0])==0);
 // Length scaling preserves the source XPBD metric without scaling compliance.
 State big,small;big.position={{0,0,0},{0,2,0},{.2f,4,0}};big.oldPosition=big.position;big.invMass={0,1,1};small=big;
 for(int i=0;i<3;i++){small.position[i]=small.position[i]*.01f;small.oldPosition[i]=small.oldPosition[i]*.01f;}
 float largeLambda=0,smallLambda=0;SolveDistance(big,1,2,1,.0001f,largeLambda,1.f/60);SolveDistance(small,1,2,.01f,.0001f,smallLambda,1.f/60);
 for(int i=0;i<3;i++)CHECK(Length(big.position[i]*.01f-small.position[i])<1e-7f);
 // Render samples must reproduce a straight guide and be C1 across stations.
 State guide;for(int i=0;i<12;i++)guide.position.push_back({float(i)*2,0,0});
 V3 center,tangent;SampleGuide(guide,12,.12f,center,tangent);
 CHECK(Length(center-V3{2.64f,0,0})<1e-6f);CHECK(Length(tangent-V3{1,0,0})<1e-6f);
 for(int i=2;i<12;i++)guide.position[i].z=-.05f*i*i;
 for(int i=1;i<11;i++){V3 ca,cb,ta,tb;SampleGuide(guide,12,i/11.f-1e-6f,ca,ta);SampleGuide(guide,12,i/11.f+1e-6f,cb,tb);CHECK(Length(ca-cb)<.0001f);CHECK(Length(ta-tb)<.0001f);}
 // Moving curved-rest rod, suspended lobes and obstacle: 10 seconds, fixed dt.
 State s;s.position={{0,0,0},{0,.13f,.02f},{0,.18f,.05f},{0,.21f,.1f},{0,.25f,.12f},{0,.28f,.18f},{0,.31f,.2f},{0,.36f,.22f},{-.05f,.14f,-.21f},{.05f,.14f,-.21f}};
 auto reference=s.position;s.oldPosition=s.position;s.velocity.resize(10);s.invMass={0,0,1,1,1,1,1,1,1,1};float lengths[7];
 for(int i=0;i<7;i++)lengths[i]=Length(s.position[i+1]-s.position[i]);
 float maxSegmentError=0;
 for(int step=0;step<600;step++){
  float dt=1.f/60;V3 offset={.025f*sinf(step*dt*5),0,0},oldOffset={.025f*sinf((step-1)*dt*5),0,0};
  auto old=s.position;s.oldPosition=s.position;
  for(int i=0;i<10;i++){if(i<2)s.position[i]=reference[i]+offset;else {s.velocity[i].z-=1.1f*dt;s.velocity[i]=s.velocity[i]*expf(-1.8f*dt);s.position[i]=s.position[i]+s.velocity[i]*dt;}}
  float ls[7]={},sl[2]={},sx[2]={},sy[2]={};V3 bl[6]={};PDBendData bd[6];
  for(int i=0;i<6;i++){float t=(i+1.f)/7;bd[i]=PrepareBend(s,i+1,.0015f*(.02f+.98f*t*t),dt,.5f);}
  for(int iteration=0;iteration<24;iteration++){
   for(int i=1;i<7;i++)SolveDistance(s,i,i+1,lengths[i],0,ls[i],dt);
   for(int i=0;i<6;i++){V3 rest=reference[i]-reference[i+1]*2+reference[i+2];SolveRestBend(s,i+1,bd[i],rest,rest,bl[i]);}
   for(int i=8;i<10;i++){
    SolveMaterialAxis(s,i,reference[i]+offset,reference[i]+oldOffset,{1,0,0},.000022f,dt,sx[i-8]);
    SolveMaterialAxis(s,i,reference[i]+offset,reference[i]+oldOffset,{0,1,0},.000022f,dt,sy[i-8]);
    float rest=Length(reference[i]);SolveSuspension(s,i,offset,oldOffset,rest,.000012f,.4f,dt,sl[i-8]);SolveReach(s,i,offset,rest*1.12f+.01f);
    ProjectMovingContact(s,i,{0,0,1},s.position[i].z+.26f,{});
   }
  }
  for(int i=2;i<10;i++){s.velocity[i]=(s.position[i]-s.oldPosition[i])/dt;CHECK(std::isfinite(Length(s.position[i])));CHECK(Length(s.position[i]-reference[i]-offset)<.3f);}
  for(int i=0;i<7;i++)maxSegmentError=max(maxSegmentError,std::abs(Length(s.position[i+1]-s.position[i])-lengths[i]));
 }
 CHECK(maxSegmentError<.005f);
 std::cout<<"moving rod maximum length error "<<maxSegmentError<<"; suspension/reach; symmetric contact; length conversion passed\n";
 std::cout<<"PASS 5000 source support cases; maximum float error "<<worst<<"; rest-bend parity; moving contact without recovery bounce\n";
}
