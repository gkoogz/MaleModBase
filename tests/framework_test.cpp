#include <malemod/character.hpp>
#include <malemod/collision.hpp>
#include <malemod/physics/chain.hpp>
#include <iostream>
#include <random>
#include <chrono>

namespace original {
using malemod::V3;using malemod::Length;using std::max;using std::min;
std::vector<V3> pdPosition,pdOldPosition;std::vector<float> pdInvMass;float physUI[8]{};
#include "data/wolverine-xpbd.inc"
}
using namespace malemod;
#define REQUIRE(x) do{if(!(x)){std::cerr<<"FAIL "<<__LINE__<<": "<<#x<<'\n';return false;}}while(0)

bool KernelParity(){
 std::mt19937 rng(418);std::uniform_real_distribution<float> random(-1,1);
 for(int test=0;test<1000;test++){
  physics::State s;for(int i=0;i<12;i++){s.position.push_back({random(rng),random(rng),random(rng)});s.oldPosition.push_back({random(rng),random(rng),random(rng)});s.invMass.push_back(i<2?0.f:1.f+std::abs(random(rng)));}
  original::pdPosition=s.position;original::pdOldPosition=s.oldPosition;original::pdInvMass=s.invMass;original::physUI[2]=37;
  float a=0,b=0;original::PDDistance(2,3,.2f,.00001f,a,1.f/120);physics::SolveDistance(s,2,3,.2f,.00001f,b,1.f/120);
  V3 oldLambda{},newLambda{};auto old=original::PDPrepareBend(3,.00008f,1.f/120);auto modern=physics::PrepareBend(s,3,.00008f,1.f/120,.37f);
  REQUIRE(old.alpha==modern.alpha&&old.gamma==modern.gamma&&old.factor==modern.factor&&a==b);
  original::PDBendPrepared(3,old,oldLambda);physics::SolveBendPrepared(s,3,modern,newLambda);
  for(int i=0;i<12;i++)REQUIRE(Length(s.position[i]-original::pdPosition[i])<1e-7f);
 }
 std::cout<<"PASS 1000 source/portable XPBD kernel cases.\n";return true;
}

bool ChainChecks(){
 std::vector<std::vector<V3>> final;
 for(int fps:{30,60,120}){
  physics::Chain chain;physics::ChainSettings c;chain.Begin(c,{});
  for(int i=0;i<fps*5;i++)chain.Advance(1.f/fps,{});
  const auto& s=chain.Get();REQUIRE(Length(s.position[0])<1e-7f);REQUIRE(Length(s.position[1]-V3{0,-c.length/(c.nodes-1),0})<1e-6f);
  for(auto p:s.position)REQUIRE(std::isfinite(Length(p)));
  for(unsigned i=1;i<c.nodes;i++)REQUIRE(std::abs(Length(s.position[i]-s.position[i-1])-c.length/(c.nodes-1))<.002f);
  final.push_back(s.position);
 }
 for(size_t n=0;n<final[0].size();n++)REQUIRE(Length(final[0][n]-final[1][n])<1e-5f&&Length(final[0][n]-final[2][n])<1e-5f);
 physics::Chain a,b;physics::ChainSettings c;a.Begin(c,{});b.Begin(c,{{2,0,0},{0,-1,0}});
 auto saved=b.Get().position;a.Advance(.8f,{});REQUIRE(b.Get().position.size()==saved.size());
 for(size_t i=0;i<saved.size();i++)REQUIRE(Length(b.Get().position[i]-saved[i])==0);
 bool rejected=false;try{c.nodes=2;a.Begin(c,{});}catch(const std::invalid_argument&){rejected=true;}REQUIRE(rejected);
 std::cout<<"PASS fixed-step chain frame-rate equivalence, tether bounds, hitches, isolation and input rejection.\n";return true;
}

bool SkinAndSockets(){
 std::vector<V3> points={{1,0,0}};std::vector<SkinBinding> binding(1);binding[0].joints={0,1,0,0};binding[0].weights={.25f,.75f,0,0};
 std::vector<Transform> palette(2);palette[1].translation={0,2,0};std::vector<V3> out;
 SkinPositions(points,binding,palette,out);REQUIRE(Length(out[0]-V3{1,1.5f,0})<1e-7f);
 AttachmentSocket socket;socket.joint=1;socket.local.translation={0,0,1};auto resolved=ResolveSocket(socket,palette);REQUIRE(Length(resolved.translation-V3{0,2,1})<1e-7f);
 bool rejected=false;binding[0].joints[1]=99;try{SkinPositions(points,binding,palette,out);}catch(const std::invalid_argument&){rejected=true;}REQUIRE(rejected);
 Projection hit;Capsule sphere{{0,0,0},{0,0,0},1,55};REQUIRE(ProjectCapsule({0,0,0},.1f,sphere,hit));REQUIRE(std::abs(Length(hit.position)-1.1f)<1e-6f&&hit.receiverId==55);
 REQUIRE(!ProjectCapsule({2,0,0},.1f,sphere,hit));
 std::cout<<"PASS skin mapping, socket composition, invalid palettes and cached hitbox projection.\n";return true;
}

void Benchmark(){
 physics::Chain chain;chain.Begin({},{});auto start=std::chrono::steady_clock::now();
 for(int i=0;i<12000;i++)chain.Advance(1.f/120,{});
 double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
 std::cout<<"BENCHMARK 12-node/24-iteration chain without contacts: "<<ms/12000<<" ms/substep (this host CPU only).\n";
}
int main(){if(!(KernelParity()&&ChainChecks()&&SkinAndSockets()))return 1;Benchmark();}
