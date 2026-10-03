#include <malemod/clinical/session.hpp>
#include <malemod/clinical/ambient.hpp>
#include <malemod/clinical/deposit_mesh.hpp>
#include <iostream>
#include <limits>

// Compile the unmodified CPU sequence as an independent comparison in this TU.
// Its Windows-dependent renderer, audio and proxy are never included.
namespace original {
using malemod::V3;using malemod::Dot;using malemod::Cross;using malemod::Length;using malemod::Unit;
using std::min;using std::max;
#include "../legacy/wolverine/src/runtime/teaching_sequence.h"
#include "../legacy/wolverine/src/runtime/teaching_volume.h"
}

namespace c=malemod::clinical;
using malemod::V3;using malemod::Length;using malemod::Unit;
#define REQUIRE(x) do{if(!(x)){std::cerr<<"FAIL line "<<__LINE__<<": "<<#x<<'\n';return false;}}while(0)

template<class F> double Ledger(const F& fluid){
 double sum=0;for(const auto& stream:fluid.streams){sum+=stream.retiredVolume;for(const auto& n:stream.nodes)sum+=n.lump;for(const auto& l:stream.links)sum+=l.volume;}
 return sum;
}

bool Parity(){
 for(int fps:{15,30,60,120}){
  original::volumeFluid::config={};original::volumeFluid::collisionSweep=nullptr;
  original::teaching::Fluid old;c::teaching::Fluid current;
  V3 tip{0,0,100},direction=Unit({1,0,.1f});
  REQUIRE(old.Begin(tip)&&current.Begin(tip));
  original::teaching::Timeline oldTime;c::teaching::Timeline newTime;
  oldTime.Start();newTime.endTime=current.endTime;newTime.Start();
  for(int frame=0;frame<fps*22;frame++){
   float dt=1.f/fps;if(frame==fps*8)dt=.8f;
   oldTime.Advance(dt);newTime.Advance(dt);
   auto a=oldTime.Get();auto b=newTime.Get();
   REQUIRE(a.blend==b.blend&&a.firm==b.firm&&a.pulse==b.pulse&&a.hangPulse==b.hangPulse&&oldTime.active==newTime.active);
   old.Advance(dt,tip,direction,{});current.Advance(dt,tip,direction,{});
   REQUIRE(old.clock==current.clock&&old.emittedVolume==current.emittedVolume&&old.steps==current.steps);
   REQUIRE(old.mesh.indices==current.mesh.indices&&old.mesh.vertices.size()==current.mesh.vertices.size());
   for(size_t i=0;i<current.mesh.vertices.size();i++){
    REQUIRE(Length(old.mesh.vertices[i].p-current.mesh.vertices[i].p)<1e-6f);
    REQUIRE(c::Session::Finite(current.mesh.vertices[i].p)&&c::Session::Finite(current.mesh.vertices[i].n));
   }
   for(int stream=0;stream<4;stream++){
    REQUIRE(old.streams[stream].nodes.size()==current.streams[stream].nodes.size());
    for(size_t i=0;i<current.streams[stream].nodes.size();i++)REQUIRE(Length(old.streams[stream].nodes[i].p-current.streams[stream].nodes[i].p)<1e-6f);
   }
  }
  double expected=0;for(float v:current.pulseVolume)expected+=v;
  REQUIRE(std::abs(Ledger(current)-expected)<expected*1e-5);
  REQUIRE(std::abs(current.emittedVolume-(expected+2*current.settings.dropVolume))<.001);
 }
 std::cout<<"PASS original timeline, fluid nodes, mesh parity and volume conservation at 15/30/60/120 FPS with an 800 ms hitch.\n";
 return true;
}

bool Isolation(){
 c::Session a,b;a.fluid.settings.feed=.3f;b.fluid.settings.feed=2.f;
 a.fluid.settings.volume=20;b.fluid.settings.volume=100;
 a.fluid.SetVariationSeed(123);b.fluid.SetVariationSeed(987);
 int queriesA=0,queriesB=0;
 a.fluid.collisionQuery.callback=[&](V3,V3,float,c::volumeFluid::FluidImpact&){++queriesA;return c::volumeFluid::SweepResult::miss;};
 b.fluid.collisionQuery.callback=[&](V3,V3,float,c::volumeFluid::FluidImpact&){++queriesB;return c::volumeFluid::SweepResult::miss;};
 REQUIRE(a.Begin({0,0,100})&&b.Begin({100,0,100}));
 c::Session solo;solo.fluid.settings=a.fluid.settings;solo.fluid.SetVariationSeed(123);REQUIRE(solo.Begin({0,0,100}));
 float aEnd=a.timeline.endTime,bEnd=b.timeline.endTime;
 for(int i=0;i<600;i++){
  REQUIRE(a.Advance(1.f/60,{{0,0,100},{1,0,0},{}},i*16));
  REQUIRE(b.Advance(1.f/60,{{100,0,100},{1,0,0},{}},i*16));
  REQUIRE(solo.Advance(1.f/60,{{0,0,100},{1,0,0},{}},i*16));
  REQUIRE(a.fluid.emittedVolume==solo.fluid.emittedVolume&&a.fluid.mesh.indices==solo.fluid.mesh.indices);
  for(size_t j=0;j<a.fluid.mesh.vertices.size();j++)REQUIRE(Length(a.fluid.mesh.vertices[j].p-solo.fluid.mesh.vertices[j].p)<1e-6f);
 }
 REQUIRE(queriesA>0&&queriesB>0&&a.timeline.endTime==aEnd&&b.timeline.endTime==bEnd);
 double bTime=b.fluid.clock;auto bNodes=b.fluid.streams[0].nodes.size();a.Cancel();
 REQUIRE(!a.timeline.active&&!a.fluid.ready&&b.fluid.clock==bTime&&b.fluid.streams[0].nodes.size()==bNodes);
 REQUIRE(!b.Advance(.1f,{{0,0,0},{0,0,0},{}},1000));
 REQUIRE(!b.Advance(std::numeric_limits<float>::quiet_NaN(),{},1000));
 std::cout<<"PASS interleaved characters, independent settings/callbacks/end times, cancellation and invalid pose rejection.\n";
 return true;
}

bool Deferred(){
 c::volumeFluid::CollisionPath path;c::volumeFluid::CollisionQuery query;
 int count=0;c::volumeFluid::FluidImpact hit{};
 query.callback=[&](V3 from,V3,float,c::volumeFluid::FluidImpact&){
  if(++count==1)return c::volumeFluid::SweepResult::deferred;
  if(from.x!=1)return c::volumeFluid::SweepResult::miss;
  return c::volumeFluid::SweepResult::hit;
 };
 REQUIRE(!path.Sweep({1,0,0},{2,0,0},.1f,hit,query)&&path.pending);
 REQUIRE(path.Sweep({2,0,0},{3,0,0},.1f,hit,query)&&!path.pending);
 std::cout<<"PASS deferred collision preserves the entire untested crossing.\n";return true;
}

static int originalQueries=0;
bool OriginalSweep(V3 from,V3 to,float,original::volumeFluid::FluidImpact& hit){
 if(++originalQueries%5==0){original::volumeFluid::collisionDeferred=true;return false;}
 if(from.x<=20&&to.x>=20&&to.x>from.x){float t=(20-from.x)/(to.x-from.x);hit.p=from+(to-from)*t;hit.n={-1,0,0};return true;}
 return false;
}
bool CollisionParity(){
 originalQueries=0;int portableQueries=0;
 original::volumeFluid::config={};original::volumeFluid::collisionSweep=OriginalSweep;
 original::teaching::Fluid old;c::teaching::Fluid current;
 current.collisionQuery.callback=[&](V3 from,V3 to,float,c::volumeFluid::FluidImpact& hit){
  if(++portableQueries%5==0)return c::volumeFluid::SweepResult::deferred;
  if(from.x<=20&&to.x>=20&&to.x>from.x){float t=(20-from.x)/(to.x-from.x);hit.p=from+(to-from)*t;hit.n={-1,0,0};return c::volumeFluid::SweepResult::hit;}
  return c::volumeFluid::SweepResult::miss;
 };
 REQUIRE(old.Begin({0,0,100})&&current.Begin({0,0,100}));double deposited=0;
 for(int i=0;i<900;i++){
  old.Advance(1.f/60,{0,0,100},{1,0,0},{});current.Advance(1.f/60,{0,0,100},{1,0,0},{});
  REQUIRE(originalQueries==portableQueries&&old.impacts.size()==current.impacts.size());
  for(size_t j=0;j<current.impacts.size();j++){
   REQUIRE(old.impacts[j].volume==current.impacts[j].volume&&Length(old.impacts[j].p-current.impacts[j].p)<1e-6);
   deposited+=current.impacts[j].volume;
  }
  REQUIRE(old.mesh.indices==current.mesh.indices&&old.mesh.vertices.size()==current.mesh.vertices.size());
  for(size_t j=0;j<current.mesh.vertices.size();j++)REQUIRE(Length(old.mesh.vertices[j].p-current.mesh.vertices[j].p)<1e-6);
 }
 original::volumeFluid::collisionSweep=nullptr;
 REQUIRE(deposited>0&&deposited<=current.emittedVolume+.01);
 std::cout<<"PASS original/portable receiver collisions, deferred budgets, impacts and mesh parity.\n";return true;
}

bool Deposits(const char* bakePath){
 c::volumeFluid::SplatBakes bakes;REQUIRE(!bakes.Load({1,2,3}));REQUIRE(bakes.LoadFile(bakePath));
 REQUIRE(!bakes.Load({})&&bakes.Open());
 c::volumeFluid::SplatModel model;model.splatBakes=bakes;
 model.project=[](const auto& contact,V3 p,auto& result){result=contact;result.p={p.x,p.y,0};result.n={0,0,1};return true;};
 float translate=0;
 model.resolve=[&](const c::volumeFluid::FluidImpact& anchor,V3& p,V3& n){p=anchor.p+V3{translate,0,0};n=anchor.n;return true;};
 c::volumeFluid::FluidImpact impact{};impact.p={0,0,0};impact.n={0,0,1};impact.velocity={10,0,-5};impact.volume=5;
 REQUIRE(model.Add({impact},100));model.Update(3000);REQUIRE(model.marks.size()==1);
 const auto& mark=model.marks[0];double volume=0;float texel=mark.span/c::volumeFluid::splatTexture;
 for(float density:mark.density)volume+=density*texel*texel;
 REQUIRE(std::abs(volume-5)<.0001);
 auto mesh=c::BuildDepositMesh(model,3000);REQUIRE(!mesh.vertices.empty()&&!mesh.indices.empty());
 for(auto id:mesh.indices)REQUIRE(id<mesh.vertices.size());
 for(auto v:mesh.vertices)REQUIRE(std::abs(v.p.z-.027f)<1e-5&&v.alpha>=0&&v.alpha<=1);
 V3 before,n,x,y,after;REQUIRE(model.Frame(mark,before,n,x,y));translate=10;REQUIRE(model.Frame(mark,after,n,x,y));
 REQUIRE(Length(after-before-V3{10,0,0})<1e-5);
 c::volumeFluid::SplatModel other;other.splatBakes=bakes;REQUIRE(other.marks.empty());
 model.Update(20200);REQUIRE(model.marks.empty());
 REQUIRE(c::BuildDepositMesh(model,20200).indices.empty());
 std::cout<<"PASS portable baked loading, malformed input, deposition volume, moving receiver and expiry.\n";return true;
}

bool AudioCues(){
 c::teaching::AudioCues cues;std::vector<unsigned> fired;
 cues.playPhase=[&](unsigned phase){fired.push_back(phase);};
 cues.Begin();cues.Advance(0,2.4f);REQUIRE(fired.empty());cues.Advance(2.4f,2.6f);cues.Advance(2.6f,8);
 cues.Advance(0,10);REQUIRE((fired==std::vector<unsigned>{0,1}));
 cues.End();cues.Advance(0,10);REQUIRE(fired.size()==2);
 cues.Begin();cues.Advance(0,10);REQUIRE(fired.size()==4);
 c::Session session;int events=0;session.audio.playPhase=[&](unsigned){++events;};REQUIRE(session.Begin({0,0,100}));
 for(int i=0;i<500;i++)REQUIRE(session.Advance(1.f/60,{{0,0,100},{1,0,0},{}},i*16));
 REQUIRE(events==2);session.Cancel();REQUIRE(!session.audio.active);
 session.fluid.settings.threadSpacing=0;REQUIRE(!session.Begin({0,0,100})&&!session.timeline.active);
 std::cout<<"PASS original audio cue times, single delivery, restart, session integration and invalid settings.\n";return true;
}

bool PassiveParity(){
 original::volumeFluid::config={};original::volumeFluid::collisionSweep=nullptr;
 original::teaching::Fluid old;c::teaching::Fluid current;
 REQUIRE(old.BeginPassive({0,0,100})&&current.BeginPassive({0,0,100}));
 REQUIRE(old.TriggerPassiveClear()&&current.TriggerPassiveClear());
 for(int frame=0;frame<600;frame++){
  if(frame==120){REQUIRE(old.TriggerPassiveClear()&&current.TriggerPassiveClear());}
  old.Advance(1.f/60,{0,0,100},{1,0,0},{});current.Advance(1.f/60,{0,0,100},{1,0,0},{});
  REQUIRE(old.emittedVolume==current.emittedVolume&&old.mesh.indices==current.mesh.indices);
  REQUIRE(old.mesh.vertices.size()==current.mesh.vertices.size());
  for(size_t i=0;i<current.mesh.vertices.size();i++)REQUIRE(Length(old.mesh.vertices[i].p-current.mesh.vertices[i].p)<1e-6f);
 }
 REQUIRE(std::abs(current.emittedVolume-2*current.settings.dropVolume)<1e-6);
 std::cout<<"PASS original passive preliminary-flow simulation and mesh parity.\n";return true;
}

bool PhaseAndTerminal(const char* bakes){
 c::Session session;REQUIRE(session.deposits.splatBakes.LoadFile(bakes));
 session.deposits.project=[](const auto& contact,V3 p,auto& result){result=contact;result.p={p.x,p.y,0};result.n={0,0,1};return true;};
 session.fluid.collisionQuery.callback=[](V3 from,V3 to,float,c::volumeFluid::FluidImpact& hit){
  if(from.z>=0&&to.z<=0&&from.z>to.z){hit.p=from+(to-from)*(from.z/(from.z-to.z));hit.n={0,0,1};return c::volumeFluid::SweepResult::hit;}return c::volumeFluid::SweepResult::miss;
 };
 REQUIRE(session.Begin({0,0,30}));session.fluid.surfaceDeposits=true;
 bool earlyClear=false,earlyOpaque=false;
 for(unsigned frame=0;frame<1320;frame++){
  REQUIRE(session.Advance(1.f/60,{{0,0,30},{1,0,0},{}},frame*1000/60));
  if(frame<7*60){earlyClear|=session.depositedVolume[0]>0;earlyOpaque|=session.depositedVolume[1]>0;}
 }
 REQUIRE(earlyClear&&!earlyOpaque);REQUIRE(session.depositedVolume[1]>0&&session.finalPumpDepositedVolume>0&&session.finalPumpDepositedImpacts>0);
 // Coincident receiver fields retain separate transparent/opaque provenance.
 c::volumeFluid::FluidImpact clear{};clear.p={0,0,0};clear.n={0,0,1};clear.velocity={1,0,-1};clear.volume=5;clear.phase=c::volumeFluid::LiquidPhase::clear;
 auto white=clear;white.phase=c::volumeFluid::LiquidPhase::opaque;
 session.deposits.marks.clear();REQUIRE(session.deposits.Add({clear,white},23000));REQUIRE(session.deposits.marks.size()==2);
 session.deposits.Update(23001);auto mesh=c::BuildDepositMesh(session.deposits,23001);bool transparent=false,opaque=false;for(auto v:mesh.vertices){transparent|=v.phase==c::volumeFluid::LiquidPhase::clear;opaque|=v.phase==c::volumeFluid::LiquidPhase::opaque;}REQUIRE(transparent&&opaque);
 // A real queued receiver may complete after timeline20. Keep this source
 // node alive while the timeline/cues remain ended and emission stays fixed.
 session.Cancel();REQUIRE(session.Begin({0,0,30}));for(unsigned i=0;i<1201;i++)REQUIRE(session.Advance(1.f/60,{{0,0,30},{1,0,0},{}},i*1000/60));
 REQUIRE(!session.timeline.active&&session.timeline.time==20);auto emitted=session.fluid.emittedVolume;auto cues=session.audio.active;
 session.fluid.settings.gravity=0;session.fluid.streams[0].Feed(5,{0,0,1},{0,0,-1},{0,0,-5},float(session.fluid.clock),.01f,session.fluid.settings);session.fluid.surfaceDeposits=true;
 unsigned attempts=0;session.fluid.collisionQuery.callback=[&](V3,V3 to,float,c::volumeFluid::FluidImpact& hit){if(++attempts<3)return c::volumeFluid::SweepResult::deferred;hit.p=to;hit.n={0,0,1};return c::volumeFluid::SweepResult::hit;};
 for(unsigned i=0;i<12;i++)REQUIRE(session.Advance(1.f/60,{{0,0,30},{1,0,0},{}},21000+i*16));
 REQUIRE(attempts>2&&session.finalPumpDepositedVolume>0&&!session.timeline.active&&session.timeline.time==20&&session.audio.active==cues&&session.fluid.emittedVolume==emitted);
 std::cout<<"PASS clear-only preliminary phase, final-born opaque ground ledger, separate coincident phase fields and post20 deferred receiver drain without emission/cues.\n";return true;
}

int main(int argc,char** argv){if(argc!=2){std::cerr<<"Provide original splat_bakes.bin path\n";return 2;}return Parity()&&Isolation()&&Deferred()&&CollisionParity()&&Deposits(argv[1])&&AudioCues()&&PassiveParity()&&PhaseAndTerminal(argv[1])?0:1;}
