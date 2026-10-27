#include <malemod/clinical/canonical_runtime.hpp>
#include <malemod/clinical/canonical_settings.hpp>
#include <iostream>
#include <sstream>
#include <limits>
namespace original {
using malemod::V3;using malemod::Dot;using malemod::Cross;using malemod::Length;using malemod::Unit;
using std::min;using std::max;
#include "../legacy/wolverine/src/runtime/teaching_sequence.h"
#include "../legacy/wolverine/src/runtime/teaching_volume.h"
}
namespace c=malemod::clinical;
#define REQUIRE(x) do{if(!(x)){std::cerr<<"FAIL "<<__LINE__<<": "<<#x<<'\n';return 1;}}while(0)
int main(){
 {
  c::CanonicalRuntime clock;malemod::surface::canonical::FeatureOutput outlet;outlet.nozzleValid=1;c::NozzlePose pose{{0,0,100},{1,0,0},{}};
  clock.Toggle(1);clock.Before(1000,true,false);REQUIRE(!clock.Active());clock.Before(1100,true,true);REQUIRE(!clock.Active());
  clock.Toggle(1);clock.Before(1200,true,true);REQUIRE(clock.Active()&&clock.FluidElapsedSeconds(1200)==0);REQUIRE(clock.After(outlet,pose,1200));
  clock.Before(1500,true,true);REQUIRE(clock.FluidElapsedSeconds(1500)==.1f);REQUIRE(clock.After(outlet,pose,1500));clock.Before(1533,true,true);REQUIRE(std::abs(clock.FluidElapsedSeconds(1533)-.033f)<1e-7f);
  clock.SetThrob(2);clock.Before(2000,true,true);REQUIRE(clock.FluidElapsedSeconds(2000)==0&&clock.After(outlet,pose,2000));clock.Before(2300,true,true);REQUIRE(clock.FluidElapsedSeconds(2300)==.1f);
  std::cout<<"PASS invisibleJedge discarded and active/passive fluid clocks retain original100msboundedhitch independently of SDKphysicsdt.\n";
 }
 {
  c::volumeFluid::Settings settings;std::string error;std::istringstream good("[Fluid]\nVolume=210\nDuration=1.3\nMeshSides=13.9\nCatchPlane=1\n");
  REQUIRE(c::ReadCanonicalFluidSettings(good,settings,error)&&settings.volume==210&&settings.feed==1.3f&&settings.meshSides==13&&settings.catchPlane);
  std::istringstream invalid("[Fluid]\nVolume=nan\n");REQUIRE(!c::ReadCanonicalFluidSettings(invalid,settings,error)&&settings.volume==210);
 }
 {
  // The source intentionally rejects some random pulse preparations. The
  // rejection belongs to fluid presentation, never to anatomy service.
  unsigned rejectedSeed=0;
  for(unsigned seed=1;seed<10000;seed++){
   c::teaching::Fluid rejected,retry;rejected.SetVariationSeed(seed);retry.SetVariationSeed(seed+1000);
   if(!rejected.BeginPassive({0,0,100})&&retry.BeginPassive({0,0,100})){rejectedSeed=seed;break;}
  }
  REQUIRE(rejectedSeed!=0);
  original::volumeFluid::config={};original::teaching::Fluid originalRejected;originalRejected.SetVariationSeed(rejectedSeed);REQUIRE(!originalRejected.BeginPassive({0,0,100}));
  c::CanonicalRuntime retry;retry.SetThrob(2);malemod::surface::canonical::FeatureOutput outlet;outlet.nozzleValid=1;outlet.sizePulse=.6f;c::NozzlePose pose{{0,0,100},{1,0,0},{}};
  auto feature=retry.Before(rejectedSeed,true,true);REQUIRE(feature.throbMode==2&&retry.After(outlet,pose,rejectedSeed));
  REQUIRE(!retry.Engine().fluid.ready&&retry.PassiveRetryPending()&&retry.PassiveRetryAt()==rejectedSeed+1000&&retry.ErrorSerial()==1&&retry.LastError()==originalRejected.error);
  const auto error=retry.LastError();
  for(unsigned frame=1;frame<60;frame++){const auto now=rejectedSeed+frame*16;feature=retry.Before(now,true,true);REQUIRE(feature.throbMode==2&&retry.After(outlet,pose,now));REQUIRE(retry.ErrorSerial()==1&&!retry.Engine().fluid.ready&&retry.LastError()==error);}
  retry.Before(rejectedSeed+1000,true,true);REQUIRE(retry.After(outlet,pose,rejectedSeed+1000)&&retry.Engine().fluid.ready&&retry.Engine().fluid.passiveMode&&!retry.PassiveRetryPending()&&retry.LastError().empty());
  c::CanonicalRuntime active;active.Toggle(1);active.Before(1000,true,true);REQUIRE(active.Active());
  active.Engine().fluid.settings.volume=4000;active.Engine().fluid.settings.nozzle=.15f;
  REQUIRE(active.After(outlet,pose,1000)&&!active.Active()&&!active.Engine().fluid.ready&&!active.LastError().empty());
  c::CanonicalRuntime prepare;prepare.Engine().fluid.settings.volume=4000;prepare.Engine().fluid.settings.nozzle=.15f;prepare.Toggle(1);feature=prepare.Before(1000,true,true);REQUIRE(!feature.teachingActive&&!prepare.Active()&&!prepare.LastError().empty());
  c::CanonicalRuntime invalid;invalid.Toggle(1);invalid.Before(1000,true,true);pose.position.x=std::numeric_limits<float>::quiet_NaN();REQUIRE(invalid.After(outlet,pose,1000)&&!invalid.Active()&&invalid.LastError().find("finite")!=std::string::npos);
  c::CanonicalRuntime wrap;wrap.SetThrob(2);wrap.Engine().fluid.settings.volume=4000;wrap.Engine().fluid.settings.nozzle=.15f;pose.position.x=0;wrap.Before(0xffffff00u,true,true);REQUIRE(wrap.After(outlet,pose,0xffffff00u)&&wrap.ErrorSerial()==1);wrap.Before(244,true,true);REQUIRE(wrap.After(outlet,pose,244)&&wrap.ErrorSerial()==1);wrap.Before(744,true,true);REQUIRE(wrap.After(outlet,pose,744)&&wrap.ErrorSerial()==2);
  std::cout<<"PASS original rejected random seed="<<rejectedSeed<<", nonfatal passive one-second retry/recovery, no premature retry, wrapped clock, active preparation cancellation and invalid emitter isolation.\n";
 }
 for(unsigned fps:{15u,30u,60u,120u}){
  c::CanonicalRuntime runtime;unsigned cues[2]{};runtime.SetAudio([&](unsigned phase){++cues[phase];},[]{});
  runtime.Toggle(0x12345678);original::volumeFluid::config={};original::volumeFluid::collisionSweep=nullptr;original::teaching::Fluid reference;reference.SetVariationSeed(0x12345678);
  const malemod::V3 tip{0,0,100},direction=malemod::Unit({1,0,.1f});REQUIRE(reference.Begin(tip));
  original::teaching::Timeline timeline;timeline.Start();
  malemod::surface::canonical::FeatureOutput outlet;outlet.nozzleValid=1;
  c::NozzlePose pose{tip,direction,{}};std::uint32_t previous=1000;
  for(unsigned frame=0;frame<fps*22;frame++){
   const auto now=1000u+std::uint32_t(frame*1000u/fps);auto feature=runtime.Before(now,true,true);
   const float dt=float((std::min)(100u,now-previous))*.001f;previous=now;
   if(frame)timeline.Advance(dt);
   REQUIRE(feature.teachingActive==unsigned(timeline.active)&&feature.teachingTime==timeline.time);
   REQUIRE(runtime.After(outlet,pose,now));
   if(timeline.active){
    if(dt>0)reference.Advance(dt,tip,direction,{});
    const auto& fluid=runtime.Engine().fluid;
    REQUIRE(fluid.clock==reference.clock&&fluid.steps==reference.steps&&fluid.emittedVolume==reference.emittedVolume);
    REQUIRE(fluid.mesh.indices==reference.mesh.indices&&fluid.mesh.vertices.size()==reference.mesh.vertices.size());
    for(unsigned i=0;i<fluid.mesh.vertices.size();i++)REQUIRE(malemod::Length(fluid.mesh.vertices[i].p-reference.mesh.vertices[i].p)<1e-6f);
   }else REQUIRE(!runtime.Engine().fluid.ready&&runtime.Engine().fluid.mesh.indices.empty());
  }
  REQUIRE(cues[0]==1&&cues[1]==1);
  runtime.SetThrob(3);REQUIRE(runtime.Throb()==3&&!runtime.Active());runtime.Before(previous+1,true,true);outlet.sizePulse=.6f;REQUIRE(runtime.After(outlet,pose,previous+1));
  REQUIRE(runtime.Engine().fluid.passiveMode&&runtime.Engine().fluid.clearScheduled[0]);
  auto teleported=pose;teleported.position.x+=101;REQUIRE(runtime.After(outlet,teleported,previous+1)&&!runtime.Engine().fluid.ready&&!runtime.Active());
  runtime.Before(previous+2,false,true);REQUIRE(!runtime.Active()&&!runtime.Engine().fluid.ready);runtime.Reset();REQUIRE(runtime.Throb()==0&&!runtime.IdleChatter());
  runtime.Toggle(1);runtime.Before(previous+10,true,true);runtime.Before(previous+60,true,true);const auto pausedTime=runtime.Seconds();runtime.Suspend(previous+10000);runtime.Before(previous+20000,true,true);REQUIRE(runtime.Active()&&runtime.Seconds()==pausedTime);
 }
 std::cout<<"PASS canonical clinical orchestration, source CPU mesh parity at 15/30/60/120 Hz, exact cues, passive trigger, focus cancellation and atomic INI validation.\n";
}
