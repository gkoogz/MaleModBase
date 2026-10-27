#pragma once
#include <malemod/clinical/session.hpp>
#include <malemod/clinical/deposit_mesh.hpp>
#include <malemod/surface/canonical_features.hpp>
#include <stdexcept>

namespace malemod::clinical {
// Canonical runtime orchestration. This deliberately retains Wolverine's
// clear-on-end policy; Session::Advance also supports a newer drain policy.
// The engine supplies the live outlet, actor translation, collision receivers,
// audio playback and presentation. No guessed world plane is created here.
class CanonicalRuntime {
 Session session_;
 teaching::PassiveThrobGate passiveGate_;
 unsigned throb_=0,reset_=0;
 bool idle_=false,focused_=true,visible_=false,startRequested_=false;
 std::uint32_t lastTick_=0,seenTick_=0,passiveTick_=0,seed_=0;
 std::uint32_t passiveRetryAt_=0;
 bool passiveRetryPending_=false;
 std::string lastError_;
 std::uint64_t errorSerial_=0;
 bool tickValid_=false,passiveTickValid_=false;
 double fluidTime_=0;
 std::function<void()> stopAudio_;
 void NoteFailure(const std::string& fallback){lastError_=session_.fluid.error.empty()?fallback:session_.fluid.error;session_.fluid.error=lastError_;++errorSerial_;}
 void Recovered(){lastError_.clear();session_.fluid.error.clear();}
 void CancelFailure(const std::string& reason){const auto error=session_.fluid.error;Cancel();session_.fluid.error=error;NoteFailure(reason);}
public:
 Session& Engine(){return session_;}
 const Session& Engine()const{return session_;}
 unsigned Throb()const{return throb_;}
 bool IdleChatter()const{return idle_;}
 bool Active()const{return session_.timeline.active;}
 float Seconds()const{return float(session_.timeline.time);}
 bool Visible()const{return visible_;}
 unsigned ClockReset()const{return reset_;}
 const std::string& LastError()const{return lastError_;}
 std::uint64_t ErrorSerial()const{return errorSerial_;}
 bool PassiveRetryPending()const{return passiveRetryPending_;}
 std::uint32_t PassiveRetryAt()const{return passiveRetryAt_;}
 float FluidElapsedSeconds(std::uint32_t now)const{
  if(!focused_||!visible_)return 0;
  if(Active())return teaching::BoundedFrameAdvance(float(session_.timeline.time-fluidTime_));
  if(!throb_||!session_.fluid.ready||!session_.fluid.passiveMode||!passiveTickValid_)return 0;
  return float((std::min)(100u,std::uint32_t(now-passiveTick_)))*.001f;
 }
 void Suspend(std::uint32_t now){lastTick_=now;tickValid_=false;passiveTick_=now;passiveTickValid_=false;++reset_;if(stopAudio_)stopAudio_();}
 void SetAudio(std::function<void(unsigned)> play,std::function<void()> stop){session_.audio.playPhase=std::move(play);stopAudio_=std::move(stop);}
 void SetThrob(unsigned mode){if(mode>3)throw std::invalid_argument("Throb mode must be 0..3");Cancel();if(throb_!=mode){throb_=mode;++reset_;}}
 void SetIdleChatter(bool enabled){Cancel();idle_=enabled;}
 void Reset(){Cancel();throb_=0;idle_=false;++reset_;}
 void Cancel(){session_.timeline.Cancel();session_.audio.End();if(stopAudio_)stopAudio_();session_.fluid.Clear();session_.depositedVolume={};session_.depositedImpacts={};session_.finalPumpDepositedVolume=0;session_.finalPumpDepositedImpacts=0;passiveGate_.Reset();passiveTickValid_=false;passiveRetryPending_=false;passiveRetryAt_=0;fluidTime_=0;startRequested_=false;}
 void Toggle(std::uint32_t seed){if(Active()||startRequested_){Cancel();return;}seed_=seed;startRequested_=true;}
 // Called once before the source worker. Both clocks use the same original
 // unsigned GetTickCount interval, including wrapping subtraction and .1s cap.
 surface::canonical::Features Before(std::uint32_t now,bool focused,bool visible){
  focused_=focused;visible_=visible;if(visible)seenTick_=now;
  const float dt=tickValid_?float((std::min)(100u,std::uint32_t(now-lastTick_)))*.001f:0.f;
  lastTick_=now;tickValid_=true;
  if(!focused){Cancel();++reset_;}
  else if((Active()||session_.fluid.passiveMode)&&std::uint32_t(now-seenTick_)>3000u){Cancel();++reset_;}
  // Source J admission is an edge while a current body exists. A rejected
  // invisible edge must not become a deferred start when the body returns.
  if(startRequested_&&!visible)startRequested_=false;
  if(startRequested_&&focused&&visible){
   Cancel();session_.fluid.SetVariationSeed(seed_);session_.fluid.settings.catchPlane=false;
   if(session_.fluid.Prepare()){Recovered();session_.timeline.endTime=session_.fluid.endTime;session_.timeline.Start();session_.audio.Begin();}
   else NoteFailure("Clinical sequence preparation failed.");
  }else if(Active()){
   const float previous=float(session_.timeline.time);session_.timeline.Advance(dt);session_.audio.Advance(previous,float(session_.timeline.time));
   if(!Active()){session_.audio.End();session_.fluid.Clear();fluidTime_=0;}
  }
  surface::canonical::Features out;out.throbMode=focused&&visible?throb_:0;out.milliseconds=now;out.resetClock=reset_;
  out.teachingActive=Active()?1u:0u;out.teachingTime=session_.timeline.time;out.teachingEnd=session_.timeline.endTime;
  for(unsigned i=0;i<4;i++){out.angleGain[i]=session_.fluid.angleGain[i];out.lateralGain[i]=session_.fluid.lateralGain[i];}
  out.lateralWobbleDegrees=session_.fluid.settings.lateralWobbleDegrees;return out;
 }
 // The pose uses persistent world coordinates in canonical source units (cm),
 // not camera-relative coordinates. Velocity is actor-root velocity only.
 bool After(const surface::canonical::FeatureOutput& output,const NozzlePose& pose,std::uint32_t now){
  if(!focused_||!visible_){passiveTick_=now;passiveTickValid_=false;fluidTime_=session_.timeline.time;return true;}
  if(!Active()&&!throb_){if(session_.fluid.passiveMode)session_.fluid.Clear();passiveGate_.Reset();passiveTickValid_=false;passiveRetryPending_=false;passiveRetryAt_=0;session_.deposits.Update(now);return true;}
  if(!output.nozzleValid){session_.fluid.error="Clinical emitter transform unavailable.";CancelFailure("Clinical emitter transform unavailable.");return true;}
  if(!Session::Finite(pose.position)||!Session::Finite(pose.direction)||Length(pose.direction)<1e-6f||!Session::Finite(pose.inheritedVelocity)){session_.fluid.error="Clinical emitter pose must be finite with a nonzero direction.";CancelFailure("Invalid clinical emitter.");return true;}
  auto& fluid=session_.fluid;fluid.settings.catchPlane=false;
  // Canonical finite relocation is a normal cancellation. It must not disable
  // unrelated anatomy rendering or turn a fast-travel load into a fatal fault.
  if(fluid.ready&&Length(pose.position-fluid.lastTip)>100.f){fluid.error="Clinical sequence cancelled after finite emitter relocation.";CancelFailure("Emitter relocated.");return true;}
  float dt=0;
  if(!Active()){
   if(!throb_){if(fluid.passiveMode)fluid.Clear();passiveGate_.Reset();passiveTickValid_=false;passiveRetryPending_=false;passiveRetryAt_=0;session_.deposits.Update(now);return true;}
   if(!fluid.ready||!fluid.passiveMode){
    // Exact source renderer admission policy: rejected random preparation
    // clears this fluid only, waits one second, then samples a fresh seed.
    if(passiveRetryPending_&&std::int32_t(now-passiveRetryAt_)<0)return true;
    if(fluid.ready)fluid.Clear();fluid.SetVariationSeed(now);
    if(!fluid.BeginPassive(pose.position)){NoteFailure("Cannot start passive clear fluid.");fluid.Clear();passiveTickValid_=false;passiveRetryAt_=now+1000;passiveRetryPending_=true;passiveGate_.Reset();return true;}
    Recovered();fluid.lastDir=Unit(pose.direction);passiveTick_=now;passiveTickValid_=true;passiveRetryPending_=false;passiveRetryAt_=0;
   }
   if(passiveGate_.Update(output.sizePulse)&&!fluid.TriggerPassiveClear()){fluid.error="Passive clear-strand pool is full; skipped one throb.";NoteFailure("Passive clear-strand pool is full.");}
   dt=passiveTickValid_?float((std::min)(100u,std::uint32_t(now-passiveTick_)))*.001f:0;
   passiveTick_=now;passiveTickValid_=true;
  }else{
   if(!fluid.ready||fluid.passiveMode){if(!fluid.Begin(pose.position)){CancelFailure("Clinical fluid initialization failed.");return true;}Recovered();fluid.lastDir=Unit(pose.direction);}
   dt=float(session_.timeline.time-fluidTime_);fluidTime_=session_.timeline.time;
  }
  fluid.surfaceDeposits=session_.deposits.splatBakes.Open();
  if(dt>0)fluid.Advance(dt,pose.position,Unit(pose.direction),pose.inheritedVelocity);
  if(!fluid.ready){if(Active())CancelFailure("Clinical fluid rejected its emitter.");else{NoteFailure("Passive fluid stopped.");passiveGate_.Reset();passiveTickValid_=false;}return true;}
  session_.DepositImpacts(now);session_.deposits.Update(now);return true;
 }
};
}
