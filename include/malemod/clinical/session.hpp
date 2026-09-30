#pragma once
#include "teaching_sequence.h"
#include "teaching_volume.h"
#include "fluid_splat_model.h"
#include "teaching_audio.h"

namespace malemod::clinical {
struct NozzlePose { V3 position{},direction{1,0,0},inheritedVelocity{}; };

// One Session per character. Callbacks are synchronous and belong to the adapter.
struct Session {
 teaching::Timeline timeline;
 teaching::Fluid fluid;
 teaching::AudioCues audio;
 volumeFluid::SplatModel deposits;
 static bool Finite(V3 v){return std::isfinite(v.x)&&std::isfinite(v.y)&&std::isfinite(v.z);}
 bool ValidSettings() const {
  const auto& s=fluid.settings;
  for(float value:{s.spacing,s.volume,s.feed,s.nozzle,s.viscosity,s.tension,s.lifetime,
      s.pulseVolumeVariation,s.pulseDurationVariation,s.angleVariation,s.pulseTaper,
      s.lateralWobbleDegrees,s.pulseForceVariation,s.gravity,s.speedLimit,s.dropVolume,
      s.flowVariation,s.catchDepth,s.dropDuration,s.dropHold,s.dropLength,s.threadSpacing,s.breakup})
   if(!std::isfinite(value))return false;
  return s.spacing>0&&s.nozzle>0&&s.lifetime>0&&s.speedLimit>0&&s.dropDuration>0&&s.dropHold>=0&&s.dropLength>0&&s.threadSpacing>0&&s.dropVolume>=0&&s.tension>=0;
 }
 bool Begin(V3 position){
  timeline.Cancel();audio.End();
  if(!Finite(position)){fluid.error="Nozzle position must be finite.";return false;}
  if(!ValidSettings()){fluid.error="Simulation settings must be finite with positive spacing, durations and speed limits.";return false;}
  if(!fluid.Begin(position))return false;
  timeline.endTime=fluid.endTime;timeline.Start();audio.Begin();return true;
 }
 void Cancel(){timeline.Cancel();audio.End();fluid.Clear();deposits.marks.clear();}
 bool Advance(float dt,const NozzlePose& pose,std::uint32_t milliseconds){
  if(!std::isfinite(dt)||dt<0||!Finite(pose.position)||!Finite(pose.direction)||Length(pose.direction)<1e-6f||!Finite(pose.inheritedVelocity)){
   fluid.error="Elapsed time and nozzle pose must be finite; direction must be nonzero.";return false;
  }
  bool active=timeline.active;float previous=float(timeline.time);timeline.Advance(dt);
  if(active)audio.Advance(previous,float(timeline.time));
  if(!timeline.active)audio.End();
  if(active){
   fluid.Advance(dt,pose.position,Unit(pose.direction),pose.inheritedVelocity);
   if(deposits.splatBakes.Open())deposits.Add(fluid.impacts,milliseconds);
  }
  deposits.Update(milliseconds);return true;
 }
};
}
