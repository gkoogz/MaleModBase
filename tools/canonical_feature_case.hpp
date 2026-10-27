#pragma once
#include <malemod/surface/canonical_packet.hpp>
// Shared observations for independent canonical-source / IPC-worker oracles.
inline void CanonicalFeatureCase(unsigned frame,malemod::surface::canonical::Packet& p){
 p.epoch=1;p.seconds=1.f/30;p.controls.fill(50);p.controls[0]=2;p.controls[2]=frame<240?50.f:75.f;p.garmentStyle=frame>=360?1u:0u;
 p.features.milliseconds=1000+frame*1000/30;p.features.throbMode=(frame/180)%4;p.features.resetClock=frame/180;
 p.features.teachingActive=frame>=30&&frame<630;p.features.teachingTime=frame<30?0.:frame<630?double(float((frame-30)/30.f)):20.;
 p.features.angleGain={.8f,.93f,1.f,.89f};p.features.lateralGain={-.7f,.4f,-.2f,.8f};p.features.lateralWobbleDegrees=3;
}
