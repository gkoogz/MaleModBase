// Generated from the immutable active Wolverine filter. Do not hand tune.
#pragma once
#include <array>
#include <cstdint>
#include <cmath>
#include <cstring>
#include <algorithm>
#include <stdexcept>
namespace malemod::motion {
class Tracker {
 public:
 using Matrix=std::array<float,12>; // row-major affine skin delta, model space
 struct State {
bool motionTracked,motionBasisReady;float motionPitchForce,motionYawForce,motionSpinSpeed,motionPrevPosition[3],motionPrevVelocity[3],motionFilteredAccel[3],motionPrevBasis[9],motionPrevAngularVelocity[3];std::uint32_t motionLastTick,motionLastCaptureTick;std::int32_t motionSamples;int motionWarmupSamples,motionQuietFrames;
float motionPelvisMatrix[12],motionLeftThighMatrix[12],motionRightThighMatrix[12];
bool motionCollisionBonesReady;
 };
 void Reset(){state_=State{};}
 const State& Read()const noexcept{return state_;}
 // Force-only input never advertises unavailable thigh skin matrices.
 void TrackPoseOnly(std::uint32_t now,const Matrix& pelvis){
  const Matrix unavailable{};Track(now,pelvis,unavailable,unavailable);state_.motionCollisionBonesReady=false;
 }
 void Track(std::uint32_t now,const Matrix& pelvis,const Matrix& left,const Matrix& right){
  for(const auto* matrix:{&pelvis,&left,&right})for(float x:*matrix)if(!std::isfinite(x))throw std::invalid_argument("Non-finite motion matrix");
  const auto* bone=pelvis.data();const auto* leftThigh=left.data();const auto* rightThigh=right.data();
  using std::min;using std::max;

  memcpy(state_.motionPelvisMatrix,bone,12*sizeof(float));memcpy(state_.motionLeftThighMatrix,leftThigh,12*sizeof(float));memcpy(state_.motionRightThighMatrix,rightThigh,12*sizeof(float));state_.motionCollisionBonesReady=true;
  float world[3]={bone[3],bone[7],bone[11]},basis[9]{};
  for(int axis=0;axis<3;axis++){for(int a=0;a<3;a++)basis[axis*3+a]=bone[axis*4+a];Normalize3(&basis[axis*3]);}
  state_.motionLastCaptureTick=now;if(!state_.motionLastTick){memcpy(state_.motionPrevPosition,world,12);memcpy(state_.motionPrevBasis,basis,sizeof(basis));state_.motionBasisReady=true;state_.motionLastTick=now;state_.motionWarmupSamples=state_.motionQuietFrames=0;state_.motionTracked=false;return;}float dt=(now-state_.motionLastTick)*.001f;if(dt<.008f)return;state_.motionLastTick=now;if(dt>.50f){memcpy(state_.motionPrevPosition,world,12);memcpy(state_.motionPrevBasis,basis,sizeof(basis));memset(state_.motionPrevVelocity,0,12);memset(state_.motionFilteredAccel,0,12);memset(state_.motionPrevAngularVelocity,0,12);state_.motionSpinSpeed=0;state_.motionPitchForce=state_.motionYawForce=0;state_.motionBasisReady=true;state_.motionWarmupSamples=0;state_.motionQuietFrames=0;state_.motionTracked=false;return;}
  float delta[3]={world[0]-state_.motionPrevPosition[0],world[1]-state_.motionPrevPosition[1],world[2]-state_.motionPrevPosition[2]};memcpy(state_.motionPrevPosition,world,12);float distance=sqrtf(delta[0]*delta[0]+delta[1]*delta[1]+delta[2]*delta[2]);float teleportDistance=max(8.f,dt*120.f);if(distance>teleportDistance){memset(state_.motionPrevVelocity,0,12);memset(state_.motionFilteredAccel,0,12);memset(state_.motionPrevAngularVelocity,0,12);state_.motionSpinSpeed=state_.motionPitchForce=state_.motionYawForce=0;state_.motionTracked=false;state_.motionWarmupSamples=0;state_.motionQuietFrames=0;memcpy(state_.motionPrevBasis,basis,sizeof(basis));return;}
  // A critically-smoothed velocity/acceleration estimate avoids the severe
  // noise amplification caused by taking two raw frame-to-frame derivatives.
  float rawVelocity[3]={delta[0]/dt,delta[1]/dt,delta[2]/dt},accel[3]{};
  float velocityAlpha=1.f-expf(-10.f*dt),accelAlpha=1.f-expf(-8.f*dt);
  for(int axis=0;axis<3;axis++){float oldVelocity=state_.motionPrevVelocity[axis];state_.motionPrevVelocity[axis]+=velocityAlpha*(rawVelocity[axis]-oldVelocity);float rawAccel=(state_.motionPrevVelocity[axis]-oldVelocity)/dt;state_.motionFilteredAccel[axis]+=accelAlpha*(rawAccel-state_.motionFilteredAccel[axis]);accel[axis]=state_.motionFilteredAccel[axis];}
  // Bone translation is already expressed in character/model space.
  float localAccel[3]={accel[0],accel[1],accel[2]};
  float worldOmega[3]{},localOmega[3]{};if(state_.motionBasisReady){for(int axis=0;axis<3;axis++){const float* a=&state_.motionPrevBasis[axis*3];const float* b=&basis[axis*3];worldOmega[0]+=(a[1]*b[2]-a[2]*b[1])*.5f/dt;worldOmega[1]+=(a[2]*b[0]-a[0]*b[2])*.5f/dt;worldOmega[2]+=(a[0]*b[1]-a[1]*b[0])*.5f/dt;}float omegaAlpha=1.f-expf(-12.f*dt);for(int axis=0;axis<3;axis++){float raw=worldOmega[0]*basis[axis*3]+worldOmega[1]*basis[axis*3+1]+worldOmega[2]*basis[axis*3+2];state_.motionPrevAngularVelocity[axis]+=omegaAlpha*(raw-state_.motionPrevAngularVelocity[axis]);localOmega[axis]=state_.motionPrevAngularVelocity[axis];}}
  memcpy(state_.motionPrevBasis,basis,sizeof(basis));state_.motionBasisReady=true;
  if(state_.motionWarmupSamples<3){state_.motionWarmupSamples++;state_.motionPitchForce=state_.motionYawForce=0;state_.motionTracked=false;return;}
  for(int axis=0;axis<3;axis++){localAccel[axis]=max(-800.f,min(800.f,localAccel[axis]));localOmega[axis]=max(-10.f,min(10.f,localOmega[axis]));}
  // Continuous, time-based filtering: idle movement must not trigger a
  // dead zone or erase the momentum of already moving tissue.
  float forceAlpha=1.f-expf(-14.f*dt);
  state_.motionSpinSpeed+=(localOmega[2]-state_.motionSpinSpeed)*forceAlpha;
  float pitch=max(-2.5f,min(2.5f,-localAccel[2]*.0052f-localAccel[0]*.0033f-localOmega[1]*.72f));
  float yaw=max(-2.5f,min(2.5f,-localAccel[1]*.0048f-localOmega[2]*1.40f));
  state_.motionQuietFrames=fabsf(pitch)+fabsf(yaw)>.035f?0:state_.motionQuietFrames+1;
  state_.motionPitchForce+=(pitch-state_.motionPitchForce)*forceAlpha;
  state_.motionYawForce+=(yaw-state_.motionYawForce)*forceAlpha;
  state_.motionTracked=true;++state_.motionSamples;
 }
 private:
 State state_{};
static void Normalize3(float* v){float n=sqrtf(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]);if(n>1e-6f){v[0]/=n;v[1]/=n;v[2]/=n;}}
};
}
