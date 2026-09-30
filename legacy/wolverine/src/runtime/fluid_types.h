#pragma once
#include <algorithm>
#include <string>
namespace volumeFluid {
template<class T> void Drop(T*& p){if(p){p->Release();p=nullptr;}}
struct Particle {V3 p;float live;V3 v;float age;};
struct F4 {float x,y,z,w;};
enum FluidImpactKind : unsigned char { FLUID_IMPACT_WORLD=0,FLUID_IMPACT_BODY=1 };
struct FluidImpact {
 V3 p,n;
 FluidImpactKind kind=FLUID_IMPACT_WORLD;
 unsigned char section=0;
 unsigned triangle=0;
 float baryU=0,baryV=0;
 void* shape=nullptr;
 V3 velocity{};
 float volume=0;
 unsigned sourceId=0,sourceTriangle=0;
 float normalSign=1;
};
typedef bool (*CollisionSweep)(V3 from,V3 to,float radius,FluidImpact& hit);
static CollisionSweep collisionSweep=nullptr;
// An exhausted query budget is not a tested miss. Keep the last checked
// position so a later sweep still includes the receiver crossing.
static bool collisionDeferred=false;
struct CollisionPath {
 V3 from{};bool pending=false;
 bool Sweep(V3 old,V3 next,float radius,FluidImpact& hit){
  if(!pending)from=old;
  collisionDeferred=false;
  bool found=collisionSweep&&collisionSweep(from,next,radius,hit);
  pending=!found&&collisionDeferred;return found;
 }
};
static constexpr int collisionCount=14;
struct Settings {
 float spacing=.32f,volume=180.f,feed=1.1f,nozzle=1.25f,viscosity=16.f,tension=12.f,lifetime=5.f;
 float pulseVolumeVariation=.35f,pulseDurationVariation=.65f,angleVariation=.55f,pulseTaper=.55f;
 float lateralWobbleDegrees=3.f;
 float pulseForceVariation=.12f;
 float gravity=98.f,speedLimit=350.f;int capacity=32768,iterations=4,viscIterations=24;
 float dropVolume=.42f,flowVariation=.3f;bool catchPlane=false;float catchDepth=45.f;
 float dropDuration=1.75f,dropHold=1.45f,dropLength=5.7f;
 bool cacheViscosity=true; // Legacy particle solver, retained only for reference tools.
 float threadSpacing=.65f,breakup=.65f;int meshSides=12;
};
static Settings config;
}
