// Reference-only target. The shared library itself has no graphics SDK.
#include "../legacy/wolverine/src/runtime/d3d9_proxy.cpp"
#include "../legacy/wolverine/tools/r14/cpu_buffer.h"
#include <malemod/surface/runtime.hpp>
using malemod::surface::Controls;
static void SourceControls(const Controls& c){
 physicsState=int(c.values[0]);const unsigned ids[7]={1,2,3,5,7,8,9};
 for(unsigned i=0;i<7;i++)sliderUI[i]=c.values[ids[i]];
 glansUI=c.values[4];hangUI=c.values[6];for(unsigned i=0;i<8;i++)physUI[i]=c.values[i+10];ApplyControlMapping();
}
static void Seed(){
 graftBuffer=new CpuVertexBuffer(50915*32);graftOffset=47050*32;void* raw=nullptr;graftBuffer->Lock(0,0,&raw,0);memset(raw,0,50915*32);
 for(UINT i=0;i<collarNormalTriangleCount*3;i++)memcpy((char*)raw+collarNormalTriangleIndices[i]*32,collarNormalTriangleBasePositions+i*3,12);
 for(UINT i=0;i<pelvisControlCount;i++)memcpy((char*)raw+pelvisControlIndices[i]*32,pelvisControlBasePositions+i*3,12);
 for(UINT i=0;i<graftCount;i++)memcpy((char*)raw+(47050+i)*32,morph_base+i*3,12);
 graftBuffer->Unlock();ApplyShape();
}
int main(){
 teachingFluid.SetVariationSeed(123);if(!teachingFluid.Prepare())return 4;
 ResetStudyControls();Controls controls;SourceControls(controls);Seed();malemod::surface::Session session(controls);
 double maximum=0,bodyMaximum=0;unsigned compared=0;
 for(unsigned frame=0;frame<270;frame++){
  malemod::surface::Frame clinicalFrame;
  clinicalFrame.clinical.throbMode=frame<120?frame/30:0;
  clinicalFrame.clinical.sizeTime=fmodf(frame/12.f,3.f);
  clinicalFrame.clinical.twitchTime=frame/12.f;while(clinicalFrame.clinical.twitchTime>=5.75f)clinicalFrame.clinical.twitchTime-=4.6f;
  clinicalFrame.clinical.active=frame>=120&&frame<240;clinicalFrame.clinical.time=frame>=120?(frame-120)/6.f:0;
  for(unsigned i=0;i<4;i++)clinicalFrame.clinical.angleGain[i]=teachingFluid.angleGain[i];
  clinicalFrame.clinical.lateralWobbleDegrees=teachingFluid.settings.lateralWobbleDegrees;
  for(unsigned i=0;i<4;i++)clinicalFrame.clinical.lateralGain[i]=teachingFluid.lateralGain[i];
  teachingTimeline.active=clinicalFrame.clinical.active;teachingTimeline.time=clinicalFrame.clinical.time;
  throbMode=int(clinicalFrame.clinical.throbMode);throbSizePulse=ThrobEnvelope(clinicalFrame.clinical.sizeTime,.20f,1.05f,false);throbTwitchPulse=ThrobEnvelope(clinicalFrame.clinical.twitchTime,1.15f,4.6f,true);throbAngleSizePulse=ThrobEnvelope(clinicalFrame.clinical.twitchTime,1.15f,1.05f,true);SourceControls(controls);
  malemod::surface::Frame f=clinicalFrame;f.pitchForce=.2f*sinf(frame*.15f);f.yawForce=.15f*cosf(frame*.1f);
  float sway=3.f*sinf(frame*.04f);
  f.thighEndpoints=std::array<malemod::surface::Point,4>{{{2,-7.8f,79},{1+sway,-8.2f,43},{2,7.8f,79},{1-sway,8.2f,43}}};
  collisionCapsuleOverride=true;overrideLeftA={2,-7.8f,79};overrideLeftB={1+sway,-8.2f,43};overrideRightA={2,7.8f,79};overrideRightB={1-sway,8.2f,43};
  UpdateConstraintSolver(f.seconds,f.pitchForce,f.yawForce);ApplyShape();session.Step(f);auto out=session.Read();
  if(out.anatomy.positions.size()!=nrCount)return 2;
  for(unsigned i=0;i<nrCount;i++){V3 p;memcpy(&p,nrPacked+i*32,12);const auto q=out.anatomy.positions[i];double error=Length(p-V3{q.x,q.y,q.z});maximum=max(maximum,error);++compared;}
  for(unsigned s=0;s<2;s++)for(unsigned i=0;i<sharedBodyCount[s];i++){V3 p;memcpy(&p,sharedBodyOutput[s]+i*32,12);const auto q=out.body[s].positions[i];bodyMaximum=max(bodyMaximum,double(Length(p-V3{q.x,q.y,q.z})));}
  if(maximum>1e-4||bodyMaximum>1e-4){printf("FAIL frame=%u anatomy=%.9g body=%.9g\n",frame,maximum,bodyMaximum);return 3;}
 }
 printf("PASS dynamic trace: frames=270 vertices=%u max=%.9g bodyMax=%.9g\n",compared,maximum,bodyMaximum);
 graftBuffer->Release();graftBuffer=nullptr;
}
