#include <windows.h>
static DWORD canonicalFeatureClock=0;
static DWORD WINAPI CanonicalFeatureTick(){return canonicalFeatureClock;}
// The source still owns UpdateThrob; only its engine clock observation changes.
#define GetTickCount CanonicalFeatureTick
#define main ResetReferenceEntry
#include "wolverine_v2_reference.cpp"
#undef main
#undef GetTickCount
#include "canonical_feature_case.hpp"
#include <iostream>
int main(int argc,char** argv){try{
 if(argc!=2)return 2;const std::filesystem::path output(argv[1]);std::filesystem::create_directories(output);
 ResetStudyControls();graftBuffer=new CpuVertexBuffer(50915*32);graftOffset=47050*32;void* memory=nullptr;graftBuffer->Lock(0,0,&memory,0);memset(memory,0,50915*32);auto* raw=static_cast<unsigned char*>(memory);
 auto reset=[&]{PrepareMenuRetargetBodyBasis(raw);for(unsigned i=0;i<collarNormalTriangleCount*3;i++)memcpy(raw+collarNormalTriangleIndices[i]*32,collarNormalTriangleBasePositions+i*3,12);for(unsigned i=0;i<pelvisControlCount;i++)memcpy(raw+pelvisControlIndices[i]*32,pelvisControlBasePositions+i*3,12);for(unsigned i=0;i<graftCount;i++)memcpy(raw+(47050+i)*32,morph_base+i*3,12);preparedShapeReady=shaftRestFrameReady=eggRestReady=constraintSolverReady=false;paBasisSaved=false;ResetCompliantDynamics();shapeDirty=true;ApplyShape();};
 std::ofstream guides(output/"guides.f32",std::ios::binary),features(output/"features.f32",std::ios::binary);std::array<float,18> previous{};unsigned previousStyle=~0u,previousMode=0,clockReset=0;auto packet=std::make_unique<malemod::surface::canonical::Packet>();auto& p=*packet;
 for(unsigned frame=0;frame<720;frame++){
  CanonicalFeatureCase(frame,p);const auto& f=p.features;const bool dirtyFeature=clockReset!=f.resetClock||previousMode!=f.throbMode;
  if(dirtyFeature){ResetThrobClock();clockReset=f.resetClock;}previousMode=throbMode=f.throbMode;canonicalFeatureClock=f.milliseconds;teachingTimeline.active=f.teachingActive!=0;teachingTimeline.time=f.teachingTime;teaching::sequenceEnd=f.teachingEnd;
  for(unsigned i=0;i<4;i++){teachingFluid.angleGain[i]=f.angleGain[i];teachingFluid.lateralGain[i]=f.lateralGain[i];}teachingFluid.settings.lateralWobbleDegrees=f.lateralWobbleDegrees;
  if(p.garmentStyle!=clothingStyle)SetJockstrapStyle(p.garmentStyle);physicsState=int(p.controls[0]);for(unsigned i=0;i<7;i++)sliderUI[i]=p.controls[i+1];glansUI=p.controls[8];hangUI=p.controls[9];for(unsigned i=0;i<8;i++)physUI[i]=p.controls[i+10];UpdateThrob();
  if(!frame)reset();UpdateConstraintSolver(p.seconds,0,0);if(malemod::garments::UpdatePouchCoveredSurface(p.garmentStyle==1,previous!=p.controls||previousStyle!=p.garmentStyle||dirtyFeature,teachingTimeline.active,frame+1))ApplyShape();previous=p.controls;previousStyle=p.garmentStyle;
  guides.write(reinterpret_cast<const char*>(shaftNodes),sizeof(shaftNodes));
  std::array<float,27> result{throbSizePulse,throbTwitchPulse,throbAngleSizePulse,float(physicsState),effectiveShapeUI[0],effectiveShapeUI[1],effectiveShapeUI[2],effectiveGlansUI,effectiveShapeUI[3],teachingTimeline.active?effectiveHangUI:hangUI,effectiveShapeUI[4],effectiveShapeUI[5],effectiveShapeUI[6],physUI[0],physUI[1],physUI[2],physUI[3],physUI[4],physUI[5],physUI[6],physUI[7]};
  auto lip=TeachingRing(r14RingCount-6),back=TeachingRing(r14CrownRing+(r14RingCount-r14CrownRing)/2),direction=Unit(lip-back);lip=lip+direction*.06f;result[21]=lip.x;result[22]=lip.y;result[23]=lip.z;result[24]=direction.x;result[25]=direction.y;result[26]=direction.z;features.write(reinterpret_cast<const char*>(result.data()),sizeof(result));
 }
 Save(output/"anatomy.packed",nrPacked,sizeof(nrPacked));for(unsigned s=0;s<2;s++)Save(output/("body"+std::to_string(s)+".packed"),sharedBodyOutput[s],sharedBodyCount[s]*32);
 std::cout<<"720 direct source frames: clinical deformation, all throb modes, effective readouts, actual outlet, bare/clothed cadence\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
