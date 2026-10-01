// Original rest-measurement functions, independent of game and graphics SDKs.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include "../include/malemod/math.hpp"
using namespace malemod;
using std::max;using std::min;using UINT=uint32_t;
#include "../legacy/wolverine/src/runtime/morph_targets_faired.h"
#include "../legacy/wolverine/src/runtime/physics_weights.h"
#include "../legacy/wolverine/src/runtime/collar_fairing.h"
#include "../legacy/wolverine/src/runtime/pelvic_root_binding.h"
#include "../legacy/wolverine/src/runtime/suspension_weights.h"
static const UINT graftCount=2388;
static const int shaftRestSampleCount=18;
static float sliderValues[7],effectiveHangUI,preparedPelvicRampBlend,
 preparedPelvicSeamLift,preparedPelvicLateralGrowth,constraintRestLength;
static const float neutralShape[7]={1.2f,1.6f,1.59f,1.53f,30.f,-.7f,.400001f};
static const float* overallWidthTargets[3][3]={
 {morph_ow_lo_lo,morph_ow_lo_def,morph_ow_lo_hi},
 {morph_ow_def_lo,morph_ow_def_def,morph_ow_def_hi},
 {morph_ow_hi_lo,morph_ow_hi_def,morph_ow_hi_hi}};
static V3 graftDeformedPositions[graftCount],shaftRestCenters[shaftRestSampleCount];
static float graftRestFlex[graftCount],preparedPelvicRootFollow[graftCount],logicalShaftBodyRadius;
static bool shaftRestFrameReady,restFrameUsesPreviousLength;
static int physicsState;
static float SampleOverallWidthVertex(UINT,float,float);
static void SampleRestShaftFrame(float,V3&,V3&);
#include "data/wolverine-authored-shape.inc"
#include "data/wolverine-rest-frame.inc"
int main(){
 for(int test=0;;test++){
  if(std::scanf("%f %f %f %f %f %f %f %f %d %f",&sliderValues[0],&sliderValues[1],
   &sliderValues[2],&sliderValues[3],&sliderValues[4],&sliderValues[5],
   &sliderValues[6],&effectiveHangUI,&physicsState,&constraintRestLength)!=10)break;
  float growth=PelvisCollarGrowth();preparedPelvicRampBlend=Smoother01((growth-.15f)/1.f);
  preparedPelvicSeamLift=.46f+(.75f+.10f*preparedPelvicRampBlend)*growth;
  preparedPelvicLateralGrowth=(.14f+.41f*preparedPelvicRampBlend)*growth;
  // Isolate rest measurement from the already separately tested early stage:
  // consume the identical caller-supplied float32 positions as the library.
  for(unsigned i=0;i<graftCount;i++)if(std::scanf("%f %f %f",
   &graftDeformedPositions[i].x,&graftDeformedPositions[i].y,&graftDeformedPositions[i].z)!=3)return 2;
  restFrameUsesPreviousLength=false;BuildShaftRestFrame();
  for(int i=0;i<18;i++){auto p=shaftRestCenters[i];unsigned vertex=i*137;
   std::printf("%d,%d,%.9g,%.9g,%.9g,%.9g,%.9g,%d,%u,%.9g,%.9g\n",test,i,
    p.x,p.y,p.z,logicalShaftBodyRadius,constraintRestLength,restFrameUsesPreviousLength,
    vertex,graftRestFlex[vertex],preparedPelvicRootFollow[vertex]);}
 }
}
