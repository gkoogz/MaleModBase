// Exercises original source functions without game/graphics SDKs.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
using std::max; using std::min; using UINT=uint32_t;
struct V3 {float x,y,z;};
#include "../legacy/wolverine/src/runtime/morph_targets_faired.h"
#include "../legacy/wolverine/src/runtime/physics_weights.h"
#include "../legacy/wolverine/src/runtime/collar_fairing.h"
#include "../legacy/wolverine/src/runtime/pelvic_root_binding.h"
#include "../legacy/wolverine/src/runtime/suspension_weights.h"
static const UINT graftCount=2388;
static float sliderValues[7],effectiveHangUI,preparedPelvicRampBlend,
 preparedPelvicSeamLift,preparedPelvicLateralGrowth;
static const float neutralShape[7]={1.2f,1.6f,1.59f,1.53f,30.f,-.7f,.400001f};
static const float* overallWidthTargets[3][3]={
 {morph_ow_lo_lo,morph_ow_lo_def,morph_ow_lo_hi},
 {morph_ow_def_lo,morph_ow_def_def,morph_ow_def_hi},
 {morph_ow_hi_lo,morph_ow_hi_def,morph_ow_hi_hi}};
static V3 graftDeformedPositions[graftCount];
static float SampleOverallWidthVertex(UINT,float,float);
#include "data/wolverine-authored-shape.inc"
int main(){
 for(int test=0;;test++){
  if(std::scanf("%f %f %f %f %f %f %f %f", &sliderValues[0],&sliderValues[1],
   &sliderValues[2],&sliderValues[3],&sliderValues[4],&sliderValues[5],
   &sliderValues[6],&effectiveHangUI)!=8)break;
  float collarGrowth=PelvisCollarGrowth();
  preparedPelvicRampBlend=Smoother01((collarGrowth-.15f)/1.0f);
  preparedPelvicSeamLift=.46f+(.75f+.10f*preparedPelvicRampBlend)*collarGrowth;
  preparedPelvicLateralGrowth=(.14f+.41f*preparedPelvicRampBlend)*collarGrowth;
  BuildEarlyAuthoredStage();
  for(unsigned i=0;i<graftCount;i+=37){auto v=graftDeformedPositions[i];
   std::printf("%d,%u,%.9g,%.9g,%.9g,%.9g,%.9g\n",test,i,v.x,v.y,v.z,
    collarGrowth,HangOffset()*suspensionWeight[i]);}
 }
}
