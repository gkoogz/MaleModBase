// Original source functions on identical caller-supplied geometry/rest frames.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include "../include/malemod/math.hpp"
using namespace malemod;using std::min;using std::max;using UINT=uint32_t;
#include "../legacy/wolverine/src/runtime/physics_weights.h"
#include "../legacy/wolverine/src/runtime/suspension_weights.h"
static const UINT graftCount=2388;static const int shaftRestSampleCount=18;
static V3 graftDeformedPositions[graftCount],shaftRestCenters[shaftRestSampleCount];
static float graftRestFlex[graftCount],preparedPelvicRootFollow[graftCount],logicalShaftBodyRadius,fixtureGrowth;
static float PelvisCollarGrowth(){return fixtureGrowth;}
#include "data/wolverine-root-profile.inc"
int main(){
 for(int test=0;;test++){
  if(std::scanf("%f %f",&fixtureGrowth,&logicalShaftBodyRadius)!=2)break;
  for(auto& p:shaftRestCenters)if(std::scanf("%f %f %f",&p.x,&p.y,&p.z)!=3)return 2;
  for(unsigned i=0;i<graftCount;i++)if(std::scanf("%f %f %f %f %f",&graftDeformedPositions[i].x,
   &graftDeformedPositions[i].y,&graftDeformedPositions[i].z,&graftRestFlex[i],&preparedPelvicRootFollow[i])!=5)return 3;
  RegularizeSharedRootProfile();
  for(unsigned i=0;i<graftCount;i++){auto p=graftDeformedPositions[i];std::printf("%d,%u,%.9g,%.9g,%.9g\n",test,i,p.x,p.y,p.z);}
 }
}
