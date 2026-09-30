#pragma once
#include "raphe_support_data.h"
// The shaft-side material strip is transported by the shaft, never by the
// hanging pouch. Reassert it after the render-only junction/lobe fairing.
static float rapheSupportMoved[r14Count];
static void PreserveShaftRaphe(){
  memset(rapheSupportMoved,0,sizeof(rapheSupportMoved));
  if(!constraintSolverReady||!shaftRestFrameReady)return;
  static V3 origin[r14Count],delta[r14Count];
  memcpy(origin,r14Positions,sizeof(origin));memset(delta,0,sizeof(delta));
  for(UINT i=0;i<r14NewStart;i++){
    float weight=rapheSupportMask[i];if(weight<=0.f)continue;
    V3 rest{r14Base[i*3],r14Base[i*3+1],r14Base[i*3+2]};
    for(UINT k=r14Offsets[i];k<r14Offsets[i+1];k++){
      UINT j=r14Sources[k];
      rest=rest+(firmLobeRestSkin[j]-V3{r14Reference[j*3],r14Reference[j*3+1],r14Reference[j*3+2]})*r14Weight[k];
    }
    float t=ClosestRestShaftFlex(rest);
    V3 rc{},rt{},lc{},lt{};SampleRestShaftFrame(t,rc,rt);SampleShaftChain(t,lc,lt);
    V3 target=lc+RotateFromTo(rest-rc,rt,lt);
    delta[i]=(target-origin[i])*weight;
  }
  for(UINT row=0;row<rapheSupportFreeCount;row++){
    V3 shift{};
    for(UINT k=0;k<rapheSupportCoreCount;k++)shift=shift+delta[rapheSupportCore[k]]*rapheSupportWeights[row*rapheSupportCoreCount+k];
    delta[rapheSupportFree[row]]=shift;
  }
  for(UINT i=0;i<r14NewStart;i++){
    r14Positions[i]=origin[i]+delta[i];rapheSupportMoved[i]=Length(delta[i]);
    undersideBlendMoved[i]+=rapheSupportMoved[i];
  }
}
