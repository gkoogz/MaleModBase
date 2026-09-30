#define UNIFIED_TEST_NO_MAIN
#include "unified_test.cpp"

int main(){
  setvbuf(stdout,nullptr,_IONBF,0);
  if(!RapheInit(2,94))return 1;
  throbMode=0;sliderUI[0]=94;sliderUI[1]=52;sliderUI[2]=50;
  ApplyControlMapping();ApplyShape();
  const unsigned startBuilds=UnifiedCollar::builds;
  if(!teachingFluid.Prepare())return 2;
  teachingTimeline.Start();
  unsigned frames=0;double surfaceSum=0,surfacePeak=0;
  while(teachingTimeline.active&&frames<1200){
    teachingTimeline.Advance(1.f/30.f);
    ApplyControlMapping();
    UpdateConstraintSolver(1.f/30.f,0,0);
    double begin=PerfClock();ApplyShape();double elapsed=PerfClock()-begin;
    surfaceSum+=elapsed;surfacePeak=(std::max)(surfacePeak,elapsed);
    if(frames%30==0&&!CheckUnified())return 3;
    ++frames;
  }
  const unsigned sequenceBuilds=UnifiedCollar::builds-startBuilds;
  printf("J sequence with Throb 0: frames=%u active=%d collarBuilds=%u surfaceMean=%.3fms peak=%.3fms\n",frames,teachingTimeline.active?1:0,sequenceBuilds,surfaceSum/frames,surfacePeak);
  if(teachingTimeline.active||sequenceBuilds>6)return 4;

  teaching::Timeline hitch;hitch.Start();hitch.Advance(.35f);
  if(!hitch.active||fabs(hitch.time-.1)>.0001)return 5;
  hitch.Advance(.8f);
  if(!hitch.active||fabs(hitch.time-.2)>.0001)return 6;
  teaching::Fluid fluid;if(!fluid.Begin({0,0,0}))return 7;
  fluid.Advance(.5f,{0,0,0},{1,0,0},{});
  if(!fluid.ready||fabs(fluid.clock-.1)>.001)return 8;
  printf("PASS J collar and 350-800ms hitch regression\n");
  return 0;
}
