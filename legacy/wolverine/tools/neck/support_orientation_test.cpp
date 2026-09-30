#define NECK_TEST_MAIN
#include "../raphe/raphe_test.cpp"
int main(){
 setvbuf(stdout,nullptr,_IONBF,0);
 for(int state=0;state<3;state++)for(float size:{50.f,100.f}){
  if(!RapheInit(state,size))return 1;
  // Tilt the actual rigid supports beyond horizontal, then let the existing
  // coupled contacts and suspension recover under alternating movement.
  for(int s=0;s<2;s++)PDRotateBody(s,{0.f,(s?1.f:-1.f)*2.1f,0.f});
  float worst=1.f;int inverted=0;double time=0;
  for(int frame=0;frame<360;frame++){
   double start=PerfClock();UpdateConstraintSolver(1.f/60.f,frame<120?sinf(frame*.12f)*.4f:0.f,frame<120?cosf(frame*.1f)*.4f:0.f);time+=PerfClock()-start;
   for(int s=0;s<2;s++){
    float alignment=Dot(cpBasis[s][2],CPDesiredUp(s));
    if(!std::isfinite(alignment)||!std::isfinite(ballNodes[s].z))return 1;
    if(frame>=120){worst=min(worst,alignment);inverted+=alignment<0.f;}
    if(fabsf(Length(cpBasis[s][2])-1.f)>.001f)return 1;
   }
  }
  printf("state=%d size=%.0f settledMinAlignment=%.6f invertedSamples=%d end=%.6f,%.6f physicsMS=%.3f\n",state,size,worst,inverted,Dot(cpBasis[0][2],CPDesiredUp(0)),Dot(cpBasis[1][2],CPDesiredUp(1)),time/360);
  // Contact can tilt the supported body; crossing the suspension's equator
  // after settling is the inversion regression, not imperfect alignment.
  if(inverted || worst<0.f)return 1;
 }
 return 0;
}
