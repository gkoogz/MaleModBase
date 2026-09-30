#define NECK_TEST_MAIN
#include "../raphe/raphe_test.cpp"

static void AttachmentFrame(V3& center,V3& down){
 V3 tangent{};SampleShaftChain(.12f,center,tangent);down={0,0,-1};
}
static bool CheckUnderside(){
 V3 center{},down{};AttachmentFrame(center,down);
 for(int s=0;s<2;s++){
  V3 p=CPCenter(s);CHECK(std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z));
  float depth=Dot(p-center,down);
  if(depth<logicalShaftBodyRadius*.10f-.002f)printf("above shaft: side=%d depth=%.6f radius=%.6f\n",s,depth,logicalShaftBodyRadius);
  CHECK(depth>=logicalShaftBodyRadius*.10f-.002f);
  CHECK(std::isfinite(Length(pdVelocity[pdBody0+s])));
 }
 return true;
}
static bool Run(){
 unsigned cases=0;float peakSpeed=0;
 for(int state=0;state<3;state++)for(float size:{50.f,100.f})for(float angle:{1.f,50.f,100.f}){
  CHECK(RapheInit(state,size));sliderUI[1]=100;sliderUI[2]=size;sliderUI[4]=angle;ApplyControlMapping();shapeDirty=true;ApplyShape();
  for(int frame=0;frame<60;frame++)UpdateConstraintSolver(1.f/60.f,0,0);
  V3 center{},down{};AttachmentFrame(center,down);
  // Reproduce a perched lobe on each side of the shaft, not just a normal
  // startup. Keep its outward velocity to exercise safe recovery as well.
  for(int s=0;s<2;s++){
   pdPosition[pdBody0+s]=center-down*(logicalShaftBodyRadius+CPRadii(s).z+1.f)+V3{0,(s?1.f:-1.f)*CPRadii(s).y*1.1f,0};
   pdVelocity[pdBody0+s]=down*-30.f;
  }
  PDSync();
  for(int frame=0;frame<180;frame++){
   if(frame==60||frame==120){AttachmentFrame(center,down);for(int s=0;s<2;s++)pdVelocity[pdBody0+s]=pdVelocity[pdBody0+s]-down*180.f;}
   float dt=frame%30==0?1.f/20.f:1.f/60.f;
   UpdateConstraintSolver(dt,sinf(frame*.13f),2.f*sinf(frame*.09f));CHECK(CheckUnderside());
   for(int s=0;s<2;s++){
    float speed=Length(pdVelocity[pdBody0+s]);peakSpeed=max(peakSpeed,speed);
    if(speed>1000.f)printf("velocity spike state=%d size=%.0f angle=%.0f frame=%d side=%d speed=%.3f\n",state,size,angle,frame,s,speed);
    CHECK(speed<1000.f);
   }
  }
  ApplyShape();for(unsigned i=0;i<nrCount;i++)CHECK(std::isfinite(nrPositions[i].x)&&std::isfinite(nrPositions[i].y)&&std::isfinite(nrPositions[i].z));
  cases++;
 }
 printf("PASS %u perched/launch cases: all modes, neutral/max size, low/neutral/high angle; underside maintained through swing and 50ms frames; peak speed %.3f\n",cases,peakSpeed);
 return true;
}
int main(){setvbuf(stdout,nullptr,_IONBF,0);return Run()?0:1;}
