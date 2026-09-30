#define UNIFIED_TEST_NO_MAIN
#include "unified_test.cpp"
int main(){
 setvbuf(stdout,nullptr,_IONBF,0);
 for(int state=0;state<3;state++){
  if(!RapheInit(state,50.f))return 1;
  for(int f=0;f<360;f++)UpdateConstraintSolver(1.f/60.f,0,0);
  for(int f=0;f<180;f++)UpdateConstraintSolver(f%40==0?1.f/20.f:1.f/60.f,sinf(f*.09f),cosf(f*.12f)*1.5f);
  for(int f=0;f<180;f++)UpdateConstraintSolver(1.f/60.f,0,0);
  shapeDirty=true;ApplyShape();char label[80];sprintf_s(label,"hanging-%d",state);Snapshot(label);
  for(int s=0;s<2;s++){
   float worst=1e9f;int segment=-1;
   for(int j=1;j<shaftNodeCount-1;j++){
    PDConstraint c;float t;V3 arm{},q{},n=PDContactNormal(c,s,pdPosition[j],pdPosition[j+1],arm,q,t);
    float gap=Dot(pdPosition[pdBody0+s]+arm-q,n)-logicalShaftBodyRadius;
    if(gap<worst){worst=gap;segment=j;}
   }
   printf("state=%d side=%d gap=%.6f segment=%d shaftRadius=%.3f center=%.3f,%.3f,%.3f\n",state,s,worst,segment,logicalShaftBodyRadius,CPCenter(s).x,CPCenter(s).y,CPCenter(s).z);
  }
 }
 return 0;
}
