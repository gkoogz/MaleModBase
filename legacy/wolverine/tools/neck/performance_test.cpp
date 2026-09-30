#define NECK_TEST_MAIN
#include "../raphe/raphe_test.cpp"
int main(int argc,char** argv){
 setvbuf(stdout,nullptr,_IONBF,0);
 FILE* snapshot=nullptr;if(argc>1)fopen_s(&snapshot,argv[1],"wb");
 for(int state=0;state<1;state++)for(int pulse=0;pulse<1;pulse++){
  if(!RapheInit(state,argc>2?(float)atof(argv[2]):70))return 1;
  if(argc>2){sliderUI[1]=100;sliderUI[2]=95;ApplyControlMapping();shapeDirty=true;ApplyShape();}
  for(auto& p:perfMetrics){p.total=p.peak=0;p.count=0;}
  for(auto& p:rapheProfile)p=0;
  double physics=0,surface=0;for(int f=0;f<90;f++){
   if(pulse){sliderUI[0]=70+2*sinf(f*.07f);ApplyControlMapping();}
   double start=PerfClock();UpdateConstraintSolver(1.f/60,sinf(f*.05f)*.3f,cosf(f*.08f)*.3f);physics+=PerfClock()-start;
   start=PerfClock();ApplyShape();surface+=PerfClock()-start;
   if(snapshot&&f%15==0){fwrite(nrPacked,1,sizeof(nrPacked),snapshot);fwrite(shaftNodes,1,sizeof(shaftNodes),snapshot);}
  }
  printf("state=%d pulse=%d physics=%.3f surface=%.3f ms per frame\n",state,pulse,physics/90,surface/90);
  for(auto& p:perfMetrics)if(p.count)printf("  %s %.3f ms\n",p.name,p.total/p.count);
  printf("  structural stages %.3f %.3f %.3f %.3f ms\n",rapheProfile[0]/90,rapheProfile[1]/90,rapheProfile[2]/90,rapheProfile[3]/90);
 }
 if(snapshot)fclose(snapshot);return 0;
}
