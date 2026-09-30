#define NECK_TEST_MAIN
#include "../raphe/raphe_test.cpp"
int main(int argc,char** argv){
 setvbuf(stdout,nullptr,_IONBF,0);
 const int state=argc>2?atoi(argv[2]):2;
 const float size=argc>3?(float)atof(argv[3]):85.f;
 const float width=argc>4?(float)atof(argv[4]):50.f;
 for(float dt:{1.f/60.f,1.f/45.f,1.f/30.f,1.f/20.f}){
  if(!RapheInit(state,size))return 1;
  sliderUI[2]=width;ApplyControlMapping();shapeDirty=true;ApplyShape();
  char path[MAX_PATH]{};FILE* capture=nullptr;
  if(argc>1){sprintf_s(path,"%s-%.0f.bin",argv[1],dt*1000);fopen_s(&capture,path,"wb");}
  double total=0,peak=0;
  for(int f=0;f<150;f++){
   double start=PerfClock();
   UpdateConstraintSolver(dt,sinf(f*.05f)*.3f,cosf(f*.08f)*.3f);
   double ms=PerfClock()-start;
   if(f>=30){total+=ms;peak=max(peak,ms);}
   if(capture){fwrite(shaftNodes,sizeof(V3),shaftNodeCount,capture);fwrite(ballNodes,sizeof(V3),2,capture);}
  }
  if(capture)fclose(capture);
  printf("dt=%.2fms physics=%.3fms/frame peak=%.3f minGap=%.5f maxTether=%.5f\n",dt*1000,total/120,peak,pdMinimumGap,pdMaxTetherRatio);
 }
 return 0;
}
