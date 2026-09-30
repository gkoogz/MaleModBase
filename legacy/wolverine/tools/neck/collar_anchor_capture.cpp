#define NECK_TEST_MAIN
#include "../raphe/raphe_test.cpp"

static void Capture(const char* path){
  FILE* file=nullptr;
  fopen_s(&file,path,"wb");
  if(!file){fprintf(stderr,"cannot open %s\n",path);exit(2);}
  fwrite(nrPacked,32,nrCount,file);
  fwrite(shaftNodes,sizeof(shaftNodes),1,file);
  void* body=nullptr;
  if(FAILED(graftBuffer->Lock(0,0,&body,0))){fclose(file);exit(3);}
  fwrite(body,32,50915,file);
  graftBuffer->Unlock();
  fclose(file);
  printf("%s root=(%.3f,%.3f,%.3f) middle=(%.3f,%.3f,%.3f) tip=(%.3f,%.3f,%.3f)\n",
         path,shaftNodes[0].x,shaftNodes[0].y,shaftNodes[0].z,
         shaftNodes[shaftNodeCount/2].x,shaftNodes[shaftNodeCount/2].y,shaftNodes[shaftNodeCount/2].z,
         shaftNodes[shaftNodeCount-1].x,shaftNodes[shaftNodeCount-1].y,shaftNodes[shaftNodeCount-1].z);
}

int main(int argc,char** argv){
  if(argc!=3)return 2;
  if(!RapheInit(2,85.f))return 1;
  sliderUI[1]=100.f;
  sliderUI[2]=95.f;
  ApplyControlMapping();shapeDirty=true;ApplyShape();
  for(int i=0;i<120;i++)UpdateConstraintSolver(1.f/60.f,0.f,0.f);
  ApplyShape();Capture(argv[1]);
  for(int i=0;i<120;i++)UpdateConstraintSolver(1.f/60.f,0.f,2.f);
  ApplyShape();Capture(argv[2]);
  return 0;
}
