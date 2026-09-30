#define NECK_TEST_MAIN
#include "../raphe/raphe_test.cpp"

static void Capture(const char* directory,int frame){
  char path[MAX_PATH];sprintf_s(path,"%s\\frame-%02d.bin",directory,frame);
  FILE* file=nullptr;fopen_s(&file,path,"wb");if(!file)exit(2);
  fwrite(nrPacked,32,nrCount,file);
  fwrite(shaftNodes,sizeof(shaftNodes),1,file);
  void* body=nullptr;if(FAILED(graftBuffer->Lock(0,0,&body,0)))exit(3);
  fwrite(body,32,50915,file);graftBuffer->Unlock();fclose(file);
  printf("frame=%02d tipY=%.3f rootX=%.3f\n",frame,shaftNodes[shaftNodeCount-1].y,shaftNodes[0].x);
}

int main(int argc,char** argv){
  if(argc!=2)return 2;
  if(!RapheInit(2,85.f))return 1;
  sliderUI[1]=100.f;sliderUI[2]=95.f;ApplyControlMapping();shapeDirty=true;ApplyShape();
  for(int frame=0;frame<120;frame++){UpdateConstraintSolver(1.f/60.f,0.f,0.f);ApplyShape();}
  for(int frame=0;frame<240;frame++){
    float side=2.f*sinf(2.f*3.1415926535f*frame/120.f);
    UpdateConstraintSolver(1.f/60.f,0.f,side);ApplyShape();
    if(frame>=120&&frame%10==0)Capture(argv[1],(frame-120)/10);
  }
  return 0;
}
