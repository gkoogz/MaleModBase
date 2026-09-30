#define NECK_TEST_MAIN
#include "../raphe/raphe_test.cpp"
static bool CheckUnified(){
 using namespace UnifiedCollar;
 CHECK(valid&&solved.allFinite());
 for(unsigned k:fixedIDs)CHECK((solved.row(masters[k])-before.row(masters[k])).norm()<1e-10);
 for(unsigned k=0;k<ucSeamCount;k++){unsigned x=ucSeamVertices[k*3],a=ucSeamVertices[k*3+1],b=ucSeamVertices[k*3+2];double u=ucSeamWeights[k];CHECK((solved.row(x)-solved.row(a)*(1-u)-solved.row(b)*u).norm()<1e-10);}
 for(unsigned i=0;i<nrCount;i++){unsigned total=0;for(unsigned j=0;j<4;j++){total+=nrPacked[i*32+24+j];CHECK(nrPacked[i*32+20+j]<5);}CHECK(total==255);CHECK(std::isfinite(nrPositions[i].x)&&std::isfinite(nrPositions[i].y)&&std::isfinite(nrPositions[i].z));}
 return true;
}
static void Snapshot(const char* label){
 using namespace UnifiedCollar;char path[MAX_PATH];FILE* f=nullptr;const char* captureDir=getenv("SURFACE_AUDIT_CAPTURE_DIR");sprintf_s(path,"%s/%s.bin",captureDir?captureDir:"../../captures/runtime-tests",label);fopen_s(&f,path,"wb");if(!f)return;
 for(unsigned i=0;i<ucCount;i++){V3 p=Point(solved,i),q=Point(before,i);float w=(float)mask[i];fwrite(&p,12,1,f);fwrite(&q,12,1,f);fwrite(&w,4,1,f);}
 if(captureDir){fwrite(nrPacked,sizeof(nrPacked),1,f);fwrite(rsPacked,sizeof(rsPacked),1,f);fwrite(r14Packed,sizeof(r14Packed),1,f);fwrite(shaftNodes,sizeof(shaftNodes),1,f);fwrite(ballNodes,sizeof(ballNodes),1,f);}
 fclose(f);
}
static bool Run(){
 using namespace UnifiedCollar;unsigned cases=0;char label[100];
 for(int state=0;state<3;state++)for(float size:{25.f,50.f,85.f,100.f}){
  CHECK(RapheInit(state,size));throbMode=0;sliderUI[1]=100;
  for(float width:{25.f,50.f,95.f,100.f}){
   sliderUI[2]=width;ApplyControlMapping();shapeDirty=true;ApplyShape();CHECK(CheckUnified());sprintf_s(label,"static-%d-%.0f-%.0f",state,size,width);Snapshot(label);cases++;
  }
 }
 printf("PASS %u static controls: finite geometry, normalized weights, protected exterior, exact weld constraints\n",cases);
 for(int state=0;state<3;state++){
  CHECK(RapheInit(state,85));throbMode=0;sliderUI[1]=100;sliderUI[2]=95;ApplyControlMapping();shapeDirty=true;ApplyShape();
  std::vector<double> times,collar;unsigned startBuilds=builds;
  for(int f=0;f<90;f++){
   float t=f/60.f;UpdateConstraintSolver(1.f/60,sinf(t*4)*.3f,cosf(t*3)*.3f);
   double begin=PerfClock();ApplyShape();times.push_back(PerfClock()-begin);collar.push_back(solveMS);CHECK(CheckUnified());
   if(f%30==0||f==89){sprintf_s(label,"motion-%d-%d",state,f);Snapshot(label);}
  }
  std::sort(times.begin(),times.end());std::sort(collar.begin(),collar.end());printf("PASS state %d: 90 frames, full surface median %.3f ms p95 %.3f ms, collar median %.3f ms, rebuilds %u\n",state,times[45],times[85],collar[45],builds-startBuilds);
 }
 for(int pulse:{1,3})for(float size:{85.f,100.f}){
  CHECK(RapheInit(2,size));sliderUI[1]=100;sliderUI[2]=size==85?95.f:100.f;throbMode=pulse;ApplyControlMapping();shapeDirty=true;ApplyShape();unsigned count=builds;std::vector<double> times;
  for(int frame=0;frame<120;frame++){
   float time=frame/40.f;throbSizePulse=ThrobEnvelope(time,.20f,1.05f,false);throbTwitchPulse=ThrobEnvelope(time,1.15f,4.6f,true);throbAngleSizePulse=ThrobEnvelope(time,1.15f,1.05f,true);ApplyControlMapping();
   UpdateConstraintSolver(1.f/40,0,0);double start=PerfClock();ApplyShape();times.push_back(PerfClock()-start);CHECK(CheckUnified());
   if(frame%20==0||frame==119){sprintf_s(label,"pulse-%d-%.0f-%d",pulse,size,frame);Snapshot(label);}
  }
  std::sort(times.begin(),times.end());printf("PASS pulse %d size %.0f: full surface median %.3f ms p95 %.3f ms, factor builds %u\n",pulse,size,times[60],times[114],builds-count);
 }
 return true;
}
#ifndef UNIFIED_TEST_NO_MAIN
int main(){setvbuf(stdout,nullptr,_IONBF,0);return Run()?0:1;}
#endif
