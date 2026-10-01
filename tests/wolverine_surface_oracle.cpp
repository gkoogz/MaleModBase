// Full authoritative runtime oracle. Verification harness only: graphics SDKs
// are confined to this reference target, never a portable shared module.
#include "../legacy/wolverine/src/runtime/d3d9_proxy.cpp"
#include "../legacy/wolverine/tools/r14/cpu_buffer.h"

template<class T> static bool Write(const char* prefix,const char* suffix,const T* data,size_t count){
 char path[4096];sprintf_s(path,"%s.%s",prefix,suffix);FILE* file=nullptr;
 if(fopen_s(&file,path,"wb")||!file)return false;
 bool ok=fwrite(data,sizeof(T),count,file)==count;fclose(file);return ok;
}
int main(int argc,char** argv){
 if(argc!=4&&argc!=5){fprintf(stderr,"usage: surface-oracle INPUT-BUFFER UI-PREFERENCES OUTPUT-PREFIX [STEPS]\n");return 2;}
 ResetStudyControls();throbMode=0;
 FILE* prefs=nullptr;if(fopen_s(&prefs,argv[2],"r")||!prefs)return 3;
 if(fscanf_s(prefs,"%d %f %f %f %f %f %f %f %f %f",&physicsState,
  &sliderUI[0],&sliderUI[1],&sliderUI[2],&sliderUI[3],&sliderUI[4],&sliderUI[5],&sliderUI[6],&glansUI,&hangUI)!=10)return 4;
 for(int i=0;i<8;i++)if(fscanf_s(prefs,"%f",&physUI[i])!=1)return 4;fclose(prefs);
 ApplyControlMapping();
 graftBuffer=new CpuVertexBuffer(50915*32);graftOffset=47050*32;void* raw=nullptr;
 graftBuffer->Lock(0,0,&raw,0);
 if(!strcmp(argv[1],"-")){
  // Source-owned rest seed, independent of captured game vertex buffers.
  memset(raw,0,50915*32);
  for(UINT i=0;i<collarNormalTriangleCount*3;i++)memcpy((char*)raw+collarNormalTriangleIndices[i]*32,collarNormalTriangleBasePositions+i*3,12);
  for(UINT i=0;i<pelvisControlCount;i++)memcpy((char*)raw+pelvisControlIndices[i]*32,pelvisControlBasePositions+i*3,12);
  for(UINT i=0;i<graftCount;i++)memcpy((char*)raw+(47050+i)*32,morph_base+i*3,12);
 }else{
  FILE* donor=nullptr;if(fopen_s(&donor,argv[1],"rb")||!donor)return 5;
  if(fread(raw,32,50915,donor)!=50915)return 6;fclose(donor);
 }
 graftBuffer->Unlock();
 preparedShapeReady=shaftRestFrameReady=eggRestReady=constraintSolverReady=false;
 paBasisSaved=false;ResetCompliantDynamics();shapeDirty=true;ApplyShape();
 // Capture a settled source resting frame with its numerical solver active.
 const unsigned steps=argc==5?unsigned(atoi(argv[4])):120;
 for(unsigned i=0;i<steps;i++){UpdateConstraintSolver(1.f/60,0,0);ApplyShape();}
 bool ok=Write(argv[3],"coarse",graftDeformedPositions,graftCount)
  &&Write(argv[3],"r14",r14Packed,sizeof(r14Packed))
  &&Write(argv[3],"support",rsPacked,sizeof(rsPacked))
  &&Write(argv[3],"final",nrPacked,sizeof(nrPacked))
  &&Write(argv[3],"final-indices",nrIndices,nrIndexCount)
  &&Write(argv[3],"body0",sharedBodyOutput[0],sharedBodyCount[0]*32)
  &&Write(argv[3],"body1",sharedBodyOutput[1],sharedBodyCount[1]*32)
  &&Write(argv[3],"rest-flex",graftRestFlex,graftCount)
  &&Write(argv[3],"rest-centers",shaftRestCenters,sizeof(shaftRestCenters)/sizeof(V3));
 printf("source surface: coarse=%u support=%u final=%u radius=%.9g length=%.9g finite=%d\n",
  graftCount,rsCount,nrCount,logicalShaftBodyRadius,constraintRestLength,UnifiedCollar::solved.allFinite());
 graftBuffer->Release();graftBuffer=nullptr;return ok?0:7;
}
