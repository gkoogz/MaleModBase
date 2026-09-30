#define RAPHE_PROFILE
#define main ExistingSequenceMain
#include "../teaching/sequence_test.cpp"
#undef main
#include <chrono>
static bool RapheInit(int state,float size){
 ResetStudyControls();physicsState=state;sliderUI[0]=sliderUI[1]=size;ApplyControlMapping();
 preparedShapeReady=shaftRestFrameReady=eggRestReady=constraintSolverReady=false;paBasisSaved=false;ResetCompliantDynamics();
 if(graftBuffer)graftBuffer->Release();graftBuffer=new CpuVertexBuffer(50915*32);graftOffset=47050*32;void* raw=nullptr;graftBuffer->Lock(0,0,&raw,0);
 FILE* f=nullptr;fopen_s(&f,"../harmonization/audit-gameplay-base.bin","rb");CHECK(f);CHECK(fread(raw,32,50915,f)==50915);fclose(f);graftBuffer->Unlock();shapeDirty=true;ApplyShape();return true;
}
static void WriteRapheFixture(const char* prefix){
 char path[MAX_PATH];FILE* f=nullptr;
 sprintf_s(path,"%s.vertices",prefix);fopen_s(&f,path,"wb");fwrite(rsPacked,32,rsCount,f);fclose(f);
 sprintf_s(path,"%s.indices",prefix);fopen_s(&f,path,"wb");fwrite(rsIndices,sizeof(unsigned short),rsIndexCount,f);fclose(f);
 sprintf_s(path,"%s.material",prefix);fopen_s(&f,path,"wb");
 for(unsigned i=0;i<rsCount;i++){
  float t=0,flex=0,ball=0,shaft=0;
  auto source=[&](unsigned j,float factor){t+=AxialMaterialFlex(j)*factor;flex+=r14Flex[j]*factor;for(unsigned k=r14Offsets[j];k<r14Offsets[j+1];k++){unsigned q=r14Sources[k];ball+=r14Weight[k]*phys_scrotum_weight[q]*factor;shaft+=r14Weight[k]*max(phys_shaft_weight[q],phys_attachment_weight[q])*factor;}};
  if(i<r14Count)source(i,1);else for(int j=0;j<3;j++)source(rsFineSource[(i-r14Count)*3+j],rsFineBary[(i-r14Count)*3+j]);
  float v[4]={t,flex,ball,shaft};fwrite(v,4,4,f);
 }fclose(f);
 sprintf_s(path,"%s.frames",prefix);fopen_s(&f,path,"wb");for(int i=0;i<=100;i++){V3 c,t;SampleShaftChain(i/100.f,c,t);fwrite(&c,12,1,f);fwrite(&t,12,1,f);}fclose(f);
 sprintf_s(path,"%s.body",prefix);fopen_s(&f,path,"wb");for(int s=0;s<2;s++)fwrite(sharedBodyOutput[s],32,sharedBodyCount[s],f);fclose(f);
 sprintf_s(path,"%s.tube",prefix);fopen_s(&f,path,"wb");fwrite(rapheTubeWall,sizeof(rapheTubeWall),1,f);fclose(f);
 sprintf_s(path,"%s.tube-indices",prefix);fopen_s(&f,path,"wb");fwrite(rapheTubeFaces,sizeof(rapheTubeFaces),1,f);fclose(f);
 sprintf_s(path,"%s.skin-delta",prefix);fopen_s(&f,path,"wb");fwrite(rapheSkinDelta,sizeof(rapheSkinDelta),1,f);fclose(f);
 printf("Tube end %.6f fold depth %.6f skin fraction %.6f changed %u\n",rapheTubeEnd,rapheTubeEndDepth,rapheSkinFraction,rapheSkinChanged);
 printf("radius %.6f length %.6f vertices %u triangles %u\n",logicalShaftBodyRadius,constraintRestLength,rsCount,rsIndexCount/3);
 for(unsigned i:{17u,16u,36u,37u,60u,67u,74u,354u,355u,356u,317u,318u,r14NewStart,r14NewStart+r14CrownRing*r14SegmentCount})printf("id %u flex %.6f material %.6f p %.4f %.4f %.4f\n",i,r14Flex[i],AxialMaterialFlex(i),rsPositions[i].x,rsPositions[i].y,rsPositions[i].z);
}
static bool RapheTests(){
 CHECK(RapheTubeRadiusRatio(0)>RapheTubeRadiusRatio(.5f)*1.25f);
 CHECK(RapheTubeRadiusRatio(.5f)==RapheTubeRadiusRatio(.4f));
 CHECK(RapheTubeRadiusRatio(.5f)<.31f);
 CHECK(RapheTubeRadiusRatio(1)<RapheTubeRadiusRatio(.5f)*.36f);
 for(int i=51;i<=100;i++)CHECK(RapheTubeRadiusRatio(i*.01f)<=RapheTubeRadiusRatio((i-1)*.01f));
 float bend[3];for(int state=0;state<3;state++){shaftMode=(float)state;bend[state]=RapheTubeBendMultiplier(.3f);CHECK(RapheTubeBendMultiplier(.8f)>bend[state]);}CHECK(bend[0]<bend[1]&&bend[1]<bend[2]);
 for(int state=0;state<3;state++)for(float size:{25.f,50.f,85.f}){
  CHECK(RapheInit(state,size));CHECK(rapheSkinFraction>0&&rapheSkinChanged>0);
  // Compare the bounded search to the original exhaustive projection. This
  // includes the extended proximal segment and points around the skin.
  for(unsigned i=0;i<rsCount;i+=17){V3 p=rsPositions[i];float best=1e30f,reference=0;
   for(unsigned k=0;k<32;k++){V3 e=rapheChain[k+1].p-rapheChain[k].p;float u=Dot(p-rapheChain[k].p,e)/max(1e-8f,Dot(e,e));u=max(k==0?-.30f*32:0.f,min(1.f,u));V3 q=p-(rapheChain[k].p+e*u);float d=Dot(q,q);if(d<best){best=d;reference=(k+u)/32.f;}}
   float actual=RapheClosest(p),u=actual*32;unsigned k=(unsigned)max(0.f,min(31.f,u));V3 q=p-(rapheChain[k].p+rapheSegments[k].edge*(u-k));
   // Nearly collinear adjacent segments can tie at float precision; compare
   // distance, not the arbitrary segment chosen to represent the same foot.
   // Reconstructing the foot from t also rounds coordinates near z=80.
   CHECK(fabsf(Dot(q,q)-best)<=max(1e-4f,best*1e-5f));
  }
  std::map<std::pair<unsigned,unsigned>,unsigned> edges;double volume=0;
  for(unsigned k=0;k<sizeof(rapheTubeFaces)/sizeof(rapheTubeFaces[0]);k+=3){unsigned a=rapheTubeFaces[k],b=rapheTubeFaces[k+1],c=rapheTubeFaces[k+2];CHECK(a<rapheTubeRings*rapheTubeSides*2&&b<rapheTubeRings*rapheTubeSides*2&&c<rapheTubeRings*rapheTubeSides*2);volume+=Dot(rapheTubeWall[a],Cross(rapheTubeWall[b],rapheTubeWall[c]))/6.;for(auto pair:{std::make_pair(a,b),std::make_pair(b,c),std::make_pair(c,a)}){if(pair.first>pair.second)std::swap(pair.first,pair.second);++edges[pair];}}
  for(auto edge:edges)CHECK(edge.second==2);CHECK(volume>0);
  bool seam[rsCount]{};for(unsigned k=0;k<paSeamCount;k++)seam[paSeamR14[k]]=true;
  float neck=0,pouch=0;for(unsigned i=0;i<rsCount;i++){
   CHECK(std::isfinite(rsPositions[i].x)&&std::isfinite(rsPositions[i].y)&&std::isfinite(rsPositions[i].z));
   if(!seam[i]&&rapheSkinBall[i]>.15f)neck=max(neck,Length(rapheSkinDelta[i]));if(rapheSkinBall[i]>.7f)pouch=max(pouch,Length(rapheSkinDelta[i]));
  }
  CHECK(neck<=logicalShaftBodyRadius*.03501f);CHECK(pouch==0);
  float worst=1.f;for(unsigned k=0;k<rsIndexCount;k+=3){unsigned a=rsIndices[k],b=rsIndices[k+1],c=rsIndices[k+2];V3 old=Cross((rsPositions[b]-rapheSkinDelta[b])-(rsPositions[a]-rapheSkinDelta[a]),(rsPositions[c]-rapheSkinDelta[c])-(rsPositions[a]-rapheSkinDelta[a]));V3 now=Cross(rsPositions[b]-rsPositions[a],rsPositions[c]-rsPositions[a]);float area=Dot(old,old);if(area>1e-10f)worst=min(worst,Dot(old,now)/area);}CHECK(worst>.049f);
  printf("PASS state=%d size=%.0f tube closed/outward volume=%.4f; neck easing=%.5f pouch=%.5f; skin fraction=%.4f min area projection=%.4f\n",state,size,volume,neck,pouch,rapheSkinFraction,worst);
 }
 // Coupled solver motion exercises rigid and yielding modes with contacts.
 for(int state=0;state<3;state++){
  CHECK(RapheInit(state,70));std::vector<double> times;for(auto& v:rapheProfile)v=0;
  for(int frame=0;frame<120;frame++){
   float t=frame/60.f;UpdateConstraintSolver(1.f/60,sinf(t*4)*.3f,cosf(t*3)*.3f);auto start=std::chrono::steady_clock::now();ApplyShape();
   times.push_back(std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count());
   for(const auto& p:rapheTubeWall)CHECK(std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z));
   for(int i=0;i<shaftNodeCount;i++)CHECK(std::isfinite(shaftNodes[i].x)&&std::isfinite(shaftNodes[i].y)&&std::isfinite(shaftNodes[i].z));
   CHECK(rapheSkinFraction>=0&&rapheSkinFraction<=1);
  }
  std::sort(times.begin(),times.end());printf("PASS coupled motion state=%d: skin CPU median %.3f ms p95 %.3f ms (offline)\n",state,times[60],times[114]);
  printf("Raphe average stages: build %.3f field %.3f safety %.3f commit %.3f ms\n",rapheProfile[0]/120,rapheProfile[1]/120,rapheProfile[2]/120,rapheProfile[3]/120);
 }
 return true;
}
#ifndef NECK_TEST_MAIN
int main(int argc,char** argv){setvbuf(stdout,nullptr,_IONBF,0);if(argc>1&&strcmp(argv[1],"--test")==0)return RapheTests()?0:1;if(!RapheInit(argc>2?atoi(argv[2]):0,argc>3?(float)atof(argv[3]):50.f))return 1;WriteRapheFixture(argc>1?argv[1]:"fixture");return 0;}

#endif
