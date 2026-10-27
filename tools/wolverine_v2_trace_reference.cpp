// Direct canonical-source oracle for changing controls, clocks and contacts.
// Uses the same source declarations as the reset harness, without installation.
#define main ResetReferenceEntry
#include "wolverine_v2_reference.cpp"
#undef main
#include <iostream>
int main(int argc,char** argv){
 try{
  if(argc!=3)throw std::runtime_error("usage: trace-reference INPUT OUTPUT");
  const std::filesystem::path output(argv[2]);std::filesystem::create_directories(output);
  std::ifstream input(argv[1]);ResetStudyControls();
  graftBuffer=new CpuVertexBuffer(50915*32);graftOffset=47050*32;void* memory=nullptr;graftBuffer->Lock(0,0,&memory,0);memset(memory,0,50915*32);auto* raw=static_cast<unsigned char*>(memory);
  auto resetEpoch=[&](){
   memset(raw,0,50915*32);PrepareMenuRetargetBodyBasis(raw);
   for(unsigned i=0;i<collarNormalTriangleCount*3;i++)memcpy(raw+collarNormalTriangleIndices[i]*32,collarNormalTriangleBasePositions+i*3,12);
   for(unsigned i=0;i<pelvisControlCount;i++)memcpy(raw+pelvisControlIndices[i]*32,pelvisControlBasePositions+i*3,12);
   for(unsigned i=0;i<graftCount;i++)memcpy(raw+(47050+i)*32,morph_base+i*3,12);
   preparedShapeReady=shaftRestFrameReady=eggRestReady=constraintSolverReady=false;paBasisSaved=false;ResetCompliantDynamics();shapeDirty=true;ApplyShape();
  };
  std::ofstream guide(output/"guides.f32",std::ios::binary);unsigned frames=0,epoch=0,epochFrame=0;
  std::ofstream timing(output/"timings.csv");timing<<"frame,physicsMs,surfaceMs,garment\n";std::string line;std::array<float,18> previousControls{};unsigned previousStyle=~0u;
  while(std::getline(input,line)){
   if(line.empty())continue;std::istringstream row(line);std::array<float,18> values{};for(float& value:values)row>>value;
   float seconds=0,pitch=0,yaw=0;int collision=0;V3 thighs[4]{};row>>seconds>>pitch>>yaw>>collision;for(auto& p:thighs)row>>p.x>>p.y>>p.z;if(!row)throw std::runtime_error("Truncated canonical trace");unsigned style=0,requestedEpoch=1;if(row>>style){if(!(row>>requestedEpoch))requestedEpoch=1;}if(!requestedEpoch)throw std::runtime_error("Invalid reference epoch");if(style>1)throw std::runtime_error("Invalid reference garment mode");if(style!=clothingStyle)SetJockstrapStyle(style);
   physicsState=int(values[0]);for(unsigned i=0;i<7;i++)sliderUI[i]=values[i+1];glansUI=values[8];hangUI=values[9];for(unsigned i=0;i<8;i++)physUI[i]=values[i+10];ApplyControlMapping();
   collisionCapsuleOverride=collision!=0;overrideLeftA=thighs[0];overrideLeftB=thighs[1];overrideRightA=thighs[2];overrideRightB=thighs[3];
   if(epoch!=requestedEpoch){resetEpoch();epoch=requestedEpoch;epochFrame=0;previousStyle=~0u;}
   double mark=PerfClock();UpdateConstraintSolver(seconds,pitch,yaw);double physics=PerfClock()-mark;mark=PerfClock();bool dirty=previousControls!=values||previousStyle!=style;if(malemod::garments::UpdatePouchCoveredSurface(style==1,dirty,false,++epochFrame))ApplyShape();previousControls=values;previousStyle=style;timing<<frames<<','<<physics<<','<<PerfClock()-mark<<','<<style<<'\n';
   guide.write(reinterpret_cast<const char*>(shaftNodes),sizeof(shaftNodes));++frames;
  }
  Save(output/"anatomy.packed",nrPacked,sizeof(nrPacked));Save(output/"anatomy.indices",nrIndices,nrIndexCount*sizeof(*nrIndices));
  for(unsigned section=0;section<2;section++)Save(output/("body"+std::to_string(section)+".packed"),sharedBodyOutput[section],sharedBodyCount[section]*32);
  std::cout<<"Direct canonical trace frames="<<frames<<'\n';return 0;
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
