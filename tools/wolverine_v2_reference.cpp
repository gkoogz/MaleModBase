// Reference-only executable: include the exact pinned canonical Wolverine code.
// It performs no graphics calls, input, settings writes, installation or launch.
// The generated entry supplies these paths; no numerical code is copied here.
#include MALEMOD_WOLVERINE_RUNTIME
#include MALEMOD_WOLVERINE_CPU_BUFFER
#include <fstream>
#include <filesystem>
#include <iomanip>

static void Save(const std::filesystem::path& path,const void* data,size_t bytes){
 std::ofstream file(path,std::ios::binary);file.write(static_cast<const char*>(data),bytes);
 if(!file)throw std::runtime_error("Reference output failed");
}
int main(int argc,char** argv){
 try{
  if(argc!=4)throw std::runtime_error("usage: reference CONTROLS OUTPUT FRAMES");
  std::ifstream controls(argv[1]);std::array<float,18> values{};
  for(auto& value:values)if(!(controls>>value)||!std::isfinite(value))throw std::runtime_error("Invalid controls");
  if(values[0]<0||values[0]>2||values[0]!=int(values[0]))throw std::runtime_error("Invalid state");
  for(unsigned i=1;i<18;i++)if(values[i]<1||values[i]>100)throw std::runtime_error("Invalid control range");
  const int frames=atoi(argv[3]);if(frames<1||frames>3600)throw std::runtime_error("Invalid frame count");
  const std::filesystem::path output(argv[2]);std::filesystem::create_directories(output);
  ResetStudyControls();physicsState=int(values[0]);
  for(unsigned i=0;i<7;i++)sliderUI[i]=values[i+1];
  glansUI=values[8];hangUI=values[9];
  for(unsigned i=0;i<8;i++)physUI[i]=values[i+10];
  ApplyControlMapping();
  // Source-owned authored body tables seed both body resources. No captures.
  const unsigned count=50915;graftBuffer=new CpuVertexBuffer(count*32);graftOffset=47050*32;
  void* raw=nullptr;graftBuffer->Lock(0,0,&raw,0);memset(raw,0,count*32);
  PrepareMenuRetargetBodyBasis(static_cast<unsigned char*>(raw));
  for(unsigned i=0;i<collarNormalTriangleCount*3;i++)memcpy(static_cast<char*>(raw)+collarNormalTriangleIndices[i]*32,collarNormalTriangleBasePositions+i*3,12);
  for(unsigned i=0;i<pelvisControlCount;i++)memcpy(static_cast<char*>(raw)+pelvisControlIndices[i]*32,pelvisControlBasePositions+i*3,12);
  for(unsigned i=0;i<graftCount;i++)memcpy(static_cast<char*>(raw)+(47050+i)*32,morph_base+i*3,12);
  graftBuffer->Unlock();
  preparedShapeReady=shaftRestFrameReady=eggRestReady=constraintSolverReady=false;
  paBasisSaved=false;ResetCompliantDynamics();shapeDirty=true;ApplyShape();
  std::ofstream trace(output/"frames.csv");trace<<"frame,physicsMs,surfaceMs,tipX,tipY,tipZ\n"<<std::setprecision(9);
  for(int frame=0;frame<frames;frame++){
   const double start=PerfClock();
   UpdateConstraintSolver(1.f/60.f,0.f,0.f);
   const double physics=PerfClock()-start;const double surfaceStart=PerfClock();ApplyShape();
   const double surface=PerfClock()-surfaceStart;const V3 tip=shaftNodes[shaftNodeCount-1];
   trace<<frame<<','<<physics<<','<<surface<<','<<tip.x<<','<<tip.y<<','<<tip.z<<'\n';
  }
  Save(output/"anatomy.packed",nrPacked,sizeof(nrPacked));
  Save(output/"anatomy.indices",nrIndices,nrIndexCount*sizeof(*nrIndices));
  Save(output/"shaft-guide.f32",shaftNodes,sizeof(shaftNodes));
  for(unsigned section=0;section<2;section++)Save(output/("body"+std::to_string(section)+".packed"),sharedBodyOutput[section],sharedBodyCount[section]*32);
  std::ofstream metadata(output/"source.json");metadata<<"{\"anatomyVertices\":"<<nrCount<<",\"anatomyTriangles\":"<<nrIndexCount/3<<",\"frames\":"<<frames<<",\"fixedSeconds\":0.016666666666666666,\"offlineOnly\":true,\"garmentEvaluated\":false,\"bodyVertices\":["<<sharedBodyCount[0]<<','<<sharedBodyCount[1]<<"]}\n";
  if(!metadata||!trace)throw std::runtime_error("Incomplete reference output");
  graftBuffer->Release();graftBuffer=nullptr;return 0;
 }catch(const std::exception& error){fprintf(stderr,"%s\n",error.what());return 1;}
}
