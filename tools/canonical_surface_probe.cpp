// Standalone SDK-free comparison target for the generated canonical kernel.
#include "canonical-kernel.inc"
#include <filesystem>
#include <fstream>
using namespace malemod::surface::source;
static void Save(const std::filesystem::path& p,const void* data,std::size_t size){
 std::ofstream f(p,std::ios::binary);f.write(static_cast<const char*>(data),size);
 if(!f)throw std::runtime_error("Canonical output failed");
}
int main(int argc,char** argv){
 if(argc!=4)return 2;
 const std::filesystem::path output(argv[2]);std::filesystem::create_directories(output);
 std::ifstream control(argv[1]);control>>physicsState;
 for(auto& x:sliderUI)control>>x;control>>glansUI>>hangUI;for(auto& x:physUI)control>>x;
 if(!control)return 3;ApplyControlMapping();
 std::vector<unsigned char> buffer(50915*32);auto* raw=buffer.data();
 PrepareMenuRetargetBodyBasis(raw);
 for(unsigned i=0;i<collarNormalTriangleCount*3;i++)memcpy(raw+collarNormalTriangleIndices[i]*32,collarNormalTriangleBasePositions+i*3,12);
 for(unsigned i=0;i<pelvisControlCount;i++)memcpy(raw+pelvisControlIndices[i]*32,pelvisControlBasePositions+i*3,12);
 for(unsigned i=0;i<graftCount;i++)memcpy(raw+(47050+i)*32,morph_base+i*3,12);
 preparedShapeReady=shaftRestFrameReady=eggRestReady=constraintSolverReady=false;
 paBasisSaved=false;ResetCompliantDynamics();EvaluateAnatomy(raw,47050);
 for(int frame=0;frame<atoi(argv[3]);frame++){
  UpdateConstraintSolver(1.f/60.f,0,0);EvaluateAnatomy(raw,47050);
 }
 Save(output/"anatomy.packed",nrPacked,sizeof(nrPacked));
 Save(output/"anatomy.indices",nrIndices,nrIndexCount*sizeof(*nrIndices));
 Save(output/"shaft-guide.f32",shaftNodes,sizeof(shaftNodes));
 for(unsigned s=0;s<2;s++)Save(output/("body"+std::to_string(s)+".packed"),sharedBodyOutput[s],sharedBodyCount[s]*32);
 return 0;
}
