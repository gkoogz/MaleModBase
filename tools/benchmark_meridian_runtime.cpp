#include <malemod/garments/meridian_runtime.hpp>
#include "meridian_recipe.h"
#include <fstream>
#include <chrono>
#include <cstdio>
#include <algorithm>
int main(int argc,char** argv){
 using namespace malemod::garments::meridian;
 if(argc!=2)return 1;std::ifstream in(argv[1],std::ios::binary);
 std::uint32_t counts[2];in.read((char*)counts,sizeof(counts));std::vector<Vec> source[2];
 for(int s=0;s<2;s++){source[s].resize(counts[s]);in.read((char*)source[s].data(),counts[s]*sizeof(Vec));}if(!in)return 2;
 Mesh mesh;std::vector<double> times;times.reserve(2000);float checksum=0;
 for(unsigned frame=0;frame<2100;frame++){
  auto began=std::chrono::steady_clock::now();
  mesh.Update(MeridianRecipe::bindings,MeridianRecipe::count,MeridianRecipe::faces,MeridianRecipe::faceCount,[&](unsigned s,unsigned id){if(id>=source[s].size())throw std::runtime_error("Donor outside benchmark source");return source[s][id];});
  checksum+=mesh.Vertices()[frame%MeridianRecipe::count].position[0];
  auto ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-began).count();if(frame>=100)times.push_back(ms);
 }
 std::sort(times.begin(),times.end());double mean=0;for(double t:times)mean+=t;mean/=times.size();
 std::printf("{\"vertices\":%u,\"triangles\":%u,\"iterations\":2000,\"meanMs\":%.6f,\"p95Ms\":%.6f,\"maxMs\":%.6f,\"checksum\":%.6f,\"nativeFPSMeasured\":false}\n",MeridianRecipe::count,MeridianRecipe::faceCount,mean,times[1900],times.back(),checksum);
}
