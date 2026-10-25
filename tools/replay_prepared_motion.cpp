#include <malemod/surface/prepared_motion.hpp>
#include <fstream>
#include <iostream>
#include <chrono>
using namespace malemod::surface;
int main(int argc,char** argv){try{
 if(argc<2||argc>4)throw std::invalid_argument("Usage: replay_prepared_motion captured-case [OBJ-prefix] [anchored-rim]");
 std::ifstream file(argv[1],std::ios::binary);std::array<unsigned,2> count{};file.read(reinterpret_cast<char*>(count.data()),8);
 if(!file||count[0]>2000000||count[1]>4000000)throw std::invalid_argument("Invalid private motion case");
 auto read=[&](auto& data,unsigned n){data.resize(n);file.read(reinterpret_cast<char*>(data.data()),n*sizeof(data[0]));if(!file)throw std::invalid_argument("Truncated motion case");};
 std::vector<PresentationPoint> before,failed,expected,proposed;std::vector<std::array<unsigned,3>> faces;std::vector<PreparedMotionWeights> weights;
 read(before,count[0]);read(failed,count[0]);read(faces,count[1]);read(expected,count[1]);read(proposed,count[0]);read(weights,count[0]);
 if(file.peek()!=EOF){
  std::array<unsigned,4> sizes{};file.read(reinterpret_cast<char*>(sizes.data()),sizeof(sizes));
  std::vector<LiveControlLoft> lofts;std::vector<LiveControlTangents> tangents;std::vector<LiveControlDonor> body,compound;
  read(lofts,sizes[0]);read(tangents,sizes[0]);PreparedMotionFrames from,to;std::array<PresentationPoint,17> scale;
  file.read(reinterpret_cast<char*>(from.data()),sizeof(from));file.read(reinterpret_cast<char*>(to.data()),sizeof(to));file.read(reinterpret_cast<char*>(scale.data()),sizeof(scale));read(body,sizes[2]);read(compound,sizes[3]);
  if(argc==4){if(std::string(argv[3])!="anchored-rim")throw std::invalid_argument("Unknown replay comparison");to[15]=from[15];to[16]=from[16];}
  PreparedMotionPlan motion(from,to,scale);proposed=motion.Evaluate(before,weights);
  DeformLiveControlDonors(body,before,proposed);TransportPreparedMotionLofts(motion,weights,lofts,tangents,before,proposed,0,sizes[1]);
  DeformLiveControlDonors(compound,before,proposed);TransportPreparedMotionLofts(motion,weights,lofts,tangents,before,proposed,sizes[1],sizes[0]);
  for(unsigned i=0;i<faces.size();i++){auto f=faces[i];PreparedMotionWeights w{};for(auto v:f)AddMotionWeights(w,weights[v],1./3);expected[i]=motion.Normal(Cross(Sub(before[f[1]],before[f[0]]),Sub(before[f[2]],before[f[0]])),w);}
 }
 if(argc>=3){std::ofstream out(std::string(argv[2])+"-proposed.obj");out.precision(17);for(auto p:proposed)out<<"v "<<p[0]<<' '<<p[1]<<' '<<p[2]<<'\n';for(auto f:faces)out<<"f "<<f[0]+1<<' '<<f[1]+1<<' '<<f[2]+1<<'\n';}
 PreparedMotionAdjacency adjacency(count[0],faces);auto after=proposed;
 auto start=std::chrono::steady_clock::now();auto correction=ConstrainPreparedMotion(before,after,faces,weights,.01,&expected,&adjacency);
 std::cout<<"PASS projections="<<correction.projections<<" maximum="<<correction.maximum<<" milliseconds="<<std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count()<<'\n';
 if(argc>=3){auto obj=[&](const char* name,const auto& points){std::ofstream out(std::string(argv[2])+name+".obj");out.precision(17);for(auto p:points)out<<"v "<<p[0]<<' '<<p[1]<<' '<<p[2]<<'\n';for(auto f:faces)out<<"f "<<f[0]+1<<' '<<f[1]+1<<' '<<f[2]+1<<'\n';};obj("-before",before);obj("-proposed",proposed);obj("-corrected",after);}
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
