#include <malemod/surface/graft_runtime.hpp>
#include <cstring>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <chrono>
using namespace malemod::surface;
template<class T>void Read(std::ifstream& f,T& value){if(!f.read(reinterpret_cast<char*>(&value),sizeof(value)))throw std::runtime_error("Truncated graft fixture");}
template<class T>void ReadList(std::ifstream& f,std::vector<T>& list){for(auto& item:list)Read(f,item);}
int main(int argc,char** argv){
 if(argc!=3)return 2;
 try{
  std::ifstream f(argv[1],std::ios::binary);char magic[8];if(!f.read(magic,8)||std::memcmp(magic,"GRAFT001",8))throw std::runtime_error("Invalid graft fixture header");
  std::uint32_t n,faces,seams,locked;Read(f,n);Read(f,faces);Read(f,seams);Read(f,locked);
  if(!n||n>1000000||faces>2000000||seams>n||locked>n)throw std::runtime_error("Oversize graft fixture");
  GraftDomain d;d.points.resize(n);d.triangles.resize(faces);d.seams.resize(seams);d.protectedVertices.resize(locked);
  ReadList(f,d.points);ReadList(f,d.triangles);
  for(auto& e:d.seams){Read(f,e.slave);Read(f,e.a);Read(f,e.b);Read(f,e.weight);}ReadList(f,d.protectedVertices);
  GraftFrame frame;Read(f,frame.root);Read(f,frame.axis);Read(f,frame.up);Read(f,frame.radius);Read(f,frame.length);Read(f,frame.sourceLengthScale);
  std::vector<PrecisePoint> delta(n);ReadList(f,delta);if(f.peek()!=std::ifstream::traits_type::eof())throw std::runtime_error("Trailing graft fixture");
  auto start=std::chrono::steady_clock::now();GraftPlan plan(d,frame);auto prepared=std::chrono::steady_clock::now();
  const auto result=plan.SolveDisplacement(delta);
  for(unsigned i=0;i<16;i++)if(plan.SolveDisplacement(delta)!=result)throw std::runtime_error("Cached graft solve changed its output");
  auto solved=std::chrono::steady_clock::now();std::ofstream out(argv[2],std::ios::binary);
  out.write(reinterpret_cast<const char*>(result.data()),result.size()*sizeof(PrecisePoint));if(!out)throw std::runtime_error("Cannot write graft result");
  std::cout<<"graft unique="<<n<<" seams="<<seams<<" protected="<<locked<<"\n";
  std::cout<<"planMs="<<std::chrono::duration<double,std::milli>(prepared-start).count()<<" cachedSolveMs="<<std::chrono::duration<double,std::milli>(solved-prepared).count()/17<<"\n";
 }catch(const std::exception& e){std::cerr<<e.what()<<"\n";return 1;}
}
