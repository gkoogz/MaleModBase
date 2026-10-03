#include <malemod/surface/graft_runtime.hpp>
#include <cstring>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <chrono>
#include <string>
#include <cmath>
using namespace malemod::surface;
template<class T>void Read(std::ifstream& f,T& value){if(!f.read(reinterpret_cast<char*>(&value),sizeof(value)))throw std::runtime_error("Truncated graft fixture");}
template<class T>void ReadList(std::ifstream& f,std::vector<T>& list){for(auto& item:list)Read(f,item);}
int main(int argc,char** argv){
 if(argc!=3)return 2;
 try{
  std::ifstream f(argv[1],std::ios::binary);char magic[8];if(!f.read(magic,8)||(std::memcmp(magic,"GRAFT001",8)&&std::memcmp(magic,"GRAFT002",8)))throw std::runtime_error("Invalid graft fixture header");
  std::uint32_t n,faces,seams,locked;Read(f,n);Read(f,faces);Read(f,seams);Read(f,locked);
  if(!n||n>1000000||faces>2000000||seams>n||locked>n)throw std::runtime_error("Oversize graft fixture");
  GraftDomain d;d.points.resize(n);d.triangles.resize(faces);d.seams.resize(seams);d.protectedVertices.resize(locked);
  ReadList(f,d.points);ReadList(f,d.triangles);
  for(auto& e:d.seams){Read(f,e.slave);Read(f,e.a);Read(f,e.b);Read(f,e.weight);}ReadList(f,d.protectedVertices);
  if(!std::memcmp(magic,"GRAFT002",8)){std::uint32_t prescribed;Read(f,prescribed);if(prescribed>n)throw std::runtime_error("Oversize prescribed boundary");d.prescribedVertices.resize(prescribed);ReadList(f,d.prescribedVertices);}
  GraftFrame frame;Read(f,frame.root);Read(f,frame.axis);Read(f,frame.up);Read(f,frame.radius);Read(f,frame.length);Read(f,frame.sourceLengthScale);
  std::vector<PrecisePoint> delta(n);ReadList(f,delta);if(f.peek()!=std::ifstream::traits_type::eof())throw std::runtime_error("Trailing graft fixture");
  auto start=std::chrono::steady_clock::now();GraftPlan plan(d,frame);auto prepared=std::chrono::steady_clock::now();
  const auto result=plan.SolveDisplacement(delta);
  for(unsigned i=0;i<16;i++)if(plan.SolveDisplacement(delta)!=result)throw std::runtime_error("Cached graft solve changed its output");
  auto solved=std::chrono::steady_clock::now();std::ofstream out(argv[2],std::ios::binary);
  out.write(reinterpret_cast<const char*>(result.data()),result.size()*sizeof(PrecisePoint));if(!out)throw std::runtime_error("Cannot write graft result");
  std::ofstream weights(std::string(argv[2])+".weights",std::ios::binary);
  for(auto p:d.points){double w=GraftRecruitmentWeight(p,frame);weights.write(reinterpret_cast<const char*>(&w),sizeof(w));}
  if(!weights)throw std::runtime_error("Cannot write graft recruitment weights");
  double updateMs=0,freshMs=0;
  std::ofstream changedWeights(std::string(argv[2])+".updated.weights",std::ios::binary);
  for(unsigned i=0;i<5;i++){
   auto changed=frame;double angle=(int(i)-2)*.011;
   changed.axis={frame.axis[0]*std::cos(angle)-frame.axis[2]*std::sin(angle),0,frame.axis[0]*std::sin(angle)+frame.axis[2]*std::cos(angle)};
   double yaw=(int(i)-2)*.013,x=changed.axis[0],y=changed.axis[1];
   changed.axis[0]=x*std::cos(yaw)-y*std::sin(yaw);changed.axis[1]=x*std::sin(yaw)+y*std::cos(yaw);
   changed.up={-changed.axis[2],0,changed.axis[0]};double norm=std::sqrt(changed.up[0]*changed.up[0]+changed.up[2]*changed.up[2]);
   for(auto& component:changed.up)component/=norm;
   changed.radius*=1+(int(i)-2)*.013;changed.length*=1+(int(i)-2)*.007;changed.root[2]+=(int(i)-2)*.03;
   auto begin=std::chrono::steady_clock::now();plan.UpdateFrame(changed);auto updated=std::chrono::steady_clock::now();
   GraftPlan fresh(d,changed);auto rebuilt=std::chrono::steady_clock::now();
   if(plan.SolveDisplacement(delta)!=fresh.SolveDisplacement(delta))throw std::runtime_error("Updated graft frame differs from a fresh exact plan");
   for(auto p:d.points){double w=GraftRecruitmentWeight(p,changed);changedWeights.write(reinterpret_cast<const char*>(&w),sizeof(w));}
   updateMs+=std::chrono::duration<double,std::milli>(updated-begin).count();freshMs+=std::chrono::duration<double,std::milli>(rebuilt-updated).count();
  }
  if(!changedWeights)throw std::runtime_error("Cannot write updated recruitment weights");
  plan.UpdateFrame(frame);if(plan.SolveDisplacement(delta)!=result)throw std::runtime_error("Restoring graft frame changed its output");
  std::cout<<"graft unique="<<n<<" seams="<<seams<<" protected="<<locked<<"\n";
  std::cout<<"planMs="<<std::chrono::duration<double,std::milli>(prepared-start).count()<<" cachedSolveMs="<<std::chrono::duration<double,std::milli>(solved-prepared).count()/17<<"\n";
  std::cout<<"updatedFrameCases=5 updateMs="<<updateMs/5<<" freshPlanMs="<<freshMs/5<<" exactUpdatedReplay=true\n";
 }catch(const std::exception& e){std::cerr<<e.what()<<"\n";return 1;}
}
