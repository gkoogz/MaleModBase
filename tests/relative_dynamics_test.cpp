#include <malemod/surface/runtime.hpp>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
using namespace malemod::surface;
static void P(std::ostream& out,Point p){out<<'['<<p.x<<','<<p.y<<','<<p.z<<']';}
int main(int argc,char** argv){try{
 if(argc==4&&std::string(argv[1])=="--clock"){
  const float dt=std::stof(argv[3]);Session session;const int count=int(std::lround(3.f/dt));
  for(int i=0;i<count;i++){Frame f;f.seconds=dt;f.pitchForce=.35f;f.yawForce=-.2f;session.Step(f);}
  const auto out=session.Read();std::ofstream log(argv[2]);log<<"{\"rootDirection\":";P(log,out.rootDirection);
  log<<",\"rod\":[";for(unsigned j=0;j<12;j++){if(j)log<<',';P(log,out.shaftGuide[j]);}log<<"]}\n";
  return 0;
 }
 if(argc!=5)throw std::runtime_error("output state size stiffness");
 Controls c;c.values[0]=float(std::stoi(argv[2]));
 c.values[1]=c.values[2]=c.values[3]=c.values[5]=float(std::stoi(argv[3]));
 c.values[10]=c.values[14]=float(std::stoi(argv[4]));c.values[7]=35;
 Session session(c);std::ofstream log(argv[1]);
 // Settle before excitation: initial gravity collapse is not walking response.
 for(int i=0;i<200;i++){Frame f;f.seconds=.1f;session.Step(f);}
 for(int i=0;i<160;i++){
  Frame f;f.seconds=.05f;const float t=i*.05f;
  f.yawForce=i<80?.65f*std::sin(t*6.2831853f):0;
  f.pitchForce=i<80?.45f*std::sin(t*9.424778f):0;
  session.Step(f);auto o=session.Read();auto d=session.ReadDiagnostics();
  log<<"{\"time\":"<<t<<",\"skin\":"<<d.scrotalSkinDisplacement<<",\"protected\":"<<d.protectedShaftDisplacement<<",\"rod\":[";
  for(unsigned j=0;j<12;j++){if(j)log<<',';P(log,o.shaftGuide[j]);}
  log<<"],\"lobes\":[";for(unsigned j=0;j<2;j++){if(j)log<<',';P(log,o.lobeCenters[j]);}
  log<<"],\"anchors\":[";for(unsigned j=0;j<2;j++){if(j)log<<',';P(log,o.lobeAnchors[j]);}
  log<<"],\"length\":"<<o.restLength<<"}\n";
  for(auto p:o.shaftGuide)if(!std::isfinite(p.x+p.y+p.z))throw std::runtime_error("Nonfinite rod");
  if(d.protectedShaftDisplacement!=0)throw std::runtime_error("Skin displaced protected rod");
 }
 std::cout<<"PASS: settled driven/release trace, continuous rod, protected skin ownership\n";
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
