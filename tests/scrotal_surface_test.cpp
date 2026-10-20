#include <malemod/surface/wire.hpp>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
using namespace malemod::surface;
static void Check(bool x,const char* why){if(!x)throw std::runtime_error(why);}
static double Distance(Point a,Point b){return std::sqrt(double(a.x-b.x)*(a.x-b.x)+double(a.y-b.y)*(a.y-b.y)+double(a.z-b.z)*(a.z-b.z));}
static void P(std::ostream& o,Point p){o<<'['<<p.x<<','<<p.y<<','<<p.z<<']';}
int main(int argc,char** argv){try{
 Check(argc==5,"state size stiffness output-prefix");Controls c;c.values[0]=float(std::stoi(argv[1]));
 c.values[1]=c.values[2]=c.values[3]=c.values[5]=float(std::stoi(argv[2]));c.values[14]=float(std::stoi(argv[3]));c.values[7]=35;
 Session session(c);std::string prefix=argv[4];std::ofstream report(prefix+".jsonl");
 double maxSkin=0,maxProtected=0,minFraction=1,maxRodStretch=0,maxLobeMovement=0;auto initial=session.Read();
 for(unsigned i=0;i<120;i++){
  Frame f;f.seconds=1.f/60;f.pitchForce=i<90?.28f*std::sin(i*.23f):0;f.yawForce=i<90?.22f*std::sin(i*.31f):0;
  session.Step(f);auto out=session.Read();auto d=session.ReadDiagnostics();
  maxSkin=std::max(maxSkin,double(d.scrotalSkinDisplacement));maxProtected=std::max(maxProtected,double(d.protectedShaftDisplacement));minFraction=std::min(minFraction,double(d.scrotalSkinFraction));
  Check(d.protectedShaftDisplacement==0,"Secondary skin displaced a shaft/body-owned vertex");
  Check(std::isfinite(d.scrotalSkinDisplacement),"Nonfinite secondary skin");
  for(unsigned j=1;j<12;j++){
   double length=Distance(out.shaftGuide[j-1],out.shaftGuide[j]);Check(length>1e-5&&std::isfinite(length),"Continuous hidden rod lost a segment");
   maxRodStretch=std::max(maxRodStretch,std::abs(length/(out.restLength/11)-1));
  }
  for(unsigned j=0;j<2;j++)maxLobeMovement=std::max(maxLobeMovement,Distance(initial.lobeCenters[j],out.lobeCenters[j]));
  if(i%10==0||i==119){
   report<<"{\"frame\":"<<i<<",\"skin\":"<<d.scrotalSkinDisplacement<<",\"fraction\":"<<d.scrotalSkinFraction<<",\"rod\":[";
   for(unsigned j=0;j<12;j++){if(j)report<<',';P(report,out.shaftGuide[j]);}report<<"],\"lobes\":[";
   for(unsigned j=0;j<2;j++){if(j)report<<',';P(report,out.lobeCenters[j]);}report<<"],\"anchors\":[";
   for(unsigned j=0;j<2;j++){if(j)report<<',';P(report,out.lobeAnchors[j]);}report<<"]}\n";
  }
  if(i==60||i==119){auto bytes=wire::Encode(out);std::ofstream b(prefix+"-"+std::to_string(i)+".bin",std::ios::binary);b.write(reinterpret_cast<const char*>(bytes.data()),bytes.size());}
 }
 Check(maxSkin>.001,"Secondary skin was entirely suppressed");Check(maxLobeMovement>.01,"Suspended contents did not move");
 report<<"{\"passed\":true,\"maxSkin\":"<<maxSkin<<",\"protectedDisplacement\":"<<maxProtected<<",\"minFraction\":"<<minFraction<<",\"maxRodStretch\":"<<maxRodStretch<<",\"maxLobeMovement\":"<<maxLobeMovement<<"}\n";
 std::cout<<"PASS skin="<<maxSkin<<" protected="<<maxProtected<<" minFraction="<<minFraction<<" rodStretch="<<maxRodStretch<<'\n';
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
