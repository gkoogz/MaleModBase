#include <malemod/surface/wire.hpp>
#include <fstream>
#include <iostream>
using namespace malemod::surface;
int main(int argc,char** argv){
 if(argc!=4)return 2;int mode=std::atoi(argv[1]);Controls controls;controls.values[0]=float(std::atoi(argv[2]));
 Frame frame;frame.garment.enabled=mode==1;
 if(mode){frame.garment.shaftAcceleration={0,0,6.05f};frame.garment.lobeAcceleration={{{0,0,3.96f},{0,0,3.96f}}};}
 Session session(controls);
 for(unsigned i=0;i<180;i++)session.Step(frame);
 auto out=session.Read();auto data=wire::Encode(out);std::ofstream file(argv[3],std::ios::binary);file.write((const char*)data.data(),data.size());
 std::cout<<out.shaftGuide.back().z<<" "<<out.lobeCenters[0].z<<" "<<out.lobeCenters[1].z<<"\n";
 return file?0:3;
}
