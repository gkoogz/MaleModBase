#include <malemod/surface/wire.hpp>
#include <fstream>
#include <iostream>
using namespace malemod::surface;
int main(int argc,char** argv){
 if(argc!=4)return 2;int mode=std::atoi(argv[1]);Controls controls;controls.values[0]=float(std::atoi(argv[2]));
 Frame frame;frame.garment.enabled=mode==1;
 if(mode==1||mode==2){frame.garment.shaftAcceleration={0,0,6.05f};frame.garment.lobeAcceleration={{{0,0,3.96f},{0,0,3.96f}}};}
 Session session(controls);
 for(unsigned i=0;i<180;i++){
  if(mode>=3){frame.garment={};if(i==174||(mode==4&&i>174)){frame.garment.enabled=true;frame.garment.contactReaction=true;frame.garment.contactEpoch=3;frame.garment.contactSerial=1;frame.garment.rodImpulseTotals[7]={0,4,8};frame.garment.lobeImpulseTotals[0]={0,3,5};frame.garment.lobeAngularImpulseTotals[0]={2,1,0};}}
  if((mode==5||mode==6)&&i==174){auto queued=frame;queued.seconds=0;if(mode==6){queued.garment.rodImpulseTotals[7]={0,2,4};queued.garment.lobeImpulseTotals[0]={0,1.5,2.5};queued.garment.lobeAngularImpulseTotals[0]={1,.5,0};frame.garment.contactSerial=2;}session.Step(queued);}
  session.Step(frame);
 }
 auto out=session.Read();auto data=wire::Encode(out);std::ofstream file(argv[3],std::ios::binary);file.write((const char*)data.data(),data.size());
 std::cout<<out.shaftGuide.back().z<<" "<<out.lobeCenters[0].z<<" "<<out.lobeCenters[1].z<<"\n";
 return file?0:3;
}
