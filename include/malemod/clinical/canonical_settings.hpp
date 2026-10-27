#pragma once
#include <malemod/clinical/fluid_types.h>
#include <istream>
#include <sstream>
#include <cstdlib>
#include <cctype>

namespace malemod::clinical {
// Same [Fluid] keys, defaults, ranges and integer truncation as Wolverine's
// LoadVolumeConfig. Reject a complete invalid document atomically.
inline bool ReadCanonicalFluidSettings(std::istream& input,volumeFluid::Settings& output,std::string& error){
 volumeFluid::Settings settings;
 float capacity=float(settings.capacity),pressure=float(settings.iterations),viscosity=float(settings.viscIterations),plane=0,sides=float(settings.meshSides);
 struct Field {const char* name;float* value;float lo,hi;};
 Field fields[]={
  {"Volume",&settings.volume,.01f,4000},{"Duration",&settings.feed,.15f,10},{"PulseVolumeVariation",&settings.pulseVolumeVariation,0,.75f},{"PulseDurationVariation",&settings.pulseDurationVariation,0,.9f},{"AngleVariation",&settings.angleVariation,0,1},{"PulseTaper",&settings.pulseTaper,0,1},
  {"Viscosity",&settings.viscosity,0,100},{"SurfaceTension",&settings.tension,0,100},{"NozzleRadius",&settings.nozzle,.15f,4},{"Spacing",&settings.spacing,.15f,.8f},{"FlowVariation",&settings.flowVariation,0,.7f},{"Lifetime",&settings.lifetime,1,15},{"DropVolume",&settings.dropVolume,.005f,20},{"DropDuration",&settings.dropDuration,.3f,5},{"DropHold",&settings.dropHold,.1f,5},{"DropLength",&settings.dropLength,.5f,15},
  {"Capacity",&capacity,1024,65536},{"PressureIterations",&pressure,2,12},{"ViscosityIterations",&viscosity,2,32},{"CatchPlane",&plane,0,1},{"CatchDepth",&settings.catchDepth,5,150},{"LateralWobbleDegrees",&settings.lateralWobbleDegrees,0,8},{"PulseForceVariation",&settings.pulseForceVariation,0,.3f},{"ThreadSpacing",&settings.threadSpacing,.25f,2},{"Breakup",&settings.breakup,0,4},{"MeshSides",&sides,8,20}
 };
 auto trim=[](std::string text){const auto begin=text.find_first_not_of(" \t\r"),end=text.find_last_not_of(" \t\r");return begin==std::string::npos?std::string{}:text.substr(begin,end-begin+1);};
 auto equal=[](const std::string& a,const char* b){if(a.size()!=std::strlen(b))return false;for(unsigned i=0;i<a.size();i++)if(std::tolower(static_cast<unsigned char>(a[i]))!=std::tolower(static_cast<unsigned char>(b[i])))return false;return true;};
 bool fluid=false;std::string line;
 while(std::getline(input,line)){
  line=trim(line);if(line.empty()||line[0]==';'||line[0]=='#')continue;
  if(line.front()=='['&&line.back()==']'){fluid=equal(trim(line.substr(1,line.size()-2)),"Fluid");continue;}
  if(!fluid)continue;const auto at=line.find('=');if(at==std::string::npos)continue;
  const auto key=trim(line.substr(0,at)),text=trim(line.substr(at+1));
  for(auto& field:fields)if(equal(key,field.name)){
   char* end=nullptr;const auto value=std::strtof(text.c_str(),&end);
   if(end==text.c_str()||*end||!std::isfinite(value)||value<field.lo||value>field.hi){error="Invalid Fluid setting: "+key;return false;}*field.value=value;break;
  }
 }
 if(input.bad()){error="Cannot read Fluid settings";return false;}
 settings.capacity=int(capacity);settings.iterations=int(pressure);settings.viscIterations=int(viscosity);settings.catchPlane=plane>.5f;settings.meshSides=int(sides);output=settings;error.clear();return true;
}
}
