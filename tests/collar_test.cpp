#include <malemod/surface/collar_field.hpp>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>

namespace original {
using malemod::V3;using malemod::Dot;using std::min;using std::max;
float Smoother01(float value){value=max(0.f,min(1.f,value));return value*value*value*(value*(value*6.f-15.f)+10.f);}
#include "data/wolverine-collar-metric.inc"
}

int main(int argc,char** argv){
 std::ofstream output;if(argc==2){output.open(argv[1],std::ios::binary);if(!output)return 2;output<<std::setprecision(17)<<"x,y,z,axis_x,axis_z,up_x,up_z,radius,length,area,mask,screen\n";}
 std::mt19937 random(17041);std::uniform_real_distribution<float> v(-1,1);
 for(int k=0;k<1024;k++){
  malemod::V3 point{15+40*v(random),25*v(random),84.3f+40*v(random)};
  float radius=5.5f+4.5f*v(random),length=40+20*v(random);double area=(1+v(random))*2;
  float angle=1.9f*v(random);malemod::V3 axis{cosf(angle),0,sinf(angle)},up{-sinf(angle),0,cosf(angle)};
  auto reference=original::EvaluateMetricRow(point,{9,0,84.3f},axis,up,radius,length,area);
  auto portable=malemod::collar::EvaluateMetricRow(point,{9,0,84.3f},axis,up,radius,length,area);
  if(reference.mask!=portable.mask||reference.screen!=portable.screen||reference.area!=portable.area)return 1;
  if(output)output<<point.x<<','<<point.y<<','<<point.z<<','<<axis.x<<','<<axis.z<<','<<up.x<<','<<up.z<<','<<radius<<','<<length<<','<<area<<','<<reference.mask<<','<<reference.screen<<'\n';
 }
 std::cout<<"PASS 1024 active-source/portable collar metric cases.\n";
}
