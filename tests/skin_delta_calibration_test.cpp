#include <malemod/surface/binding.hpp>
#include <iostream>
int main(){
 try{
  malemod::surface::CoordinateCalibration c;
  c.basis={{{0,-1,0},{1,0,0},{0,0,1}}};c.sourceRoot={10.3688097,.12673235,83.4225998};c.targetRoot={0,.11,.975};c.targetUnitsPerSourceUnit=.010255218584141074;
  auto apply=[](const std::array<double,16>& m,malemod::surface::PrecisePoint p){malemod::surface::PrecisePoint out{};for(unsigned i=0;i<3;i++){out[i]=m[i*4+3];for(unsigned j=0;j<3;j++)out[i]+=m[i*4+j]*p[j];}return out;};
  for(unsigned k=0;k<500;k++){
   double angle=k*.013;std::array<double,16> native={std::cos(angle),-std::sin(angle),0,.02*std::sin(k*.1),std::sin(angle),std::cos(angle),0,.03*std::cos(k*.1),0,0,1,.04*std::sin(k*.03),0,0,0,1};
   auto source=c.SkinDeltaToSource(native);malemod::surface::PrecisePoint point={k*.2-30,17-k*.09,80+k*.01};
   auto expected=apply(native,c.PointToTarget(point));auto actual=c.PointToTarget(apply(source,point));
   for(unsigned i=0;i<3;i++)if(std::abs(expected[i]-actual[i])>1e-12)throw std::runtime_error("Calibrated skin delta fails commuting transform");
  }
  std::array<double,16> bad{};try{c.SkinDeltaToSource(bad);throw std::runtime_error("Non-affine transform accepted");}catch(const std::invalid_argument&){}
  std::cout<<"PASS: 500 calibrated skin deltas preserve complete affine origin/basis/scale mapping.\n";
 }catch(const std::exception& e){std::cerr<<e.what()<<"\n";return 1;}
}
