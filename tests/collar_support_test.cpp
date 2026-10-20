#include <malemod/surface/collar_support.hpp>
#include <iostream>
using namespace malemod::surface;
void Require(bool value,const char* why){if(!value)throw std::runtime_error(why);}
int main(){try{
 constexpr unsigned rays=32;
 std::vector<PrecisePoint> normalized;
 for(double scale:{.01,1.,100.}){
  GraftDomain d;std::vector<bool> body;std::vector<double> part,angle,station;
  const double levels[]={0,.08,.31,.41};
  for(unsigned ring=0;ring<4;ring++)for(unsigned ray=0;ray<rays;ray++){
   const double phi=6.28318530717958647692*ray/rays;
   d.points.push_back({25*levels[ring]*scale,3*scale*std::cos(phi),3*scale*std::sin(phi)});
   body.push_back(ring==0);part.push_back(ring?1:0);angle.push_back(phi);station.push_back(levels[ring]);
  }
  for(unsigned ring=0;ring<3;ring++)for(unsigned ray=0;ray<rays;ray++){
   unsigned a=ring*rays+ray,b=ring*rays+(ray+1)%rays,c=a+rays,e=b+rays;
   d.triangles.push_back({a,c,b});d.triangles.push_back({b,c,e});
  }
  auto reference=d.points,expanded=d.points;
  for(unsigned ring=0;ring<4;ring++)for(unsigned ray=0;ray<rays;ray++){
   unsigned i=ring*rays+ray;double r=ring==0?4:ring==1?2:6;
   reference[i][1]=r*scale*std::cos(angle[i]);reference[i][2]=r*scale*std::sin(angle[i]);expanded[i]=reference[i];
   if(ring==0){expanded[i][1]=7.2*scale*std::cos(angle[i]);expanded[i][2]=7.2*scale*std::sin(angle[i]);}
  }
  CollarSupport support(d,body,part,angle,station);
  GraftFrame frame{{0,0,0},{1,0,0},{0,0,1},6*scale,25*scale,scale};
  auto result=support.Apply(reference,expanded,frame,2*scale);
  Require(result[rays+rays/4][2]>reference[rays+rays/4][2]+scale,"Dorsal depression retained");
  for(unsigned i=0;i<rays;i++)Require(result[i]==expanded[i],"Recruited body changed during collar fit");
  for(unsigned i=3*rays;i<4*rays;i++)Require(result[i]==expanded[i],"Healthy shaft changed");
  frame.radius=2*scale;Require(support.Apply(reference,expanded,frame,2*scale)==expanded,"Neutral shape changed");
  for(auto& p:result)for(auto& x:p)x/=scale;
  if(normalized.empty())normalized=result;else for(unsigned i=0;i<result.size();i++)for(unsigned a=0;a<3;a++)Require(std::abs(result[i][a]-normalized[i][a])<1e-6,"Collar support depends on units");
 }
 std::cout<<"Collar support: dorsal repair, fixed recruited body/healthy shaft, neutral preservation and unit covariance passed\n";
 return 0;
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
