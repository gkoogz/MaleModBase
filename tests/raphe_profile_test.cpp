#include <malemod/surface/raphe_profile.hpp>
#include <iostream>
using namespace malemod::surface;
using namespace malemod::surface::modular;
void Check(bool value,const char* message){if(!value)throw std::runtime_error(message);}
int main(){try{
 {
  std::vector<Vec> before{{0,0,0},{1,0,0},{0,1,0}},after{{0,0,0},{1,0,0},{0,-1,0}};
  const double fraction=RapheCorrectionFraction(before,after,{{0,1,2}});
  Check(fraction>0&&fraction<.375,"Inverting profile correction was not bounded");
  Check(1-2*fraction>=.25,"Profile backoff loses its oriented area margin");
  Check(RapheCorrectionFraction(before,before,{{0,1,2}})==1,"Safe profile unnecessarily weakened");
 }
 PresentationFrames frames;for(unsigned i=1;i<=12;i++)frames[i].origin={double(i-1)*10,0,0};
 std::vector<Vec> p;std::vector<Binding> owners;std::vector<double> crown;
 for(unsigned i=0;i<=220;i++){
  double x=i*.5,s=x/110,wave=s>.2&&s<.6?.12*std::sin((s-.2)*40):0;
  for(unsigned j=0;j<4;j++){
   p.push_back({x,0,j==0?-1.3-wave:j==1?1.:j==2?-3.:-1.3});
   Binding b;b.influences[0]={j==2?13u:1u,1,{}};owners.push_back(b);crown.push_back(j==3||s>=.75?1:0);
  }
 }
 auto d=RapheProfileCorrection(p,owners,frames,crown,1);
 double before=0,after=0;
 for(unsigned i=0;i<p.size();i++){
  auto s=p[i][0]/110;
  if(i%4||s<=.12||s>=.74)Check(d[i]==Vec{},"Protected root, dorsal surface, lobe or crown changed");
  if(i%4==0&&s>.33&&s<.47){before+=std::abs(p[i][2]+1.3);after+=std::abs(p[i][2]+d[i][2]+1.3);}
  Check(d[i][0]==0&&d[i][1]==0,"Profile correction moved an axial station or width");
 }
 Check(before>.1&&after<before*.001,"Midshaft longitudinal waviness was retained");
 auto transformed=p;auto rotated=frames;
 auto rotate=[](Vec v){return Vec{-v[2],v[1],v[0]};};
 for(auto& v:transformed)v=Add(Scale(rotate(v),3.2),{9,7,-4});
 for(auto& f:rotated){f.origin=Add(Scale(rotate(f.origin),3.2),{9,7,-4});for(auto& b:f.basis)b=rotate(b);}
 auto e=RapheProfileCorrection(transformed,owners,rotated,crown,3.2);
 for(unsigned i=0;i<d.size();i++){auto error=Sub(e[i],Scale(rotate(d[i]),3.2));Check(Dot(error,error)<1e-20,"Profile is not scale/angle invariant");}
 std::cout<<"PASS ventral contour, protected surfaces and rigid transform/scale invariance\n";
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
