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
 {
  auto smooth=[](double t){t=std::clamp(t,0.,1.);return t*t*t*(10+t*(-15+6*t));};
  std::vector<Vec> folded;std::vector<Binding> ownership;std::vector<std::array<unsigned,3>> faces;
  for(unsigned i=0;i<=300;i++)for(unsigned j=0;j<9;j++){
   const double s=i/300.,y=(double(j)-4)*.2,depth=1.3-.4*smooth((s-.72)/.21)+.35*std::exp(-std::pow((s-.79)/.004,2)-y*y/.02)+.18*std::exp(-std::pow((s-.88)/.025,2)-y*y/.2);
   folded.push_back({120*s,y,-depth});Binding b;b.influences[0]={1,1,{}};ownership.push_back(b);
   if(i<300&&j<8){unsigned k=i*9+j;faces.push_back({k,k+1,k+9});faces.push_back({k+1,k+10,k+9});}
  }
  const unsigned duplicate=unsigned(folded.size());folded.push_back(folded[237*9+4]);ownership.push_back(ownership[237*9+4]);
  faces.push_back({duplicate,237*9+5,238*9+4});
  auto correction=FrenulumProfileCorrection(folded,ownership,frames,faces,1);
  Check(correction[237*9+4][2]>.25,"Distal spur survived final material fairing");
  Check(correction[264*9+4][2]>.10,"Bulb interrupts the dipping raphe transition");
  Check(correction[duplicate]==correction[237*9+4],"Coincident material aliases separated");
  for(unsigned i=0;i<folded.size();i++){
   Check(correction[i][1]==0,"Frenulum fairing moved lateral material coordinates");
   if(std::abs(folded[i][1])>=1.0||folded[i][0]<=.69*120||folded[i][0]>=.94*120)Check(correction[i]==Vec{},"Frenulum fairing changed a protected surface");
  }
  auto changed=folded;for(auto& v:changed)v=Add(Scale(rotate(v),3.2),{9,7,-4});
  auto result=FrenulumProfileCorrection(changed,ownership,rotated,faces,3.2);
  for(unsigned i=0;i<result.size();i++){auto error=Sub(result[i],Scale(rotate(correction[i]),3.2));Check(Dot(error,error)<1e-19,"Frenulum fairing depends on angle or units");}
 }
 std::cout<<"PASS ventral contour, protected surfaces and rigid transform/scale invariance\n";
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
