#include <malemod/surface/suspension_profile.hpp>
#include <iostream>
using namespace malemod::surface;
void Check(bool value,const char* message){if(!value)throw std::runtime_error(message);}
int main(){try{
 for(double h:{1.,10.,25.,50.,75.,100.}){
  Check(SuspensionSectionScale(0,h)==1&&SuspensionSectionScale(1,h)==1,"Attachment endpoints moved");
  const double epsilon=1e-5;
  Check(std::abs(SuspensionSectionScale(epsilon,h)-1)/epsilon<.001,"Lower join tangent changed");
  Check(std::abs(SuspensionSectionScale(1-epsilon,h)-1)/epsilon<.001,"Upper join tangent changed");
  for(unsigned i=0;i<=100;i++)Check(SuspensionSectionScale(i/100.,h)>=.619,"Neck collapsed");
 }
 double previous=2;
 for(unsigned h=1;h<=100;h++){double value=SuspensionSectionScale(2./3.,h);Check(value<=previous,"Hang waist is not monotone");previous=value;}
 Check(SuspensionSectionScale(2./3.,1)>1,"Low Hang is not fuller");
 Check(SuspensionSectionScale(2./3.,100)<.65,"Long Hang lacks a waist");
 std::vector<PresentationPoint> points{{2,0,3},{0,2,3},{-2,0,3},{0,-2,3},{10,20,30}};
 const auto original=points;std::vector<SuspensionSection> sections{{2./3.,{0,1,2,3}}};
 ShapeSuspensionSections(points,sections,100);Check(points[4]==original[4],"Retained tissue changed");
 auto translated=original;for(auto& p:translated)p=Add(Scale(p,3),{9,-4,7});
 ShapeSuspensionSections(translated,sections,100);
 for(unsigned i=0;i<points.size();i++){auto error=Sub(translated[i],Add(Scale(points[i],3),{9,-4,7}));Check(Dot(error,error)<1e-20,"Profile depends on scale/origin");}
 bool rejected=false;try{SuspensionSectionScale(.5,0);}catch(const std::invalid_argument&){rejected=true;}Check(rejected,"Invalid Hang accepted");
 // A curved trumpet around two retained contents, with irregular meridian
 // spacing like the native clipped compound strip. Only its free skin moves.
 std::vector<PresentationPoint> strip;std::vector<std::vector<unsigned>> rings;
 std::vector<double> parameters{0,.04,.12,.20,.32,.45,.6,.76,.9,.96,1};
 for(double t:parameters){std::vector<unsigned> ring;double radius=1+.7*(2*t-1)*(2*t-1);
  for(unsigned j=0;j<24;j++){double angle=6.283185307179586*j/24;ring.push_back(unsigned(strip.size()));strip.push_back({radius*std::cos(angle)+.25*t*t,radius*std::sin(angle),6*t});}rings.push_back(ring);
 }
 const auto unprofiled=strip;
 Check(ShapeSuspensionStrip(strip,rings,parameters,1,100)==1,"Ordinary neck unnecessarily limited");
 for(unsigned row=0;row<rings.size();row++)for(auto id:rings[row]){
  if(parameters[row]<=.20||parameters[row]>=.9)Check(strip[id]==unprofiled[id],"Compound tangent band moved");
  auto tangent=PresentationPoint{.5*parameters[row],0,6};
  auto delta=Sub(strip[id],unprofiled[id]);
  // The finite centerline tangent is exact for symmetric interior samples.
  if(row==5){const double a=parameters[row-1],b=parameters[row+1];tangent={.25*(a+b),0,6};Check(std::abs(Dot(delta,tangent))<1e-12,"Profile changed axial height");}
 }
 auto changed=strip;Check(ShapeSuspensionStrip(strip,rings,parameters,100,1)==1,"Reverse ordinary neck limited");
 auto flat=unprofiled;ShapeSuspensionStrip(flat,parameters,24,1,100);Check(flat==changed,"Flat and indexed profiles differ");
 for(unsigned i=0;i<strip.size();i++)Check(Dot(Sub(strip[i],unprofiled[i]),Sub(strip[i],unprofiled[i]))<1e-25,"Profile accumulated or failed reversal");
 auto intermediate=unprofiled;ShapeSuspensionStrip(intermediate,rings,parameters,1,50);ShapeSuspensionStrip(intermediate,rings,parameters,50,100);
 for(unsigned i=0;i<strip.size();i++)Check(Dot(Sub(intermediate[i],changed[i]),Sub(intermediate[i],changed[i]))<1e-25,"Two-step profile differs from direct profile");
 auto partial=unprofiled;for(unsigned i=0;i<partial.size();i++)partial[i]=Add(unprofiled[i],Scale(Sub(changed[i],unprofiled[i]),.37));
 auto removed=partial;ShapeSuspensionStrip(removed,rings,parameters,100,1,.37);
 for(unsigned i=0;i<strip.size();i++)Check(Dot(Sub(removed[i],unprofiled[i]),Sub(removed[i],unprofiled[i]))<1e-25,"Partially applied profile did not remove exactly");
 ShapeSuspensionStrip(partial,parameters,24,100,100,.37);
 for(unsigned i=0;i<strip.size();i++)Check(Dot(Sub(partial[i],changed[i]),Sub(partial[i],changed[i]))<1e-25,"Partially applied profile did not reach target");
 auto transformed=unprofiled;for(auto& p:transformed)p={3*p[2]+5,3*p[0]-2,3*p[1]+9};
 ShapeSuspensionStrip(transformed,rings,parameters,1,100);
 for(unsigned i=0;i<strip.size();i++){auto p=changed[i];auto expected=PresentationPoint{3*p[2]+5,3*p[0]-2,3*p[1]+9};Check(Dot(Sub(transformed[i],expected),Sub(transformed[i],expected))<1e-24,"Compound profile depends on world axes or scale");}
 auto compact=unprofiled;for(unsigned row=0;row<rings.size();row++)for(auto id:rings[row]){compact[id][0]-=.25*parameters[row]*parameters[row];compact[id][2]*=.01;}const auto compactBefore=compact;
 ShapeSuspensionStrip(compact,rings,parameters,1,100);Check(compact==compactBefore,"Compact neck gained a waist without free span");
 for(double t:{.20,.9}){double eps=1e-5;Check(std::abs(SuspensionStripScale(t+eps,100,2)-SuspensionStripScale(t-eps,100,2))/(2*eps)<1e-5,"Compound profile tangent band is not C1");}
 rejected=false;try{ShapeSuspensionStrip(strip,rings,parameters,0,100);}catch(const std::invalid_argument&){rejected=true;}Check(rejected,"Invalid compound Hang accepted");
 bool limited=false;
 for(unsigned stress=1;stress<=32;stress++){
  auto warped=unprofiled;
  // Irregular, short cut ears next to a broad lobe: the waist must not
  // overturn these faces even when the full requested change cannot fit.
  for(unsigned j=0;j<24;j++){auto& p=warped[rings[5][j]];p[2]+=.04*stress*std::sin(j*1.7);p[0]+=.05*stress*std::cos(j*.9);}
  const auto before=warped;double fraction=ShapeSuspensionStrip(warped,rings,parameters,1,100);limited|=fraction<1;
  auto verify=[&](unsigned a,unsigned b,unsigned c){auto n=Cross(Sub(before[b],before[a]),Sub(before[c],before[a]));auto m=Cross(Sub(warped[b],warped[a]),Sub(warped[c],warped[a]));Check(Dot(n,m)>=.099999*Dot(n,n),"Profile collapsed or inverted a cut ear");};
  for(unsigned row=0;row+1<rings.size();row++)for(unsigned j=0;j<24;j++){unsigned k=(j+1)%24;verify(rings[row][j],rings[row][k],rings[row+1][j]);verify(rings[row][k],rings[row+1][k],rings[row+1][j]);}
 }
 Check(limited,"Stress case never exercised orientation limit");
 std::cout<<"PASS attachment tangents, bounded monotone Hang waist, retained ownership and scale invariance\n";return 0;
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
