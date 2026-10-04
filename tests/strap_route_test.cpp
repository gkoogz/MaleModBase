#include <malemod/garments/jockstrap.hpp>
#include <iostream>
using namespace malemod::garments;
static void Check(bool v,const char* m){if(!v)throw std::runtime_error(m);}
static double Turn(const std::vector<Sample>& path){double maximum=0;for(unsigned k=1;k+1<path.size();k++){auto a=Unit(Sub(path[k].position,path[k-1].position)),b=Unit(Sub(path[k+1].position,path[k].position));maximum=(std::max)(maximum,std::acos(std::clamp(Dot(a,b),-1.,1.)));}return maximum;}
int main(){try{
 std::vector<Sample> path;for(unsigned k=0;k<81;k++){double t=double(k)/80,angle=t*1.7;Sample s;s.position={std::sin(angle),std::cos(angle),.04*std::sin(t*55)};s.normal={0,0,1};s.lineage.donors[0]={Surface::Body,k,1};path.push_back(s);}
 unsigned calls=0;auto project=[&](Sample& s){Check(Finite(s.position),"Nonfinite trial route");s.position[2]=(std::max)(s.position[2],-.05);calls++;};
 auto smooth=drape::SmoothRoute(path,12,project);
 Check(calls==12*(path.size()-2),"Each physical trial point must be projected each sweep");
 Check(smooth.front().position==path.front().position&&smooth.back().position==path.back().position,"Fixed route endpoint moved");
 Check(Turn(smooth)<Turn(path)*.65,"Measured ribbon bends did not become smoother");
 for(unsigned k=0;k<path.size();k++){Check(smooth[k].position[2]>=-.05,"Route crossed physical support");for(unsigned j=0;j<path[k].lineage.donors.size();j++){auto a=path[k].lineage.donors[j],b=smooth[k].lineage.donors[j];Check(a.surface==b.surface&&a.vertex==b.vertex&&a.weight==b.weight,"Weighted source attachment ancestry changed");}}
 auto converted=path;for(auto& s:converted)s.position=Add(Mul(Point{-s.position[1],s.position[0],s.position[2]},100),{70,-30,90});
 auto moved=drape::SmoothRoute(converted,12,[](Sample& s){s.position[2]=(std::max)(s.position[2],85.);});
 for(unsigned k=0;k<path.size();k++){auto expected=Add(Mul(Point{-smooth[k].position[1],smooth[k].position[0],smooth[k].position[2]},100),{70,-30,90});Check(Length(Sub(moved[k].position,expected))<1e-10,"Strap fairing depends on game origin, axes or units");}
 auto unchanged=drape::SmoothRoute(path,0,[](Sample&){});for(unsigned k=0;k<path.size();k++)Check(unchanged[k].position==path[k].position,"Zero-sweep route changed");
 bool rejected=false;try{drape::SmoothRoute(std::vector<Sample>(2),12,[](Sample&){});}catch(const std::invalid_argument&){rejected=true;}Check(rejected,"Incomplete measured route accepted");
 Mesh ribbon;for(unsigned k=0;k<path.size();k++)for(unsigned j=0;j<4;j++){Vertex v;v.position=Add(path[k].position,{0,j<2?.012:-.012,j%2?.001:-.001});v.lineage=path[k].lineage;ribbon.vertices.push_back(v);}
 auto original=ribbon;RibbonLayout layout{0,unsigned(path.size()),4};unsigned groups=0;
 drape::BendRibbonGroups(ribbon,layout,[&](unsigned id,Point delta){groups++;for(unsigned j=0;j<4;j++)ribbon.vertices[id+j].position=Add(ribbon.vertices[id+j].position,delta);});
 Check(groups==path.size()-2,"Bending step did not address every nonattached material section");
 for(unsigned k=0;k<path.size();k++)for(unsigned j=0;j<4;j++){
  unsigned id=k*4+j;Check(Length(Sub(Sub(ribbon.vertices[id].position,ribbon.vertices[k*4].position),Sub(original.vertices[id].position,original.vertices[k*4].position)))<1e-12,"Bending deformed ribbon cross section");
  Check(ribbon.vertices[id].lineage.donors[0].vertex==original.vertices[id].lineage.donors[0].vertex,"Bending changed source lineage");
  if(k==0||k+1==path.size())Check(ribbon.vertices[id].position==original.vertices[id].position,"Bending changed endpoint attachment frame");
 }
 auto convertedRibbon=original;for(auto& v:convertedRibbon.vertices)v.position=Add(Mul(Point{-v.position[1],v.position[0],v.position[2]},100),{70,-30,90});
 drape::BendRibbonGroups(convertedRibbon,layout,[&](unsigned id,Point delta){for(unsigned j=0;j<4;j++)convertedRibbon.vertices[id+j].position=Add(convertedRibbon.vertices[id+j].position,delta);});
 for(unsigned id=0;id<ribbon.vertices.size();id++){auto p=ribbon.vertices[id].position;auto expected=Add(Mul(Point{-p[1],p[0],p[2]},100),{70,-30,90});Check(Length(Sub(convertedRibbon.vertices[id].position,expected))<1e-10,"Ribbon bending depends on units or frame");}
 // A kink immediately before a fixed underside cap must be fairable. The
 // old centered-only stencil never moved that section at all.
 Mesh cap;for(unsigned k=0;k<5;k++)for(unsigned j=0;j<4;j++){Vertex v;v.position={double(k),k==3?1.:0.,j%2?.01:-.01};cap.vertices.push_back(v);}
 auto before=cap;drape::BendRibbonGroups(cap,{0,5,4},[&](unsigned id,Point delta){for(unsigned j=0;j<4;j++)cap.vertices[id+j].position=Add(cap.vertices[id+j].position,delta);});
 Check(cap.vertices[12].position[1]<before.vertices[12].position[1],"Penultimate cap kink cannot relax");
 for(unsigned j=0;j<4;j++)Check(cap.vertices[j].position==before.vertices[j].position&&cap.vertices[16+j].position==before.vertices[16+j].position,"Cap fairing moved actual sewn endpoints");
 std::cout<<"PASS physical projection, smooth dense curve, exact fixed attachments and donor ancestry, unit/frame invariance\n";return 0;
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
