#include <malemod/surface/live_controls.hpp>
#include <iostream>
#include <stdexcept>
using namespace malemod::surface;
static void check(bool v,const char* s){if(!v)throw std::runtime_error(s);}
static double error(PresentationPoint a,PresentationPoint b){auto d=Sub(a,b);return std::sqrt(Dot(d,d));}
int main(){try{
 LiveControlFrame f;f.basis={{{1,0,0},{0,1,0},{0,0,1}}};f.root={4,0,80};f.tip={30,0,80};f.lobes={6,0,74};f.bodyOrigin=f.root;f.radius=3;f.bodyRadius=4;
 Controls a,b;
 LiveControlBinding shaft{1,0,0,0,false},fixed{0,0,0,0,true},body{0,0,0,1,false};
 for(float size:{1.f,25.f,50.f,75.f,100.f})for(float angle:{10.f,15.f,25.f,50.f,75.f,90.f}){
  b.values[1]=b.values[3]=size;b.values[7]=angle;
  LiveControlDeformation m(f,a,b);
  check(error(m.Point(f.root,shaft),f.root)<1e-12,"Rotation left the root");
  auto p=PresentationPoint{4,3,80};
  check(error(m.Point(p,fixed),p)==0,"Protected body moved");
  auto j=m.Differential({15,2,81},shaft);auto normal=m.Normal({0,1,0},j),tangent=m.Tangent({1,0,0},j);
  check(std::abs(Dot(normal,tangent))<1e-9,"Nonuniform scale broke the tangent plane");
 }
 b=a;LiveControlDeformation identity(f,a,b);
 check(error(identity.Point({13,1,84},shaft),{13,1,84})==0,"Equal controls changed geometry");
 b.values[3]=100;LiveControlDeformation width(f,a,b);
 check(width.Point({4,3,80},body)[1]>3,"Body opening failed to dilate with width");
 check(error(width.Point({4,3,80},body),width.Point({4,3,80},shaft))<1e-12,"Radial connection separated at the root");
 // A subdivided edge has to remain on its parent edge even when the nonlinear
 // field initially places its midpoint elsewhere. UV aliases share donors.
 std::vector<PresentationPoint> before{{0,0,0},{2,0,0},{0,2,0},{1,0,0},{1,0,0}};
 auto after=before;after[0]={0,1,0};after[1]={4,3,0};after[3]={1,8,0};after[4]={1,-2,0};
 std::vector<LiveControlDonor> donors{{3,{0,1,2},{.5,.5,0}},{4,{0,1,2},{.5,.5,0}}};
 DeformLiveControlDonors(donors,before,after);
 check(error(after[3],Scale(Add(after[0],after[1]),.5))<1e-12,"Subdivision bent the source edge");
 check(after[3]==after[4],"Source-edge aliases separated");
 // Loft identity must preserve sculpted residual detail, not flatten the
 // connector to a new analytic surface whenever a presentation is prepared.
 before={{4,0,80},{8,0,80},{6,1,80}};after=before;
 std::vector<LiveControlBinding> bindings(3,shaft);
 DeformLiveControlLofts(identity,bindings,{{2,0,1,.5}},{{{4,0,0},{4,0,0}}},before,after);
 check(error(after[2],before[2])<1e-8,"Identity erased loft residual detail");
 // A connector can start on another loft, with an affine edge subdivision
 // between them. Independent point morphing must not bend that shared edge.
 before={{4,0,80},{8,2,80},{6,1,81},{7,1.5,80.5},{8,0,84},{7.5,.75,82.25}};
 bindings.assign(before.size(),shaft);bindings[2]={0,1,0,0,false};
 std::vector<LiveControlLoft> staged{{2,0,1,.5},{5,3,4,.5}};
 std::vector<LiveControlTangents> handles{{{4,2,0},{4,2,0}},{{1,-1.5,3.5},{1,-1.5,3.5}}};
 after=before;for(unsigned i=0;i<after.size();i++)after[i]=width.Point(before[i],bindings[i]);
 const auto untouched=after[5];malemod::TaskArena arena(2);
 DeformLiveControlLofts(width,bindings,staged,handles,before,after,&arena,0,1);
 check(after[5]==untouched,"First loft stage wrote a later stage");
 DeformLiveControlDonors({{3,{2,1,0},{.5,.5,0}}},before,after);
 check(error(after[3],Scale(Add(after[2],after[1]),.5))<1e-12,"Dependent split left the updated edge");
 const auto endpoint=after[3];
 DeformLiveControlLofts(width,bindings,staged,handles,before,after,&arena,1);
 check(after[3]==endpoint,"Second loft stage moved its shared endpoint");
 auto serial=before;for(unsigned i=0;i<serial.size();i++)serial[i]=width.Point(before[i],bindings[i]);
 DeformLiveControlLofts(width,bindings,{staged[0]},{handles[0]},before,serial);
 DeformLiveControlDonors({{3,{2,1,0},{.5,.5,0}}},before,serial);
 DeformLiveControlLofts(width,bindings,{staged[1]},{handles[1]},before,serial);
 for(unsigned i=0;i<serial.size();i++)check(error(after[i],serial[i])<1e-12,"Staged parallel result differs from dependency order");
 bool rejected=false;try{DeformLiveControlLofts(width,bindings,staged,handles,before,after,&arena,2,1);}catch(const std::invalid_argument&){rejected=true;}
 check(rejected,"Invalid loft stage range was accepted");
 std::cout<<"PASS live controls: root pivot, protected boundary, dilation, normal differentials, source-edge donors and loft detail\n";
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
