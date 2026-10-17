#include <malemod/surface/modular_motion.hpp>
#include <malemod/surface/modular_transfer.hpp>
#include <iostream>
using namespace malemod::surface;
using namespace malemod::surface::modular;
void Check(bool v,const char* message){if(!v)throw std::runtime_error(message);}
int main(){try{
 State rest;std::vector<Vec> p{{0,0,0},{1,0,0},{1,0,-2},{1,0,-1}};
 std::vector<Binding> b(4);for(unsigned i=0;i<4;i++)b[i].influences[0]={i==2?13u:i==0?0u:1u,1,p[i]};
 Loft l{3,2,1,.5,{0,0,2},{0,0,2},p[3],b[2],b[1]};
 Plan plan(p,b,{l},rest);auto baseline=plan.Evaluate(rest);
 {
  std::vector<Vec> q{{3,-1,0},{3,1,0},{3,0,2},{0,-1,-2},{0,1,-2},{0,0,-2},{1.5,-1,-1},{1.5,1,-1},{1.5,0,0}};
  std::vector<Binding> owners(q.size());for(unsigned i=0;i<q.size();i++)owners[i].influences[0]={i<3?1u:13u,1,q[i]};
  std::vector<Loft> skin;for(unsigned i=0;i<3;i++)skin.push_back({i+6,i+3,i,.5,{0,0,1},{0,0,1},q[i+6],owners[i+3],owners[i]});
  Plan suspension(q,owners,skin,rest);auto offset=suspension.SuspensionRestOffset(q,rest);
  Check(std::abs(offset[0]-3)<1e-12&&std::abs(offset[1])<1e-12&&offset[2]==0,"Suspended rest position is not directly below its attachment");
  auto translated=q;suspension.TranslateSuspension(translated,offset);
  for(unsigned i=0;i<3;i++)Check(translated[i]==q[i],"Suspended alignment moved the complete shaft");
  for(unsigned i=3;i<6;i++)Check(translated[i]==Add(q[i],offset),"Suspended alignment distorted the retained skin");
 }
 for(unsigned i=0;i<p.size();i++)Check(Dot(Sub(baseline[i],p[i]),Sub(baseline[i],p[i]))<1e-24,"Rest geometry changed");
 State moved=rest;moved.frames[13].origin={.4,.2,-.3};auto result=plan.Evaluate(moved);
 Check(result[0]==p[0]&&result[1]==p[1],"Suspended motion changed fixed body or complete shaft");
 Check(result[2]==Add(p[2],moved.frames[13].origin),"Suspended skin failed to follow its lobe");
 Check(result[3]!=p[3],"Connecting skin is rigid");
 moved=rest;moved.frames[1].origin={0,0,1};result=plan.Evaluate(moved);
 Check(result[2]==p[2],"Shaft motion was incorrectly baked into the suspended lobe");
 Check(result[1]==Add(p[1],Vec{0,0,1}),"Shaft frame failed to drive shaft");
 moved=rest;moved.scales[1]={1,2,2};result=plan.Evaluate(moved);Check(result[2]==p[2],"Shaft width changed lobe size");
 // A second source-length correction must not invert axial station order.
 // Cross-sectional morphology is retained in the moving rod frame.
 auto section=CrossSectionResidual(b[1],rest,{-5,2,3});
 Check(section==Vec{0,2,3},"Legacy axial correction changed the complete rod");
 Check(CrossSectionResidual(b[1],rest,{-5,2,3},1)==Vec{-5,2,3},"Authored distal shape lost its independent axial morphology");
 moved=rest;moved.frames[1].basis={Vec{0,0,1},Vec{0,1,0},Vec{-1,0,0}};
 section=CrossSectionResidual(b[1],moved,{3,2,-5});
 Check(section==Vec{3,2,0},"Section transfer does not follow the root angle");
 State anchored;anchored.hasShapeFrames=true;
 for(unsigned i=1;i<=12;i++){
  anchored.shapeFrames[i].origin={double(i-1),0,0};
  anchored.frames[i].origin={double(i-1),.5,0};
 }
 anchored.frames[13].origin={2,3,-4};anchored.frames[14].origin={2,-3,-5};
 AnchorRootPhysics(anchored,2,20);
 Check(anchored.frames[1].origin==anchored.shapeFrames[1].origin,"Root physics moved the fixed opening");
 Check(anchored.frames[2].origin[1]>0&&anchored.frames[2].origin[1]<.5,"Root release is not gradual");
 Check(anchored.frames[3].origin[1]==.5,"Root constraint still locks a quarter of the rod");
 Check(anchored.frames[5].origin[1]==.5&&anchored.frames[12].origin[1]==.5,"Root constraint suppressed distal physics");
 Check(anchored.frames[13].origin==Vec{2,3,-4}&&anchored.frames[14].origin==Vec{2,-3,-5},"Root constraint moved independent lobes");
 // A section must retain its diameter through a ninety-degree bend, and
 // the crown must retain all pairwise distances while its frame rotates.
 State material;material.hasShapeFrames=true;
 for(unsigned i=1;i<=12;i++){material.shapeFrames[i].origin={double(i-1),0,0};material.frames[i]=material.shapeFrames[i];}
 Binding middle;middle.influences[0]={5,.5,{.5,1,0}};middle.influences[1]={6,.5,{-.5,1,0}};
 auto remapped=BindMaterialStations({{8.25,1,0}}, {middle},material.shapeFrames);
 Check(remapped[0].influences[0].frame==9&&std::abs(remapped[0].influences[1].weight-.25)<1e-12,"Morphology retained stale material stations");
 auto restTransport=MaterialPoint(remapped[0],material,{8.25,1,0});
 Check(restTransport==Vec{8.25,1,0},"Rebinding moved selected rest geometry");
 // Test the surface, not just guide positions: a wide section amplifies a
 // discontinuous angular derivative into the root step seen during walking.
 {
  auto bent=material;
  for(unsigned i=2;i<=12;i++){
   const double angle=.7*std::sin((i-1)*.3);
   bent.frames[i].basis={Vec{std::cos(angle),0,std::sin(angle)},Vec{0,1,0},Vec{-std::sin(angle),0,std::cos(angle)}};
   bent.frames[i].origin=Add(bent.frames[i-1].origin,Scale(bent.frames[i].basis[0],1.));
  }
  auto sample=[&](double station,double ray){
   unsigned i=std::min(11u,1u+unsigned(std::max(0.,station)));
   const double t=std::clamp(station-double(i-1),0.,1.);
   Binding b;b.influences={Influence{i,1-t,{}},Influence{i+1,t,{}}};
   return MaterialPoint(b,bent,{station,2*std::cos(ray),2*std::sin(ray)});
  };
  const double e=1e-5;
  for(unsigned knot=0;knot<11;knot++)for(unsigned ray=0;ray<16;ray++){
   const double x=double(knot),a=ray*6.283185307179586/16;
   auto left=Scale(Sub(sample(x,a),sample(x-e,a)),1/e);
   auto right=Scale(Sub(sample(x+e,a),sample(x,a)),1/e);
   Check(Dot(Sub(left,right),Sub(left,right))<1e-7,"Material section has a root or guide-knot derivative step");
  }
 }
 material.frames[6].basis={Vec{0,0,1},Vec{0,1,0},Vec{-1,0,0}};
 auto top=MaterialPoint(middle,material,{4.5,0,1}),bottom=MaterialPoint(middle,material,{4.5,0,-1});
 Check(std::abs(std::sqrt(Dot(Sub(top,bottom),Sub(top,bottom)))-2)<1e-10,"Section diameter collapses while bending");
 material.frames[12].basis=material.frames[6].basis;
 auto crownA=MaterialPoint(middle,material,{10,2,1},1),crownB=MaterialPoint(middle,material,{12,-1,-2},1);
 Check(std::abs(Dot(Sub(crownA,crownB),Sub(crownA,crownB))-22)<1e-10,"Crown shape is not rigid");
 for(unsigned i=2;i<=12;i++)material.frames[i].origin={.3*(i-1),.1*(i-1),0};
 PreserveMaterialLengths(material);
 for(unsigned i=2;i<=12;i++){auto d=Sub(material.frames[i].origin,material.frames[i-1].origin);Check(std::abs(Dot(d,d)-1)<1e-10,"Material rod compressed");}
 {
  State rod;rod.hasShapeFrames=true;rod.sectionRadius=2;
  for(unsigned i=1;i<=12;i++){rod.shapeFrames[i].origin={double(i-1),0,0};rod.frames[i].origin={0,double(i-1),0};}
  rod.frames[13].origin={2,3,-4};auto lobes=rod.frames[13];
  PreserveMaterialLengths(rod);Vec previous{1,0,0};
  for(unsigned i=2;i<=12;i++){
   auto d=Sub(rod.frames[i].origin,rod.frames[i-1].origin);Check(std::abs(Dot(d,d)-1)<1e-10,"Bend bound changed segment length");
   const double turn=std::acos(std::clamp(Dot(previous,Unit(d)),-1.,1.));
   Check(turn<=.2*(i==2?.5:1.)+1e-10,"Wide material section exceeds its bend radius");previous=Unit(d);
  }
  Check(rod.frames[2].origin[1]>0,"Bend-radius constraint froze the first free interval");
  Check(rod.frames[13].origin==lobes.origin,"Bend-radius constraint moved independent suspension");
  rod.frames=rod.shapeFrames;PreserveMaterialLengths(rod);
  for(unsigned i=1;i<=12;i++)Check(rod.frames[i].origin==rod.shapeFrames[i].origin,"Bend-radius constraint changed the rest shape");
 }
 bool rejected=false;try{moved.scales[1][1]=0;plan.Evaluate(moved);}catch(const std::invalid_argument&){rejected=true;}Check(rejected,"Collapsed morphology accepted");
 std::cout<<"PASS independent shaft/lobe ownership, live connecting skin, exact rest and invalid-scale rejection\n";
 return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
