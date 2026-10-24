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
 // A rigid rotation supplied by the solver must reach the proximal surface.
 // Neither anchoring nor socket resampling may turn it into distal-only bend.
 for(double angle:{-.65,-.2,.2,.65})for(double radius:{.1,1.,3.}){
  State rod;rod.hasShapeFrames=true;rod.sectionRadius=radius;
  const PresentationBasis rotated{Vec{std::cos(angle),0,std::sin(angle)},Vec{0,1,0},Vec{-std::sin(angle),0,std::cos(angle)}};
  for(unsigned i=1;i<=12;i++){
   rod.shapeFrames[i].origin={double(i-1),0,0};
   rod.frames[i].basis=rotated;rod.frames[i].origin=Scale(rotated[0],double(i-1));
  }
  rod.frames[13].origin={2,3,-4};rod.frames[14].origin={2,-3,-5};
  AnchorRootPhysics(rod,radius,11.);
  Check(Dot(rod.frames[1].basis[0],rotated[0])>1-1e-12,"Anchoring erased simulated root rotation");
  PreserveMaterialLengths(rod);
  for(double start:{0.,1.25,3.}){
   auto socket=SocketMaterialState(rod,start);
   Check(Dot(Sub(socket.frames[1].origin,socket.shapeFrames[1].origin),Sub(socket.frames[1].origin,socket.shapeFrames[1].origin))<1e-20,"Root rotation translated the socket center");
   Check(Dot(socket.frames[1].basis[0],rotated[0])>1-1e-12,"Socket erased simulated root rotation");
   auto early=Sub(socket.frames[2].origin,socket.frames[1].origin);
   Check(Dot(Unit(early),rotated[0])>1-1e-12,"Root rotation was deferred beyond the first interval");
   Binding rim;rim.influences[0]={1,1,{}};
   auto center=socket.shapeFrames[1].origin;
   auto a=MaterialPoint(rim,socket,Add(center,Vec{0,0,radius}));
   auto b=MaterialPoint(rim,socket,Add(center,Vec{0,0,-radius}));
   Check(std::abs(Dot(Sub(a,b),Sub(a,b))-4*radius*radius)<1e-10,"Root participation collapsed the circumference");
   Check(socket.frames[13].origin==rod.frames[13].origin&&socket.frames[14].origin==rod.frames[14].origin,"Root participation moved independent lobes");
  }
 }
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
  auto socket=SocketMaterialState(rod,1.25);
  Check(std::abs(socket.shapeFrames[1].origin[0]-1.25)<1e-10,"Material rod did not start at the final socket");
  Check(socket.shapeFrames[12].origin==rod.shapeFrames[12].origin,"Socket resampling moved the rest tip");
  for(unsigned i=1;i<=12;i++){auto d=Sub(socket.frames[i].origin,socket.shapeFrames[i].origin);Check(Dot(d,d)<1e-20,"Socket resampling changed the zero-motion curve");}
  rod.frames[3].origin[1]=.4;socket=SocketMaterialState(rod,1.25);
  Check(socket.frames[1].origin==socket.shapeFrames[1].origin&&socket.frames[1].basis==socket.shapeFrames[1].basis,"Socket motion moved the attached rim");
  Check(socket.frames[2].origin[1]!=0,"Socket resampling froze the free material rod");
 }
 {
  State pulse;pulse.hasShapeFrames=true;pulse.materialScale={1.12,1.3,1.3};
  for(unsigned i=1;i<=12;i++)pulse.frames[i].origin=pulse.shapeFrames[i].origin={double(i-1),0,0};
  auto original=pulse.shapeFrames;PreserveMaterialLengths(pulse);PreserveMaterialLengths(pulse);
  Check(pulse.shapeFrames[12].origin==original[12].origin,"Pulse modified immutable morphology");
  Check(std::abs(pulse.frames[12].origin[0]-12.32)<1e-10,"Pulse length missing or compounded on repeated constraints");
  Binding root;root.influences={Influence{1,1,{}},Influence{2,0,{}}};
  Check(MaterialPoint(root,pulse,{-.2,1,0})==Vec{-.2,1,0},"Pulse moved the attached circumference");
  Binding middle;middle.influences={Influence{6,.5,{}},Influence{7,.5,{}}};
  auto a=MaterialPoint(middle,pulse,{5.5,1,0}),b=MaterialPoint(middle,pulse,{5.5,-1,0});
  Check(std::abs(std::sqrt(Dot(Sub(a,b),Sub(a,b)))-2.6)<1e-10,"Live pulse failed to dilate the material surface");
  auto socket=SocketMaterialState(pulse,1.25);
  Check(socket.frames[1].origin==socket.shapeFrames[1].origin,"Pulsed socket detached from body");
  Check(std::abs(std::sqrt(Dot(Sub(socket.frames[12].origin,socket.frames[1].origin),Sub(socket.frames[12].origin,socket.frames[1].origin)))-9.75*1.12)<1e-9,"Socket compounded pulse length");
 }
 {
  // Cached sections must reproduce the uncached field across quaternion
  // hemisphere changes, curved guides, material strain and crown blending.
  State s;s.materialScale={1.12,.85,1.3};
  for(unsigned sample=0;sample<24;sample++){
   for(unsigned i=1;i<=12;i++){
    double a=sample*.29+i*.13;
    s.frames[i].origin={double(i),std::sin(a),.2*std::cos(a)};
    s.frames[i].basis={Vec{std::cos(a),std::sin(a),0},Vec{-std::sin(a),std::cos(a),0},Vec{0,0,1}};
    s.shapeFrames[i].origin={double(i),0,0};
   }
   PreparedMaterialSections rest(s.shapeFrames),live(s.frames);
   for(unsigned i=1;i<12;i++)for(unsigned step=0;step<=16;step++){
    const double t=step/16.;Binding b;b.influences={Influence{i,1-t,{}},Influence{i+1,t,{}}};
    const Vec p{double(i)+t,.7,-.3};
    for(double crown:{0.,.4,1.}){
     auto a=MaterialPoint(b,s,p,crown),c=MaterialPoint(b,s,p,crown,&rest,&live);
     Check(Dot(Sub(a,c),Sub(a,c))<1e-24,"Prepared material field changed the surface");
    }
   }
  }
 }
 {
  // Rest construction must not inherit displacement residuals measured in a
  // different live frame. The suspended material is immutable across motion.
  Binding skin;skin.influences={Influence{13,.35,{1,2,3}},Influence{14,.65,{2,-1,4}}};
  for(double angle:{-.9,0.,.9})for(double scale:{.25,1.,3.}){
   State live;live.hasShapeFrames=true;live.materialScale={1.1,1.2,1.2};live.sectionRadius=2.4;
   live.frames[13].origin={4,-3,2};live.frames[14].origin={-2,3,-1};
   live.frames[13].basis={Vec{std::cos(angle),std::sin(angle),0},Vec{-std::sin(angle),std::cos(angle),0},Vec{0,0,1}};
   live.scales[13]=live.scales[14]={scale,scale,scale};
   auto material=std::make_shared<std::vector<Vec>>(1,Vec{2,3,4});live.shaftMaterial=material;
   auto offsets=std::make_shared<std::vector<Vec>>(1,Sub(Vec{7,8,9},malemod::surface::modular::Point(skin,live)));live.offsets=offsets;
   auto rest=RestMaterialState({skin},live);auto reconstructed=Add(malemod::surface::modular::Point(skin,rest),rest.offsets->front());
   Check(Dot(Sub(reconstructed,material->front()),Sub(reconstructed,material->front()))<1e-24,"Live offsets contaminated immutable rest material");
   auto stillLive=Add(malemod::surface::modular::Point(skin,live),live.offsets->front());
   Check(Dot(Sub(stillLive,Vec{7,8,9}),Sub(stillLive,Vec{7,8,9}))<1e-24,"Rest reconstruction changed live motion");
   Check(rest.materialScale==Vec{1,1,1}&&std::abs(rest.sectionRadius-2)<1e-12,"Rest state retained transient strain");
  }
 }
 {
  State s;s.hasShapeFrames=true;s.sectionRadius=1.5;
  for(unsigned i=1;i<=12;i++){
   s.shapeFrames[i].origin={double(i-1),0,0};
   s.frames[i].origin={double(i-1),.03*(i-1)*(i-1),0};
   if(i>1)s.frames[i].basis={Vec{1,0,0},Vec{0,-1,0},Vec{0,0,-1}};
  }
  PreserveMaterialLengths(s);
  for(unsigned i=2;i<=12;i++){
   Check(s.frames[i].basis[2][2]>.999999,"Unsimulated section roll twisted the rod");
   Check(std::abs(Dot(s.frames[i].basis[0],s.frames[i].basis[1]))<1e-12,"Material frame lost orthogonality");
  }
  auto end=s.frames[12].origin;
  Check(end[1]>.1&&end[0]>0,"Solid rod lost flexible bending");
  PreserveMaterialLengths(s);
  Check(Dot(Sub(s.frames[12].origin,end),Sub(s.frames[12].origin,end))<1e-20,"Solid rod correction accumulated");
 }
 bool rejected=false;try{moved.scales[1][1]=0;plan.Evaluate(moved);}catch(const std::invalid_argument&){rejected=true;}Check(rejected,"Collapsed morphology accepted");
 std::cout<<"PASS independent shaft/lobe ownership, live connecting skin, exact rest and invalid-scale rejection\n";
 return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
