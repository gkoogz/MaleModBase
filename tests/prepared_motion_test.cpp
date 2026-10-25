#include <malemod/surface/prepared_motion.hpp>
#include <iostream>
using namespace malemod::surface;
int main(){try{
 PreparedMotionFrames a{},b{};for(auto& f:a)f.basis={{{1,0,0},{0,1,0},{0,0,1}}};b=a;
 std::array<PresentationPoint,17> scales;for(auto& s:scales)s={1,1,1};
 std::vector<PresentationPoint> p{{1,2,3},{1,-2,3},{0,0,4},{5,6,7}};
 std::vector<PreparedMotionWeights> w(4);w[0][13]=1;w[1][14]=1;w[2][15]=.5;w[2][16]=.5;
 auto near=[](auto p,auto q){auto d=Sub(p,q);if(Dot(d,d)>1e-22)throw std::runtime_error("Prepared motion changed an independent owner");};
 for(unsigned step=0;step<100;step++){
  b[13].origin={.1*step,0,0};b[14].origin={0,0,-.07*step};b[15].origin=b[16].origin={.05*step,0,-.035*step};
  PreparedMotionPlan plan(a,b,scales);auto out=plan.Evaluate(p,w);
  near(out[0],Add(p[0],b[13].origin));near(out[1],Add(p[1],b[14].origin));near(out[2],Add(p[2],b[15].origin));near(out[3],p[3]);
 }
 // The same prepared material is evaluated at every new sample; no mutable
 // integration or previous mesh result may affect the next publication.
 PreparedMotionPlan identity(a,a,scales);auto reset=identity.Evaluate(p,w);for(unsigned i=0;i<p.size();i++)near(reset[i],p[i]);
 {
  std::vector<PresentationPoint> rest{{0,0,0},{1,0,0},{.5,.0001,0}},moved=rest;
  moved[2][1]=-.00001;std::vector<PreparedMotionWeights> weights(3);weights[2][13]=1;
  auto fixedA=moved[0],fixedB=moved[1];auto result=ConstrainPreparedMotion(rest,moved,{{0,1,2}},weights,.001);
  if(result.projections==0||moved[2][1]<=0||moved[0]!=fixedA||moved[1]!=fixedB)throw std::runtime_error("Thin-face correction did not preserve fixed boundary/orientation");
  moved=rest;moved[2][1]=-.1;bool rejected=false;try{ConstrainPreparedMotion(rest,moved,{{0,1,2}},weights,.001);}catch(const std::runtime_error&){rejected=true;}
  if(!rejected)throw std::runtime_error("Large folding hidden by local correction");
  // Reversing world-facing direction by a proper rotation is not inversion.
  PreparedMotionFrames turned=a;turned[13].basis={{{1,0,0},{0,-1,0},{0,0,-1}}};PreparedMotionPlan turn(a,turned,scales);
  for(auto& w:weights){w={};w[13]=1;}auto rotated=turn.Evaluate(rest,weights);
  auto n=Cross(Sub(rest[1],rest[0]),Sub(rest[2],rest[0]));std::vector<PresentationPoint> expected{turn.Normal(n,weights[0])};
  auto rotationResult=ConstrainPreparedMotion(rest,rotated,{{0,1,2}},weights,.001,&expected);
  if(rotationResult.projections)throw std::runtime_error("Rigid rotation mistaken for surface inversion");
  ValidatePreparedMotionAreas(rotated,{{0,1,2}},{n});
  auto collapsed=rotated;collapsed[2]=Scale(Add(collapsed[0],collapsed[1]),.5);bool collapseRejected=false;
  try{ValidatePreparedMotionAreas(collapsed,{{0,1,2}},{n});}catch(const std::runtime_error&){collapseRejected=true;}
  if(!collapseRejected)throw std::runtime_error("Zero-area moving triangle accepted");
  std::vector<PresentationPoint> strip{{0,0,0},{1,0,0},{.5,.2,.1}};
  std::vector<LiveControlLoft> loft{{2,0,1,.5}};
  std::vector<LiveControlTangents> tangents{{{1,1,0},{1,-1,0}}};
  auto posed=turn.Evaluate(strip,weights);auto expectedStrip=posed;
  TransportPreparedMotionLofts(turn,weights,loft,tangents,strip,posed,0,1);
  for(unsigned i=0;i<strip.size();i++)near(posed[i],expectedStrip[i]);
  auto unchanged=strip;TransportPreparedMotionLofts(identity,weights,loft,tangents,strip,unchanged,0,1);
  for(unsigned i=0;i<strip.size();i++)near(unchanged[i],strip[i]);
 }
 std::cout<<"PASS independent bodies, shared rim, fixed body and history-free fresh samples\n";
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
