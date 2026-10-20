#include <malemod/surface/fixed_root_bend.hpp>
#include <malemod/surface/attachment_hinge.hpp>
#include <iostream>
using namespace malemod::surface::fixed_root;
using namespace malemod::surface::root_motion;
void Check(bool v){if(!v)throw std::runtime_error("Fixed-root bend regression");}
int main(){
 for(double angle:{-.7,-.2,0.,.2,.7}){
  Bend bend({0,0,0},{1,0,0},{0,0,1},20,4,angle);
  for(double x:{-10.,-1.,0.})for(double z:{-5.,0.,5.}){Point p{x,2,z};Check(bend.Apply(p)==p);}
  for(double x:{1.,5.,10.,20.,30.}){
   auto center=bend.Apply({x,0,0});
   for(unsigned i=0;i<24;i++){double a=i*6.283185307179586/24;auto p=bend.Apply({x,4*std::cos(a),4*std::sin(a)});auto d=Add(p,Scale(center,-1));Check(std::abs(Dot(d,d)-16)<1e-8);}
  }
  Check(std::abs(bend.Angle())<=.35*20/(1.875*4)+1e-12);
  auto near=bend.Apply({1e-5,0,4});Check(std::abs(near[2]-4)<1e-9);
  Point at{10,1,2},tangent{1,0,0},normal{0,0,1};
  auto t=bend.Vector(at,tangent),n=bend.Vector(at,normal,true);
  Check(std::abs(Dot(t,n))<1e-10);
  auto numeric=Scale(Add(bend.Apply(Add(at,Scale(tangent,1e-4))),Scale(bend.Apply(Add(at,Scale(tangent,-1e-4))),-1)),5000);
  Check(Dot(Add(numeric,Scale(t,-1)),Add(numeric,Scale(t,-1)))<1e-10);
 }
 Bend shortWide({0,0,0},{1,0,0},{0,0,1},2,10,1);Check(shortWide.Angle()<.04);
 Bend compact({0,0,0},{1,0,0},{0,0,1},8,4,.7853981633974483,.2);
 Check(std::abs(compact.Angle()-.7853981633974483)<1e-12);
 for(double x:{0.,2.,4.,6.,8.,16.})for(double z:{-4.,0.,4.}){
  auto derivative=compact.Vector({x,0,z},{1,0,0});
  Check(Dot(derivative,derivative)>=.2*.2-1e-12);
 }
 // The section beyond the transition has the requested orientation; its
 // centre travels substantially, ruling out a numerically safe no-op.
 auto a=compact.Apply({8,0,0}),b=compact.Apply({24,0,0});
 Check(std::abs(std::atan2(b[2]-a[2],b[0]-a[0])-.7853981633974483)<1e-12);
 Check(b[2]>12);
 using Flow=malemod::surface::attachment_hinge::Flow;
 Flow flow({0,0,0},{1,0,0},{0,0,1},{0,-4,-12},{8,4,-6},8,.5);
 Flow undo({0,0,0},{1,0,0},{0,0,1},{0,-4,-12},{8,4,-6},8,-.5);
 for(Point p:{Point{-1,2,4},Point{0,0,0},Point{4,0,-8}})Check(flow.Apply(p)==p);
 for(Point p:{Point{2,0,2},Point{6,1,-3},Point{12,0,0},Point{30,0,0}}){
  auto delta=Add(undo.Apply(flow.Apply(p)),Scale(p,-1));Check(Dot(delta,delta)<1e-10);
 }
 auto far=flow.Apply({30,0,0});Check(std::abs(std::atan2(far[2],far[0])-.5)<1e-12);
 auto proximal=flow.Apply({10,0,0});Check(proximal[2]>2);
 // Fully recruited shaft sections rotate rigidly, with identical girth.
 auto radial=Add(flow.Apply({30,0,3}),Scale(far,-1));Check(std::abs(Dot(radial,radial)-9)<1e-10);
 Check(malemod::surface::attachment_hinge::AdmissibleGain([](double g){return g<=.6;})==.5);
 Check(malemod::surface::attachment_hinge::AdmissibleGain([](double g){return g==0;})==0);
 std::cout<<"Fixed-root invariance, section radii, continuity and travel bounds passed\n";
}
